from __future__ import annotations

import queue
import threading
import time

from config import load_json
from control.fuzzy_drop_controller import FuzzyDropController
from control.servo_controller import ServoController
from core.shared_state import SharedState
from core.event_bus import EventBus
from flight.mavlink_connection import MavlinkConnection
from flight.mavlink_rx import MavlinkRxWorker
from flight.mavlink_tx import MavlinkTx
from flight.mavlink_stream_config import MavlinkStreamConfigurator
from flight.freshness_policy import FreshnessPolicy
from flight.rangefinder_state import RangefinderStatePolicy
from flight.command_manager import CommandManager
from flight.mission_planner_gateway import MissionPlannerGateway
from flight.lidar_takeoff import ProfiledLidarTakeoff
from mission.fc_mission_sync import FCMissionSync
from mission.full_mission_config import load_full_mission_config
from mission.waypoint_manager import WaypointManager
from mission.mission_manager import MissionManager
from safety.safety_manager import SafetyManager
from streaming.gcs_server import GcsServer
from streaming.vision_stream_session import VisionStreamSession
from system.health import HealthWorker
from system.vehicle_monitor import VehicleMonitor
from system.readiness import ReadinessSupervisor
from system.runtime_logger import RuntimeLogger


class KingphoenixRuntime:
    def __init__(
        self,
        *,
        enable_gcs=True,
        enable_mission_manager=True,
        enable_runtime_logger=True,
        enable_safety_manager=True,
        enable_readiness=True,
    ):
        self.enable_gcs = bool(enable_gcs)
        self.enable_mission_manager = bool(enable_mission_manager)
        self.enable_runtime_logger = bool(enable_runtime_logger)
        self.enable_safety_manager = bool(enable_safety_manager)
        self.enable_readiness = bool(enable_readiness)
        self.cfg = load_json("system.json")
        self.full_cfg = load_full_mission_config()

        # full_mission.json is the operator-facing source of truth for race
        # detection thresholds. Keep the Hailo decoder floor low enough for
        # both fast AUTO-drop confirmation and fuzzy fallback.
        mission_conf = min(
            float(self.full_cfg["vision"]["confidence_threshold"]),
            float(self.full_cfg["guided_fuzzy"]["confidence_threshold"]),
        )
        self.cfg.setdefault("ai", {}).setdefault("detector", {})[
            "confidence_threshold"
        ] = mission_conf
        # BboxControlEstimator has its own confidence gate. Keep it aligned
        # with the mission JSON too, otherwise lowering YOLO confidence in the
        # race config could still yield ex/ey=None and silently block a drop.
        self.cfg.setdefault("vision_control", {})["confidence_min"] = mission_conf

        # Race preflight policy also lives in full_mission.json so field
        # tuning does not require editing multiple config files.
        readiness_cfg = dict(self.cfg.get("readiness", {}) or {})
        readiness_cfg.update(dict(self.full_cfg.get("preflight", {}) or {}))
        self.cfg["readiness"] = readiness_cfg

        self.state = SharedState()
        self.events = EventBus()
        self.stop_event = threading.Event()

        mav_cfg = self.cfg["mavlink"]
        requested_rates = mav_cfg.get("message_rates_hz", {})
        freshness_cfg = mav_cfg.get("freshness_s", {})
        rangefinder_cfg = self.cfg.get("rangefinder", {})

        self.state.system["mav_source_system"] = int(mav_cfg.get("source_system", 1))
        self.state.system["mav_source_component"] = int(mav_cfg.get("source_component", 191))
        self.state.system["mav_target_system"] = int(mav_cfg.get("target_system", 1))
        self.state.system["mav_target_component"] = int(mav_cfg.get("target_component", 1))

        self.connection = MavlinkConnection(mav_cfg)
        self.tx = MavlinkTx(self.connection)
        self.freshness = FreshnessPolicy(self.state.vehicle, freshness_cfg)
        self.rangefinder_policy = RangefinderStatePolicy(
            self.state.vehicle,
            max_age_s=rangefinder_cfg.get("valid_age_s", 0.20),
            expected_orientation=rangefinder_cfg.get("expected_orientation", 25),
            allow_legacy_fallback=rangefinder_cfg.get("allow_legacy_fallback", True),
        )
        self.command = CommandManager(
            self.tx, self.state, self.events,
            freshness_policy=self.freshness, vct_cfg=mav_cfg.get("vct", {}),
        )

        self.waypoints = WaypointManager(self.state, self.full_cfg)
        mp_cfg = self.cfg.get("mission_planner_gateway", {})
        self.mp_mirror_queue = queue.Queue(maxsize=int(mp_cfg.get("mirror_queue_size", 512)))
        self.mission_protocol_queue = queue.Queue(maxsize=256)

        self.rx = MavlinkRxWorker(
            self.connection, self.state, self.events, self.stop_event,
            heartbeat_timeout_s=mav_cfg.get("heartbeat_timeout_s", 2.5),
            mirror_queue=self.mp_mirror_queue if bool(mp_cfg.get("enabled", False)) else None,
            mission_protocol_queue=self.mission_protocol_queue,
        )
        self.stream_config = MavlinkStreamConfigurator(
            self.tx, self.state, self.stop_event, requested_rates,
            retry_interval_s=mav_cfg.get("rate_retry_interval_s", 10.0),
        )

        self.vision_stream = VisionStreamSession(self.cfg, self.state, self.stop_event)
        self.servo = ServoController(self.tx, self.state, self.full_cfg.get("servo", {}))
        sync_cfg = dict(self.full_cfg.get("mission_sync", {}))
        sync_cfg["acceptance_radius_m"] = float(
            self.full_cfg["waypoint"].get("acceptance_radius_m", 0.50)
        )
        self.fc_sync = FCMissionSync(
            self.tx, self.state, self.mission_protocol_queue, sync_cfg
        )
        self.takeoff = ProfiledLidarTakeoff(
            self.state, self.command, self.tx, self.full_cfg["takeoff"]
        )
        self.fuzzy = FuzzyDropController(
            self.state, self.command, self.tx, self.vision_stream,
            self.full_cfg["guided_fuzzy"],
        )
        self.mission = MissionManager(
            self.state, self.events, self.command, self.waypoints, self.fc_sync,
            self.vision_stream, self.servo, self.takeoff, self.fuzzy,
            self.full_cfg, self.stop_event,
        )

        self.safety = None
        if self.enable_safety_manager:
            self.safety = SafetyManager(self.state, self.events, self.stop_event)
        self.gcs = None
        if self.enable_gcs:
            self.gcs = GcsServer(
                self.state, self.events, self.waypoints, self.stop_event,
                command_manager=self.command, servo_controller=self.servo,
            )

        self.mp_gateway = None
        if bool(mp_cfg.get("enabled", False)):
            self.mp_gateway = MissionPlannerGateway(
                self.state, self.tx, self.mp_mirror_queue, self.stop_event, cfg=mp_cfg
            )

        self.health = HealthWorker(
            self.state, self.stop_event, cfg=self.cfg.get("pi_health", {}),
            hz=self.cfg.get("rates_hz", {}).get("health", 1.0),
        )
        self.vehicle_monitor = VehicleMonitor(
            self.state, self.stop_event, requested_rates_hz=requested_rates,
            freshness_policy=self.freshness,
            hz=self.cfg.get("rates_hz", {}).get("vehicle_monitor", 2.0),
        )
        self.readiness = ReadinessSupervisor(
            self.state, self.stop_event, cfg=self.cfg.get("readiness", {}),
            hz=self.cfg.get("rates_hz", {}).get("readiness", 2.0),
        )
        logging_cfg = dict(self.full_cfg.get("logging", {}) or {})
        self.runtime_logger = RuntimeLogger(
            self.state, self.stop_event, logging_cfg
        )

        # Production full-mission uses the compact RuntimeLogger. The legacy
        # VehicleMonitor full-screen telemetry table is opt-in for diagnostics
        # only because at 2 Hz it floods the field console and hides mission
        # events, AI transitions, servo ACKs and safety messages.
        self.workers = [
            self.rx, self.stream_config, self.health,
        ]
        if self.enable_readiness:
            self.workers.append(self.readiness)
        if self.safety is not None:
            self.workers.insert(2, self.safety)
        if self.enable_runtime_logger:
            self.workers.append(self.runtime_logger)
        if self.gcs is not None:
            self.workers.insert(3, self.gcs)
        if bool(logging_cfg.get("print_vehicle_state_table", False)):
            self.workers.insert(-2, self.vehicle_monitor)
        if self.mp_gateway is not None:
            self.workers.insert(4, self.mp_gateway)

    def start(self):
        with self.state.lock:
            self.state.system["running"] = True
            self.state.system["startup_state"] = "INITIALIZING"
            self.state.system["ready_for_mission"] = False

        self.connection.open()
        for worker in self.workers:
            worker.start()

        # Both cameras start immediately and stay warm independently of GCS.
        # AI is explicitly OFF until the WP6 drop-arming event.
        self.vision_stream.start()
        self.vision_stream.disable_ai(source="FULL_MISSION_STARTUP")
        with self.state.lock:
            self.state.system["startup_state"] = "SELF_TEST"

        deadline = time.monotonic() + 5.0
        while not self.state.vehicle.connected and time.monotonic() < deadline:
            time.sleep(0.05)

        vision_deadline = time.monotonic() + 3.0
        while time.monotonic() < vision_deadline:
            front = self.state.get_vision_snapshot("front")
            bottom = self.state.get_vision_snapshot("bottom")
            if bool(front.get("camera_open")) and bool(bottom.get("camera_open")):
                break
            time.sleep(0.05)

        ok, message = self.servo.initialize()
        print(("[SERVO] " if ok else "[WARN] ") + message)

        if self.enable_mission_manager:
            self.mission.start()
        with self.state.lock:
            self.state.system["startup_state"] = "READY_FOR_OPERATOR"

    def wait(self):
        while not self.stop_event.is_set():
            time.sleep(0.2)

    def stop(self):
        if self.stop_event.is_set():
            return
        print("[RUNTIME] stopping")
        self.stop_event.set()
        with self.state.lock:
            self.state.system["running"] = False
            self.state.system["ready_for_mission"] = False
            self.state.system["startup_state"] = "STOPPED"
        for worker in self.workers:
            if hasattr(worker, "join"):
                worker.join(timeout=1.0)
        if self.enable_mission_manager and self.mission.is_alive():
            self.mission.join(timeout=1.0)
        self.vision_stream.join(timeout=1.0)
        self.connection.close()
