from __future__ import annotations

import threading
import time

from config import load_json
from core.shared_state import SharedState
from core.event_bus import EventBus
from flight.mavlink_connection import MavlinkConnection
from flight.mavlink_rx import MavlinkRxWorker
from flight.mavlink_tx import MavlinkTx
from flight.mavlink_stream_config import MavlinkStreamConfigurator
from flight.freshness_policy import FreshnessPolicy
from flight.rangefinder_state import RangefinderStatePolicy
from flight.command_manager import CommandManager
from mission.mission_loader import MissionLoader
from mission.waypoint_manager import WaypointManager
from mission.mission_manager import MissionManager
from safety.safety_manager import SafetyManager
from streaming.gcs_server import GcsServer
from streaming.vision_stream_session import VisionStreamSession
from system.health import HealthWorker
from system.vehicle_monitor import VehicleMonitor


class KingphoenixRuntime:
    def __init__(self):
        self.cfg = load_json("system.json")
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

        # V1.3 central freshness policy. It consumes VehicleState only.
        self.freshness = FreshnessPolicy(self.state.vehicle, freshness_cfg)

        self.rangefinder_policy = RangefinderStatePolicy(
            self.state.vehicle,
            max_age_s=rangefinder_cfg.get("valid_age_s", 0.15),
            expected_orientation=rangefinder_cfg.get("expected_orientation", 25),
            allow_legacy_fallback=rangefinder_cfg.get("allow_legacy_fallback", True),
        )

        self.command = CommandManager(
            self.tx,
            self.state,
            self.events,
            freshness_policy=self.freshness,
            vct_cfg=mav_cfg.get("vct", {}),
        )

        self.loader = MissionLoader()
        self.waypoints = WaypointManager(self.state)

        self.rx = MavlinkRxWorker(
            self.connection,
            self.state,
            self.events,
            self.stop_event,
            heartbeat_timeout_s=mav_cfg.get("heartbeat_timeout_s", 2.5),
        )

        self.stream_config = MavlinkStreamConfigurator(
            self.tx,
            self.state,
            self.stop_event,
            requested_rates,
            retry_interval_s=mav_cfg.get("rate_retry_interval_s", 10.0),
        )

        self.mission = MissionManager(
            self.state,
            self.events,
            self.command,
            self.loader,
            self.waypoints,
            self.stop_event,
        )
        self.safety = SafetyManager(self.state, self.events, self.stop_event)
        self.gcs = GcsServer(self.state, self.events, self.waypoints, self.stop_event)
        self.health = HealthWorker(
            self.state,
            self.stop_event,
            cfg=self.cfg.get("pi_health", {}),
            hz=self.cfg.get("rates_hz", {}).get("health", 1.0),
        )
        self.vision_stream = VisionStreamSession(self.cfg, self.state, self.stop_event)

        self.vehicle_monitor = VehicleMonitor(
            self.state,
            self.stop_event,
            requested_rates_hz=requested_rates,
            freshness_policy=self.freshness,
            hz=self.cfg.get("rates_hz", {}).get("vehicle_monitor", 2.0),
        )

        self.workers = [
            self.rx,
            self.stream_config,
            self.safety,
            self.mission,
            self.gcs,
            self.health,
            self.vehicle_monitor,
        ]

    def start(self):
        print("[RUNTIME] starting Kingphoenix V2.1 Pi Health + Lightweight YOLO26")
        self.state.system["running"] = True
        self.connection.open()

        for worker in self.workers:
            worker.start()

        # V1.9.x vision/streaming branch. This sends video + metadata only.
        # It never sends movement MAVLink commands.
        self.vision_stream.start()

    def wait(self):
        while not self.stop_event.is_set():
            time.sleep(0.2)

    def stop(self):
        if self.stop_event.is_set():
            return

        print("[RUNTIME] stopping")
        self.stop_event.set()
        self.state.system["running"] = False

        for worker in self.workers:
            if hasattr(worker, "join"):
                worker.join(timeout=1.0)

        self.vision_stream.join(timeout=1.0)
        self.connection.close()
