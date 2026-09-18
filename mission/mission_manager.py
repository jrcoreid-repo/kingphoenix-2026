from __future__ import annotations

import threading
import time

from control.fuzzy_drop_controller import FuzzyDropError
from flight.lidar_takeoff import LidarTakeoffError
from mission.fc_mission_sync import MissionSyncError


class MissionRunError(RuntimeError):
    pass


class MissionManager(threading.Thread):
    """Kingphoenix full-race mission orchestrator."""

    daemon = True

    def __init__(self, state, events, command, waypoints, fc_sync, vision, servo,
                 takeoff, fuzzy, full_cfg, stop_event):
        super().__init__(name="MissionManager")
        self.state = state
        self.events = events
        self.command = command
        self.waypoints = waypoints
        self.fc_sync = fc_sync
        self.vision = vision
        self.servo = servo
        self.takeoff = takeoff
        self.fuzzy = fuzzy
        self.cfg = full_cfg
        self.stop_event = stop_event
        self.abort_event = threading.Event()
        self._mission_lock = threading.RLock()
        self._mission_thread = None
        self._sync_lock = threading.RLock()
        self._reached_lock = threading.RLock()
        self._reached = set()
        self._drop_lock = threading.RLock()
        self.drop_done = False
        self.drop_confirm = 0
        self.drop_last_frame = None
        self.drop_armed = False

    @property
    def mission_active(self):
        return self._mission_thread is not None and self._mission_thread.is_alive()

    def _set_status(self, status: str, **extra):
        with self.state.lock:
            self.state.mission.status = str(status)
            for k, v in extra.items():
                setattr(self.state.mission, k, v)

    def _startup_report(self):
        m = self.cfg["mission"]; t = self.cfg["takeoff"]; w = self.cfg["waypoint"]
        v = self.cfg["vision"]; f = self.cfg["guided_fuzzy"]
        print("============================================================")
        print(" KINGPHOENIX 2026 - FULL RACE RUNTIME")
        print("============================================================")
        print(
            f"[PLAN] WP={m['total_waypoints']} | TO={t['target_alt_m']:.2f}m@{t['fast_vz_mps']:.2f}m/s | "
            f"WPalt={w['default_alt_m']:.2f}m | AI@WP{m['ai_enable_wp']} | DROP@WP{m['drop_wp']} | "
            f"YOLO={v['confidence_threshold']:.2f}/{v['confirm_frames']}F | FUZZY={f['timeout_s']:.0f}s | "
            f"FINAL={w['final_wp_alt_m']:.2f}m"
        )
        print("[PLAN] AUTO speed/accel/cornering: FC PARAMETERS (NO Pi override)")
        mav_ok = self.state.vehicle.connected and self.state.vehicle.heartbeat_age_s() <= 2.5
        front = self.state.get_vision_snapshot("front")
        bottom = self.state.get_vision_snapshot("bottom")
        hailo_ok = self.vision.shared_ai is not None and bool(
            getattr(self.vision.shared_ai, "ready", False)
        )
        servo_state = str(getattr(self.servo, "logical_state", "UNKNOWN"))
        print(
            f"[INIT] MAVLink={'OK' if mav_ok else 'WAIT'} | "
            f"FrontCam={'OK' if front.get('camera_open') else 'WAIT'} | "
            f"BottomCam={'OK' if bottom.get('camera_open') else 'WAIT'} | "
            f"Hailo={'OK' if hailo_ok else 'WAIT'} | Servo={servo_state}"
        )
        for strategy in ("LEFT", "RIGHT"):
            ok, detail = self.waypoints.validate_strategy(strategy, int(m["total_waypoints"]))
            print(f"[MISSION] {'PASS' if ok else 'WARN'} {strategy}: {detail}")
        print("[READY] Waiting START L / START R | live preflight runs on START")

    def _activate_drop_ai(self):
        if self.drop_done or self.drop_armed:
            return
        cfg = self.cfg["vision"]
        self.vision.set_ai_route(
            "bottom", model_mode=str(cfg.get("model_mode", "YOLO26_ONLY")),
            task=str(cfg.get("task", "DROP_ALIGNMENT")),
            detector_class_id=int(cfg.get("detector_class_id", 0)),
            source="MISSION_WP6",
        )
        with self._drop_lock:
            self.drop_armed = True
            self.drop_confirm = 0
            self.drop_last_frame = None
        print("[MISSION] WP6 REACHED | BOTTOM AI ON | DROP ARMED")

    def _disable_ai(self, source):
        try:
            self.vision.disable_ai(source=source)
        except Exception as exc:
            print(f"[WARN] AI disable failed: {exc}")

    def _drop_monitor_once(self):
        with self._drop_lock:
            if not self.drop_armed or self.drop_done:
                return self.drop_done
        c = self.cfg["vision"]
        snap = self.state.get_vision_snapshot("bottom", max_age_s=float(c.get("freshness_s", 0.20)))
        frame_id = snap.get("frame_id")
        if frame_id is None or frame_id == self.drop_last_frame:
            return False
        self.drop_last_frame = frame_id
        conf = snap.get("confidence"); ex = snap.get("ex"); ey = snap.get("ey")
        valid = bool(
            snap.get("usable") and conf is not None
            and float(conf) >= float(c["confidence_threshold"])
            and ex is not None and ey is not None
        )
        if valid and bool(c.get("coarse_center_required", True)):
            lim = float(c.get("coarse_center_error_max", 0.35))
            valid = abs(float(ex)) <= lim and abs(float(ey)) <= lim
        self.drop_confirm = self.drop_confirm + 1 if valid else 0
        needed = int(c.get("confirm_frames", 2))
        if self.drop_confirm < needed:
            return False
        ok, message = self.servo.open(source="VISION_DROP")
        if not ok:
            raise MissionRunError(message)
        with self._drop_lock:
            self.drop_done = True
            self.drop_armed = False
        print(f"[DROP] TARGET CONFIRMED {self.drop_confirm}/{needed} -> {message}")
        if bool(c.get("disable_ai_after_drop", True)):
            self._disable_ai("DROP_COMPLETE")
        return True

    def _preflight(self, strategy: str):
        if bool(self.waypoints.edit_unlocked):
            raise MissionRunError("START blocked: waypoint SETUP/EDIT mode is still unlocked")
        total = int(self.cfg["mission"]["total_waypoints"])
        ok, detail = self.waypoints.validate_strategy(strategy, total)
        if not ok:
            raise MissionRunError(detail)
        with self.state.lock:
            readiness = dict(self.state.system.get("readiness") or {})
            ready_for_mission = bool(self.state.system.get("ready_for_mission", False))
        if not ready_for_mission:
            failed = list(readiness.get("failed_critical") or [])
            detail = ",".join(failed) if failed else str(readiness.get("status", "NOT_READY"))
            raise MissionRunError(f"START blocked: readiness NOT READY ({detail})")

        v = self.state.vehicle
        if not v.connected or v.heartbeat_age_s() > 2.5:
            raise MissionRunError("MAVLink heartbeat not ready")
        if v.armed:
            raise MissionRunError("START blocked: vehicle already ARMED")
        if v.gps_fix_type is not None and int(v.gps_fix_type) < 3:
            raise MissionRunError(f"GPS fix insufficient ({v.gps_fix_type})")
        if not v.rangefinder_is_fresh(float(self.cfg["takeoff"].get("lidar_stale_s", 0.60))):
            raise MissionRunError("LiDAR not fresh")
        if self.vision.shared_ai is None or not bool(getattr(self.vision.shared_ai, "ready", False)):
            raise MissionRunError("Hailo/YOLO not ready")
        servo_cfg = self.cfg.get("servo") or {}
        if bool(servo_cfg.get("require_closed_before_start", True)):
            servo_state = str(getattr(self.servo, "logical_state", "UNKNOWN")).upper()
            if servo_state != "CLOSE":
                raise MissionRunError(
                    f"START blocked: payload servo is {servo_state}; CLOSE it manually from GCS SETUP first"
                )
        bottom = self.state.get_vision_snapshot("bottom")
        if not bool(bottom.get("camera_open", False)):
            raise MissionRunError("bottom camera not open")

    def _wait_reached(self, seq: int, *, monitor_drop=False):
        timeout = float(self.cfg["mission"].get("auto_wp_timeout_s", 60.0))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.abort_event.is_set():
                raise MissionRunError("ABORTED")
            if monitor_drop:
                self._drop_monitor_once()
            with self._reached_lock:
                if int(seq) in self._reached:
                    return
            time.sleep(0.01 if monitor_drop else 0.03)
        raise MissionRunError(f"AUTO timeout waiting WP{seq}")

    def _switch_auto(self, seq: int):
        self.fc_sync.set_current(int(seq))
        result = self.command.set_mode("AUTO")
        if not result.ok:
            raise MissionRunError(f"AUTO failed: {result.reason}")
        print(f"[MISSION] AUTO -> WP{seq}")

    def _run_mission(self, strategy: str):
        strategy = str(strategy).upper()
        try:
            with self._mission_lock:
                self.abort_event.clear()
                with self._reached_lock:
                    self._reached.clear()
                with self._drop_lock:
                    self.drop_done = False
                    self.drop_confirm = 0
                    self.drop_last_frame = None
                    self.drop_armed = False

                self._set_status("PREFLIGHT", strategy=strategy, abort_reason="")
                self._disable_ai("MISSION_START")
                print(f"[MISSION] START {strategy}")
                self._preflight(strategy)
                self._set_status("SYNC_MISSION")
                self.fc_sync.ensure(self.waypoints, strategy, self.cfg)

                self._set_status("TAKEOFF")
                self.takeoff.run(abort_event=self.abort_event)

                self._set_status("AUTO")
                self._switch_auto(1)

                ai_wp = int(self.cfg["mission"]["ai_enable_wp"])
                drop_wp = int(self.cfg["mission"]["drop_wp"])
                final_wp = int(self.cfg["mission"]["final_wp"])

                for wp in range(1, ai_wp + 1):
                    self._wait_reached(wp)
                self._activate_drop_ai()
                self._wait_reached(drop_wp, monitor_drop=True)

                if not self.drop_done:
                    print(f"[DROP] WP{drop_wp} reached without release -> GUIDED FUZZY fallback")
                    self._set_status("FUZZY_DROP")
                    cfg = self.cfg["vision"]
                    self.vision.set_ai_route(
                        "bottom", model_mode=str(cfg.get("model_mode", "YOLO26_ONLY")),
                        task=str(cfg.get("task", "DROP_ALIGNMENT")),
                        detector_class_id=int(cfg.get("detector_class_id", 0)),
                        source="FUZZY_FALLBACK",
                    )
                    centered = self.fuzzy.run(abort_event=self.abort_event)
                    if not centered:
                        raise MissionRunError("fuzzy drop timeout; payload NOT released")
                    ok, message = self.servo.open(source="FUZZY_DROP")
                    if not ok:
                        raise MissionRunError(message)
                    with self._drop_lock:
                        self.drop_done = True
                        self.drop_armed = False
                    print(f"[DROP] FUZZY CENTERED -> {message}")
                    self._disable_ai("FUZZY_DROP_COMPLETE")
                    self._set_status("AUTO")
                    self._switch_auto(drop_wp + 1)

                for wp in range(drop_wp + 1, final_wp + 1):
                    self._wait_reached(wp)

                print(f"[MISSION] WP{final_wp} REACHED -> LAND")
                self._set_status("LANDING")
                land = self.command.land()
                if not land.ok:
                    raise MissionRunError(f"LAND command failed: {land.reason}")

                land_timeout = float((self.cfg.get("landing") or {}).get("timeout_s", 45.0))
                deadline = time.monotonic() + land_timeout
                while self.state.vehicle.armed and time.monotonic() < deadline:
                    if self.abort_event.is_set():
                        raise MissionRunError("ABORTED")
                    time.sleep(0.10)

                if self.state.vehicle.armed:
                    raise MissionRunError(
                        f"LAND timeout after {land_timeout:.1f}s; vehicle still ARMED"
                    )

                self._set_status("COMPLETE")
                print("[MISSION] COMPLETE | vehicle DISARMED")

        except (MissionRunError, MissionSyncError, LidarTakeoffError, FuzzyDropError) as exc:
            self._disable_ai("MISSION_ERROR")
            self.tx_zero_best_effort()
            if str(exc) == "ABORTED":
                self._set_status("ABORTED", abort_reason="GCS_ABORT")
                print("[MISSION] ABORTED")
            else:
                self._set_status("ERROR", abort_reason=str(exc))
                print(f"[ERROR] MISSION: {exc}")
                self._safe_hold("MISSION_ERROR")
        except Exception as exc:
            self._disable_ai("MISSION_EXCEPTION")
            self.tx_zero_best_effort()
            self._set_status("ERROR", abort_reason=f"{type(exc).__name__}: {exc}")
            print(f"[ERROR] MISSION UNEXPECTED: {type(exc).__name__}: {exc}")
            self._safe_hold("MISSION_EXCEPTION")

    def tx_zero_best_effort(self):
        try:
            self.fuzzy._zero()
        except Exception:
            pass

    def _safe_hold(self, reason):
        if not self.state.vehicle.armed or self.state.vehicle.mode == "LAND":
            return
        mode = str(self.cfg["mission"].get("abort_mode", "LOITER")).upper()
        result = self.command.set_mode(mode)
        if result.ok:
            print(f"[MISSION] {reason} -> {mode}")
        else:
            print(f"[WARN] {reason}: could not enter {mode}: {result.reason}")

    def _abort_now(self, reason):
        self.abort_event.set()
        self._disable_ai("GLOBAL_ABORT")
        self.tx_zero_best_effort()
        with self.state.lock:
            self.state.mission.abort_reason = str(reason)
        self._safe_hold("ABORT")

    def sync_strategy(self, strategy: str):
        strategy = str(strategy).upper()
        if self.mission_active:
            print(f"[WARN] waypoint sync {strategy} deferred: mission active")
            return
        if self.state.vehicle.armed:
            print(f"[WARN] waypoint sync {strategy} blocked: vehicle ARMED")
            return
        try:
            with self._sync_lock:
                ok, detail = self.waypoints.validate_strategy(
                    strategy, int(self.cfg["mission"]["total_waypoints"])
                )
                if not ok:
                    print(f"[MISSION] WARN {strategy} sync skipped: {detail}")
                    return
                self.fc_sync.ensure(self.waypoints, strategy, self.cfg)
        except Exception as exc:
            print(f"[ERROR] mission sync {strategy}: {exc}")

    def run(self):
        self._startup_report()
        while not self.stop_event.is_set():
            event = self.events.get(timeout=0.10)
            if event is None:
                continue
            name = str(event.name)
            payload = event.payload or {}

            if name == "MISSION_ITEM_REACHED":
                # Ignore delayed/stale reached packets before AUTO begins.
                if str(self.state.mission.status).upper() != "AUTO":
                    continue
                seq = int(payload.get("seq", -1))
                with self._reached_lock:
                    first = seq not in self._reached
                    self._reached.add(seq)
                if first and seq > 0:
                    print(f"[MISSION] WP{seq} REACHED")
                if seq == int(self.cfg["mission"]["ai_enable_wp"]):
                    self._activate_drop_ai()
                continue

            if name == "START_MISSION":
                strategy = str(payload.get("strategy") or self.state.mission.strategy).upper()
                if self.mission_active:
                    print("[WARN] START ignored: mission already active")
                    continue
                self._mission_thread = threading.Thread(
                    target=self._run_mission, args=(strategy,),
                    name=f"FullMission-{strategy}", daemon=True,
                )
                self._mission_thread.start()
                continue

            if name == "GLOBAL_ABORT":
                self._abort_now(payload.get("reason", "GLOBAL_ABORT"))
                continue

            if name == "WAYPOINT_EDIT_LOCKED":
                strategy = str(payload.get("strategy") or self.state.mission.strategy).upper()
                threading.Thread(
                    target=self.sync_strategy, args=(strategy,),
                    name=f"MissionSync-{strategy}", daemon=True,
                ).start()
