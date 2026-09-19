from __future__ import annotations

import threading
import time

from control.fuzzy_drop_controller import FuzzyDropError
from flight.lidar_takeoff import LidarTakeoffError
from mission.fc_mission_sync import MissionSyncError
from safety.pilot_takeover import PilotTakeover


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
        self._setup_ai_stop = threading.Event()
        self._setup_ai_thread = None

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
        v = self.cfg["vision"]; f = self.cfg["guided_fuzzy"]; d = self.cfg["drop"]
        fc_preloaded = str(m.get("source", "")).upper() == "FC_PRELOADED"
        print("============================================================")
        print(" KINGPHOENIX 2026 - FULL RACE RUNTIME")
        print("============================================================")
        print(
            f"[PLAN] source={'FC_PRELOADED' if fc_preloaded else 'JSON_SYNC'} | "
            f"TO={t['target_alt_m']:.2f}m@{t['fast_vz_mps']:.2f}m/s | "
            f"START=SEQ{int(m.get('start_seq', 1))} | AI=SEQ{int(m.get('vision_enable_seq', m.get('ai_enable_wp', 6)))} | "
            f"DROP=SEQ{int(m.get('drop_seq', m.get('drop_wp', 7)))} | "
            f"YOLO={v['confidence_threshold']:.2f}/{v['confirm_frames']}F | "
            f"SUCCESS_SKIP=SEQ{int(m.get('resume_seq', 8))} | "
            f"LAND=SEQ{int(m.get('land_seq', m.get('final_wp', 11)))}"
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
        if fc_preloaded:
            print("[MISSION] FC mission is authoritative | Pi will READ only; no waypoint upload/sync")
        else:
            for strategy in ("LEFT", "RIGHT"):
                ok, detail = self.waypoints.validate_strategy(strategy, int(m["total_waypoints"]))
                print(f"[MISSION] {'PASS' if ok else 'WARN'} {strategy}: {detail}")
        print("[STANDBY] Waiting START | live preflight gates every START")

    def _activate_drop_ai(self):
        if self.drop_done or self.drop_armed:
            return
        cfg = self.cfg["vision"]
        self.vision.set_ai_route(
            "bottom", model_mode=str(cfg.get("model_mode", "YOLO26_ONLY")),
            task=str(cfg.get("task", "DROP_ALIGNMENT")),
            detector_class_id=int(cfg.get("detector_class_id", 0)),
            source="MISSION_VISION_TRIGGER",
        )
        with self._drop_lock:
            self.drop_armed = True
            self.drop_confirm = 0
            self.drop_last_frame = None
        print("[MISSION] VISION TRIGGER -> BOTTOM AI ON | DROP ARMED")

    def _disable_ai(self, source):
        try:
            self.vision.disable_ai(source=source)
        except Exception as exc:
            print(f"[WARN] AI disable failed: {exc}")


    def _setup_ai_loop(self):
        cfg = dict(self.cfg.get("setup_ai") or {})
        interval = max(0.5, float(cfg.get("switch_interval_s", 10.0)))
        model_mode = str(cfg.get("model_mode", "YOLO26_ONLY"))
        sequence = (
            ("front", dict(cfg.get("front") or {})),
            ("bottom", dict(cfg.get("bottom") or {})),
        )
        print(f"[SETUP-AI] ACTIVE | FRONT <-> BOTTOM every {interval:.1f}s")
        index = 0
        while not self._setup_ai_stop.is_set():
            if not self.waypoints.edit_unlocked or self.mission_active:
                break
            camera, route_cfg = sequence[index % len(sequence)]
            self.vision.set_ai_route(
                camera,
                model_mode=model_mode,
                task=str(route_cfg.get("task", "--")),
                detector_class_id=route_cfg.get("detector_class_id"),
                source="SETUP_AI_TIMER",
            )
            print(
                f"[SETUP-AI] {camera.upper()} | model={model_mode} | "
                f"task={route_cfg.get('task', '--')} | "
                f"class={route_cfg.get('detector_class_id')} | next={interval:.1f}s"
            )
            index += 1
            if self._setup_ai_stop.wait(interval):
                break
        print("[SETUP-AI] SWITCHER STOPPED")

    def _start_setup_ai(self):
        cfg = dict(self.cfg.get("setup_ai") or {})
        if not bool(cfg.get("enabled", True)):
            return
        if self.mission_active:
            print("[WARN] SETUP AI ignored: mission active")
            return
        if self.state.vehicle.armed:
            print("[WARN] SETUP AI ignored: vehicle ARMED")
            return
        if self._setup_ai_thread is not None and self._setup_ai_thread.is_alive():
            return
        self._setup_ai_stop.clear()
        self._setup_ai_thread = threading.Thread(
            target=self._setup_ai_loop,
            name="SetupAiSwitcher",
            daemon=True,
        )
        self._setup_ai_thread.start()

    def _stop_setup_ai(self, source="SETUP_EXIT"):
        self._setup_ai_stop.set()
        thread = self._setup_ai_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=0.25)
        self._setup_ai_thread = None
        self._disable_ai(source)
        print(f"[SETUP-AI] OFF | source={source}")

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
        mission_cfg = dict(self.cfg.get("mission") or {})
        fc_preloaded = str(mission_cfg.get("source", "")).upper() == "FC_PRELOADED"
        if not fc_preloaded:
            total = int(mission_cfg["total_waypoints"])
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
            if str(self.state.vehicle.mode).upper() != "AUTO":
                raise PilotTakeover(
                    f"operator changed mode during AUTO mission to {self.state.vehicle.mode}"
                )
            if monitor_drop:
                self._drop_monitor_once()
            with self._reached_lock:
                if int(seq) in self._reached:
                    return
            time.sleep(0.01 if monitor_drop else 0.03)
        raise MissionRunError(f"AUTO timeout waiting WP{seq}")

    def _switch_auto(self, seq: int):
        if self.abort_event.is_set():
            raise MissionRunError("ABORTED")
        self.fc_sync.set_current(int(seq))
        if self.abort_event.is_set():
            raise MissionRunError("ABORTED")
        result = self.command.set_mode("AUTO")
        if not result.ok:
            raise MissionRunError(f"AUTO failed: {result.reason}")
        print(f"[MISSION] AUTO -> WP{seq}")

    def _validate_fc_preloaded_mission(self):
        """Read/validate Mission Planner mission without modifying FC."""
        mission_cfg = dict(self.cfg.get("mission") or {})
        actual = self.fc_sync.download()
        required = [
            int(mission_cfg.get("start_seq", 3)),
            int(mission_cfg.get("vision_enable_seq", 8)),
            int(mission_cfg.get("drop_seq", 9)),
            int(mission_cfg.get("resume_seq", 13)),
            int(mission_cfg.get("land_seq", 17)),
        ]
        count = len(actual)
        missing = [seq for seq in required if seq < 0 or seq >= count]
        if missing:
            raise MissionRunError(
                f"FC mission missing required seq {missing}; count={count}"
            )
        print(
            f"[MISSION] FC mission READ OK | items={count} | "
            f"start={required[0]} ai={required[1]} drop={required[2]} "
            f"resume={required[3]} land={required[4]}"
        )
        return actual

    def _monitor_vision_until_drop_point(self, drop_seq: int, resume_seq: int):
        """Vision success skips MP fallback block; failure leaves FC mission untouched."""
        timeout = float(self.cfg["mission"].get("auto_wp_timeout_s", 60.0))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.abort_event.is_set():
                raise MissionRunError("ABORTED")
            if str(self.state.vehicle.mode).upper() != "AUTO":
                raise PilotTakeover(
                    f"operator changed mode during AUTO mission to {self.state.vehicle.mode}"
                )

            if self._drop_monitor_once():
                # Physical release happened. Give servo command a short settling
                # window, then skip MP DELAY/DO_SET_SERVO fallback block.
                settle = float((self.cfg.get("drop") or {}).get("post_release_settle_s", 0.25))
                if settle > 0:
                    time.sleep(settle)
                try:
                    self.fc_sync.set_current(int(resume_seq))
                    print(
                        f"[DROP] VISION SUCCESS -> fallback block skipped -> SEQ{int(resume_seq)}"
                    )
                    return "VISION"
                except MissionSyncError as exc:
                    # Fail-safe behavior: do not fight the FC. Its original
                    # DELAY/DO_SET_SERVO mission remains intact.
                    print(
                        f"[WARN] vision drop done but skip to SEQ{int(resume_seq)} failed: {exc} | "
                        "FC fallback mission remains active"
                    )
                    return "VISION_SKIP_FAILED"

            with self._reached_lock:
                if int(drop_seq) in self._reached:
                    self._disable_ai("FC_FALLBACK_BLOCK")
                    with self._drop_lock:
                        self.drop_armed = False
                    print(
                        f"[DROP] SEQ{int(drop_seq)} reached without vision release -> "
                        "leave FC DELAY/DO_SET_SERVO fallback untouched"
                    )
                    return "FC_FALLBACK"

            time.sleep(0.01)

        raise MissionRunError(f"AUTO timeout waiting drop SEQ{int(drop_seq)}")

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
                self._stop_setup_ai("MISSION_START")
                print("[MISSION] START | FC-PRELOADED SIMPLE RACE")
                self._preflight(strategy)

                mission_cfg = dict(self.cfg.get("mission") or {})
                fc_preloaded = str(mission_cfg.get("source", "")).upper() == "FC_PRELOADED"

                if fc_preloaded:
                    self._validate_fc_preloaded_mission()
                else:
                    self._set_status("SYNC_MISSION")
                    self.fc_sync.ensure(self.waypoints, strategy, self.cfg)

                # Keep the proven Kingphoenix LiDAR takeoff profile.
                self._set_status("TAKEOFF")
                self.takeoff.run(abort_event=self.abort_event)

                start_seq = int(mission_cfg.get("start_seq", 1))
                vision_seq = int(mission_cfg.get("vision_enable_seq", mission_cfg.get("ai_enable_wp", 6)))
                drop_seq = int(mission_cfg.get("drop_seq", mission_cfg.get("drop_wp", 7)))
                resume_seq = int(mission_cfg.get("resume_seq", drop_seq + 1))
                land_seq = int(mission_cfg.get("land_seq", mission_cfg.get("final_wp", 11)))

                self._set_status("AUTO")
                self._switch_auto(start_seq)

                # FC owns all navigation. Pi waits only for the one event it
                # needs: enable bottom vision at the configured mission seq.
                self._wait_reached(vision_seq)
                self._activate_drop_ai()
                self._monitor_vision_until_drop_point(drop_seq, resume_seq)

                # Do not command LAND or rewrite the mission. Mission Planner's
                # uploaded mission remains authoritative through its LAND item.
                print(f"[MISSION] FC autonomous mission continues -> LAND SEQ{land_seq}")
                self._set_status("AUTO")

                # Wait for FC LAND/disarm. This loop is observational only.
                land_timeout = float((self.cfg.get("landing") or {}).get("timeout_s", 45.0))
                while not self.abort_event.is_set():
                    if not self.state.vehicle.armed:
                        break
                    mode = str(self.state.vehicle.mode).upper()
                    if mode == "LAND":
                        self._set_status("LANDING")
                    time.sleep(0.10)

                if self.abort_event.is_set():
                    raise MissionRunError("ABORTED")

                self._set_status("COMPLETE")
                print("[MISSION] COMPLETE | vehicle DISARMED")

        except PilotTakeover as exc:
            self._disable_ai("PILOT_TAKEOVER")
            self.tx_zero_best_effort()
            self._set_status("ABORTED", abort_reason=str(exc))
            print(f"[PILOT TAKEOVER] {exc} | autonomous mode command suppressed")
        except (MissionRunError, MissionSyncError, LidarTakeoffError, FuzzyDropError) as exc:
            self._disable_ai("MISSION_ERROR")
            self.tx_zero_best_effort()
            if self.abort_event.is_set() or str(exc) == "ABORTED":
                reason = self.state.mission.abort_reason or "GCS_ABORT"
                self._set_status("ABORTED", abort_reason=reason)
                print(f"[MISSION] ABORTED | reason={reason}")
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
        mode = str(self.cfg["mission"].get("abort_mode", "RTL")).upper()
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
        if str((self.cfg.get("mission") or {}).get("source", "")).upper() == "FC_PRELOADED":
            print("[MISSION] FC_PRELOADED | EXIT SETUP -> no mission upload/sync")
            return
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
                    print(f"[MISSION] SEQ{seq} REACHED")
                vision_seq = int(
                    self.cfg["mission"].get(
                        "vision_enable_seq",
                        self.cfg["mission"].get("ai_enable_wp", 6),
                    )
                )
                if seq == vision_seq:
                    self._activate_drop_ai()
                continue

            if name == "WAYPOINT_EDIT_UNLOCKED":
                self._start_setup_ai()
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
                self._stop_setup_ai("SETUP_EXIT")
                strategy = str(payload.get("strategy") or self.state.mission.strategy).upper()
                threading.Thread(
                    target=self.sync_strategy, args=(strategy,),
                    name=f"MissionSync-{strategy}", daemon=True,
                ).start()
