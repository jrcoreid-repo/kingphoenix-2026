#!/usr/bin/env python3
from __future__ import annotations

import signal
import sys
import threading
import time
from collections import deque

from config import load_json
from control.fuzzy_drop_controller import FuzzyDropController, FuzzyDropError
from core.runtime import KingphoenixRuntime
from mission.fc_mission_sync import MissionSyncError
from safety.pilot_takeover import PilotTakeover
from vision.daemon_client import VisionDaemonClient


class RaceAbort(RuntimeError):
    pass


class TerminalRace:
    def __init__(self):
        self.cfg = load_json("race_terminal.json")
        self.runtime = KingphoenixRuntime(
            enable_gcs=False,
            enable_mission_manager=False,
            enable_runtime_logger=False,
            enable_safety_manager=False,
            enable_readiness=False,
            enable_servo_initialize=False,
            enable_vision_stream=False,
        )
        vdcfg = dict(self.cfg.get("vision_daemon", {}) or {})
        self.vision = VisionDaemonClient(
            data_bind_ip=vdcfg.get("data_bind_ip", "127.0.0.1"),
            data_port=vdcfg.get("data_port", 14600),
            control_host=vdcfg.get("control_host", "127.0.0.1"),
            control_port=vdcfg.get("control_port", 14601),
            stale_s=vdcfg.get("stale_s", 0.30),
            rate_window_s=vdcfg.get("rate_window_s", 1.5),
        )
        self.abort_requested = False
        self.abort_event = threading.Event()
        self.started = False
        self.ai_active = False
        self.drop_done = False
        self.drop_confirm = 0
        self.drop_last_frame = None
        self.last_seq = None
        self.touchdown_confirmed = False
        self.landing_commanded = False
        self.final_reached = False
        self.last_land_force_ts = 0.0
        self.fuzzy = None
        self.fuzzy_attempted = False
        self.fuzzy_active = False
        self.fuzzy_result = "IDLE"

        display_cfg = dict(self.cfg.get("display", {}) or {})
        self.display_hz = max(1.0, float(display_cfg.get("refresh_hz", 5.0)))
        self.use_ansi = bool(display_cfg.get("ansi", True)) and sys.stdout.isatty()
        self.last_display_s = 0.0
        self.phase = "BOOT"
        self.status_detail = "starting runtime"
        self.event_log = deque(maxlen=5)
        self.mission_count = None
        self.mission_required = {}

    @property
    def v(self):
        return self.runtime.state.vehicle

    def _signal_abort(self, *_):
        self.abort_requested = True
        self.abort_event.set()

    def install_signals(self):
        signal.signal(signal.SIGINT, self._signal_abort)
        signal.signal(signal.SIGTERM, self._signal_abort)

    def _preflight_snapshot(self):
        p = self.cfg["preflight"]
        vision = self.vision.snapshot()
        lidar_age = self.v.rangefinder_age_s()
        checks = {
            "FC": bool(self.v.connected)
            and self.v.heartbeat_age_s() <= float(p["heartbeat_max_age_s"]),
            "GPS": self.v.gps_fix_type is not None
            and int(self.v.gps_fix_type) >= int(p["gps_min_fix_type"]),
            "LIDAR": self.v.rangefinder_distance_m is not None
            and float(self.v.rangefinder_distance_m) > 0.0
            and lidar_age <= float(p["lidar_max_age_s"]),
            "VISION_LINK": bool(vision.get("link_ok")),
            "BOTTOM_CAM": bool(vision.get("camera_ready")),
            "HAILO": bool(vision.get("hailo_ready")),
        }
        return checks

    @staticmethod
    def _fmt(value, digits=2, fallback="-"):
        if value is None:
            return fallback
        try:
            return f"{float(value):.{digits}f}"
        except Exception:
            return str(value)

    def log(self, message=""):
        text = str(message)
        if text:
            self.event_log.append(text)
        self.status_detail = text or self.status_detail
        self.display_if_due(force=True)

    def dashboard(self):
        vision = self.vision.snapshot()
        lidar = self.v.rangefinder_distance_m
        lidar_age = self.v.rangefinder_age_s()
        seq = self.v.current_mission_item
        mode = str(self.v.mode or "-").upper()
        gps_fix = self.v.gps_fix_type
        hb_age = self.v.heartbeat_age_s()

        vision_age = vision.get("rx_age_ms")
        vision_age_text = "-" if vision_age is None else f"{float(vision_age):.0f} ms"

        fresh_cfg = dict(self.cfg.get("freshness_monitor", {}) or {})
        def msg_health(name, max_age_s):
            age_s = self.v.message_age_s(name)
            rate_hz = self.v.message_rate_hz(name)
            if age_s == float("inf"):
                return "MISSING", rate_hz, None
            return (
                "FRESH" if age_s <= float(max_age_s) else "STALE",
                rate_hz,
                age_s * 1000.0,
            )

        lidar_msg = (
            "RANGEFINDER"
            if self.v.rangefinder_source == "RANGEFINDER_LEGACY"
            else "DISTANCE_SENSOR"
        )
        lidar_state, lidar_hz, lidar_msg_age_ms = msg_health(
            lidar_msg, fresh_cfg.get("lidar_max_age_s", 0.20)
        )
        gps_state, gps_hz, gps_age_ms = msg_health(
            "GPS_RAW_INT", fresh_cfg.get("gps_max_age_s", 0.50)
        )
        hb_state, hb_hz, hb_msg_age_ms = msg_health(
            "HEARTBEAT", fresh_cfg.get("heartbeat_max_age_s", 2.50)
        )
        mission_state, mission_hz, mission_age_ms = msg_health(
            "MISSION_CURRENT", fresh_cfg.get("mission_max_age_s", 1.00)
        )
        gpos_state, gpos_hz, gpos_age_ms = msg_health(
            "GLOBAL_POSITION_INT", fresh_cfg.get("global_position_max_age_s", 0.30)
        )

        def age_text(ms):
            return "-" if ms is None else f"{ms:.0f}ms"
        lidar_text = "-" if lidar is None else f"{float(lidar):.2f} m"
        lidar_age_text = "-" if lidar is None else f"{float(lidar_age) * 1000.0:.0f} ms"
        seq_text = "-" if seq is None else str(int(seq))
        final_seq = self.cfg.get("mission", {}).get("final_seq", "-")

        conf = vision.get("confidence")
        ex = vision.get("ex")
        ey = vision.get("ey")
        target = "YES" if vision.get("target_detected") else "NO"
        vision_link = "OK" if vision.get("link_ok") else "STALE"

        drop_state = "DONE" if self.drop_done else ("AI ACTIVE" if self.ai_active else "WAIT")
        fuzzy_state = "ACTIVE" if self.fuzzy_active else self.fuzzy_result
        land_state = (
            "TOUCHDOWN"
            if self.touchdown_confirmed
            else ("LAND CMD" if self.landing_commanded else "WAIT")
        )

        lines = [
            "======================================================================",
            " KINGPHOENIX AR-1 | RACE RUNNER",
            "======================================================================",
            (
                f" PHASE  : {self.phase:<12} | FC: {'OK' if self.v.connected else 'WAIT':<4} "
                f"| MODE: {mode:<9} | ARMED: {'YES' if self.v.armed else 'NO'}"
            ),
            (
                f" NAV    : GPS FIX={gps_fix if gps_fix is not None else '-'} "
                f"SAT={self.v.gps_satellites if self.v.gps_satellites is not None else '-'} "
                f"HDOP={self._fmt(self.v.gps_hdop, 2):>4} | "
                f"LiDAR={lidar_text:<8} | HB={hb_age:.2f}s"
            ),
            (
                f" MISSION: SEQ={seq_text:<3} / FINAL={final_seq} | "
                f"ITEMS={self.mission_count if self.mission_count is not None else '-'} "
                f"| DROP={drop_state:<9} | LAND={land_state}"
            ),
            "----------------------------------------------------------------------",
            (
                f" FRESH  : LIDAR {lidar_state:<7} {lidar_hz:5.1f}Hz {age_text(lidar_msg_age_ms):>6} | "
                f"GPS {gps_state:<7} {gps_hz:5.1f}Hz {age_text(gps_age_ms):>6}"
            ),
            (
                f" LINK   : HB {hb_state:<7} {hb_hz:5.1f}Hz {age_text(hb_msg_age_ms):>6} | "
                f"GPOS {gpos_state:<7} {gpos_hz:5.1f}Hz {age_text(gpos_age_ms):>6}"
            ),
            (
                f" MISSION: STREAM {mission_state:<7} {mission_hz:5.1f}Hz "
                f"{age_text(mission_age_ms):>6} | SRC LiDAR={lidar_msg}"
            ),
            "----------------------------------------------------------------------",
            (
                f" VISION : LINK={vision_link:<5} | RX={float(vision.get('rx_rate_hz') or 0.0):5.1f} pkt/s "
                f"| AGE={vision_age_text:<7} | RX#={int(vision.get('rx_count') or 0)}"
            ),
            (
                f" AI     : MODE={str(vision.get('mode', '-')):<8} | "
                f"FPS={float(vision.get('model_fps') or 0.0):5.1f} | "
                f"FRAME={str(vision.get('frame_id', '-')):<8} | TARGET={target}"
            ),
            (
                f" TARGET : CONF={self._fmt(conf, 2):>5} | "
                f"ex={self._fmt(ex, 3):>7} | ey={self._fmt(ey, 3):>7} | "
                f"CONFIRM={self.drop_confirm}"
            ),
            (
                f" FALLBACK: FUZZY={fuzzy_state:<8} | "
                f"ATTEMPTED={'YES' if self.fuzzy_attempted else 'NO ':<3} | "
                f"FC BLOCK={'READY' if not self.drop_done else 'DONE'}"
            ),
            "----------------------------------------------------------------------",
            f" STATUS : {self.status_detail or '-'}",
            " EVENTS :",
        ]
        events = list(self.event_log)
        if not events:
            events = ["-"]
        lines.extend(f"   {line}" for line in events[-5:])
        lines.extend([
            "======================================================================",
            " Ctrl+C = abort / safety handling",
        ])
        return "\n".join(lines)

    def display_if_due(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_display_s < 1.0 / self.display_hz:
            return
        self.last_display_s = now
        text = self.dashboard()
        if self.use_ansi:
            sys.stdout.write("\033[2J\033[H" + text + "\n")
            sys.stdout.flush()
        elif force:
            sys.stdout.write(text + "\n")
            sys.stdout.flush()

    def wait_preflight(self):
        self.phase = "PREFLIGHT"
        self.status_detail = "checking FC, GPS, LiDAR, camera, Hailo and vision link"
        self.display_if_due(force=True)
        self.log("")
        self.log("============================================================")
        self.log(" KINGPHOENIX 2026 - TERMINAL RACE")
        self.log(" Mission Planner mission on FC = navigation authority")
        self.log(" GCS / waypoint JSON / video downlink = NOT USED")
        self.log("============================================================")

        last = None
        while not self.abort_requested:
            checks = self._preflight_snapshot()
            self.display_if_due()
            signature = tuple(checks.items())
            if signature != last:
                states = " | ".join(
                    f"{name}={'OK' if ok else 'WAIT'}"
                    for name, ok in checks.items()
                )
                self.log(f"[PREFLIGHT] {states}")
                last = signature
            if all(checks.values()):
                self.log(self.vision_link_line(prefix="[VISION-LINK]"))
                return
            time.sleep(0.25)
        raise RaceAbort("operator abort during preflight")

    def read_fc_mission(self):
        m = self.cfg["mission"]
        actual = self.runtime.fc_sync.download()
        required = {
            "START": int(m["start_seq"]),
            "AI": int(m["vision_enable_seq"]),
            "DROP": int(m["drop_seq"]),
            "FALLBACK": int(m["fallback_start_seq"]),
            "RESUME": int(m["resume_seq"]),
            "FINAL": int(m["final_seq"]),
        }
        count = len(actual)
        self.mission_count = count
        self.mission_required = dict(required)
        missing = [
            f"{name}=SEQ{seq}"
            for name, seq in required.items()
            if seq < 0 or seq >= count
        ]
        if missing:
            raise RuntimeError(
                f"FC mission has {count} items; missing {', '.join(missing)}"
            )

        self.log(
            f"[MISSION] FC READ OK | items={count} | "
            + " | ".join(f"{k}=SEQ{v}" for k, v in required.items())
        )
        self.log(
            f"[MISSION] SEQ{int(m['final_seq'])} = FINAL POINT | "
            "Pi will command LAND there; FC mission items after it are ignored"
        )
        return actual

    def vision_link_line(self, prefix="[VISION]"):
        snap = self.vision.snapshot()
        age = snap.get("rx_age_ms")
        age_text = "-" if age is None else f"{float(age):.0f}ms"
        return (
            f"{prefix} "
            f"{'OK' if snap.get('link_ok') else 'STALE'} | "
            f"RX={float(snap.get('rx_rate_hz') or 0.0):.1f} pkt/s | "
            f"AGE={age_text} | "
            f"RX#={int(snap.get('rx_count') or 0)} | "
            f"MODE={snap.get('mode', '-')} | "
            f"FRAME={snap.get('frame_id', '-')} | "
            f"AI={float(snap.get('model_fps') or 0.0):.1f} fps"
        )

    def activate_ai(self):
        if self.ai_active or self.drop_done:
            return
        ok, reply = self.vision.set_mode("ACTIVE")
        if not ok:
            self.log(
                f"[WARN] Vision daemon ACTIVE failed: "
                f"{reply.get('detail', 'no reply')} | FC fallback remains available"
            )
            return
        self.ai_active = True
        self.drop_confirm = 0
        self.drop_last_frame = None
        self.log(self.vision_link_line(prefix="[VISION] ACTIVE"))

    def disable_ai(self, source):
        if not self.ai_active:
            return
        ok, reply = self.vision.set_mode("STANDBY")
        self.ai_active = False
        if ok:
            self.log(f"[VISION] STANDBY | {source}")
        else:
            self.log(
                f"[WARN] Vision daemon STANDBY failed: "
                f"{reply.get('detail', 'no reply')} | {source}"
            )

    def _get_fuzzy(self):
        if self.fuzzy is None:
            cfg = dict(self.cfg.get("fuzzy_fallback", {}) or {})
            self.fuzzy = FuzzyDropController(
                self.runtime.state,
                self.runtime.command,
                self.runtime.tx,
                self.vision,
                cfg,
                log=self.log,
            )
        return self.fuzzy

    def run_fuzzy_fallback(self):
        """Run one bounded fuzzy attempt; FC fallback remains the final safety net."""
        if self.drop_done or self.fuzzy_attempted:
            return False

        cfg = dict(self.cfg.get("fuzzy_fallback", {}) or {})
        if not bool(cfg.get("enabled", True)):
            self.fuzzy_attempted = True
            self.fuzzy_result = "DISABLED"
            self.log("[FUZZY] disabled -> FC fallback")
            return False

        self.fuzzy_attempted = True
        self.fuzzy_active = True
        self.fuzzy_result = "ACTIVE"
        self.phase = "FUZZY"
        self.status_detail = "vision primary missed; fuzzy fallback aligning"
        self.disable_ai("FUZZY_FALLBACK")
        # Fuzzy consumes the same daemon telemetry, so reactivate perception only.
        ok, reply = self.vision.set_mode("ACTIVE")
        if not ok:
            self.fuzzy_active = False
            self.fuzzy_result = "NO VISION"
            self.log(
                f"[WARN] fuzzy fallback vision ACTIVE failed: "
                f"{reply.get('detail', 'no reply')} -> FC fallback"
            )
            return False

        max_duration_s = float(cfg.get("max_duration_s", 8.0))
        try:
            centered = self._get_fuzzy().run(
                abort_event=self.abort_event,
                max_duration_s=max_duration_s,
            )
        except FuzzyDropError as exc:
            if self.abort_event.is_set():
                raise RaceAbort("operator abort during fuzzy fallback") from exc
            centered = False
            self.log(f"[WARN] fuzzy fallback stopped: {exc}")
        finally:
            try:
                self.runtime.tx.send_body_velocity(0.0, 0.0, 0.0)
            except Exception:
                pass
            self.fuzzy_active = False

        if not centered:
            self.fuzzy_result = "FAILED"
            try:
                self.vision.set_mode("STANDBY", timeout_s=0.10)
            except Exception:
                pass
            self.phase = "AUTO"
            self.status_detail = "fuzzy fallback failed; handing back to FC fallback"
            return False

        ok, message = self.runtime.servo.open(source="RACE_FUZZY_FALLBACK_DROP")
        if not ok:
            raise RuntimeError(message)

        self.drop_done = True
        self.fuzzy_result = "SUCCESS"
        try:
            self.vision.set_mode("STANDBY", timeout_s=0.10)
        except Exception:
            pass
        self.log(f"[FUZZY] CENTERED -> DROP | {message}")
        return True

    def vision_drop_once(self):
        if not self.ai_active or self.drop_done:
            return False

        c = self.cfg["vision"]
        snap = self.vision.snapshot()
        frame_id = snap.get("frame_id")
        if frame_id is None or frame_id == self.drop_last_frame:
            return False
        self.drop_last_frame = frame_id

        conf = snap.get("confidence")
        ex = snap.get("ex")
        ey = snap.get("ey")
        result_age_ms = snap.get("result_age_ms")
        valid = bool(
            snap.get("link_ok")
            and snap.get("usable")
            and result_age_ms is not None
            and float(result_age_ms) <= float(c["freshness_s"]) * 1000.0
            and conf is not None
            and float(conf) >= float(c["confidence_threshold"])
            and ex is not None
            and ey is not None
            and abs(float(ex)) <= float(c["coarse_center_error_max"])
            and abs(float(ey)) <= float(c["coarse_center_error_max"])
        )

        self.drop_confirm = self.drop_confirm + 1 if valid else 0
        needed = int(c["confirm_frames"])
        if self.drop_confirm < needed:
            return False

        ok, message = self.runtime.servo.open(source="RACE_VISION_DROP")
        if not ok:
            raise RuntimeError(message)

        self.drop_done = True
        self.disable_ai("VISION_DROP_COMPLETE")
        self.log(
            f"[DROP] VISION CONFIRMED {self.drop_confirm}/{needed} | "
            f"conf={float(conf):.2f} ex={float(ex):+.3f} ey={float(ey):+.3f}"
        )
        self.log(f"[SERVO] {message}")
        return True

    def safe_rtl(self, reason):
        self.disable_ai(reason)
        if self.touchdown_confirmed:
            self.log(f"[SAFETY] {reason} after touchdown -> no RTL command")
            return
        if not self.v.armed:
            return
        self.log(f"[SAFETY] {reason} -> RTL")
        result = self.runtime.command.set_mode("RTL")
        if not result.ok:
            self.log(f"[WARN] RTL command failed: {result.reason}")

    def start_flight(self):
        self.phase = "READY"
        self.status_detail = "preflight passed; waiting operator START"
        self.display_if_due(force=True)
        self.log("")
        self.log("Press ENTER to START. Type q then ENTER to quit.")
        self.display_if_due(force=True)
        answer = input("> ").strip().lower()
        self.phase = "TAKEOFF"
        self.status_detail = "operator START accepted"
        self.display_if_due(force=True)
        if answer in {"q", "quit", "exit"}:
            raise RaceAbort("operator quit before start")

        if self.abort_requested:
            raise RaceAbort("operator abort before start")

        # Race terminal never touches the payload servo before the drop.
        # The only race servo command from Pi is OPEN at confirmed vision drop.

        # Race takeoff is intentionally minimal:
        # GUIDED -> ARM -> NAV_TAKEOFF(1m) -> LiDAR >= trigger -> AUTO SEQ3.
        # No profiled velocity handoff, taper, or hard LiDAR ceiling.
        tcfg = self.runtime.full_cfg["takeoff"]
        trigger_m = float(tcfg["target_alt_m"]) - float(tcfg["tolerance_m"])
        kick_alt_m = float(tcfg.get("kick_alt_m", 1.0))
        stale_s = float(tcfg.get("lidar_stale_s", 0.60))
        timeout_s = float(tcfg.get("timeout_s", 15.0))
        arm_settle_s = float(tcfg.get("arm_settle_s", 0.35))

        self.log(
            f"[TAKEOFF] SIMPLE | kick={kick_alt_m:.2f}m | "
            f"AUTO trigger LiDAR>={trigger_m:.2f}m"
        )

        result = self.runtime.command.set_mode("GUIDED")
        if not result.ok:
            raise RuntimeError(f"GUIDED failed: {result.reason}")

        if not self.v.armed:
            result = self.runtime.command.arm()
            if not result.ok:
                raise RuntimeError(f"ARM failed: {result.reason}")

        settle_end = time.monotonic() + arm_settle_s
        while time.monotonic() < settle_end:
            if self.abort_requested:
                raise RaceAbort("operator abort during arm settle")
            time.sleep(0.02)

        if not self.runtime.tx.takeoff(kick_alt_m):
            raise RuntimeError("NAV_TAKEOFF TX failed")

        start_seq = int(self.cfg["mission"]["start_seq"])
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            self.display_if_due()
            if self.abort_requested:
                raise RaceAbort("operator abort during takeoff")
            if not self.v.connected or self.v.heartbeat_age_s() > 2.5:
                raise RuntimeError("FC heartbeat lost during takeoff")
            if str(self.v.mode or "").upper() != "GUIDED":
                raise RaceAbort(
                    f"pilot/mode takeover during takeoff: {self.v.mode}"
                )

            if self.v.rangefinder_is_fresh(stale_s):
                lidar = float(self.v.rangefinder_distance_m)
                if lidar >= trigger_m:
                    self.log(
                        f"[TAKEOFF] LiDAR={lidar:.2f}m -> AUTO SEQ{start_seq}"
                    )
                    self.runtime.fc_sync.set_current(start_seq)
                    result = self.runtime.command.set_mode("AUTO")
                    if not result.ok:
                        raise RuntimeError(f"AUTO failed: {result.reason}")
                    self.started = True
                    self.phase = "AUTO"
                    self.status_detail = f"AUTO mission running from SEQ{start_seq}"
                    self.log(f"[MISSION] AUTO START -> SEQ{start_seq}")
                    return

            time.sleep(0.01)

        raise RuntimeError("takeoff timeout before LiDAR AUTO trigger")

    def handle_land_touchdown(self):
        """Confirm physical touchdown with LiDAR before switching LAND -> ALT_HOLD."""
        c = dict(self.cfg.get("landing") or {})
        threshold_m = float(c.get("lidar_touchdown_m", 0.20))
        confirm_s = float(c.get("confirm_s", 0.8))
        lidar_max_age_s = float(c.get("lidar_max_age_s", 0.30))
        max_groundspeed_mps = float(c.get("max_groundspeed_mps", 0.30))
        target_modes = [
            str(x).upper()
            for x in c.get("post_touchdown_modes", ["ALT_HOLD", "STABILIZE"])
        ]

        self.phase = "LAND"
        self.status_detail = "LAND active; waiting touchdown confirmation"
        self.log(
            f"[LAND] LAND active | waiting LiDAR <= {threshold_m:.2f}m "
            f"for {confirm_s:.1f}s"
        )

        low_since = None
        while self.v.armed:
            self.display_if_due()
            if self.abort_requested:
                raise RaceAbort("operator Ctrl+C during LAND")

            if not self.v.connected or self.v.heartbeat_age_s() > 2.5:
                raise RuntimeError("FC heartbeat lost during LAND")

            mode = str(self.v.mode or "").upper()
            if mode != "LAND":
                self.log(f"[LAND] mode changed to {mode} before touchdown confirmation")
                return False

            lidar_ok = self.v.rangefinder_is_fresh(lidar_max_age_s)
            lidar = self.v.rangefinder_distance_m
            gs = self.v.groundspeed_mps
            speed_ok = gs is None or abs(float(gs)) <= max_groundspeed_mps

            low = bool(
                lidar_ok
                and lidar is not None
                and 0.0 < float(lidar) <= threshold_m
                and speed_ok
            )

            now = time.monotonic()
            if low:
                if low_since is None:
                    low_since = now
                if now - low_since >= confirm_s:
                    self.touchdown_confirmed = True
                    self.phase = "TOUCHDOWN"
                    self.status_detail = "touchdown confirmed; waiting manual disarm"
                    self.log(
                        f"[LAND] TOUCHDOWN CONFIRMED | LiDAR={float(lidar):.2f}m | "
                        f"GS={float(gs or 0.0):.2f}m/s"
                    )
                    selected_mode = None
                    last_reason = ""
                    for target_mode in target_modes:
                        result = self.runtime.command.set_mode(target_mode)
                        if result.ok:
                            selected_mode = target_mode
                            break
                        last_reason = result.reason
                        self.log(
                            f"[WARN] {target_mode} after touchdown failed: "
                            f"{result.reason}"
                        )
                    if selected_mode is None:
                        raise RuntimeError(
                            f"post-touchdown mode switch failed: {last_reason}"
                        )
                    self.log(
                        f"[LAND] {selected_mode} ACTIVE | waiting MANUAL DISARM"
                    )

                    while self.v.armed:
                        if self.abort_requested:
                            self.log("[LAND] Ctrl+C after touchdown -> exit without RTL")
                            return True
                        time.sleep(0.05)

                    self.log("[MISSION] COMPLETE | vehicle DISARMED")
                    return True
            else:
                low_since = None

            time.sleep(0.02)

        self.log("[MISSION] COMPLETE | vehicle auto-DISARMED in LAND")
        return True

    def poll_runtime_events(self):
        """Consume lightweight FC events; standalone race owns this event bus."""
        while True:
            event = self.runtime.events.get(timeout=0.0)
            if event is None:
                return
            name = str(event.name)
            payload = event.payload or {}
            if name == "MISSION_ITEM_REACHED":
                seq = int(payload.get("seq", -1))
                self.log(f"[MISSION] REACHED SEQ{seq}")
                if seq == int(self.cfg["mission"]["final_seq"]):
                    self.final_reached = True

    def run_auto(self):
        m = self.cfg["mission"]
        ai_seq = int(m["vision_enable_seq"])
        fallback_seq = int(m["fallback_start_seq"])
        resume_seq = int(m["resume_seq"])
        settle_s = float(self.cfg["drop"].get("post_release_settle_s", 0.25))

        self.phase = "AUTO"
        self.status_detail = "race mission active"
        while True:
            self.display_if_due()
            if self.abort_requested:
                raise RaceAbort("operator Ctrl+C")

            if not self.v.connected or self.v.heartbeat_age_s() > 2.5:
                raise RuntimeError("FC heartbeat lost")

            self.poll_runtime_events()

            seq = self.v.current_mission_item
            if seq is not None and seq != self.last_seq:
                self.last_seq = int(seq)
                self.log(f"[MISSION] SEQ{self.last_seq} | " + self.vision_link_line(prefix="VISION"))

            final_seq = int(m["final_seq"])
            if (
                self.started
                and self.v.armed
                and not self.landing_commanded
                and self.final_reached
            ):
                self.disable_ai("FINAL_POINT")
                self.landing_commanded = True
                self.phase = "LAND"
                self.status_detail = f"final SEQ{final_seq} reached; LAND latch active"
                self.log(
                    f"[LAND] REACHED SEQ{final_seq} -> LAND LATCH ACTIVE"
                )

            mode = str(self.v.mode or "").upper()

            if self.started and self.v.armed and self.landing_commanded:
                if mode != "LAND":
                    now = time.monotonic()
                    if now - self.last_land_force_ts >= 0.5:
                        self.last_land_force_ts = now
                        self.log(f"[LAND] latch force {mode} -> LAND")
                        result = self.runtime.command.set_mode("LAND")
                        if not result.ok:
                            self.log(f"[WARN] LAND reassert failed: {result.reason}")
                    time.sleep(0.02)
                    continue

            if self.started and self.v.armed and mode == "LAND":
                if self.handle_land_touchdown():
                    return
                mode = str(self.v.mode or "").upper()

            if self.started and self.v.armed and mode not in {"AUTO", "LAND", "RTL"}:
                self.disable_ai("PILOT_TAKEOVER")
                self.log(f"[PILOT] FC mode changed to {mode}; Pi stops autonomous intervention")
                return

            if seq is not None and int(seq) >= ai_seq and int(seq) < fallback_seq:
                self.activate_ai()

            if self.vision_drop_once():
                if settle_s > 0:
                    time.sleep(settle_s)
                try:
                    self.runtime.fc_sync.set_current(resume_seq)
                    self.log(
                        f"[DROP] SUCCESS -> SKIP FC FALLBACK -> SEQ{resume_seq}"
                    )
                except MissionSyncError as exc:
                    self.log(
                        f"[WARN] drop released but skip failed: {exc} | "
                        "FC fallback remains available"
                    )

            if (
                not self.drop_done
                and seq is not None
                and int(seq) >= fallback_seq
                and not self.fuzzy_attempted
            ):
                self.log("[DROP] primary vision missed -> fuzzy fallback")
                fuzzy_success = self.run_fuzzy_fallback()

                if fuzzy_success:
                    if settle_s > 0:
                        time.sleep(settle_s)
                    try:
                        self.runtime.fc_sync.set_current(resume_seq)
                        result = self.runtime.command.set_mode("AUTO")
                        if not result.ok:
                            raise RuntimeError(f"AUTO resume failed: {result.reason}")
                        self.phase = "AUTO"
                        self.status_detail = f"fuzzy drop complete; resumed at SEQ{resume_seq}"
                        self.log(f"[FUZZY] SUCCESS -> AUTO SEQ{resume_seq}")
                    except MissionSyncError as exc:
                        self.log(
                            f"[WARN] fuzzy drop released but resume failed: {exc}"
                        )
                else:
                    # Fuzzy already zeroed XY. Resume AUTO at the FC fallback block.
                    try:
                        self.runtime.fc_sync.set_current(fallback_seq)
                        result = self.runtime.command.set_mode("AUTO")
                        if not result.ok:
                            raise RuntimeError(
                                f"AUTO fallback resume failed: {result.reason}"
                            )
                        self.phase = "AUTO"
                        self.status_detail = "FC fallback block owns drop"
                        self.log(
                            f"[FUZZY] FAILED -> AUTO FC FALLBACK SEQ{fallback_seq}"
                        )
                    except MissionSyncError as exc:
                        raise RuntimeError(
                            f"failed to hand back to FC fallback: {exc}"
                        )

            if (
                not self.drop_done
                and self.fuzzy_attempted
                and not self.fuzzy_active
                and seq is not None
                and int(seq) > fallback_seq
            ):
                # Once the FC fallback block has started, Pi no longer intervenes.
                self.disable_ai("FC_FALLBACK")
                self.fuzzy_result = (
                    self.fuzzy_result
                    if self.fuzzy_result == "SUCCESS"
                    else "FC"
                )

            if self.started and not self.v.armed:
                self.log("[MISSION] COMPLETE | vehicle DISARMED")
                return

            time.sleep(0.01)

    def run(self):
        self.install_signals()
        try:
            self.runtime.start()
            self.phase = "PREFLIGHT"
            self.status_detail = "runtime started"
            self.display_if_due(force=True)
            self.wait_preflight()
            self.read_fc_mission()
            self.start_flight()
            self.run_auto()
            return 0

        except PilotTakeover as exc:
            self.phase = "PILOT"
            self.status_detail = str(exc)
            self.log(f"[PILOT] {exc} | Pi autonomous intervention stopped")
            return 0

        except RaceAbort as exc:
            self.phase = "ABORT"
            self.status_detail = str(exc)
            self.log(f"[RACE] ABORT | {exc}")
            self.safe_rtl("OPERATOR_ABORT")
            return 130

        except Exception as exc:
            self.phase = "ERROR"
            self.status_detail = f"{type(exc).__name__}: {exc}"
            self.log(f"[ERROR] {type(exc).__name__}: {exc}")
            self.safe_rtl("RACE_ERROR")
            return 1

        finally:
            try:
                self.vision.set_mode("STANDBY", timeout_s=0.10)
            except Exception:
                pass
            self.vision.close()
            self.runtime.stop()


def main():
    return TerminalRace().run()


if __name__ == "__main__":
    sys.exit(main())
