#!/usr/bin/env python3
from __future__ import annotations

import signal
import sys
import time

from config import load_json
from core.runtime import KingphoenixRuntime
from mission.fc_mission_sync import MissionSyncError
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

    @property
    def v(self):
        return self.runtime.state.vehicle

    def _signal_abort(self, *_):
        self.abort_requested = True

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

    def wait_preflight(self):
        print("")
        print("============================================================")
        print(" KINGPHOENIX 2026 - TERMINAL RACE")
        print(" Mission Planner mission on FC = navigation authority")
        print(" GCS / waypoint JSON / video downlink = NOT USED")
        print("============================================================")

        last = None
        while not self.abort_requested:
            checks = self._preflight_snapshot()
            signature = tuple(checks.items())
            if signature != last:
                states = " | ".join(
                    f"{name}={'OK' if ok else 'WAIT'}"
                    for name, ok in checks.items()
                )
                print(f"[PREFLIGHT] {states}")
                last = signature
            if all(checks.values()):
                print(self.vision_link_line(prefix="[VISION-LINK]"))
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
        missing = [
            f"{name}=SEQ{seq}"
            for name, seq in required.items()
            if seq < 0 or seq >= count
        ]
        if missing:
            raise RuntimeError(
                f"FC mission has {count} items; missing {', '.join(missing)}"
            )

        print(
            f"[MISSION] FC READ OK | items={count} | "
            + " | ".join(f"{k}=SEQ{v}" for k, v in required.items())
        )
        print(
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
            print(
                f"[WARN] Vision daemon ACTIVE failed: "
                f"{reply.get('detail', 'no reply')} | FC fallback remains available"
            )
            return
        self.ai_active = True
        self.drop_confirm = 0
        self.drop_last_frame = None
        print(self.vision_link_line(prefix="[VISION] ACTIVE"))

    def disable_ai(self, source):
        if not self.ai_active:
            return
        ok, reply = self.vision.set_mode("STANDBY")
        self.ai_active = False
        if ok:
            print(f"[VISION] STANDBY | {source}")
        else:
            print(
                f"[WARN] Vision daemon STANDBY failed: "
                f"{reply.get('detail', 'no reply')} | {source}"
            )

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
        print(
            f"[DROP] VISION CONFIRMED {self.drop_confirm}/{needed} | "
            f"conf={float(conf):.2f} ex={float(ex):+.3f} ey={float(ey):+.3f}"
        )
        print(f"[SERVO] {message}")
        return True

    def safe_rtl(self, reason):
        self.disable_ai(reason)
        if self.touchdown_confirmed:
            print(f"[SAFETY] {reason} after touchdown -> no RTL command")
            return
        if not self.v.armed:
            return
        print(f"[SAFETY] {reason} -> RTL")
        result = self.runtime.command.set_mode("RTL")
        if not result.ok:
            print(f"[WARN] RTL command failed: {result.reason}")

    def start_flight(self):
        print("")
        print("Press ENTER to START. Type q then ENTER to quit.")
        answer = input("> ").strip().lower()
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

        print(
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
                    print(
                        f"[TAKEOFF] LiDAR={lidar:.2f}m -> AUTO SEQ{start_seq}"
                    )
                    self.runtime.fc_sync.set_current(start_seq)
                    result = self.runtime.command.set_mode("AUTO")
                    if not result.ok:
                        raise RuntimeError(f"AUTO failed: {result.reason}")
                    self.started = True
                    print(f"[MISSION] AUTO START -> SEQ{start_seq}")
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

        print(
            f"[LAND] LAND active | waiting LiDAR <= {threshold_m:.2f}m "
            f"for {confirm_s:.1f}s"
        )

        low_since = None
        while self.v.armed:
            if self.abort_requested:
                raise RaceAbort("operator Ctrl+C during LAND")

            if not self.v.connected or self.v.heartbeat_age_s() > 2.5:
                raise RuntimeError("FC heartbeat lost during LAND")

            mode = str(self.v.mode or "").upper()
            if mode != "LAND":
                print(f"[LAND] mode changed to {mode} before touchdown confirmation")
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
                    print(
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
                        print(
                            f"[WARN] {target_mode} after touchdown failed: "
                            f"{result.reason}"
                        )
                    if selected_mode is None:
                        raise RuntimeError(
                            f"post-touchdown mode switch failed: {last_reason}"
                        )
                    print(
                        f"[LAND] {selected_mode} ACTIVE | waiting MANUAL DISARM"
                    )

                    while self.v.armed:
                        if self.abort_requested:
                            print("[LAND] Ctrl+C after touchdown -> exit without RTL")
                            return True
                        time.sleep(0.05)

                    print("[MISSION] COMPLETE | vehicle DISARMED")
                    return True
            else:
                low_since = None

            time.sleep(0.02)

        print("[MISSION] COMPLETE | vehicle auto-DISARMED in LAND")
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
                print(f"[MISSION] REACHED SEQ{seq}")
                if seq == int(self.cfg["mission"]["final_seq"]):
                    self.final_reached = True

    def run_auto(self):
        m = self.cfg["mission"]
        ai_seq = int(m["vision_enable_seq"])
        fallback_seq = int(m["fallback_start_seq"])
        resume_seq = int(m["resume_seq"])
        settle_s = float(self.cfg["drop"].get("post_release_settle_s", 0.25))

        while True:
            if self.abort_requested:
                raise RaceAbort("operator Ctrl+C")

            if not self.v.connected or self.v.heartbeat_age_s() > 2.5:
                raise RuntimeError("FC heartbeat lost")

            self.poll_runtime_events()

            seq = self.v.current_mission_item
            if seq is not None and seq != self.last_seq:
                self.last_seq = int(seq)
                print(f"[MISSION] SEQ{self.last_seq} | " + self.vision_link_line(prefix="VISION"))

            final_seq = int(m["final_seq"])
            if (
                self.started
                and self.v.armed
                and not self.landing_commanded
                and self.final_reached
            ):
                self.disable_ai("FINAL_POINT")
                self.landing_commanded = True
                print(
                    f"[LAND] REACHED SEQ{final_seq} -> LAND LATCH ACTIVE"
                )

            mode = str(self.v.mode or "").upper()

            if self.started and self.v.armed and self.landing_commanded:
                if mode != "LAND":
                    now = time.monotonic()
                    if now - self.last_land_force_ts >= 0.5:
                        self.last_land_force_ts = now
                        print(f"[LAND] latch force {mode} -> LAND")
                        result = self.runtime.command.set_mode("LAND")
                        if not result.ok:
                            print(f"[WARN] LAND reassert failed: {result.reason}")
                    time.sleep(0.02)
                    continue

            if self.started and self.v.armed and mode == "LAND":
                if self.handle_land_touchdown():
                    return
                mode = str(self.v.mode or "").upper()

            if self.started and self.v.armed and mode not in {"AUTO", "LAND", "RTL"}:
                self.disable_ai("PILOT_TAKEOVER")
                print(f"[PILOT] FC mode changed to {mode}; Pi stops autonomous intervention")
                return

            if seq is not None and int(seq) >= ai_seq and int(seq) < fallback_seq:
                self.activate_ai()

            if self.vision_drop_once():
                if settle_s > 0:
                    time.sleep(settle_s)
                try:
                    self.runtime.fc_sync.set_current(resume_seq)
                    print(
                        f"[DROP] SUCCESS -> SKIP FC FALLBACK -> SEQ{resume_seq}"
                    )
                except MissionSyncError as exc:
                    print(
                        f"[WARN] drop released but skip failed: {exc} | "
                        "FC fallback remains available"
                    )

            if (
                not self.drop_done
                and seq is not None
                and int(seq) >= fallback_seq
            ):
                self.disable_ai("FC_FALLBACK")
                print("[DROP] Vision not released -> FC fallback block owns drop")
                self.drop_done = True

            if self.started and not self.v.armed:
                print("[MISSION] COMPLETE | vehicle DISARMED")
                return

            time.sleep(0.01)

    def run(self):
        self.install_signals()
        try:
            self.runtime.start()
            self.wait_preflight()
            self.read_fc_mission()
            self.start_flight()
            self.run_auto()
            return 0

        except RaceAbort as exc:
            print(f"[RACE] ABORT | {exc}")
            self.safe_rtl("OPERATOR_ABORT")
            return 130

        except Exception as exc:
            print(f"[ERROR] {type(exc).__name__}: {exc}")
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
