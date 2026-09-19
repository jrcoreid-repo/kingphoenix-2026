#!/usr/bin/env python3
from __future__ import annotations

import signal
import sys
import time

from config import load_json
from core.runtime import KingphoenixRuntime
from mission.fc_mission_sync import MissionSyncError


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
        )
        self.abort_requested = False
        self.started = False
        self.ai_active = False
        self.drop_done = False
        self.drop_confirm = 0
        self.drop_last_frame = None
        self.last_seq = None
        self.touchdown_confirmed = False

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
        bottom = self.runtime.state.get_vision_snapshot("bottom")
        hailo = self.runtime.vision_stream.shared_ai
        lidar_age = self.v.rangefinder_age_s()
        checks = {
            "FC": bool(self.v.connected)
            and self.v.heartbeat_age_s() <= float(p["heartbeat_max_age_s"]),
            "GPS": self.v.gps_fix_type is not None
            and int(self.v.gps_fix_type) >= int(p["gps_min_fix_type"]),
            "LIDAR": self.v.rangefinder_distance_m is not None
            and float(self.v.rangefinder_distance_m) > 0.0
            and lidar_age <= float(p["lidar_max_age_s"]),
            "BOTTOM_CAM": bool(bottom.get("camera_open")),
            "HAILO": hailo is not None and bool(getattr(hailo, "ready", False)),
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
            "LAND": int(m["land_seq"]),
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
        return actual

    def close_payload(self):
        ok, message = self.runtime.servo.close(source="RACE_PRESTART")
        if not ok:
            raise RuntimeError(message)
        print(f"[SERVO] {message}")

    def activate_ai(self):
        if self.ai_active or self.drop_done:
            return
        vcfg = self.runtime.full_cfg["vision"]
        self.runtime.vision_stream.set_ai_route(
            "bottom",
            model_mode=str(vcfg.get("model_mode", "YOLO26_ONLY")),
            task=str(vcfg.get("task", "DROP_ALIGNMENT")),
            detector_class_id=int(vcfg.get("detector_class_id", 0)),
            source="RACE_SEQ_TRIGGER",
        )
        self.ai_active = True
        self.drop_confirm = 0
        self.drop_last_frame = None
        print("[VISION] BOTTOM AI ON")

    def disable_ai(self, source):
        if not self.ai_active:
            return
        self.runtime.vision_stream.disable_ai(source=source)
        self.ai_active = False
        print(f"[VISION] AI OFF | {source}")

    def vision_drop_once(self):
        if not self.ai_active or self.drop_done:
            return False

        c = self.cfg["vision"]
        snap = self.runtime.state.get_vision_snapshot(
            "bottom",
            max_age_s=float(c["freshness_s"]),
        )
        frame_id = snap.get("frame_id")
        if frame_id is None or frame_id == self.drop_last_frame:
            return False
        self.drop_last_frame = frame_id

        conf = snap.get("confidence")
        ex = snap.get("ex")
        ey = snap.get("ey")
        valid = bool(
            snap.get("usable")
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

        self.close_payload()

        print("[TAKEOFF] Kingphoenix LiDAR takeoff")
        self.runtime.takeoff.run()

        if self.abort_requested:
            raise RaceAbort("operator abort after takeoff")

        start_seq = int(self.cfg["mission"]["start_seq"])
        self.runtime.fc_sync.set_current(start_seq)
        result = self.runtime.command.set_mode("AUTO")
        if not result.ok:
            raise RuntimeError(f"AUTO failed: {result.reason}")

        self.started = True
        print(f"[MISSION] AUTO START -> SEQ{start_seq}")

    def handle_land_touchdown(self):
        """Confirm physical touchdown with LiDAR before switching LAND -> ALT_HOLD."""
        c = dict(self.cfg.get("landing") or {})
        threshold_m = float(c.get("lidar_touchdown_m", 0.20))
        confirm_s = float(c.get("confirm_s", 0.8))
        lidar_max_age_s = float(c.get("lidar_max_age_s", 0.30))
        max_groundspeed_mps = float(c.get("max_groundspeed_mps", 0.30))
        target_mode = str(c.get("post_touchdown_mode", "ALT_HOLD")).upper()

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
                    result = self.runtime.command.set_mode(target_mode)
                    if not result.ok:
                        raise RuntimeError(
                            f"{target_mode} after touchdown failed: {result.reason}"
                        )
                    print(
                        f"[LAND] {target_mode} ACTIVE | waiting MANUAL DISARM"
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

            seq = self.v.current_mission_item
            if seq is not None and seq != self.last_seq:
                self.last_seq = int(seq)
                print(f"[MISSION] SEQ{self.last_seq}")

            mode = str(self.v.mode or "").upper()

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
            self.runtime.stop()


def main():
    return TerminalRace().run()


if __name__ == "__main__":
    sys.exit(main())
