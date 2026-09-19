#!/usr/bin/env python3
from __future__ import annotations

import signal
import sys
import time

from config import load_json
from core.runtime import KingphoenixRuntime
from mission.fc_mission_sync import MissionSyncError
from pymavlink import mavutil


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

    def ensure_terminal_land(self, actual):
        """Ensure final WP is SEQ17 and append/repair SEQ18 as NAV_LAND."""
        m = self.cfg["mission"]
        final_wp_seq = int(m.get("final_wp_seq", 17))
        land_seq = int(m.get("land_seq", 18))

        if self.v.armed:
            raise RuntimeError("mission rewrite blocked: vehicle ARMED")
        if final_wp_seq >= len(actual):
            raise RuntimeError(
                f"FC mission missing final waypoint SEQ{final_wp_seq}; items={len(actual)}"
            )
        if len(actual) > land_seq + 1:
            raise RuntimeError(
                f"FC mission has items beyond SEQ{land_seq}; refusing automatic rewrite"
            )

        final_item = dict(actual[final_wp_seq])
        changed = False

        # Legacy mission may still have LAND at SEQ17. Convert it back to a
        # normal final waypoint at the exact same coordinate.
        if int(final_item["command"]) == int(mavutil.mavlink.MAV_CMD_NAV_LAND):
            final_item["command"] = int(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
            final_item["param1"] = 0.0
            final_item["param2"] = float(
                self.runtime.full_cfg["waypoint"].get("acceptance_radius_m", 0.50)
            )
            final_item["param3"] = 0.0
            final_item["param4"] = float("nan")
            final_item["autocontinue"] = 1
            actual[final_wp_seq] = final_item
            changed = True
            print(
                f"[MISSION] SEQ{final_wp_seq} legacy LAND -> converted to final WAYPOINT"
            )

        land_item = {
            "seq": land_seq,
            "command": int(mavutil.mavlink.MAV_CMD_NAV_LAND),
            "frame": int(final_item["frame"]),
            "current": 0,
            "autocontinue": 1,
            "param1": 0.0,
            "param2": 0.0,
            "param3": 0.0,
            "param4": float("nan"),
            "lat": float(final_item["lat"]),
            "lon": float(final_item["lon"]),
            "alt_m": 0.0,
        }

        if len(actual) == land_seq:
            actual.append(land_item)
            changed = True
            print(
                f"[MISSION] append SEQ{land_seq}=LAND at SEQ{final_wp_seq} coordinate"
            )
        elif len(actual) == land_seq + 1:
            got = actual[land_seq]
            same_land = (
                int(got["command"]) == int(mavutil.mavlink.MAV_CMD_NAV_LAND)
                and abs(float(got["lat"]) - float(land_item["lat"])) <= 1e-7
                and abs(float(got["lon"]) - float(land_item["lon"])) <= 1e-7
            )
            if not same_land:
                actual[land_seq] = land_item
                changed = True
                print(
                    f"[MISSION] repair SEQ{land_seq} -> LAND at SEQ{final_wp_seq} coordinate"
                )
        else:
            raise RuntimeError(
                f"FC mission count={len(actual)} cannot create SEQ{land_seq} safely"
            )

        if changed:
            self.runtime.fc_sync.upload_exact(actual)
            verify = self.runtime.fc_sync.download()
            if len(verify) != land_seq + 1:
                raise RuntimeError(
                    f"mission verify failed: expected {land_seq + 1} items, got {len(verify)}"
                )
            if int(verify[final_wp_seq]["command"]) != int(
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            ):
                raise RuntimeError(
                    f"mission verify failed: SEQ{final_wp_seq} is not WAYPOINT"
                )
            if int(verify[land_seq]["command"]) != int(
                mavutil.mavlink.MAV_CMD_NAV_LAND
            ):
                raise RuntimeError(
                    f"mission verify failed: SEQ{land_seq} is not LAND"
                )
            print(
                f"[MISSION] VERIFIED | SEQ{final_wp_seq}=WAYPOINT | "
                f"SEQ{land_seq}=LAND"
            )
            return verify

        print(
            f"[MISSION] terminal LAND already valid | "
            f"SEQ{final_wp_seq}=WAYPOINT | SEQ{land_seq}=LAND"
        )
        return actual

    def read_fc_mission(self):
        m = self.cfg["mission"]
        actual = self.runtime.fc_sync.download()
        actual = self.ensure_terminal_land(actual)
        required = {
            "START": int(m["start_seq"]),
            "AI": int(m["vision_enable_seq"]),
            "DROP": int(m["drop_seq"]),
            "FALLBACK": int(m["fallback_start_seq"]),
            "RESUME": int(m["resume_seq"]),
            "FINAL_WP": int(m["final_wp_seq"]),
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
                if self.last_seq == int(m["land_seq"]):
                    print(
                        "[LAND] Final mission item active | waiting FC LAND mode, "
                        "then LiDAR touchdown confirmation"
                    )

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
