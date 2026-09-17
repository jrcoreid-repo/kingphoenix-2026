#!/usr/bin/env python3
"""Kingphoenix hybrid drop V3: race launch + live streams + GCS servo pretest.

This runner keeps V1/V2 untouched.

Flow:
  real GCS connects and FRONT+BOTTOM streams become LIVE
  shared Hailo/model preloads once; AI inference stays OFF
  route HOME->WP1->WP2 is uploaded/verified DISARMED
  GCS SERVO_CONTROL OPEN/CLOSE (or TEST) drives the real drop servos
  runner requires OPEN then CLOSE and a final CLOSED state
  operator still types GO in the terminal
  GUIDED LiDAR launch to 0.50m -> immediate AUTO WP1 while climbing to WP alt
  WP1 -> bottom quick-drop -> AUTO WP2 -> LAND

SERVO_CONTROL is accepted only during the explicit disarmed preflight window.
It is disabled immediately before GO so the GCS cannot disturb the in-flight
payload logic.
"""

from __future__ import annotations

import threading
import time

from streaming.gcs_protocol import ack
from tools.flight_test_common import DirectFlightTest, FlightTestError
from tools.run_bottom_drop_aggressive_headless import (
    require_word,
    set_servo_pair,
    verify_servo_functions,
)
from tools.run_bottom_target_pd_tuning import bottom_config, zero_xy
from tools.run_hybrid_drop_mission_v1 import (
    EventBus,
    GcsServer,
    SharedState,
    VisionStreamSession,
    WaypointManager,
    PilotTakeover,
    disable_ai,
    run_quick_drop,
    wait_streams,
)
from tools.run_hybrid_drop_race_launch_v2 import (
    parse_args,
    validate,
    race_lidar_launch,
    fly_auto_to_wp_race_launch,
)
from tools.run_hybrid_mission_test import (
    FC_WP1_SEQ,
    FC_WP2_SEQ,
    distance_m,
    fly_auto_to_wp,
    load_waypoint,
    upload_two_wp_route,
    verify_two_wp_route,
)


class PreflightServoBridge:
    """Small, disarmed-only bridge from GCS SERVO_CONTROL to DO_SET_SERVO."""

    def __init__(self, test: DirectFlightTest, args):
        self.test = test
        self.a = args
        self.lock = threading.Lock()
        self.enabled = False
        self.state = "UNKNOWN"
        self.seen_open = False
        self.seen_close = False

    def enable(self):
        with self.lock:
            self.enabled = True
            self.state = "UNKNOWN"
            self.seen_open = False
            self.seen_close = False
        print("[GCS-SERVO] PRE-FLIGHT backend ENABLED (DISARMED only)")

    def disable(self):
        with self.lock:
            was_enabled = self.enabled
            self.enabled = False
        if was_enabled:
            print("[GCS-SERVO] PRE-FLIGHT backend DISABLED")

    def snapshot(self):
        with self.lock:
            return {
                "enabled": self.enabled,
                "state": self.state,
                "seen_open": self.seen_open,
                "seen_close": self.seen_close,
            }

    def command(self, action: str):
        action = str(action).upper().strip()
        with self.lock:
            if not self.enabled:
                return False, "SERVO PRE-FLIGHT WINDOW NOT ACTIVE", self.snapshot_unlocked()
            if self.test.master is None:
                return False, "FC MAVLINK BACKEND NOT READY", self.snapshot_unlocked()
            if bool(self.test.armed):
                self.enabled = False
                return False, "SERVO GCS CONTROL BLOCKED: VEHICLE ARMED", self.snapshot_unlocked()

            if action == "OPEN":
                set_servo_pair(
                    self.test,
                    self.a.servo_a,
                    self.a.servo_b,
                    self.a.open_pwm,
                    "GCS OPEN",
                    self.a.servo_settle,
                )
                self.state = "OPEN"
                self.seen_open = True
            elif action == "CLOSE":
                set_servo_pair(
                    self.test,
                    self.a.servo_a,
                    self.a.servo_b,
                    self.a.close_pwm,
                    "GCS CLOSE",
                    self.a.servo_settle,
                )
                self.state = "CLOSED"
                self.seen_close = True
            elif action == "TEST":
                set_servo_pair(
                    self.test,
                    self.a.servo_a,
                    self.a.servo_b,
                    self.a.open_pwm,
                    "GCS TEST OPEN",
                    self.a.servo_settle,
                )
                self.state = "OPEN"
                self.seen_open = True
                set_servo_pair(
                    self.test,
                    self.a.servo_a,
                    self.a.servo_b,
                    self.a.close_pwm,
                    "GCS TEST CLOSE",
                    self.a.servo_settle,
                )
                self.state = "CLOSED"
                self.seen_close = True
            else:
                return False, f"SERVO ACTION REJECTED: {action or '--'}", self.snapshot_unlocked()

            data = self.snapshot_unlocked()
            data.update(
                {
                    "action": action,
                    "servo_a": int(self.a.servo_a),
                    "servo_b": int(self.a.servo_b),
                    "open_pwm": int(self.a.open_pwm),
                    "close_pwm": int(self.a.close_pwm),
                    "executed": True,
                    "backend": "DIRECT_MAVLINK_DO_SET_SERVO",
                }
            )
            return True, f"SERVO {action} EXECUTED", data

    def snapshot_unlocked(self):
        return {
            "enabled": self.enabled,
            "state": self.state,
            "seen_open": self.seen_open,
            "seen_close": self.seen_close,
        }


class RaceGcsServer(GcsServer):
    """Normal GCS/stream server with a V3-only servo execution backend."""

    def __init__(self, state, events, waypoints, stop_event, servo_bridge):
        super().__init__(state, events, waypoints, stop_event, command_manager=None)
        self.servo_bridge = servo_bridge

    def handle(self, msg, peer=None):
        kind = str(msg.get("type", "")).upper()
        if kind != "SERVO_CONTROL":
            return super().handle(msg, peer)

        tx = msg.get("tx_id")
        action = str(msg.get("action") or "").upper().strip()
        if action not in self.SERVO_ACTION_ALLOWLIST:
            return ack(tx, False, f"SERVO ACTION REJECTED: {action or '--'}")

        self._touch_gcs_heartbeat()
        try:
            ok, message, data = self.servo_bridge.command(action)
        except Exception as exc:
            print(f"[GCS-SERVO] {action} exception: {exc}")
            return ack(tx, False, f"SERVO {action} EXCEPTION: {exc}")

        print(
            f"[GCS-SERVO] action={action} ok={ok} "
            f"state={data.get('state')} executed={data.get('executed', False)}"
        )
        return ack(tx, ok, message, data=data)


def wait_gcs_servo_pretest(test: DirectFlightTest, bridge: PreflightServoBridge, timeout_s=180.0):
    print("[GROUND SERVO PRETEST - GCS]")
    print("  Use the GCS SERVO controls: OPEN then CLOSE (or TEST).")
    print("  Mission cannot GO until OPEN and CLOSE have both been executed and state=CLOSED.")

    deadline = time.monotonic() + float(timeout_s)
    last_status = 0.0
    while time.monotonic() < deadline:
        test.poll(0.10)
        if test.armed:
            raise FlightTestError("vehicle armed during GCS servo pretest")

        s = bridge.snapshot()
        if s["seen_open"] and s["seen_close"] and s["state"] == "CLOSED":
            print("[GCS-SERVO] PRETEST PASS: OPEN seen + CLOSE seen + final state CLOSED")
            return

        now = time.monotonic()
        if now - last_status >= 2.0:
            print(
                f"[GCS-SERVO] waiting | state={s['state']} "
                f"open_seen={s['seen_open']} close_seen={s['seen_close']}"
            )
            last_status = now
        time.sleep(0.02)

    raise FlightTestError("GCS servo pretest timeout before OPEN+CLOSE+CLOSED")


def main() -> int:
    a = parse_args()
    validate(a)

    if a.headless:
        print("[BLOCKED] V3 GCS-servo test requires the real GCS; remove --headless")
        return 2

    try:
        wp1 = load_waypoint(a.strategy, a.wp1)
        wp2 = load_waypoint(a.strategy, a.wp2)
    except FlightTestError as exc:
        print(f"[BLOCKED] {exc}")
        return 2

    route_distance = distance_m(wp1["lat"], wp1["lon"], wp2["lat"], wp2["lon"])
    if route_distance < 0.50:
        print(f"[BLOCKED] WP1 -> WP2 only {route_distance:.2f}m; need >=0.50m")
        return 2

    cfg = bottom_config(float(a.confidence))
    detector_class_id = int(cfg.get("ai", {}).get("bottom", {}).get("detector_class_id", 0))

    stop_event = threading.Event()
    state = SharedState()
    events = EventBus()
    waypoints = WaypointManager(state)
    test = DirectFlightTest()
    servo_bridge = PreflightServoBridge(test, a)
    gcs = RaceGcsServer(state, events, waypoints, stop_event, servo_bridge)
    vision = VisionStreamSession(cfg, state, stop_event)
    disable_ai(vision, "HYBRID_RACE_GCS_INIT")

    flight_open = False
    drop_phase_active = False
    result = None

    print("============================================================")
    print(" KINGPHOENIX HYBRID DROP V3 - RACE LAUNCH + GCS SERVO")
    print("============================================================")
    print(
        f"[PLAN] LiDAR {a.race_launch_alt:.2f}m -> immediate AUTO {wp1['label']} "
        f"(WP alt={wp1['alt_m']:.2f}m) -> DROP <= {a.drop_timeout:.2f}s -> "
        f"AUTO {wp2['label']} -> LAND"
    )
    print("[STREAM] FRONT + BOTTOM stay ON throughout; real GCS is required")
    print("[AI] shared model preloaded; inference OFF until bottom DROP stage")
    print("[SERVO PRETEST] OPEN/CLOSE from GCS only; GO remains terminal-only")
    print("[GCS SERVO SAFETY] backend is DISARMED-only and disabled before GO")
    print("------------------------------------------------------------")

    try:
        gcs.start()
        vision.start()
        if vision.shared_ai is None or not bool(getattr(vision.shared_ai, "ready", False)):
            raise FlightTestError("shared Hailo YOLO26 did not become READY")
        wait_streams(state, float(a.stream_wait))

        test.open()
        flight_open = True
        test.collect_preflight(require_auto=True)
        if test.global_pos is None:
            raise FlightTestError("GLOBAL_POSITION_INT unavailable")

        cur_lat, cur_lon, prearm_rel = test.global_pos
        for wp in (wp1, wp2):
            d = distance_m(cur_lat, cur_lon, wp["lat"], wp["lon"])
            if d > 100.0:
                raise FlightTestError(f"{wp['label']} is {d:.1f}m away; test limit 100m")
            print(
                f"[WP] {wp['label']} lat={wp['lat']:.7f} lon={wp['lon']:.7f} "
                f"distance={d:.2f}m relAlt={wp['alt_m']:.2f}m"
            )
        print(f"[PREARM] FCrelAlt={prearm_rel:.2f}m DIAGNOSTIC ONLY")

        upload_two_wp_route(test, wp1, wp2, acceptance_radius_m=float(a.accept_radius))
        verify_two_wp_route(test, wp1, wp2)
        print("[SAFE] HOME -> WP1 -> WP2 verified while DISARMED")

        if a.preload_only:
            print("[PRELOAD-ONLY] streams/model/route verified; vehicle remains DISARMED")
            return 0

        if not a.skip_servo_function_check:
            verify_servo_functions(test, (a.servo_a, a.servo_b))
        else:
            print("[SERVO FUNCTION] CHECK SKIPPED by operator")

        servo_bridge.enable()
        wait_gcs_servo_pretest(test, servo_bridge)

        require_word(
            "GCS servo pretest PASS and servos CLOSED. Safety pilot ready. Type GO: ",
            "GO",
        )
        # Close the GCS actuator window BEFORE any arming/launch action.
        servo_bridge.disable()

        disable_ai(vision, "RACE_LAUNCH_AUTO_WP1")
        race_lidar_launch(test, a)
        rel = None if test.global_pos is None else test.global_pos[2]
        print(
            f"[RACE HANDOFF] LiDAR={test.lidar:.2f}m "
            f"FCrelAlt={rel if rel is not None else float('nan'):.2f}m -> AUTO WP1 NOW"
        )

        fly_auto_to_wp_race_launch(test, seq=FC_WP1_SEQ, label=wp1["label"], a=a)

        drop_phase_active = True
        drop_start = time.monotonic()
        print("[DROP TIMER] START @ WP1 REACHED")
        result = run_quick_drop(test, state, vision, a, detector_class_id, drop_start)

        zero_xy(test)
        disable_ai(vision, "DROP_DONE" if result["dropped"] else "DROP_TIMEOUT")
        drop_phase_active = False

        if result["dropped"]:
            print(
                f"[DROP RESULT] SUCCESS | servo command={result['servo_cmd_elapsed']:.3f}s "
                f"complete={result['complete_elapsed']:.3f}s"
            )
        else:
            print(
                f"[DROP RESULT] TIMEOUT | budget={a.drop_timeout:.2f}s | servo stayed CLOSED"
            )

        print("[MISSION] DROP phase finished -> AI OFF -> AUTO WP2")
        fly_auto_to_wp(
            test,
            seq=FC_WP2_SEQ,
            label=wp2["label"],
            floor_m=float(a.auto_floor),
            ceiling_m=float(a.auto_ceiling),
            timeout=float(a.wp_timeout),
        )

        test.land_and_wait()
        if result["dropped"]:
            print("[RESULT] HYBRID DROP V3 RACE-LAUNCH + GCS-SERVO PASS")
            return 0
        print("[RESULT] HYBRID ROUTE COMPLETE, BUT DROP TIMED OUT")
        return 4

    except PilotTakeover as exc:
        print(f"[PILOT TAKEOVER] {exc}")
        try:
            if flight_open:
                zero_xy(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "PILOT_TAKEOVER")
        except Exception:
            pass
        print("[RC TAKEOVER] no software LAND commanded")
        return 130

    except KeyboardInterrupt:
        print("\n[STOP] operator interrupt")
        try:
            if flight_open:
                zero_xy(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "OPERATOR_INTERRUPT")
        except Exception:
            pass
        print("[RC TAKEOVER] no software LAND commanded")
        return 130

    except FlightTestError as exc:
        print(f"[BLOCKED/ABORT] {exc}")
        try:
            if flight_open:
                zero_xy(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "FLIGHT_ABORT")
        except Exception:
            pass
        if flight_open and bool(getattr(test, "armed", False)):
            where = " during DROP" if drop_phase_active else ""
            print(f"[RC TAKEOVER]{where}; software LAND suppressed")
        return 2

    except Exception as exc:
        print(f"[UNEXPECTED] {type(exc).__name__}: {exc}")
        try:
            if flight_open:
                zero_xy(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "UNEXPECTED_ABORT")
        except Exception:
            pass
        if flight_open and bool(getattr(test, "armed", False)):
            print("[RC TAKEOVER] software LAND suppressed")
        return 3

    finally:
        servo_bridge.disable()
        stop_event.set()
        try:
            if flight_open:
                test.close()
        except Exception:
            pass
        try:
            vision.join(timeout=1.0)
        except Exception:
            pass
        try:
            gcs.join(timeout=1.0)
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
