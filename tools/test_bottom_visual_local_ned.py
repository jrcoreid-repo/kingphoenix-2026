#!/usr/bin/env python3
"""Standalone bottom-vision -> LOCAL_NED position-target flight test.

Purpose
-------
Validate the new Kingphoenix drop-alignment concept without AUTO and without
raw XY velocity control:

  LiDAR takeoff -> bottom YOLO -> image error -> metric BODY offset
  -> BODY offset rotated by EKF yaw -> absolute LOCAL_NED position target
  -> ArduPilot position controller performs accel/braking/hold
  -> operator lands manually from laptop.

There is no payload drop and no software LAND in this runner.

Important
---------
- This runner intentionally does NOT use fuzzy/PID/PD vision velocity control.
- A valid bottom-camera calibration (HFOV/VFOV) is required, either from
  config/calibration/bottom_camera.json or explicit --hfov-deg/--vfov-deg.
- Mission Planner heading must match the real vehicle heading before flight.
  A 90-degree yaw error rotates the visual correction by 90 degrees.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from core.event_bus import EventBus
from core.shared_state import SharedState
from mission.waypoint_manager import WaypointManager
from streaming.gcs_server import GcsServer
from streaming.vision_stream_session import VisionStreamSession
from tools.flight_test_common import DirectFlightTest, FlightTestError
from tools.mission_lidar_takeoff import profiled_lidar_takeoff
from tools.run_bottom_drop_aggressive_headless import require_word
from tools.run_bottom_target_pd_headless import HeadlessLoopbackGcs
from tools.run_bottom_target_pd_tuning import bottom_config, clamp
from tools.run_hybrid_drop_mission_v1 import (
    PilotTakeover,
    activate_bottom_ai,
    check_flight_health,
    check_lidar_guard,
    disable_ai,
    wait_streams,
)


CALIB_PATH = PROJECT_ROOT / "config" / "calibration" / "bottom_camera.json"


class VisualPositionFlightTest(DirectFlightTest):
    """DirectFlightTest plus fresh EKF yaw from ATTITUDE."""

    def __init__(self):
        super().__init__()
        self.yaw_rad = None
        self.yaw_ts = 0.0
        self.local_vel = None

    def _request_streams(self):
        super()._request_streams()
        # ATTITUDE message ID 30; 20 Hz is enough for the 10 Hz visual target loop.
        self._request_interval(30, 20.0)

    def _handle(self, msg):
        super()._handle(msg)
        if msg is None or msg.get_type() == "BAD_DATA":
            return
        now = time.monotonic()
        if msg.get_type() == "ATTITUDE":
            self.yaw_rad = float(msg.yaw)
            self.yaw_ts = now
        elif msg.get_type() == "LOCAL_POSITION_NED":
            self.local_vel = (float(msg.vx), float(msg.vy), float(msg.vz))

    def yaw_ok(self, max_age_s: float = 0.35) -> bool:
        return (
            self.yaw_rad is not None
            and (time.monotonic() - self.yaw_ts) <= float(max_age_s)
        )

    def poll_drain(self, first_timeout: float = 0.02, max_messages: int = 80):
        """Consume one blocking MAVLink message, then drain queued messages.

        The direct test connection requests many telemetry streams whose combined
        rate is much higher than the 20 Hz vision loop. Reading only one MAVLink
        packet per loop can leave ATTITUDE buried in the serial receive backlog
        and make yaw look stale even while the FC itself is flying normally.
        """
        self.poll(first_timeout)
        drained = 1
        while drained < int(max_messages):
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            self._handle(msg)
            drained += 1
        return drained


def parse_args():
    ap = argparse.ArgumentParser(
        description=(
            "Bottom YOLO -> metric offset -> LOCAL_NED position target; "
            "2.5m standalone test; manual landing"
        )
    )

    # Standalone takeoff profile.
    ap.add_argument("--takeoff-alt", type=float, default=2.50)
    ap.add_argument("--takeoff-rate", type=float, default=0.45)
    ap.add_argument("--near-rate", type=float, default=0.20)
    ap.add_argument("--taper-zone", type=float, default=0.30)
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--takeoff-ceiling", type=float, default=2.85)
    ap.add_argument("--zero-flush", type=float, default=0.15)
    ap.add_argument("--takeoff-timeout", type=float, default=20.0)

    # Vision.
    ap.add_argument("--confidence", type=float, default=0.50)
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--loop-hz", type=float, default=20.0)
    ap.add_argument("--target-hz", type=float, default=10.0)
    ap.add_argument("--center-m", type=float, default=0.10)
    ap.add_argument("--center-frames", type=int, default=2)
    ap.add_argument("--lost-timeout", type=float, default=0.15)

    # Camera geometry. If omitted, valid calibration JSON must provide HFOV/VFOV.
    ap.add_argument("--hfov-deg", type=float, default=None)
    ap.add_argument("--vfov-deg", type=float, default=None)

    # Position-target shaping/safety.
    ap.add_argument(
        "--max-correction-m",
        type=float,
        default=0.60,
        help="max visual BODY offset applied per target update",
    )
    ap.add_argument(
        "--max-anchor-radius-m",
        type=float,
        default=1.00,
        help="hard XY radius from visual-stage entry position",
    )
    ap.add_argument(
        "--target-alpha",
        type=float,
        default=0.45,
        help="EMA alpha for absolute LOCAL_NED target",
    )

    # 2.5m standalone altitude guard.
    ap.add_argument("--guard-floor", type=float, default=1.90)
    ap.add_argument("--guard-ceiling", type=float, default=2.85)
    ap.add_argument("--guard-outside-s", type=float, default=0.35)
    ap.add_argument("--lidar-stale-s", type=float, default=0.60)

    # Streaming.
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--stream-wait", type=float, default=60.0)
    return ap.parse_args()


def validate(a):
    if not 0.50 <= a.takeoff_alt <= 3.0:
        raise SystemExit("--takeoff-alt must be 0.50..3.00m")
    if a.takeoff_ceiling <= a.takeoff_alt:
        raise SystemExit("--takeoff-ceiling must be above --takeoff-alt")
    if not 0.10 <= a.takeoff_rate <= 0.80:
        raise SystemExit("--takeoff-rate must be 0.10..0.80m/s")
    if not 0.08 <= a.near_rate <= a.takeoff_rate:
        raise SystemExit("--near-rate must be 0.08..takeoff-rate")
    if not 10 <= a.loop_hz <= 40:
        raise SystemExit("--loop-hz must be 10..40Hz")
    if not 2 <= a.target_hz <= a.loop_hz:
        raise SystemExit("--target-hz must be 2..loop-hz")
    if not 3 <= a.duration <= 180:
        raise SystemExit("--duration must be 3..180s")
    if not 0.03 <= a.center_m <= 0.50:
        raise SystemExit("--center-m must be 0.03..0.50m")
    if not 1 <= a.center_frames <= 20:
        raise SystemExit("--center-frames must be 1..20")
    if not 0.0 <= a.lost_timeout <= 1.0:
        raise SystemExit("--lost-timeout must be 0..1s")
    if not 0.10 <= a.max_correction_m <= 1.50:
        raise SystemExit("--max-correction-m must be 0.10..1.50m")
    if not 0.20 <= a.max_anchor_radius_m <= 3.0:
        raise SystemExit("--max-anchor-radius-m must be 0.20..3.00m")
    if not 0.05 <= a.target_alpha <= 1.0:
        raise SystemExit("--target-alpha must be 0.05..1.0")
    if not 0.30 <= a.guard_floor < a.guard_ceiling <= 3.0:
        raise SystemExit("invalid LiDAR guard")


def load_fov(a) -> tuple[float, float, str]:
    if a.hfov_deg is not None or a.vfov_deg is not None:
        if a.hfov_deg is None or a.vfov_deg is None:
            raise FlightTestError("provide BOTH --hfov-deg and --vfov-deg")
        hfov = float(a.hfov_deg)
        vfov = float(a.vfov_deg)
        source = "CLI"
    else:
        try:
            raw = json.loads(CALIB_PATH.read_text(encoding="utf-8"))
        except Exception as exc:
            raise FlightTestError(
                f"bottom camera calibration unavailable: {CALIB_PATH}: {exc}"
            )
        if not bool(raw.get("valid", False)):
            raise FlightTestError(
                "bottom_camera.json is not valid; calibrate it or pass "
                "--hfov-deg and --vfov-deg"
            )
        hfov = float(raw.get("hfov_deg"))
        vfov = float(raw.get("vfov_deg"))
        source = str(CALIB_PATH)

    if not 20.0 <= hfov <= 160.0 or not 20.0 <= vfov <= 160.0:
        raise FlightTestError(
            f"implausible FOV hfov={hfov:.2f} vfov={vfov:.2f}; expected 20..160 deg"
        )
    return hfov, vfov, source


def vector_cap(x: float, y: float, cap: float) -> tuple[float, float]:
    mag = math.hypot(x, y)
    if mag <= float(cap) or mag <= 1e-9:
        return x, y
    s = float(cap) / mag
    return x * s, y * s


def body_offset_from_detection(
    ex: float,
    ey: float,
    height_m: float,
    hfov_deg: float,
    vfov_deg: float,
) -> tuple[float, float]:
    """Convert normalized YOLO center error to BODY forward/right metres.

    Project's physically verified bottom-camera mapping:
      image X left->right = drone FRONT->REAR
      image Y top->bottom = drone RIGHT->LEFT

    Therefore:
      BODY forward = -ex * ground_half_extent_along_image_X
      BODY right   = -ey * ground_half_extent_along_image_Y

    ex/ey are normalized by half-frame dimensions, so a pinhole projection is:
      offset = normalized_error * h * tan(FOV/2)
    """
    half_x_m = float(height_m) * math.tan(math.radians(float(hfov_deg)) / 2.0)
    half_y_m = float(height_m) * math.tan(math.radians(float(vfov_deg)) / 2.0)
    body_forward_m = -float(ex) * half_x_m
    body_right_m = -float(ey) * half_y_m
    return body_forward_m, body_right_m


def body_to_ned(
    body_forward_m: float,
    body_right_m: float,
    yaw_rad: float,
) -> tuple[float, float]:
    c = math.cos(float(yaw_rad))
    s = math.sin(float(yaw_rad))
    dn = body_forward_m * c - body_right_m * s
    de = body_forward_m * s + body_right_m * c
    return dn, de


def clamp_target_to_anchor(
    target_n: float,
    target_e: float,
    anchor_n: float,
    anchor_e: float,
    radius_m: float,
) -> tuple[float, float, bool]:
    dn = float(target_n) - float(anchor_n)
    de = float(target_e) - float(anchor_e)
    mag = math.hypot(dn, de)
    if mag <= float(radius_m) or mag <= 1e-9:
        return target_n, target_e, False
    s = float(radius_m) / mag
    return anchor_n + dn * s, anchor_e + de * s, True


def send_local_position_target(
    test: VisualPositionFlightTest,
    north_m: float,
    east_m: float,
    down_m: float,
):
    """Absolute LOCAL_NED position target; velocity/accel/yaw fields ignored."""
    # Use x/y/z position, ignore VX/VY/VZ, AX/AY/AZ, yaw and yaw-rate.
    type_mask = (
        (1 << 3) | (1 << 4) | (1 << 5)
        | (1 << 6) | (1 << 7) | (1 << 8)
        | (1 << 10) | (1 << 11)
    )
    test.master.mav.set_position_target_local_ned_send(
        0,
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        float(north_m),
        float(east_m),
        float(down_m),
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0,
    )


def hold_current_position(test: VisualPositionFlightTest) -> tuple[float, float, float]:
    if test.local is None:
        raise FlightTestError("LOCAL_POSITION_NED unavailable for hold")
    n, e, d = map(float, test.local)
    send_local_position_target(test, n, e, d)
    return n, e, d


def run_visual_position_tracking(
    test: VisualPositionFlightTest,
    state: SharedState,
    vision: VisionStreamSession,
    a,
    detector_class_id: int,
    hfov_deg: float,
    vfov_deg: float,
):
    activate_bottom_ai(vision, detector_class_id)
    if test.mode != "GUIDED":
        test.set_mode("GUIDED")

    # Collect fresh LOCAL_NED + yaw after takeoff.
    ready_deadline = time.monotonic() + 3.0
    while time.monotonic() < ready_deadline:
        test.poll(0.05)
        if test.local is not None and test.yaw_ok() and test.lidar_ok():
            break
    if test.local is None:
        raise FlightTestError("LOCAL_POSITION_NED unavailable before visual stage")
    if not test.yaw_ok():
        raise FlightTestError("ATTITUDE yaw unavailable/stale before visual stage")
    if not test.lidar_ok():
        raise FlightTestError("LiDAR unavailable/stale before visual stage")

    anchor_n, anchor_e, anchor_d = map(float, test.local)
    target_n, target_e, target_d = anchor_n, anchor_e, anchor_d
    send_local_position_target(test, target_n, target_e, target_d)

    print(
        f"[VISPOS] ACTIVE anchor N/E/D={anchor_n:+.2f}/{anchor_e:+.2f}/{anchor_d:+.2f}m "
        f"| HFOV/VFOV={hfov_deg:.2f}/{vfov_deg:.2f}deg"
    )
    print(
        f"[VISPOS] target update={a.target_hz:.1f}Hz loop={a.loop_hz:.1f}Hz "
        f"| max correction={a.max_correction_m:.2f}m "
        f"| anchor radius={a.max_anchor_radius_m:.2f}m"
    )
    print("[VISPOS] NO raw XY velocity control; ArduPilot owns position braking/hold")

    period = 1.0 / float(a.loop_hz)
    target_period = 1.0 / float(a.target_hz)
    deadline = time.monotonic() + float(a.duration)
    next_target_tx = 0.0
    last_print = 0.0
    last_frame = None
    center_streak = 0
    lost_since = None
    outside_since = lidar_loss_since = None
    filt_target_n = None
    filt_target_e = None
    last_valid_body = (0.0, 0.0)
    phase = "SEARCH"

    while time.monotonic() < deadline:
        loop_start = time.monotonic()
        test.poll_drain(0.02)
        check_flight_health(test)

        if test.mode != "GUIDED":
            raise PilotTakeover(
                f"operator changed mode during visual position tracking to {test.mode}"
            )
        if not test.yaw_ok():
            # Do not fly a visual correction on stale heading. Hold the current
            # LOCAL_NED position and allow telemetry a short recovery window.
            if test.local is not None:
                cur_n, cur_e, cur_d = map(float, test.local)
                send_local_position_target(test, cur_n, cur_e, cur_d)
            yaw_recover_deadline = time.monotonic() + 1.0
            while time.monotonic() < yaw_recover_deadline:
                test.poll_drain(0.02)
                if test.yaw_ok():
                    print("[VISPOS] ATTITUDE yaw telemetry recovered")
                    break
            else:
                raise FlightTestError(
                    "ATTITUDE yaw telemetry stale >1.0s; visual target frozen"
                )

        outside_since, lidar_loss_since = check_lidar_guard(
            test,
            floor_m=float(a.guard_floor),
            ceiling_m=float(a.guard_ceiling),
            outside_since=outside_since,
            lidar_loss_since=lidar_loss_since,
            max_outside_s=float(a.guard_outside_s),
            max_stale_s=float(a.lidar_stale_s),
        )

        now = time.monotonic()
        with state.lock:
            b = dict(state.vision.get("bottom") or {})

        conf = b.get("confidence")
        ex = b.get("ex")
        ey = b.get("ey")
        frame_id = b.get("frame_id")
        ts_ns = int(b.get("frame_timestamp_ns") or 0)
        age = (
            float("inf")
            if ts_ns <= 0
            else max(0.0, (time.monotonic_ns() - ts_ns) / 1e9)
        )
        valid = bool(
            b.get("vision_valid")
            and conf is not None
            and float(conf) >= float(a.confidence)
            and ex is not None
            and ey is not None
            and age <= 0.20
        )

        raw_target_n = target_n
        raw_target_e = target_e
        body_f = body_r = 0.0
        anchor_clamped = False

        if valid and frame_id != last_frame:
            last_frame = frame_id
            lost_since = None

            height_m = float(test.lidar)
            body_f, body_r = body_offset_from_detection(
                float(ex),
                float(ey),
                height_m,
                hfov_deg,
                vfov_deg,
            )
            body_f, body_r = vector_cap(
                body_f,
                body_r,
                float(a.max_correction_m),
            )
            last_valid_body = (body_f, body_r)

            # Convert the current camera/body residual to world-frame offset.
            dn, de = body_to_ned(body_f, body_r, float(test.yaw_rad))
            cur_n, cur_e, _cur_d = map(float, test.local)
            raw_target_n = cur_n + dn
            raw_target_e = cur_e + de
            raw_target_n, raw_target_e, anchor_clamped = clamp_target_to_anchor(
                raw_target_n,
                raw_target_e,
                anchor_n,
                anchor_e,
                float(a.max_anchor_radius_m),
            )

            # Light target filtering only; FC performs the actual motion control.
            if filt_target_n is None:
                filt_target_n = raw_target_n
                filt_target_e = raw_target_e
            else:
                alpha = float(a.target_alpha)
                filt_target_n = alpha * raw_target_n + (1.0 - alpha) * filt_target_n
                filt_target_e = alpha * raw_target_e + (1.0 - alpha) * filt_target_e

            target_n = float(filt_target_n)
            target_e = float(filt_target_e)
            target_d = anchor_d

            visual_error_m = math.hypot(body_f, body_r)
            if visual_error_m <= float(a.center_m):
                center_streak += 1
            else:
                center_streak = 0

            if center_streak >= int(a.center_frames):
                phase = "LOCK"
            else:
                phase = "TRACK"

        elif not valid:
            center_streak = 0
            lost_since = lost_since or now
            phase = "TARGET LOST"

            # Never chase the last stale target. Freeze at the current vehicle
            # position as soon as target data is invalid.
            if test.local is not None:
                cur_n, cur_e, cur_d = map(float, test.local)
                target_n, target_e, target_d = cur_n, cur_e, cur_d
                filt_target_n, filt_target_e = cur_n, cur_e

            if now - lost_since >= float(a.lost_timeout):
                last_valid_body = (0.0, 0.0)

        if now >= next_target_tx:
            send_local_position_target(test, target_n, target_e, target_d)
            next_target_tx = now + target_period

        if now - last_print >= 0.20:
            conf_t = "--" if conf is None else f"{float(conf):.2f}"
            ex_t = "--" if ex is None else f"{float(ex):+.3f}"
            ey_t = "--" if ey is None else f"{float(ey):+.3f}"
            yaw_deg = math.degrees(float(test.yaw_rad))
            cur_n, cur_e, cur_d = map(float, test.local)
            speed_xy = (
                0.0
                if test.local_vel is None
                else math.hypot(float(test.local_vel[0]), float(test.local_vel[1]))
            )
            print(
                f"[VISPOS] {phase:11s} conf={conf_t} ex={ex_t} ey={ey_t} "
                f"| BODY={body_f:+.2f}/{body_r:+.2f}m "
                f"| yaw={yaw_deg:+.1f}deg "
                f"| CUR N/E={cur_n:+.2f}/{cur_e:+.2f} "
                f"-> TGT={target_n:+.2f}/{target_e:+.2f} "
                f"| Vxy={speed_xy:.2f}m/s "
                f"| center={center_streak}/{a.center_frames}"
                + (" | ANCHOR-CAP" if anchor_clamped else "")
            )
            last_print = now

        sleep_s = period - (time.monotonic() - loop_start)
        if sleep_s > 0:
            time.sleep(min(sleep_s, 0.02))

    disable_ai(vision, "VISPOS_DURATION_COMPLETE")
    hold_current_position(test)
    print(
        f"[VISPOS] duration {a.duration:.1f}s complete -> hold current position; "
        "manual LAND/change mode from laptop"
    )


def wait_for_manual_landing(test: VisualPositionFlightTest):
    if test.local is None:
        raise FlightTestError("LOCAL_POSITION_NED unavailable for final hold")
    hold_n, hold_e, hold_d = map(float, test.local)
    print("[MANUAL LAND] visual control finished; absolute LOCAL_NED hold active")
    print("[MANUAL LAND] change mode / LAND from laptop; no software LAND here")

    next_tx = 0.0
    while True:
        test.poll_drain(0.05)
        if not test.heartbeat_ok():
            raise FlightTestError("heartbeat lost while waiting for manual landing")
        if test.mode != "GUIDED":
            print(f"[MANUAL LAND] FC mode changed to {test.mode}; releasing controller")
            return
        if not test.armed:
            print("[MANUAL LAND] vehicle DISARMED")
            return
        now = time.monotonic()
        if now >= next_tx:
            send_local_position_target(test, hold_n, hold_e, hold_d)
            next_tx = now + 0.10


def main() -> int:
    a = parse_args()
    validate(a)

    try:
        hfov_deg, vfov_deg, fov_source = load_fov(a)
    except FlightTestError as exc:
        print(f"[BLOCKED] {exc}")
        return 2

    cfg = bottom_config(float(a.confidence))
    detector_class_id = int(
        cfg.get("ai", {}).get("bottom", {}).get("detector_class_id", 0)
    )

    stop_event = threading.Event()
    state = SharedState()
    events = EventBus()
    waypoints = WaypointManager(state)
    gcs = (
        HeadlessLoopbackGcs(state, events, waypoints, stop_event, command_manager=None)
        if a.headless
        else GcsServer(state, events, waypoints, stop_event, command_manager=None)
    )
    vision = VisionStreamSession(cfg, state, stop_event)
    disable_ai(vision, "VISPOS_INIT")

    test = VisualPositionFlightTest()
    flight_open = False

    print("============================================================")
    print(" KINGPHOENIX VISUAL -> LOCAL_NED POSITION TEST")
    print("============================================================")
    print(
        f"[PLAN] LiDAR takeoff {a.takeoff_alt:.2f}m -> GUIDED visual position tracking "
        "-> manual LAND"
    )
    print(f"[CAMERA] HFOV={hfov_deg:.2f} VFOV={vfov_deg:.2f} deg | source={fov_source}")
    print("[CONTROL] NO fuzzy/PID/PD velocity control; position targets only")
    print("[HEADING] BLOCK RULE: Mission Planner heading must match real aircraft heading")
    print("[LAND] no software LAND in this runner")

    try:
        gcs.start()
        vision.start()
        if vision.shared_ai is None or not bool(getattr(vision.shared_ai, "ready", False)):
            raise FlightTestError("shared Hailo YOLO26 did not become READY")
        wait_streams(state, float(a.stream_wait))

        test.open()
        flight_open = True

        # Position-target tracking needs valid GPS/local state even though this
        # is not an AUTO mission.
        test.collect_preflight(require_auto=True)

        # Explicitly request/check yaw before arming.
        yaw_deadline = time.monotonic() + 2.0
        while time.monotonic() < yaw_deadline and not test.yaw_ok():
            test.poll_drain(0.05)
        if not test.yaw_ok():
            raise FlightTestError("ATTITUDE yaw unavailable/stale")

        yaw_deg = math.degrees(float(test.yaw_rad))
        print(
            f"[PREARM] GPS fix={test.gps_fix} sats={test.gps_sats} "
            f"| yaw={yaw_deg:+.1f}deg | LiDAR={float(test.lidar):.2f}m"
        )
        print(
            "[PREARM] Verify the Mission Planner HUD heading physically matches "
            "the aircraft nose BEFORE typing GO."
        )

        require_word(
            f"Props clear. Heading verified. Takeoff to {a.takeoff_alt:.2f}m and start VISPOS. Type GO: ",
            "GO",
        )

        profiled_lidar_takeoff(
            test,
            float(a.takeoff_alt),
            tolerance_m=float(a.tolerance),
            handoff_m=float(a.handoff),
            fast_rate_mps=float(a.takeoff_rate),
            near_rate_mps=float(a.near_rate),
            taper_zone_m=float(a.taper_zone),
            kick_alt_m=float(a.kick_alt),
            timeout_s=float(a.takeoff_timeout),
            ceiling_m=float(a.takeoff_ceiling),
            zero_flush_s=float(a.zero_flush),
        )

        run_visual_position_tracking(
            test,
            state,
            vision,
            a,
            detector_class_id,
            hfov_deg,
            vfov_deg,
        )
        wait_for_manual_landing(test)
        return 0

    except PilotTakeover as exc:
        print(f"[PILOT TAKEOVER] {exc}")
        try:
            disable_ai(vision, "PILOT_TAKEOVER")
        except Exception:
            pass
        return 130

    except KeyboardInterrupt:
        print("\n[STOP] operator interrupt; no software LAND commanded")
        try:
            if flight_open and test.mode == "GUIDED" and test.local is not None:
                hold_current_position(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "OPERATOR_INTERRUPT")
        except Exception:
            pass
        return 130

    except FlightTestError as exc:
        print(f"[BLOCKED/ABORT] {exc}")
        try:
            if flight_open and test.mode == "GUIDED" and test.local is not None:
                hold_current_position(test)
        except Exception:
            pass
        try:
            disable_ai(vision, "VISPOS_ABORT")
        except Exception:
            pass
        return 2

    finally:
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
