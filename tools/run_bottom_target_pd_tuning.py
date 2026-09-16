#!/usr/bin/env python3
"""Kingphoenix BOTTOM-camera target alignment / cruise tuner.

Mount geometry (field-observed by operator)
--------------------------------------------
Canonical image pixel (0,0) is physically at DRONE FRONT-RIGHT.
Canonical image pixel (W,H) is physically at DRONE REAR-LEFT.
Therefore, with ArduPilot BODY_OFFSET_NED (+X forward, +Y right):

    image +X  -> drone LEFT   -> body Y error = -ex
    image +Y  -> drone REAR   -> body X error = -ey

So a target seen at front-right commands +VX/+VY; a target seen at
rear-left commands -VX/-VY.

Modes
-----
Default is SHADOW ONLY: bottom camera + YOLO26 class 0 + GCS stream, no MAVLink.
Add --flight to run validated LiDAR takeoff -> GUIDED bottom XY alignment -> LAND.
Yaw is hard OFF in this tool. Vertical velocity is hard OFF; LiDAR guard remains
active. Target loss sends horizontal ZERO immediately.

Do not run main.py, MAVProxy, another flight tool, or another camera/Hailo owner
at the same time.
"""

from __future__ import annotations

import argparse
import copy
import math
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from config import load_json
from core.event_bus import EventBus
from core.shared_state import SharedState
from mission.waypoint_manager import WaypointManager
from streaming.gcs_server import GcsServer
from streaming.vision_stream_session import VisionStreamSession
from tools.flight_test_common import DirectFlightTest, FlightTestError
from tools.mission_lidar_takeoff import profiled_lidar_takeoff
from tools.run_hybrid_mission_test import check_flight_health, check_lidar_guard


def clamp(v: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(v)))


def deadband_remap(v: float, db: float) -> float:
    v = clamp(v, -1.0, 1.0)
    db = clamp(abs(db), 0.0, 0.95)
    if abs(v) <= db:
        return 0.0
    s = 1.0 if v >= 0 else -1.0
    return s * (abs(v) - db) / max(1.0 - db, 1e-6)


def response_curve(v: float, mode: str, expo: float) -> float:
    v = clamp(v, -1.0, 1.0)
    if mode == "linear":
        return v
    a = abs(v)
    e = clamp(expo, 0.0, 1.0)
    shaped = (1.0 - e) * a + e * a**3
    return math.copysign(shaped, v)


def slew(current: float, target: float, rate_per_s: float, dt: float) -> float:
    dmax = max(0.0, rate_per_s) * max(0.0, dt)
    return current + clamp(target - current, -dmax, dmax)


def vector_cap(x: float, y: float, cap: float) -> tuple[float, float]:
    mag = math.hypot(x, y)
    if mag <= cap or mag <= 1e-9:
        return x, y
    s = cap / mag
    return x * s, y * s


def bottom_config(confidence: float) -> dict:
    cfg = copy.deepcopy(load_json("system.json"))
    ai = cfg.setdefault("ai", {})
    ai["enabled"] = True
    ai["engine"] = "YOLO26_ONLY"
    ai.setdefault("detector", {})["confidence_threshold"] = float(confidence)
    ai.setdefault("test_switch", {})["enabled"] = False
    ai.setdefault("route", {}).update({
        "camera": "bottom",
        "model_mode": "YOLO26_ONLY",
        "task": "DROP_ALIGNMENT",
        "detector_class_id": int(ai.get("bottom", {}).get("detector_class_id", 0)),
        "source": "BOTTOM_PD_TUNING",
    })
    cfg.setdefault("vision_control", {})["movement_tx_enabled"] = False
    cfg["vision_control"]["confidence_min"] = float(confidence)
    cfg.setdefault("run_mode", {})["profile"] = "VISION_AI_TEST"
    return cfg


def send_body_xy(test: DirectFlightTest, vx: float, vy: float):
    """BODY_OFFSET_NED: +VX forward, +VY right; VZ and yaw hard OFF."""
    test.master.mav.set_position_target_local_ned_send(
        0,
        test.master.target_system,
        test.master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        1479,
        0.0, 0.0, 0.0,
        float(vx), float(vy), 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0,
    )


def zero_xy(test: DirectFlightTest):
    send_body_xy(test, 0.0, 0.0)


def wait_bottom_stream(state: SharedState, timeout_s: float):
    print("[GCS] waiting for Windows GCS CONNECT ...")
    deadline = time.monotonic() + timeout_s
    seen = False
    while time.monotonic() < deadline:
        with state.lock:
            connected = bool(state.system.get("gcs_connected", False))
            bottom = dict(state.vision.get("bottom") or {})
        if connected and not seen:
            print("[GCS] CONNECTED - waiting for BOTTOM stream")
            seen = True
        if connected and str(bottom.get("video_state", "")).upper() == "LIVE":
            print(f"[GCS] BOTTOM LIVE | AI={bottom.get('ai_state')} mode={bottom.get('vision_mode')}")
            return
        time.sleep(0.1)
    raise FlightTestError("GCS/BOTTOM stream not ready before timeout")


def args_parser():
    ap = argparse.ArgumentParser(description="Bottom-camera body-frame PD alignment tuner")
    ap.add_argument("--flight", action="store_true", help="enable actual GUIDED XY movement; default is shadow only")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--control-hz", type=float, default=20.0)

    ap.add_argument("--kp-x", type=float, default=1.2, help="forward/back P gain, m/s per normalized body-X error")
    ap.add_argument("--kd-x", type=float, default=0.12)
    ap.add_argument("--kp-y", type=float, default=1.2, help="right/left P gain, m/s per normalized body-Y error")
    ap.add_argument("--kd-y", type=float, default=0.12)
    ap.add_argument("--xy-cap", type=float, default=0.8, help="total horizontal speed cap m/s")
    ap.add_argument("--xy-slew", type=float, default=2.5, help="per-axis command slew m/s^2")

    ap.add_argument("--deadband-x", type=float, default=0.04)
    ap.add_argument("--deadband-y", type=float, default=0.04)
    ap.add_argument("--filter-alpha", type=float, default=0.40)
    ap.add_argument("--d-alpha", type=float, default=0.25)
    ap.add_argument("--response", choices=["linear", "expo"], default="linear")
    ap.add_argument("--expo", type=float, default=0.35)
    ap.add_argument("--confidence", type=float, default=0.50)
    ap.add_argument("--lock-frames", type=int, default=3)
    ap.add_argument("--lost-timeout", type=float, default=0.15)

    ap.add_argument("--alt", type=float, default=1.20)
    ap.add_argument("--climb-rate", type=float, default=0.45)
    ap.add_argument("--near-rate", type=float, default=0.20)
    ap.add_argument("--taper-zone", type=float, default=0.25)
    ap.add_argument("--handoff", type=float, default=0.25)
    ap.add_argument("--tolerance", type=float, default=0.05)
    ap.add_argument("--kick-alt", type=float, default=3.0)
    ap.add_argument("--takeoff-ceiling", type=float, default=1.45)
    ap.add_argument("--zero-flush", type=float, default=0.15)
    ap.add_argument("--guard-floor", type=float, default=0.80)
    ap.add_argument("--guard-ceiling", type=float, default=1.45)
    ap.add_argument("--guard-outside-s", type=float, default=0.35)
    ap.add_argument("--lidar-stale-s", type=float, default=0.60)
    ap.add_argument("--gcs-wait", type=float, default=60.0)
    ap.add_argument("--yes", action="store_true")
    return ap.parse_args()


def validate(a):
    if not 1 <= a.duration <= 180: raise SystemExit("--duration must be 1..180s")
    if not 10 <= a.control_hz <= 30: raise SystemExit("--control-hz must be 10..30Hz")
    if not 0 <= a.kd_x <= 2 or not 0 <= a.kd_y <= 2: raise SystemExit("Kd must be 0..2")
    if not 0.05 <= a.kp_x <= 5 or not 0.05 <= a.kp_y <= 5: raise SystemExit("Kp must be 0.05..5")
    if not 0.05 <= a.xy_cap <= 2.5: raise SystemExit("--xy-cap must be 0.05..2.5m/s")
    if not 0.2 <= a.xy_slew <= 12: raise SystemExit("--xy-slew must be 0.2..12m/s^2")
    if not 0.01 <= a.deadband_x <= 0.20 or not 0.01 <= a.deadband_y <= 0.20: raise SystemExit("deadband must be 0.01..0.20")
    if not 0.05 <= a.filter_alpha <= 1 or not 0.05 <= a.d_alpha <= 1: raise SystemExit("filter alpha must be 0.05..1")
    if not 0 <= a.expo <= 1: raise SystemExit("--expo must be 0..1")
    if not 0.25 <= a.confidence <= 0.95: raise SystemExit("--confidence must be 0.25..0.95")
    if not 1 <= a.lock_frames <= 20: raise SystemExit("--lock-frames must be 1..20")
    if not 0 <= a.lost_timeout <= 1: raise SystemExit("--lost-timeout must be 0..1s")
    if not 0.50 <= a.alt <= 3.0: raise SystemExit("--alt must be 0.50..3.00m")
    if a.takeoff_ceiling <= a.alt: raise SystemExit("takeoff ceiling must be above target")
    if not 0.30 <= a.guard_floor < a.guard_ceiling <= 2.0: raise SystemExit("invalid altitude guard")


def main() -> int:
    a = args_parser(); validate(a)
    cfg = bottom_config(a.confidence)
    stop_event = threading.Event()
    state = SharedState(); events = EventBus(); waypoints = WaypointManager(state)
    gcs = GcsServer(state, events, waypoints, stop_event, command_manager=None)
    vision = VisionStreamSession(cfg, state, stop_event)
    vision.workers = [vision.bottom]

    test = DirectFlightTest(); flight_open = False

    print("============================================================")
    print(" KINGPHOENIX BOTTOM TARGET PD TUNING")
    print("============================================================")
    print(f"[MODE] {'FLIGHT' if a.flight else 'SHADOW ONLY'}")
    print("[MOUNT] pixel (0,0) = DRONE FRONT-RIGHT")
    print("[MOUNT] pixel (W,H) = DRONE REAR-LEFT")
    print("[MAP] body X error = -ey | + = FORWARD")
    print("[MAP] body Y error = -ex | + = RIGHT")
    print(f"[PD] X Kp={a.kp_x:.2f} Kd={a.kd_x:.2f} | Y Kp={a.kp_y:.2f} Kd={a.kd_y:.2f}")
    print(f"[LIMIT] XY vector cap={a.xy_cap:.2f}m/s slew={a.xy_slew:.2f}m/s^2")
    print("[HARD OFF] yaw=0 | vertical velocity=0")
    print("------------------------------------------------------------")

    try:
        gcs.start(); vision.start()
        if vision.shared_ai is None or not bool(getattr(vision.shared_ai, "ready", False)):
            raise FlightTestError("Hailo YOLO26 did not become READY")
        wait_bottom_stream(state, a.gcs_wait)

        if a.flight:
            test.open(); flight_open = True
            test.collect_preflight(require_auto=True)
            if not a.yes:
                ans = input("Type GO to arm/takeoff and enable BOTTOM XY control: ").strip().upper()
                if ans != "GO":
                    print("[CANCELLED] vehicle remains DISARMED"); return 0
            profiled_lidar_takeoff(
                test, a.alt, tolerance_m=a.tolerance, handoff_m=a.handoff,
                fast_rate_mps=a.climb_rate, near_rate_mps=a.near_rate,
                taper_zone_m=a.taper_zone, kick_alt_m=a.kick_alt,
                timeout_s=15.0, ceiling_m=a.takeoff_ceiling,
                zero_flush_s=a.zero_flush,
            )
            if test.mode != "GUIDED": test.set_mode("GUIDED")
            zero_xy(test)
            print("[BOTTOM] FLIGHT ACTIVE - move target under drone")
        else:
            print("[BOTTOM] SHADOW ACTIVE - move target under camera; no MAVLink opened")

        end = time.monotonic() + a.duration
        period = 1.0 / a.control_hz
        next_tx = 0.0; last_print = 0.0
        filt_bx = filt_by = None
        dfx = dfy = 0.0
        prev_bx = prev_by = None; prev_t = None; last_frame = None
        valid_streak = 0; lost_since = None
        vx_cmd = vy_cmd = 0.0
        outside_since = lidar_loss_since = None

        while time.monotonic() < end:
            if a.flight:
                test.poll(0.02); check_flight_health(test)
                if test.mode != "GUIDED": raise FlightTestError(f"mode changed: {test.mode}")
                outside_since, lidar_loss_since = check_lidar_guard(
                    test, floor_m=a.guard_floor, ceiling_m=a.guard_ceiling,
                    outside_since=outside_since, lidar_loss_since=lidar_loss_since,
                    max_outside_s=a.guard_outside_s, max_stale_s=a.lidar_stale_s,
                )

            now = time.monotonic()
            with state.lock:
                b = dict(state.vision.get("bottom") or {})
                connected = bool(state.system.get("gcs_connected", False))
            if not connected:
                if a.flight: zero_xy(test)
                raise FlightTestError("GCS disconnected")

            conf = b.get("confidence"); ex = b.get("ex"); ey = b.get("ey")
            frame_id = b.get("frame_id")
            ts_ns = int(b.get("frame_timestamp_ns") or 0)
            age = float("inf") if ts_ns <= 0 else max(0.0, (time.monotonic_ns()-ts_ns)/1e9)
            valid = bool(b.get("vision_valid") and conf is not None and float(conf)>=a.confidence and ex is not None and ey is not None and age<=0.20)

            if valid and frame_id != last_frame:
                last_frame = frame_id; valid_streak += 1; lost_since = None
                # Physical mount transform: image -> BODY error.
                bx = clamp(-float(ey), -1, 1)   # + forward
                by = clamp(-float(ex), -1, 1)   # + right
                if filt_bx is None:
                    filt_bx, filt_by = bx, by
                else:
                    al=a.filter_alpha; filt_bx=al*bx+(1-al)*filt_bx; filt_by=al*by+(1-al)*filt_by
                ebx=response_curve(deadband_remap(filt_bx,a.deadband_y),a.response,a.expo)
                eby=response_curve(deadband_remap(filt_by,a.deadband_x),a.response,a.expo)

                if prev_t is not None and prev_bx is not None:
                    dt=max(1e-3,now-prev_t)
                    raw_dx=(ebx-prev_bx)/dt; raw_dy=(eby-prev_by)/dt
                    da=a.d_alpha; dfx=da*raw_dx+(1-da)*dfx; dfy=da*raw_dy+(1-da)*dfy
                prev_bx,prev_by,prev_t=ebx,eby,now

                if valid_streak>=a.lock_frames:
                    tx=a.kp_x*ebx+a.kd_x*dfx; ty=a.kp_y*eby+a.kd_y*dfy
                    tx,ty=vector_cap(tx,ty,a.xy_cap)
                    dt_cmd=period
                    vx_cmd=slew(vx_cmd,tx,a.xy_slew,dt_cmd)
                    vy_cmd=slew(vy_cmd,ty,a.xy_slew,dt_cmd)
                else:
                    vx_cmd=vy_cmd=0.0
            elif not valid:
                valid_streak=0; lost_since=lost_since or now
                vx_cmd=vy_cmd=0.0
                if now-lost_since>=a.lost_timeout:
                    filt_bx=filt_by=None; prev_bx=prev_by=prev_t=None; dfx=dfy=0.0

            if a.flight and now>=next_tx:
                send_body_xy(test,vx_cmd,vy_cmd); next_tx=now+period

            if now-last_print>=0.25:
                quad="SEARCH"
                if valid:
                    fb="FRONT" if (-float(ey))>a.deadband_y else ("REAR" if (-float(ey))<-a.deadband_y else "CENTER")
                    lr="RIGHT" if (-float(ex))>a.deadband_x else ("LEFT" if (-float(ex))<-a.deadband_x else "CENTER")
                    quad=f"{fb}-{lr}"
                print(f"[BOTTOM] {quad:13s} conf={'--' if conf is None else f'{float(conf):.2f}'} ex={'--' if ex is None else f'{float(ex):+.3f}'} ey={'--' if ey is None else f'{float(ey):+.3f}'} | BODY VX={vx_cmd:+.2f} VY={vy_cmd:+.2f} | {'TX' if a.flight else 'SHADOW'}")
                last_print=now
            time.sleep(0.005)

        if a.flight:
            zero_xy(test); print("[BOTTOM] duration complete -> ZERO -> LAND")
            test.land_and_wait()
        print("[RESULT] BOTTOM TARGET TEST COMPLETE")
        return 0

    except KeyboardInterrupt:
        print("\n[STOP] operator interrupt")
        if flight_open:
            try: zero_xy(test)
            except Exception: pass
            test.land_best_effort("operator interrupt")
        return 130
    except FlightTestError as exc:
        print(f"[BLOCKED/ABORT] {exc}")
        if flight_open:
            try: zero_xy(test)
            except Exception: pass
            test.land_best_effort(str(exc))
        return 2
    finally:
        stop_event.set()
        if flight_open: test.close()
        try:
            if gcs.is_alive(): gcs.join(timeout=1.0)
        except Exception: pass
        try: vision.join(timeout=1.0)
        except Exception: pass


if __name__ == "__main__":
    raise SystemExit(main())
