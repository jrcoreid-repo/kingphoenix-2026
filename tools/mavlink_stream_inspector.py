#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
from collections import defaultdict, deque

from config import load_json

WATCH = (
    "HEARTBEAT",
    "GPS_RAW_INT",
    "GLOBAL_POSITION_INT",
    "DISTANCE_SENSOR",
    "RANGEFINDER",
    "MISSION_CURRENT",
    "SYS_STATUS",
)

MESSAGE_IDS = {
    "GPS_RAW_INT": 24,
    "GLOBAL_POSITION_INT": 33,
    "SYS_STATUS": 1,
    "MISSION_CURRENT": 42,
    "DISTANCE_SENSOR": 132,
    "RANGEFINDER": 173,
}

DEFAULT_RATES = {
    "GPS_RAW_INT": 10.0,
    "GLOBAL_POSITION_INT": 20.0,
    "SYS_STATUS": 5.0,
    "MISSION_CURRENT": 2.0,
    "DISTANCE_SENSOR": 20.0,
}


def fmt(v, digits=2):
    if v is None:
        return "--"
    try:
        return f"{float(v):.{digits}f}"
    except Exception:
        return str(v)


def describe(msg):
    t = msg.get_type()
    if t == "HEARTBEAT":
        armed = bool(int(getattr(msg, "base_mode", 0)) & 128)
        return f"armed={armed} base_mode={getattr(msg,'base_mode',None)} custom_mode={getattr(msg,'custom_mode',None)}"
    if t == "GPS_RAW_INT":
        lat = getattr(msg, "lat", None)
        lon = getattr(msg, "lon", None)
        alt = getattr(msg, "alt", None)
        eph = getattr(msg, "eph", None)
        return (
            f"fix={getattr(msg,'fix_type',None)} sats={getattr(msg,'satellites_visible',None)} "
            f"lat={fmt(None if lat is None else lat/1e7,7)} "
            f"lon={fmt(None if lon is None else lon/1e7,7)} "
            f"alt={fmt(None if alt is None else alt/1000.0)}m "
            f"eph={fmt(None if eph in (None,65535) else eph/100.0)}"
        )
    if t == "GLOBAL_POSITION_INT":
        return (
            f"lat={fmt(getattr(msg,'lat',0)/1e7,7)} "
            f"lon={fmt(getattr(msg,'lon',0)/1e7,7)} "
            f"alt={fmt(getattr(msg,'alt',0)/1000.0)}m "
            f"rel_alt={fmt(getattr(msg,'relative_alt',0)/1000.0)}m"
        )
    if t == "DISTANCE_SENSOR":
        cm = getattr(msg, "current_distance", None)
        return (
            f"dist={fmt(None if cm is None else cm/100.0)}m "
            f"min={fmt(getattr(msg,'min_distance',0)/100.0)}m "
            f"max={fmt(getattr(msg,'max_distance',0)/100.0)}m "
            f"orientation={getattr(msg,'orientation',None)} id={getattr(msg,'id',None)}"
        )
    if t == "RANGEFINDER":
        return f"distance={fmt(getattr(msg,'distance',None))}m voltage={fmt(getattr(msg,'voltage',None))}"
    if t == "MISSION_CURRENT":
        return f"seq={getattr(msg,'seq',None)}"
    if t == "SYS_STATUS":
        v = getattr(msg, "voltage_battery", None)
        a = getattr(msg, "current_battery", None)
        rem = getattr(msg, "battery_remaining", None)
        return (
            f"V={fmt(None if v in (None,0,65535) else v/1000.0)} "
            f"A={fmt(None if a is None or int(a)<0 else a/100.0)} rem={rem}%"
        )
    return ""


def request_rate(master, name, hz):
    mid = MESSAGE_IDS.get(name)
    if mid is None:
        return
    interval_us = int(round(1_000_000.0 / float(hz)))
    master.mav.command_long_send(
        master.target_system or 1,
        master.target_component or 1,
        511,
        0,
        float(mid),
        float(interval_us),
        0,0,0,0,0,
    )
    print(f"[REQ] {name:<22} target={hz:5.1f} Hz")


def main():
    parser = argparse.ArgumentParser(description="Direct Kingphoenix MAVLink stream inspector")
    parser.add_argument("--seconds", type=float, default=20.0, help="inspection duration")
    parser.add_argument("--no-request", action="store_true", help="observe only; do not request message rates")
    args = parser.parse_args()

    cfg = load_json("system.json")["mavlink"]
    device = cfg.get("connection", "/dev/ttyAMA0")
    baud = int(cfg.get("baud", 921600))

    from pymavlink import mavutil

    print("============================================================")
    print(" KINGPHOENIX MAVLINK STREAM INSPECTOR")
    print("============================================================")
    print("IMPORTANT: stop main_v2_2.py / MAVProxy first; this tool owns the serial port.")
    print(f"[OPEN] {device} @ {baud}")

    master = mavutil.mavlink_connection(
        device,
        baud=baud,
        source_system=int(cfg.get("source_system", 191)),
        source_component=int(cfg.get("source_component", 191)),
        autoreconnect=True,
    )

    print("[WAIT] heartbeat...")
    hb = master.wait_heartbeat(timeout=8)
    if hb is None:
        raise SystemExit("[FAIL] no heartbeat within 8s")
    print(f"[PASS] heartbeat from {master.target_system}:{master.target_component}")

    if not args.no_request:
        rates = dict(DEFAULT_RATES)
        configured = cfg.get("message_rates_hz", {})
        for k in list(rates):
            if k in configured:
                rates[k] = float(configured[k])
        for name, hz in rates.items():
            request_rate(master, name, hz)
            time.sleep(0.04)

    times = defaultdict(lambda: deque(maxlen=512))
    latest = {}
    started = time.monotonic()
    next_report = started + 1.0

    while time.monotonic() - started < args.seconds:
        msg = master.recv_match(blocking=True, timeout=0.25)
        now = time.monotonic()
        if msg is not None and msg.get_type() != "BAD_DATA":
            t = msg.get_type()
            times[t].append(now)
            if t in WATCH:
                latest[t] = msg

        if now >= next_report:
            print("\n[MAVINSPECT]")
            for name in WATCH:
                q = times.get(name)
                hz = 0.0
                age = None
                if q:
                    age = now - q[-1]
                    if len(q) >= 2:
                        span = q[-1] - q[0]
                        if span > 0:
                            hz = (len(q)-1)/span
                if not q:
                    print(f"{name:<22} MISSING")
                else:
                    detail = describe(latest.get(name)) if latest.get(name) is not None else ""
                    print(f"{name:<22} {hz:6.2f} Hz age={age:5.2f}s | {detail}")
            next_report = now + 1.0

    print("\n=== FINAL SUMMARY ===")
    now = time.monotonic()
    for name in WATCH:
        q = times.get(name)
        if not q:
            print(f"{name:<22} MISSING")
            continue
        hz = 0.0
        if len(q) >= 2 and q[-1] > q[0]:
            hz = (len(q)-1)/(q[-1]-q[0])
        print(f"{name:<22} {hz:6.2f} Hz last_age={now-q[-1]:.2f}s | {describe(latest.get(name))}")

    try:
        master.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
