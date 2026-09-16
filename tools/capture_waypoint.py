#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.flight_test_common import load_json


def send_request(sock, address, payload, timeout=2.0):
    tx_id = f"wp-{time.monotonic_ns()}"
    message = dict(payload)
    message["tx_id"] = tx_id
    sock.sendto(json.dumps(message).encode("utf-8"), address)

    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        remaining = max(0.05, deadline - time.monotonic())
        sock.settimeout(remaining)
        data, _ = sock.recvfrom(65535)
        response = json.loads(data.decode("utf-8"))
        if response.get("tx_id") != tx_id:
            continue
        return response
    raise TimeoutError(f"no ACK for {payload.get('type')}")


def require_ok(response, action):
    if not bool(response.get("ok", False)):
        raise RuntimeError(f"{action} failed: {response.get('message', 'unknown error')}")
    return str(response.get("message", "OK"))


def main():
    ap = argparse.ArgumentParser(
        description="Capture current FC global position into config/waypoints.json through a running main.py"
    )
    ap.add_argument("--strategy", choices=["LEFT", "RIGHT"], required=True)
    ap.add_argument("--wp", type=int, required=True, choices=range(1, 13))
    ap.add_argument("--host", default="127.0.0.1", help="main.py control host; default localhost")
    args = ap.parse_args()

    cfg = load_json("system.json")
    port = int(cfg["stream"]["control_port"])
    address = (args.host, port)

    print("============================================================")
    print(" KINGPHOENIX WAYPOINT CAPTURE")
    print("============================================================")
    print(f"Target runtime : {args.host}:{port}")
    print(f"Waypoint       : {args.strategy}/WP{args.wp}")
    print("Source         : FC GLOBAL_POSITION_INT lat/lon")
    print("Requirement    : python3 main.py must already be running")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    locked = True
    try:
        response = send_request(sock, address, {"type": "UNLOCK_EDIT"})
        print("[EDIT]", require_ok(response, "UNLOCK_EDIT"))
        locked = False

        response = send_request(
            sock,
            address,
            {
                "type": "SAVE_WAYPOINT",
                "strategy": args.strategy,
                "wp": int(args.wp),
            },
        )
        message = require_ok(response, "SAVE_WAYPOINT")
        print("[SAVE]", message)

        response = send_request(sock, address, {"type": "LOCK_EDIT"})
        print("[EDIT]", require_ok(response, "LOCK_EDIT"))
        locked = True

        print("[RESULT] CAPTURE PASS")
        return 0
    except Exception as exc:
        print(f"[RESULT] CAPTURE FAILED: {type(exc).__name__}: {exc}")
        return 2
    finally:
        if not locked:
            try:
                response = send_request(sock, address, {"type": "LOCK_EDIT"}, timeout=0.8)
                if response.get("ok"):
                    print("[EDIT] emergency relock OK")
            except Exception:
                print("[EDIT] WARNING: emergency relock ACK not received")
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
