#!/usr/bin/env python3
from __future__ import annotations

import signal
import sys
import time

from core.runtime import KingphoenixRuntime


def _fmt_ready(runtime):
    with runtime.state.lock:
        readiness = dict(runtime.state.system.get("readiness") or {})
    failed = list(readiness.get("failed_critical") or [])
    if readiness.get("ready_for_mission"):
        return "READY"
    return "NOT READY" + (f" | failed={','.join(failed)}" if failed else "")


def _wait_operator_start(runtime):
    print("")
    print("============================================================")
    print(" KINGPHOENIX 2026 - TERMINAL RACE")
    print(" FC mission = authority | GCS = DISABLED")
    print("============================================================")
    print("[RACE] Mission Planner must already WRITE mission to FC.")
    print("[RACE] Expected sequence: START=3 | AI=8 | DROP=9 | RESUME=13 | LAND=17")
    print("[RACE] Waiting for critical hardware...")
    print("")

    last = None
    while not runtime.stop_event.is_set():
        status = _fmt_ready(runtime)
        if status != last:
            print(f"[RACE] {status}")
            last = status
        if status == "READY":
            break
        time.sleep(0.5)

    if runtime.stop_event.is_set():
        return False

    try:
        actual = runtime.fc_sync.download()
    except Exception as exc:
        print(f"[RACE] FC mission READ FAILED: {exc}")
        return False

    mission_cfg = runtime.full_cfg.get("mission", {})
    required = {
        "START": int(mission_cfg.get("start_seq", 3)),
        "AI": int(mission_cfg.get("vision_enable_seq", 8)),
        "DROP": int(mission_cfg.get("drop_seq", 9)),
        "RESUME": int(mission_cfg.get("resume_seq", 13)),
        "LAND": int(mission_cfg.get("land_seq", 17)),
    }
    count = len(actual)
    missing = [f"{name}=SEQ{seq}" for name, seq in required.items() if seq < 0 or seq >= count]
    if missing:
        print(f"[RACE] BLOCKED: FC mission count={count}; missing {', '.join(missing)}")
        return False

    print(f"[RACE] FC mission loaded | items={count}")
    print(
        "[RACE] "
        + " | ".join(f"{name}=SEQ{seq}" for name, seq in required.items())
    )
    print("[RACE] Vision success -> servo release -> skip fallback -> SEQ13")
    print("[RACE] Vision fail -> FC keeps SEQ10-12 DELAY/DO_SET_SERVO fallback")
    print("")
    print("Press ENTER to START race. Type q then ENTER to quit.")

    answer = input("> ").strip().lower()
    if answer in {"q", "quit", "exit"}:
        return False
    return True


def main():
    runtime = KingphoenixRuntime(enable_gcs=False)

    def shutdown(*_):
        runtime.stop()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        runtime.start()

        if not _wait_operator_start(runtime):
            return 1

        print("[RACE] START REQUESTED")
        runtime.events.publish("START_MISSION", {"strategy": "FC"})
        runtime.wait()
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        runtime.stop()


if __name__ == "__main__":
    sys.exit(main())
