#!/usr/bin/env python3
"""Kingphoenix 2026 full-race Raspberry Pi entrypoint."""

from __future__ import annotations


def main():
    runtime = None
    try:
        # Keep production imports inside the guarded startup path so a missing
        # Python dependency produces one clean operator message, not a traceback.
        from core.runtime import KingphoenixRuntime

        runtime = KingphoenixRuntime()
    except Exception as exc:
        print("============================================================")
        print(" KINGPHOENIX 2026 - STARTUP BLOCKED")
        print("============================================================")
        print(f"[STARTUP] FAIL {type(exc).__name__}: {exc}")
        print("[START] BLOCKED - fix dependency/config/mission files before running")
        return 2

    try:
        runtime.start()
        runtime.wait()
        return 0
    except KeyboardInterrupt:
        print("\n[STOP] operator interrupted")
        return 130
    except Exception as exc:
        print(f"[ERROR] RUNTIME {type(exc).__name__}: {exc}")
        return 3
    finally:
        runtime.stop()


if __name__ == "__main__":
    raise SystemExit(main())
