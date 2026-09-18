#!/usr/bin/env python3
"""Kingphoenix 2026 full-race Raspberry Pi entrypoint."""

from __future__ import annotations

from core.runtime import KingphoenixRuntime


def main():
    runtime = KingphoenixRuntime()
    try:
        runtime.start()
        runtime.wait()
    except KeyboardInterrupt:
        pass
    finally:
        runtime.stop()


if __name__ == "__main__":
    main()
