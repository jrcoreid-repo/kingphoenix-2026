#!/usr/bin/env python3
import json
import time

from system.health import PiHealthMonitor


def main():
    monitor = PiHealthMonitor()
    print("=== KINGPHOENIX PI HEALTH BENCH TEST ===")
    print("Ctrl+C to stop\n")

    try:
        while True:
            health = monitor.sample()
            print(json.dumps(health, indent=2))
            print("-" * 72)
            time.sleep(2.0)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
