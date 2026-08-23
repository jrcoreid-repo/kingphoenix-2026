# Kingphoenix V2.1 - Raspberry Pi Health Integration

This checkpoint adds a low-overhead Raspberry Pi health layer without enabling
flight movement TX.

## Live metrics
- CPU temperature
- Raspberry Pi `get_throttled` raw/hex
- current and historical undervoltage
- current and historical frequency cap
- current and historical throttling
- current and historical soft temperature limit
- CPU load
- RAM usage
- disk free/usage
- uptime

## Default policies
- Temperature: PASS < 75 C, WARN 75-79.9 C, FAIL >= 80 C
- Current undervoltage: FAIL
- Historical undervoltage: WARN
- Current throttle/cap/temp-limit: FAIL
- Historical throttle/cap/temp-limit: WARN
- CPU/RAM >= 85% WARN, >= 95% FAIL
- Disk free <= 2 GB WARN, <= 0.5 GB FAIL

Thresholds are configurable in `config/system.json -> pi_health`.

## Data path
HealthWorker
-> SharedState.system.pi_health
-> video metadata
-> FINAL GCS
-> SYSTEM/RANGE panel + PRE-FLIGHT popup

## Bench test
From the Raspberry Pi project root:

```bash
python3 tools/pi_health_test.py
```

## Safety checkpoint
- Flight movement TX remains disabled.
- Mission controls remain UI-only.
- Servo TX remains disabled.
- No new MAVLink actuation is introduced here.
