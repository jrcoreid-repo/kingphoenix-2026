# Kingphoenix Development Workflow

## Active branch policy

- `dev` is the active development and field-test branch.
- Raspberry Pi runtime, Windows GCS, flight-test tools, waypoint capture, vision, and mission development must be validated on `dev` first.
- `main` is treated as a stable/release branch and should not receive experimental field changes directly.
- Promote `dev` changes to `main` only after bench/field validation.

## Operator update

Raspberry Pi and Windows development machines should use:

```bash
git checkout dev
git pull origin dev
```

## Field-test ownership

Standalone flight tools own the FC MAVLink serial stream while running. Do not run `main.py`, MAVProxy, or another process consuming the same serial link at the same time.

Current guarded flight tests:

```bash
python3 tools/run_takeoff_test.py --alt 1.2
python3 tools/run_waypoint_auto.py --strategy LEFT --wp 1 --alt 1.2
python3 tools/run_waypoint_auto.py --strategy LEFT --wp 1-2 --alt 1.2
```

Waypoint AUTO altitude follows `config/flight_defaults.json` / saved DEV waypoint `alt_m` (nominally 1.2 m). Takeoff completion is LiDAR-confirmed and the physical takeoff command uses the configured test compensation (default +15%). Successful tests end in LAND and DISARM.
