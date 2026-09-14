# LiDAR takeoff braking — feat/auto-waypoint-test

`CommandManager.takeoff()` now provides one implementation for
`runtestwp.py`, `python -m tools.flight_command_test takeoff`, and TAKEOFF steps
in `mission/step_executor.py`. Low-level MAVLink TX only sends commands; it is
not a verified flight API. GCS takeoff buttons remain disabled.

## Meaning of the height

`takeoff(1)` requests approximately 1 m **vertical sensor-to-ground distance**
measured by the downward LiDAR. It is not 1 m of additional rise from the launch
position, and no landing-gear/sensor mounting offset is added. The initial target
sent to the FC is translated between references:

```
FC relative target = current FC relative altitude
                   + (requested LiDAR height - current LiDAR height) * 1.05
```

Example: sensor starts 0.10 m above ground, FC relative altitude is 0 m,
and requested LiDAR height is 1 m: initial FC target is 0.945 m relative to home.
The 5% margin applies to the remaining rise. It is not the success threshold.
A barometer error larger than the margin can still cause under-climb and timeout;
this tool does not keep raising the target or bypass FC arming/navigation checks.

## Sequence and completion

1. Require fresh heartbeat, GPS 3D fix, attitude and downward `DISTANCE_SENSOR`.
   LiDAR must have a known min/max range and sensor ID. Target is 0.5–3 m and
   at least 0.2 m above the current LiDAR height. A fresh finite relative altitude
   is needed to translate the initial FC target, before ARM and again before TX.
2. Verify GUIDED via a new heartbeat. Verify ARM using accepted command ACK and
   a post-command armed heartbeat. Monitor LiDAR, GPS and mode during ARM.
   Send TAKEOFF immediately after verified ARM, with no fixed motor delay.
3. Independently of whether the TAKEOFF ACK has arrived, the first fresh LiDAR
   sample at/above target requests **BRAKE mode**. This replaces the GUIDED climb
   with FC braking/holding; merely setting a Python DONE flag would not stop it.
4. Success requires accepted TAKEOFF ACK, a post-request BRAKE heartbeat, and at
   least five independent LiDAR samples spanning >=0.5 s within **±0.10 m** of target,
   with successive LiDAR vertical-speed estimates <=0.15 m/s. Repeated reads of
   one packet never count as multiple samples. A one-sample spike cannot prove DONE.
5. Takeoff-only leaves the aircraft armed in BRAKE. The waypoint runner checks
   the same sensor and height again during mission-index selection and before
   requesting AUTO, then lets the FC execute waypoints and LAND.

LiDAR age limit is 0.25 s. The test fixture emits LiDAR at 10 Hz; the configured
MAVLink request is 20 Hz, which does not prove actual sensor update frequency.
Legacy RANGEFINDER is refused because it does not identify orientation/sensor.
Explicit invalid quality (1) is refused; MAVLink unknown quality (0/absent) is
allowed. Nonfinite values, missing limits, out-of-range values, a changed sensor
ID, stale attitude, or roll/pitch beyond 20 degrees fail verification.
Height uses `distance * cos(roll) * cos(pitch)`, assuming approximately level
terrain and a downward-mounted sensor. Obstacles/sloped ground change the reading.

## Failure behavior and limits

On timeout, lost/invalid LiDAR, lost ACK, exception or Ctrl+C after takeoff TX,
attempt a verified BRAKE if still armed in GUIDED/BRAKE with a fresh FC link.
Preserve pilot/FC changes to other modes. No automatic DISARM, climb retry or
barometer fallback is issued. If BRAKE cannot be confirmed, the FC command may
remain active. An overshoot can stop outside tolerance: that reports NOT DONE,
and the tool does not automatically descend to correct it.

Braking has latency and stopping distance; this is **not a promise of stopping
at exactly 1.00 m**. BRAKE maintains position using the FC estimator, not a
continuous Raspberry Pi LiDAR altitude servo. The takeoff-only process stops
its runtime/stream after reporting; FC BRAKE remains active. To take over,
**switch flight mode on the RC**: BRAKE ignores pilot stick inputs until mode changes.
Waypoint cruise heights remain `alt_m` relative to FC home, typically 1.2 m;
this change does not add LiDAR terrain following to AUTO.

The 0.8 `takeoff_verify_ratio` argument remains accepted for compatibility but no
longer sets completion tolerance. LiDAR completion uses the fixed limits above.

## Invocation

Run from the repo root in the Pi environment, with no other process owning FC UART:

```bash
python3 -m tools.flight_command_test takeoff --altitude 1 --confirm-flight-area-clear
python3 runtestwp.py --strat r --goto 2-2
# Flight execution after SITL validation and physical preflight:
python3 runtestwp.py --strat r --goto 2-2 --execute --confirm-flight-area-clear
```

WP runner takeoff height follows the first selected waypoint (default 1.2 m),
now interpreted as a LiDAR takeoff target. The waypoint itself remains home-relative.
Both flight tools reserve GCS command ownership; ABORT_MISSION is supported,
Mission Planner gateway uses RACE policy, and streaming follows runtime config.

## Validation

46 deterministic unit tests pass, including real VCT/CommandManager execution
against simulated FC telemetry and the complete takeoff→BRAKE→AUTO→landing sequence.
Coverage includes delayed/rejected/missing ACK, immediate dispatch after ARM,
wrong barometer height, LiDAR stale/frozen/invalid/overshoot, BRAKE unconfirmed,
sensor identity changes, mode override and LiDAR loss during AUTO handoff.

**Not yet tested in ArduPilot SITL, the installed FC firmware, or physical flight.**
Firmware/SITL validation must establish BRAKE stopping distance at the configured
climb speed, sensor rate, timeout behavior and mode takeover before flight use.

References: [ArduPilot BRAKE](https://ardupilot.org/copter/docs/brake-mode.html),
[BRAKE controller source](https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode_brake.cpp),
[MAVLink distance sensor](https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR).
