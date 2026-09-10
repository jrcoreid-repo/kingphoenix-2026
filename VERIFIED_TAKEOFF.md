# Verified immediate takeoff

This change applies to `CommandManager.takeoff()` used by the standalone
`tools.flight_command_test` tool. GCS takeoff buttons are not enabled by this change.

## Behavior

- Require a fresh heartbeat and a finite, fresh GLOBAL_POSITION_INT relative
  altitude before starting. The target must be at least 0.2 m above current altitude.
- Verify GUIDED, then ARM using an accepted ACK and an armed heartbeat received
  after ARM transmission. If already armed, require the current heartbeat to be fresh.
- Dispatch TAKEOFF immediately after verified ARM, with no fixed post-arm sleep.
  This is command timing, not a promise of immediate physical lift-off. FC motor
  spool, arming checks and climb-rate parameters remain in control.
- Correlate ACK command ID, source system/component and receive timestamp.
  IN_PROGRESS waits for a final ACK under a bounded timeout; it is not success.
- Verify TAKEOFF with accepted ACK plus three distinct fresh GLOBAL_POSITION_INT
  samples at the target (default tolerance is +/- min(0.15 m, 20% of target)).
  Samples must follow TAKEOFF transmission and show at least 0.2 m climb from
  the pre-send baseline. LiDAR ground clearance cannot substitute for altitude
  relative to home. Target attainment is not a hover-stability assessment.
- Stop verification if the link becomes stale/disconnected, GUIDED is lost,
  or the vehicle disarms. This does not cancel a command already on the FC.
- Never automatically resend an ACK-required command after successful TX.
  An ACK/state timeout reports UNCONFIRMED even if the drone may be moving.
  Do not blindly retry; inspect telemetry and control the vehicle via RC/FC.
  No automatic DISARM is sent on an uncertain flight result.

ARM waits up to 8 seconds for state, or up to 10 seconds from the first
IN_PROGRESS for completion. These are maximum waits, not inserted delays.
Takeoff observation retains the flight tool's configured timeout (default 10 s).

## Invocation after selecting this branch

Run from the repository root in the existing Pi environment:

```bash
python3 -m tools.flight_command_test takeoff --altitude 1.2 --confirm-flight-area-clear
```

This performs GUIDED + ARM + TAKEOFF; a separate ARM invocation is unnecessary.
The confirmation flag is only for a physically inspected, clear flight area.
Do not run another runtime/flight tool against the same FC serial device, and
use one command source during the test (no simultaneous MP/GCS command input).
This tool still starts streaming and stops its runtime when the test returns;
it does not provide a persistent hover/landing console.

## Validation

Run `python3 -m unittest discover -s tests -v`.
Tests use a deterministic clock, real VehicleState ACK histories and simulated
FC telemetry/command responses. No motors, serial port or cameras are opened.
Covered: immediate dispatch after armed confirmation, slow ARM, IN_PROGRESS,
rejection, lost ACKs, ACK without state change, no climb, stale/frozen altitude,
wrong-source ACK, LiDAR/home mismatch, mode override, disarm, link loss,
overshoot, and non-finite/out-of-range targets.

Not yet validated with ArduPilot SITL, the actual FC firmware, or physical flight.
MAVLink ACKs have no unique transaction ID: use one command authority; timestamps
cannot distinguish every delayed ACK from an earlier invocation of the same command.

Protocol reference: https://mavlink.io/en/services/command.html
