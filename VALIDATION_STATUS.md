# Kingphoenix V1.7.1 Validation Status

## Hardware validated

- MAVLink link / identity `1:191 -> 1:1`
- Telemetry RX foundation
- Mode VCT round trip:
  - `STABILIZE -> GUIDED -> STABILIZE`
  - one-attempt success
  - state verification from central `VehicleState`
- ARM expected-rejection path:
  - `COMMAND_ACK` correlated to FC `1:1`
  - terminal rejection classified as `REJECTED`
  - no blind retry
  - vehicle remained disarmed
  - ArduPilot `STATUSTEXT` reasons captured through the dedicated RX thread

## Implemented but not yet hardware validated

- Successful ARM path
- DISARM after successful ARM
- TAKEOFF physical verification
- LAND physical verification
- AUTO mission progression
- AUTO <-> GUIDED resume
- Guided realtime velocity/position streaming
- Rangefinder hardware telemetry
- Full mission integration
- Vision/movement integration

## Safety architecture retained

- `mavlink_rx.py` remains the only `recv_match()` owner
- mission / control / tools consume central `VehicleState`
- terminal FC rejection is not blindly retried
- LAND retains priority over mission commands
- no vision module directly owns MAVLink TX
