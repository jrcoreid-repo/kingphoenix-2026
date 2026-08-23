#!/usr/bin/env python3
"""Kingphoenix no-hardware foundation integration self-check."""

from __future__ import annotations

from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.shared_state import SharedState
from core.event_bus import EventBus
from flight.vehicle_state import VehicleState
from flight.freshness_policy import FreshnessPolicy
from flight.vct import VerifiedCommandTransaction
from flight.command_manager import CommandManager
from flight.mavlink_stream_config import MavlinkStreamConfigurator


def check(name, condition):
    if not condition:
        raise AssertionError(name)
    print(f"[PASS] {name}")


class FakeTx:
    target_system = 1
    target_component = 1

    def __init__(self, state):
        self.state = state
        self.mode_sends = 0

    def send_mode(self, mode):
        self.mode_sends += 1
        self.state.vehicle.mode = str(mode).upper()
        return True

    def set_message_interval(self, message_id, rate_hz):
        return True


class DummyStop:
    def is_set(self):
        return False


def main():
    print("======================================================")
    print(" KINGPHOENIX V1.7.1 FOUNDATION SELF-CHECK")
    print("======================================================")

    check("VCT.run callable", callable(getattr(VerifiedCommandTransaction, "run", None)))
    check("FreshnessPolicy.require callable", callable(getattr(FreshnessPolicy, "require", None)))
    check("VehicleState.message_status callable", callable(getattr(VehicleState, "message_status", None)))
    check("VehicleState.find_command_ack callable", callable(getattr(VehicleState, "find_command_ack", None)))
    check("VehicleState.record_status_text callable", callable(getattr(VehicleState, "record_status_text", None)))
    check("VehicleState.recent_status_texts callable", callable(getattr(VehicleState, "recent_status_texts", None)))

    # Freshness behavior
    v = VehicleState()
    policy = FreshnessPolicy(v, {"HEARTBEAT": 0.2})
    check("heartbeat initially missing", not policy.check("HEARTBEAT").ok)
    now = time.monotonic_ns()
    v.mark_message("HEARTBEAT", now)
    v.last_heartbeat_ns = now
    check("heartbeat becomes fresh", policy.check("HEARTBEAT").ok)

    # ACK source/time correlation
    t0 = time.monotonic_ns()
    v.record_command_ack(400, 0, 1, 1, timestamp_ns=t0 + 10)
    check(
        "ACK source/time correlation",
        v.find_command_ack(400, t0, 1, 1) is not None
        and v.find_command_ack(400, t0, 2, 1) is None,
    )

    # STATUSTEXT source/time filtering
    v.record_status_text("PreArm: test reason", 4, 1, 1, timestamp_ns=t0 + 20)
    v.record_status_text("other component", 6, 1, 191, timestamp_ns=t0 + 30)
    texts = v.recent_status_texts(t0, 1, 1)
    check(
        "STATUSTEXT capture/filter",
        len(texts) == 1 and texts[0].text == "PreArm: test reason",
    )

    # Rangefinder message IDs
    dummy_state = SharedState()
    sc = MavlinkStreamConfigurator(
        FakeTx(dummy_state),
        dummy_state,
        DummyStop(),
        {"DISTANCE_SENSOR": 20, "RANGEFINDER": 10},
    )
    check("DISTANCE_SENSOR id=132", sc._message_id("DISTANCE_SENSOR") == 132)
    check("RANGEFINDER id=173", sc._message_id("RANGEFINDER") == 173)

    # CommandManager -> VCT -> TX -> VehicleState mode path
    state = SharedState()
    state.vehicle.connected = True
    state.vehicle.mode = "STABILIZE"
    now = time.monotonic_ns()
    state.vehicle.last_heartbeat_ns = now
    state.vehicle.mark_message("HEARTBEAT", now)
    state.vehicle.autopilot_system_id = 1
    state.vehicle.autopilot_component_id = 1

    tx = FakeTx(state)
    freshness = FreshnessPolicy(state.vehicle, {"HEARTBEAT": 2.5})
    command = CommandManager(
        tx,
        state,
        EventBus(),
        freshness_policy=freshness,
        vct_cfg={
            "ack_timeout_s": 0.01,
            "state_timeout_s": 0.05,
            "max_attempts": 3,
            "backoff_s": [0.01, 0.01, 0.01],
        },
    )

    mode_result = command.set_mode("GUIDED", require_disarmed=True)
    check("CommandManager mode VCT succeeds", mode_result.ok)
    check("mode verified as GUIDED", state.vehicle.mode == "GUIDED")
    check("TX path used exactly once", tx.mode_sends == 1)

    # Retry semantics
    holder = {"value": False, "sends": 0}

    def send_retry():
        holder["sends"] += 1
        if holder["sends"] >= 2:
            holder["value"] = True
        return True

    retry = VerifiedCommandTransaction(
        "RETRY_TEST",
        send_fn=send_retry,
        verify_fn=lambda: holder["value"],
        state_timeout_s=0.01,
        attempts=3,
        backoff=(0.01, 0.01, 0.01),
    ).run()

    check("VCT retry succeeds", retry.ok)
    check("VCT retry took two attempts", retry.attempts == 2)

    # Terminal FC rejection semantics
    reject_state = VehicleState()
    reject_holder = {"sends": 0}

    def send_reject():
        reject_holder["sends"] += 1
        reject_state.record_command_ack(
            400,
            4,  # MAV_RESULT_FAILED
            1,
            1,
            timestamp_ns=time.monotonic_ns(),
        )
        return True

    rejected = VerifiedCommandTransaction(
        "ARM_REJECT_TEST",
        send_fn=send_reject,
        verify_fn=lambda: False,
        vehicle_state=reject_state,
        expected_command=400,
        expected_ack_system=1,
        expected_ack_component=1,
        require_ack=True,
        terminal_reject_results=(1, 2, 3, 4, 6, 7, 8, 9),
        ack_timeout_s=0.05,
        state_timeout_s=0.01,
        attempts=3,
        backoff=(0.01, 0.01, 0.01),
    ).run()

    check("terminal ACK classified REJECTED", rejected.outcome == "REJECTED")
    check(
        "terminal ACK not blindly retried",
        rejected.attempts == 1 and reject_holder["sends"] == 1,
    )

    print("======================================================")
    print(" FOUNDATION SELF-CHECK: PASS")
    print("======================================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
