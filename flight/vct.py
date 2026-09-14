from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable


@dataclass
class VctResult:
    ok: bool
    attempts: int
    reason: str = ""
    ack_result: int | None = None
    ack_source: str | None = None
    elapsed_s: float = 0.0
    outcome: str = "FAILED"   # SUCCESS | REJECTED | FAILED | UNCONFIRMED


class VerifiedCommandTransaction:
    """Verified Command Transaction.

    Correlate ACK identity/time while monitoring state and bounded progress.
    Critical ACK-required transactions are never resent after successful TX.
    UNCONFIRMED means an action may still be active on the FC.
    """

    def __init__(
        self,
        name: str,
        send_fn: Callable[[], bool],
        verify_fn: Callable[[], bool],
        precheck_fn: Callable[[], tuple[bool, str]] | None = None,
        *,
        vehicle_state=None,
        expected_command: int | None = None,
        expected_ack_system: int | None = None,
        expected_ack_component: int | None = None,
        ack_timeout_s: float = 0.8,
        state_timeout_s: float = 1.5,
        attempts: int = 3,
        backoff=(0.25, 0.5, 0.9),
        require_ack: bool = False,
        accepted_ack_results=(0,),
        terminal_reject_results=(2, 3, 4, 6, 7, 8, 9),
        trace_fn=None,
        progress_timeout_s=10.0,
        abort_fn=None,
        allow_already_satisfied=True,
    ):
        self.name = str(name)
        self.send_fn = send_fn
        self.verify_fn = verify_fn
        self.precheck_fn = precheck_fn or (lambda: (True, ""))

        self.vehicle_state = vehicle_state
        self.expected_command = expected_command
        self.expected_ack_system = expected_ack_system
        self.expected_ack_component = expected_ack_component

        self.ack_timeout_s = float(ack_timeout_s)
        self.state_timeout_s = float(state_timeout_s)
        self.attempts = max(1, int(attempts))
        self.backoff = tuple(float(x) for x in backoff) or (0.25,)
        self.require_ack = bool(require_ack)
        self.accepted_ack_results = tuple(int(x) for x in accepted_ack_results)
        self.terminal_reject_results = tuple(int(x) for x in terminal_reject_results)
        self.trace_fn = trace_fn
        self.progress_timeout_s = float(progress_timeout_s)
        self.abort_fn = abort_fn or (lambda: (True, ""))
        self.allow_already_satisfied = bool(allow_already_satisfied)

    def _trace(self, event: str, **data):
        if self.trace_fn is None:
            return
        try:
            self.trace_fn(event, data)
        except Exception:
            # Diagnostics must never break the transaction.
            pass

    def run(self) -> VctResult:
        started = time.monotonic()
        last_ack = None
        attempt = 0

        def result(ok, reason, outcome="FAILED"):
            self._trace(outcome, reason=reason, name=self.name)
            return VctResult(ok, attempt, reason,
                             None if last_ack is None else last_ack.result,
                             None if last_ack is None else
                             f"{last_ack.source_system}:{last_ack.source_component}",
                             time.monotonic() - started, outcome)

        self._trace("START", name=self.name)
        for attempt in range(1, self.attempts + 1):
            ok, reason = self.precheck_fn()
            if not ok:
                return result(False, f"precheck: {reason}")
            if self.allow_already_satisfied and self.verify_fn():
                attempt -= 1
                return result(True, "already satisfied", "SUCCESS")
            since_ns = time.monotonic_ns()
            self._trace("ATTEMPT", attempt=attempt, total=self.attempts)
            try:
                sent = bool(self.send_fn())
            except Exception as exc:
                return result(False, f"send exception; command delivery unknown: {exc}", "UNCONFIRMED")
            self._trace("SEND", ok=sent)
            if not sent:
                reason = "send failed"
            else:
                ack_deadline = time.monotonic() + self.ack_timeout_s
                # State monitoring starts with transmission, including if ACK is lost.
                state_deadline = time.monotonic() + self.state_timeout_s
                progress_deadline = None
                last_ack = None
                observed_state = False
                while True:
                    ok, reason = self.abort_fn()
                    if not ok:
                        return result(False, f"monitor stopped: {reason}; FC command may still be active", "UNCONFIRMED")
                    now = time.monotonic()
                    ack = None
                    if self.vehicle_state is not None and self.expected_command is not None:
                        ack = self.vehicle_state.find_command_ack(
                            self.expected_command, since_ns,
                            self.expected_ack_system, self.expected_ack_component)
                    if ack is not None and (last_ack is None or ack.timestamp_ns > last_ack.timestamp_ns):
                        last_ack = ack
                        self._trace("ACK", result=ack.result,
                                    source=f"{ack.source_system}:{ack.source_component}")
                        if ack.result in self.terminal_reject_results:
                            return result(False, f"FC rejected command, ACK result={ack.result}", "REJECTED")
                        if ack.result == 5 and progress_deadline is None:
                            progress_deadline = now + self.progress_timeout_s
                            state_deadline = max(state_deadline, progress_deadline)
                            self._trace("IN_PROGRESS")
                    observed_state = bool(self.verify_fn())
                    accepted = last_ack is not None and last_ack.result in self.accepted_ack_results
                    if observed_state and (accepted or not self.require_ack):
                        return result(True, "ACK + state verified" if self.require_ack else "state verified", "SUCCESS")
                    # No retransmission after acceptance or IN_PROGRESS. A timeout
                    # is not evidence that the physical action did not happen.
                    deadline = state_deadline
                    if self.require_ack and last_ack is None:
                        deadline = max(deadline, ack_deadline)
                    if now >= deadline:
                        detail = "state observed" if observed_state else "state not verified"
                        if self.require_ack:
                            return result(False, f"{detail}; final ACK/state timeout; inspect FC before retry", "UNCONFIRMED")
                        reason = "state not verified"
                        break
                    time.sleep(0.02)
            if attempt < self.attempts:
                delay = self.backoff[min(attempt - 1, len(self.backoff) - 1)]
                self._trace("BACKOFF", seconds=delay, reason=reason)
                time.sleep(delay)
        return result(False, reason)
