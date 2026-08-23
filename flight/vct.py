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
    outcome: str = "FAILED"   # SUCCESS | REJECTED | FAILED


class VerifiedCommandTransaction:
    """Verified Command Transaction.

    Flow:
      pre-check -> send -> optional ACK correlation -> state verification
      -> adaptive retry/backoff -> structured result.

    V1.7 distinguishes explicit FC rejection from transport/verification failure.
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

    def _trace(self, event: str, **data):
        if self.trace_fn is None:
            return
        try:
            self.trace_fn(event, data)
        except Exception:
            # Diagnostics must never break the transaction.
            pass

    def _wait_for_ack(self, since_ns: int):
        if self.vehicle_state is None or self.expected_command is None:
            return None

        deadline = time.monotonic() + self.ack_timeout_s
        while time.monotonic() < deadline:
            ack = self.vehicle_state.find_command_ack(
                self.expected_command,
                since_ns,
                self.expected_ack_system,
                self.expected_ack_component,
            )
            if ack is not None:
                return ack
            time.sleep(0.01)
        return None

    def _wait_for_state(self) -> bool:
        deadline = time.monotonic() + self.state_timeout_s
        while time.monotonic() < deadline:
            if self.verify_fn():
                return True
            time.sleep(0.02)
        return bool(self.verify_fn())

    def run(self) -> VctResult:
        started = time.monotonic()
        self._trace("START", name=self.name)

        ok, reason = self.precheck_fn()
        self._trace("PRECHECK", ok=bool(ok), reason=str(reason or ""))
        if not ok:
            return VctResult(
                ok=False,
                attempts=0,
                reason=f"precheck: {reason}",
                elapsed_s=time.monotonic() - started,
                outcome="FAILED",
            )

        if self.verify_fn():
            self._trace("ALREADY_SATISFIED")
            return VctResult(
                ok=True,
                attempts=0,
                reason="already satisfied",
                elapsed_s=time.monotonic() - started,
                outcome="SUCCESS",
            )

        last_reason = "verification timeout"
        last_ack = None

        for attempt in range(1, self.attempts + 1):
            self._trace("ATTEMPT", attempt=attempt, total=self.attempts)
            send_time_ns = time.monotonic_ns()

            try:
                sent = bool(self.send_fn())
            except Exception as exc:
                sent = False
                last_reason = f"send exception: {exc}"

            self._trace("SEND", ok=sent)

            if sent:
                ack = self._wait_for_ack(send_time_ns)
                last_ack = ack

                self._trace(
                    "ACK",
                    received=ack is not None,
                    result=None if ack is None else ack.result,
                    source=None if ack is None else (
                        f"{ack.source_system}:{ack.source_component}"
                    ),
                )

                if self.require_ack:
                    if ack is None:
                        last_reason = "ACK timeout"

                    elif ack.result in self.terminal_reject_results:
                        self._trace(
                            "REJECTED",
                            result=ack.result,
                            source=f"{ack.source_system}:{ack.source_component}",
                        )
                        return VctResult(
                            ok=False,
                            attempts=attempt,
                            reason=f"FC rejected command, ACK result={ack.result}",
                            ack_result=ack.result,
                            ack_source=f"{ack.source_system}:{ack.source_component}",
                            elapsed_s=time.monotonic() - started,
                            outcome="REJECTED",
                        )

                    elif ack.result not in self.accepted_ack_results:
                        last_reason = f"non-accepted ACK result={ack.result}"

                    else:
                        state_ok = self._wait_for_state()
                        self._trace("STATE_VERIFY", ok=state_ok)
                        if state_ok:
                            return VctResult(
                                ok=True,
                                attempts=attempt,
                                reason="ACK + state verified",
                                ack_result=ack.result,
                                ack_source=f"{ack.source_system}:{ack.source_component}",
                                elapsed_s=time.monotonic() - started,
                                outcome="SUCCESS",
                            )
                        last_reason = "ACK accepted but state not verified"

                else:
                    state_ok = self._wait_for_state()
                    self._trace("STATE_VERIFY", ok=state_ok)
                    if state_ok:
                        return VctResult(
                            ok=True,
                            attempts=attempt,
                            reason="state verified",
                            ack_result=None if ack is None else ack.result,
                            ack_source=None if ack is None else (
                                f"{ack.source_system}:{ack.source_component}"
                            ),
                            elapsed_s=time.monotonic() - started,
                            outcome="SUCCESS",
                        )
                    last_reason = "state not verified"

            elif not last_reason.startswith("send exception"):
                last_reason = "send failed"

            if attempt < self.attempts:
                delay = self.backoff[min(attempt - 1, len(self.backoff) - 1)]
                self._trace("BACKOFF", seconds=delay, reason=last_reason)
                time.sleep(delay)

        self._trace("FAILED", reason=last_reason)
        return VctResult(
            ok=False,
            attempts=self.attempts,
            reason=last_reason,
            ack_result=None if last_ack is None else last_ack.result,
            ack_source=None if last_ack is None else (
                f"{last_ack.source_system}:{last_ack.source_component}"
            ),
            elapsed_s=time.monotonic() - started,
            outcome="FAILED",
        )
