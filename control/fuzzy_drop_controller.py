from __future__ import annotations

import math
import time

from safety.pilot_takeover import PilotTakeover


class FuzzyDropError(RuntimeError):
    pass


def _clamp(v, lo, hi):
    return max(lo, min(hi, float(v)))


def _slew(current, target, rate, dt):
    delta = _clamp(float(target) - float(current), -abs(rate) * dt, abs(rate) * dt)
    return float(current) + delta


class FuzzyDropController:
    """Production 3-state pure fuzzy bottom-centering controller."""

    def __init__(self, state, command, tx, vision, cfg: dict, log=print):
        self.state = state
        self.command = command
        self.tx = tx
        self.vision = vision
        self.cfg = dict(cfg or {})
        self.log = log

    def _memberships(self, radial: float):
        c = float(self.cfg["center_error"])
        m = float(self.cfg["mid_error"])
        f = float(self.cfg["far_error"])
        r = max(0.0, float(radial))
        if r <= c:
            return 1.0, 0.0, 0.0
        if r < m:
            t = _clamp((r - c) / (m - c), 0.0, 1.0)
            return 1.0 - t, t, 0.0
        if r < f:
            t = _clamp((r - m) / (f - m), 0.0, 1.0)
            return 0.0, 1.0 - t, t
        return 0.0, 0.0, 1.0

    def _speed(self, radial: float):
        mu = self._memberships(radial)
        speed = (
            mu[0] * float(self.cfg["minimum_speed_mps"])
            + mu[1] * float(self.cfg["mid_speed_mps"])
            + mu[2] * float(self.cfg["far_speed_mps"])
        )
        return speed, mu

    def _zero(self):
        self.tx.send_body_velocity(0.0, 0.0, 0.0)

    def run(self, abort_event=None, max_duration_s=None) -> bool:
        if abort_event is not None and abort_event.is_set():
            raise FuzzyDropError("aborted before fuzzy GUIDED transition")
        mode = self.command.set_mode("GUIDED")
        if not mode.ok:
            raise FuzzyDropError(f"GUIDED failed: {mode.reason}")
        self._zero()

        hz = float(self.cfg.get("control_hz", 20.0))
        period = 1.0 / hz
        confidence = float(self.cfg.get("confidence_threshold", 0.25))
        freshness = float(self.cfg.get("vision_freshness_s", 0.20))
        center_error = float(self.cfg["center_error"])
        center_frames = int(self.cfg.get("center_confirm_frames", 2))
        offset_x = float(self.cfg.get("center_offset_x", 0.0))
        offset_y = float(self.cfg.get("center_offset_y", 0.0))
        alpha = float(self.cfg.get("filter_alpha", 0.40))
        slew_rate = float(self.cfg.get("xy_slew_mps2", 1.50))
        x_sign = float(self.cfg.get("x_sign", -1))
        y_sign = float(self.cfg.get("y_sign", -1))
        lost_timeout = float(self.cfg.get("lost_timeout_s", 0.15))
        guard_floor = float(self.cfg.get("guard_floor_m", 0.60))
        guard_ceiling = float(self.cfg.get("guard_ceiling_m", 1.40))
        guard_outside_s = float(self.cfg.get("guard_outside_s", 0.35))
        lidar_stale_s = float(self.cfg.get("lidar_stale_s", 0.60))

        deadline = None if max_duration_s is None else time.monotonic() + float(max_duration_s)
        last_frame = None
        center_streak = 0
        filt_bx = filt_by = None
        vx = vy = 0.0
        lost_since = None
        lidar_lost_since = None
        outside_since = None
        next_tx = 0.0
        next_diag = 0.0

        self.log("[FUZZY] ACTIVE | 3-state MIN/MID/FAR | target-loss => XY ZERO")

        while True:
            if deadline is not None and time.monotonic() >= deadline:
                self._zero()
                self.log(f"[FUZZY] FALLBACK WINDOW {float(max_duration_s):.1f}s EXPIRED -> XY ZERO")
                return False
            if abort_event is not None and abort_event.is_set():
                self._zero()
                raise FuzzyDropError("aborted during fuzzy alignment")
            v = self.state.vehicle
            if v.mode != "GUIDED":
                self._zero()
                raise PilotTakeover(f"operator changed mode during fuzzy alignment to {v.mode}")
            if not v.connected or v.heartbeat_age_s() > 2.5:
                self._zero()
                raise FuzzyDropError("heartbeat lost during fuzzy alignment")

            now = time.monotonic()
            if not v.rangefinder_is_fresh(lidar_stale_s):
                self._zero()
                lidar_lost_since = lidar_lost_since or now
                if now - lidar_lost_since > lidar_stale_s:
                    raise FuzzyDropError("LiDAR stale during fuzzy alignment")
                time.sleep(0.01)
                continue
            lidar_lost_since = None

            agl = float(v.rangefinder_distance_m)
            if agl < guard_floor or agl > guard_ceiling:
                outside_since = outside_since or now
                if now - outside_since > guard_outside_s:
                    self._zero()
                    raise FuzzyDropError(f"LiDAR guard violation {agl:.2f}m")
            else:
                outside_since = None

            # Vision source may be the legacy SharedState-backed runtime or the
            # standalone VisionDaemonClient used by the race runner.
            if hasattr(self.vision, "snapshot"):
                b = dict(self.vision.snapshot() or {})
                age_ms = b.get("result_age_ms")
                b["usable"] = bool(
                    b.get("link_ok")
                    and b.get("usable")
                    and age_ms is not None
                    and float(age_ms) <= freshness * 1000.0
                )
            else:
                b = self.state.get_vision_snapshot("bottom", max_age_s=freshness)

            frame_id = b.get("frame_id")
            conf = b.get("confidence")
            ex = b.get("ex")
            ey = b.get("ey")
            valid = bool(
                b.get("usable")
                and conf is not None
                and float(conf) >= confidence
                and ex is not None
                and ey is not None
            )

            phase = "SEARCH"
            radial = 0.0
            speed = 0.0
            mu = (0.0, 0.0, 0.0)

            if valid and frame_id != last_frame:
                last_frame = frame_id
                lost_since = None
                adj_ex = _clamp(float(ex) - 2.0 * offset_x, -1.0, 1.0)
                adj_ey = _clamp(float(ey) - 2.0 * offset_y, -1.0, 1.0)

                if abs(adj_ex) <= center_error and abs(adj_ey) <= center_error:
                    center_streak += 1
                else:
                    center_streak = 0

                if center_streak >= center_frames:
                    self._zero()
                    self.log(f"[FUZZY] CENTERED {center_streak}/{center_frames} -> XY ZERO")
                    return True

                bx = _clamp(-adj_ex * x_sign, -1.0, 1.0)
                by = _clamp(-adj_ey * y_sign, -1.0, 1.0)
                if filt_bx is None:
                    filt_bx, filt_by = bx, by
                else:
                    filt_bx = alpha * bx + (1.0 - alpha) * filt_bx
                    filt_by = alpha * by + (1.0 - alpha) * filt_by

                radial = math.hypot(filt_bx, filt_by)
                speed, mu = self._speed(radial)
                if mu[2] >= max(mu[0], mu[1]):
                    phase = "FAR"
                elif mu[1] >= mu[0]:
                    phase = "MID"
                else:
                    phase = "MINIMUM"

                norm = math.hypot(filt_bx, filt_by)
                if norm > 1e-6:
                    target_vx = speed * filt_bx / norm
                    target_vy = speed * filt_by / norm
                else:
                    target_vx = target_vy = 0.0
                vx = _slew(vx, target_vx, slew_rate, period)
                vy = _slew(vy, target_vy, slew_rate, period)

            elif not valid:
                center_streak = 0
                lost_since = lost_since or now
                vx = vy = 0.0
                self._zero()
                phase = "TARGET_LOST"
                if now - lost_since >= lost_timeout:
                    filt_bx = filt_by = None

            if now >= next_tx:
                self.tx.send_body_velocity(vx, vy, 0.0)
                next_tx = now + period

            if now >= next_diag:
                conf_text = "--" if conf is None else f"{float(conf):.2f}"
                self.log(
                    f"[CTRL] FUZZY {phase} | conf={conf_text} | r={radial:.3f} | "
                    f"VX={vx:+.2f} VY={vy:+.2f} | center={center_streak}/{center_frames}"
                )
                next_diag = now + 2.0

            time.sleep(0.005)

