"""Dry-run proportional gate-alignment controller; NEVER transmits MAVLink."""
from dataclasses import dataclass

@dataclass(frozen=True)
class GuidanceConfig:
    kp_s_inverse: float = 0.5
    deadband_m: float = 0.05
    velocity_limit_mps: float = 0.30
    tolerance_m: float = 0.10
    settle_s: float = 1.0
    feedback_loss_limit_s: float = 2.0
    attempt_timeout_s: float = 15.0

@dataclass(frozen=True)
class GuidanceIntent:
    state: str
    lateral_mps: float
    vertical_mps: float
    reason: str
    tx_enabled: bool = False

class GuidanceController:
    """Offline only. Inputs are errors expressed in controlled vehicle axes."""
    def __init__(self,cfg=GuidanceConfig()):
        self.cfg=cfg
        self.started_s=None
        self.last_s=None
        self.missing_since_s=None
        self.aligned_since_s=None
        self.done=None

    def _speed(self,error_m):
        if abs(error_m)<=self.cfg.deadband_m: return 0.0
        v=-self.cfg.kp_s_inverse*error_m
        limit=self.cfg.velocity_limit_mps
        return max(-limit,min(limit,v))

    def step(self,timestamp_s,lateral_error_m=None,vertical_error_m=None,feedback_valid=False):
        if self.last_s is not None and timestamp_s<self.last_s:
            raise ValueError("timestamps must be monotonic")
        if self.started_s is None: self.started_s=timestamp_s
        self.last_s=timestamp_s
        if self.done is not None:
            return GuidanceIntent(self.done,0.,0.,"terminal")
        if timestamp_s-self.started_s>=self.cfg.attempt_timeout_s:
            self.done="ABORT"
            return GuidanceIntent("ABORT",0.,0.,"alignment_timeout")
        if not feedback_valid or lateral_error_m is None or vertical_error_m is None:
            self.aligned_since_s=None
            if self.missing_since_s is None: self.missing_since_s=timestamp_s
            if timestamp_s-self.missing_since_s>=self.cfg.feedback_loss_limit_s:
                self.done="ABORT"
                return GuidanceIntent("ABORT",0.,0.,"feedback_lost_too_long")
            return GuidanceIntent("HOLD",0.,0.,"waiting_for_valid_feedback")
        self.missing_since_s=None
        if abs(lateral_error_m)<=self.cfg.tolerance_m and abs(vertical_error_m)<=self.cfg.tolerance_m:
            if self.aligned_since_s is None: self.aligned_since_s=timestamp_s
            if timestamp_s-self.aligned_since_s>=self.cfg.settle_s:
                self.done="SUCCESS"
                return GuidanceIntent("SUCCESS",0.,0.,"stable_alignment")
        else:
            self.aligned_since_s=None
        return GuidanceIntent("ALIGN",self._speed(lateral_error_m),self._speed(vertical_error_m),
                              "valid_guidance_intent")
