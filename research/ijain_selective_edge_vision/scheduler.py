"""IJAIN selective detector refresh reference; OFFLINE ONLY, no UAV control."""
from dataclasses import dataclass
from typing import Optional

POLICIES = ("continuous", "fixed_2", "fixed_5", "fixed_10", "confidence_only",
            "geometry_only", "confidence_geometry", "full_reliability")

@dataclass(frozen=True)
class Thresholds:
    confidence: float = 0.50
    geo_normalized: float = 0.01
    temporal_normalized: float = 0.03
    max_detector_age_s: float = 0.50
    max_reprojection_rmse_px: float = 3.0
    max_feedback_age_s: float = 0.15

@dataclass(frozen=True)
class Observation:
    frame_id: int
    capture_s: float
    mission_active: bool
    structural_valid: bool
    reprojection_rmse_px: Optional[float]
    geometry_normalized: Optional[float]
    temporal_normalized: Optional[float]
    positive_depth: bool
    ambiguity_resolved: bool
    feedback_age_s: float
    candidate_detector_confidence: Optional[float] = None

@dataclass(frozen=True)
class Decision:
    policy: str
    refresh_requested: bool
    inference_executed: bool
    coalesced: bool
    accepted: bool
    detector_age_s: Optional[float]
    reason: str

class Scheduler:
    """One busy detector; pending requests coalesced; detector age is independent of geometry age."""
    def __init__(self, policy="full_reliability", thresholds=Thresholds(), inference_s=0.043):
        if policy not in POLICIES:
            raise ValueError("unsupported policy: "+str(policy))
        if inference_s < 0:
            raise ValueError("negative inference duration")
        self.policy, self.thresholds, self.inference_s = policy, thresholds, inference_s
        self.last_detector_capture_s = None
        self.last_detector_confidence = None
        self.busy_until_s = float("-inf")
        self.last_frame_s = float("-inf")
        self.active_frame_count = 0

    def _trigger(self, o, age):
        p, h = self.policy, self.thresholds
        if p == "continuous": return "continuous"
        if p.startswith("fixed_"):
            k = int(p.split("_")[1])
            return "fixed_interval" if (self.active_frame_count - 1) % k == 0 else ""
        if age is None: return "initial_detection"
        if not o.structural_valid: return "invalid_structure"
        if age >= h.max_detector_age_s: return "maximum_age"
        if p in ("confidence_only", "confidence_geometry", "full_reliability"):
            if self.last_detector_confidence is None or self.last_detector_confidence < h.confidence:
                return "low_confidence"
        if p in ("geometry_only", "confidence_geometry", "full_reliability"):
            if o.geometry_normalized is None or o.geometry_normalized > h.geo_normalized:
                return "geometric_disagreement"
        if p == "full_reliability":
            if o.temporal_normalized is None or o.temporal_normalized > h.temporal_normalized:
                return "temporal_disagreement"
        return ""

    def step(self, o):
        if o.capture_s < self.last_frame_s:
            raise ValueError("timestamps are not monotonic")
        self.last_frame_s = o.capture_s
        age = None if self.last_detector_capture_s is None else o.capture_s-self.last_detector_capture_s
        if not o.mission_active:
            return Decision(self.policy, False, False, False, False, age, "outside_alignment_segment")
        self.active_frame_count += 1
        why = self._trigger(o, age)
        requested = bool(why)
        executed = requested and o.capture_s >= self.busy_until_s
        coalesced = requested and not executed
        if executed:
            self.last_detector_capture_s = o.capture_s
            self.last_detector_confidence = o.candidate_detector_confidence
            self.busy_until_s = o.capture_s+self.inference_s
        h = self.thresholds
        accepted = (o.structural_valid and o.positive_depth and o.ambiguity_resolved
                    and o.reprojection_rmse_px is not None
                    and o.reprojection_rmse_px <= h.max_reprojection_rmse_px
                    and 0 <= o.feedback_age_s <= h.max_feedback_age_s)
        return Decision(self.policy, requested, executed, coalesced, accepted, age, why or "no_refresh")
