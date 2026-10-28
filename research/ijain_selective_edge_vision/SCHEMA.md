# Frame-level trace CSV schema

Run the replay tool on **recorded** frame metadata, not invented manuscript metrics.

Fields:
- frame_id: monotonically ordered frame identifier.
- capture_s: monotonically nondecreasing capture time (seconds, same monotonic clock).
- mission_active: 1 only during the designated GUIDED gate-alignment segment.
- x1,y1,x2,y2,x3,y3,x4,y4: four physical gate corners TL,TR,BR,BL in native-resolution pixels (blank for invalid observation).
- detector_confidence: confidence reported on this frame if the detector were executed. Offline common-trace replay needs per-frame detector observations acquired independently of the evaluated scheduler policy.
- geo_normalized: RMS projection-vs-observation consistency normalized by the mean gate diagonal; blank if missing.
- reprojection_rmse_px: actual PnP reprojection RMSE at native camera coordinates.
- positive_depth: 1 if planar pose solution has positive depth.
- ambiguity_resolved: 1 if ambiguous planar solutions have been resolved.
- feedback_age_s: capture-to-available geometric feedback age measured via timestamps.

Optional synchronized, common-frame RTK pairs:
- rtk_valid, visual_x_m, visual_y_m, rtk_x_m, rtk_y_m.

RTK validity must reflect fixed solution, accuracy, correction age, and synchronization, not merely fix status. When RTK pairs are absent, localization RMSE and high-error acceptance remain null. The replay CLI does not generate data or claim Hailo hardware execution. It does not reproduce physical closed-loop control. The inference-ms flag is an assumed busy window rather than logged inference latency.
