# Progress Log

## Session 2026-05-11 (Experiment Re-Run)

### Setup
- All 9 fixes applied to codebase before re-run
- Old data cleared: `sup_tracking_to_planning/` and `sup_occlusion_reid/`
- Monitor PID: bu1uycqsl (persistent)

### Experiment 1: Tracking-to-Planning
- Started: 21:21
- Command: `--methods "dodt fapp ours_no_clip ours_no_sam ours_full stage1_only" --scenes "pair_crossing:2 front_crossing:3 random_multi:0"`
- 3 scenes × 6 methods = 18 runs
- Log: `/tmp/exp1_full.log`

Progress (run count: 2/18):
- [x] pair_crossing / dodt — complete
- [x] pair_crossing / fapp — in post-processing
- [ ] pair_crossing / ours_no_clip
- [ ] pair_crossing / ours_no_sam
- [ ] pair_crossing / ours_full
- [ ] pair_crossing / stage1_only
- [ ] front_crossing / (6 methods)
- [ ] random_multi / (6 methods)

### Experiment 2: Controlled Occlusion + Re-ID
- Status: Pending
- Planned: `--durations "0.5 1.0 2.0" --methods "stage1_only ours_no_sam ours_no_clip ours_no_fastsam ours_full dodt fapp"`
- 3 durations × 7 methods = 21 runs
- Log: `/tmp/exp2_full.log`

## Session 2026-06-19 (Issue-1 Investigation Planning)

### Objective
- Build a focused diagnosis plan for: `/rematch/tracker_details` may already be wrong in `odom` once the robot moves

### Progress
- Reviewed current frame-honest tracker path and existing diagnostic scripts
- Rejected planner/follower as first-pass targets for this investigation
- Appended a dedicated Issue-1 investigation section to `task_plan.md`
- Pending user review before any script writing or test execution

## Session 2026-06-19 (Issue-1 Investigation Execution)

### Tooling Audit
- Audited existing diagnostic assets:
  - `diagnostic_frames.launch`
  - `stage1_perfect_meas.launch`
  - `stage2_real_umap.launch`
  - `analyze_frame_errors.py`
- Decision: use them only as references; create a narrower Issue-1-specific pipeline

### New Artifacts Added
- `src/exp/launch/issue1_static_obstacle.launch`
- `src/exp/scripts/run_issue1_diag.sh`
- `src/exp/scripts/analyze_issue1_tracking_drift.py`
- `src/exp/configs/issue1.yaml`

### Verification
- `python3 -m py_compile` passed for the new analysis script
- `bash -n` passed for the new runner
- `roslaunch --files sup_exp issue1_static_obstacle.launch` passed

### Notes
- The new analysis path compares:
  - `UMAP_camera_vs_GT_camera`
  - `Coarse_odom_vs_GT_odom`
  - `Tracker_odom_vs_GT_odom`
  - `Tracker_base_vs_GT_base`
- Primary bag-level outputs:
  - `<prefix>_summary.json`
  - `<prefix>_samples.csv`
  - `<prefix>_timeseries.png`
  - `<prefix>_errors.png`
  - `<prefix>_displacement.png`

### Coarse Candidate Investigation
- Added `src/exp/scripts/analyze_coarse_candidate_drop.py`
- Verified on `issue1_static.bag`:
  - GT-near UMAP frames: 187
  - GT-near coarse-published frames: 1
- Verified on `issue1_move_real.bag`:
  - GT-near UMAP frames: 122
  - GT-near coarse-published frames: 16
- Read early-frame coarse/rematch outputs directly from bag:
  - correct GT-near candidate appears
  - then disappears from `/tracker/dynamic_obstacles`
  - wrong side-fragment IDs persist and dominate downstream rematch
- Identified initialization hazard:
  - `ObstacleTracker::init()` sets `isDynamic = true`

## Session 2026-06-25 (Prediction Trust Investigation)

### Objective
- Investigate `prediction_trust_briefing.md` and decide how to replace the `cv_horizon` hard switch in `doa_to_tmpc.py`.

### Progress
- Restored existing planning context.
- Read the briefing.
- Added a dedicated Prediction Trust Geometry Investigation section to `task_plan.md`.
- Initial code search found:
  - `doa_to_tmpc.py` contains `cv_horizon` and `emit_uncertainty`.
  - `mpc_nav/msg/Obstacle.msg` already contains `sigma2_*` covariance fields for the older `mpc_nav` obstacle message.
  - `tmpc_doa.launch` currently sets `cv_horizon=1.5` and `emit_uncertainty=false`.

### Outcome
- Verified `/rematch/tracker_details` does carry KF covariance from `detailed_matcher.cpp`.
- Verified jackalsimulator's generated solver already has ellipsoid `major/minor/chi` parameters; using the existing ellipsoid Gaussian path should not require solver regeneration.
- Found the key caveat: current Gaussian/ellipsoid path is a hard inflated obstacle constraint. Growing uncertainty makes constraints stricter, not weaker, so it does not implement "far prediction trust decay."
- Recommendation recorded in `findings.md`: avoid full-CV + growing hard Gaussian as the main fix; prefer a bridge-only smooth trust-decay baseline plus a measured capped-Gaussian variant, while treating true trust decay as a solver-level soft/stage-weighted constraint change.

## Session 2026-06-25 (Prediction Trust Investigation, T-MPC Input Reread)

### Correction
- User pointed out the key semantic error: KF covariance is tracking covariance, not future prediction covariance.
- Re-read from the T-MPC input side first, then compared backward to DOA output.

### Findings
- Current generated jackalsimulator modules are not `GuidanceConstraints`; they are `MPCBaseModule`, `GoalModule`, `PathReferenceVelocity`, and `EllipsoidConstraints`.
- The active generated solver therefore appears to be LMPCC-style hard ellipsoid constraints, despite comments/config labels mentioning T-MPC.
- Active ellipsoid path consumes `N-1` future prediction steps (`mode[k-1]` for solver stages `k=1..N-1`).
- Some helper paths (`propagatePredictionUncertainty`, `ensureObstacleSize` when obstacles exceed max) assume `N` prediction steps and can index past the current bridge's `N-1` output.
- DOA output gives current track state plus current tracking covariance. It does not provide a behavior prediction distribution.

### Revised Direction
- Bridge should own an explicit deterministic mean-prediction policy from DOA current state to T-MPC stage-indexed obstacle trajectory.
- `sigma2_*` should be treated as track-quality metadata at most, not as direct future prediction covariance.
- Do not enable probabilistic propagation until array length and semantics are fixed.

## Session 2026-06-25 (Prediction Trust Investigation, Bridge Contract Follow-up)

### Progress
- Re-read active planner input consumer from `/input/obstacles` through `ros1_jackalsimulator.cpp`, `data_preparation.cpp`, `ellipsoid_constraints.cpp`, and latent `guidance_constraints.cpp`.
- Confirmed active runtime contract:
  - `ObstacleGMM` carries current pose plus one Gaussian mode.
  - current active ellipsoid path consumes `N-1` future prediction entries.
  - bridge unit tests explicitly assert this `N-1` length and current freeze behavior.
- Confirmed latent mismatch:
  - `propagatePredictionUncertainty()` and obstacle-overflow sorting assume `N` entries.
  - guidance, if regenerated, builds `[current + prediction]` and expects/extends to `N+1`.
- Re-read DOA tracker output and KF implementation:
  - covariance is current tracking covariance from KF `P`, not a future behavior covariance.
  - `TrackerDetail.msg` comments partly encode an intended downstream propagation formula that the current bridge does not implement.

### Interim Recommendation
- Treat the bridge as the owner of the motion prior.
- Near-term bridge should stay deterministic and replace hard `cv_horizon` freeze with a smooth saturating velocity-trust function.
- Do not enable Gaussian/probabilistic mode until prediction length and semiaxis semantics are made explicit.
