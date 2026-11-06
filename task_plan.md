# Task Plan: Exp Scripts Reorganization (2026-05-12)

## Goal
Investigate and reorganize scripts under `exp/` by three dimensions: Scene (场景), Adaptor (适配器), Statistics & Visualization (统计与可视化). Eliminate redundancies, deprecate legacy scripts, consolidate near-duplicates.

## Phases

### Phase 1: Investigation — Map all scripts and identify redundancies
- [x] Read all .py and .sh files under exp/scripts/
- [x] Read all .launch files under exp/launch/
- [x] Read SUPPLEMENTAL_EXPERIMENTS.md, commands.md, plan.md
- [x] Classify each script by role (scene/adaptor/stat-viz/runner)
- [x] Identify redundancies and overlaps
- [x] Write investigation report to findings.md
- **Status:** complete

### Phase 2: Review report with user
- [x] User reviewed, confirmed: keep dodt_adapter (don't merge), delete old runners, use subdirs
- **Status:** complete

### Phase 3: Execute cleanup
- [x] Delete duplicate scripts: gen_report.py, compare_bench.py
- [x] Delete legacy runners: run_exp.sh, run_onboard_exp.sh
- [x] Remove __pycache__ directories
- [x] Update commands.md → redirect to SUPPLEMENTAL_EXPERIMENTS.md
- [x] dodt_adapter.py kept (user decision: one adaptor per external method)
- **Status:** complete

### Phase 4: Reorganize directory structure
- [x] Create scene/, adaptor/, stats_viz/ subdirs
- [x] Move scripts: 1 scene, 5 adaptors, 13 stats_viz
- [x] Keep 5 shell runners at top level
- [x] Update CMakeLists.txt install rules
- [x] Update all path references in 5 runners + 3 launch files + SUPPLEMENTAL_EXPERIMENTS.md
- **Status:** complete

### Phase 5: Verification
- [x] bash -n: all 5 .sh scripts passed
- [x] python3 -m py_compile: all 19 .py scripts passed
- [x] roslaunch --files: all 5 .launch files passed
- [x] Remove stale __pycache__ directories
- **Status:** complete

---

# Task Plan: Real-Robot Tracking Pose Investigation (2026-06-19)

## Goal
Determine whether Issue 1 is true: `/rematch/tracker_details` is already wrong in `odom` once the chassis moves, before any planner-facing base/planner-frame conversion.

## Scope
- Focus only on the chain up to `/rematch/tracker_details`
- Exclude planner/follower behavior
- Exclude odom-source quality as primary cause
- Use controlled experiments, logging, plots, and frame-by-frame error decomposition

## Success Criteria
- Produce a yes/no answer for: "Is `/rematch/tracker_details` already drifting in `odom` with robot motion?"
- Quantify where the error first appears:
  - camera-frame measurement stage
  - camera->odom TF stage
  - coarse/detailed tracker state stage
- Leave behind reusable scripts and a repeatable runbook

## Phases

### Phase 1: Define observables and pass/fail metrics
- [x] Fix the exact topics to compare: `GT_odom`, `GT_base`, `UMAP_camera`, `Tracker_odom`, `Tracker_base`, `/odom`, `/tf`
- [x] Define the primary metric: `Tracker_odom_vs_GT_odom`
- [x] Define secondary metrics: `UMAP_camera_vs_GT_camera`, `Tracker_base_vs_GT_base`
- [x] Define motion-coupling metrics: error vs robot displacement, error vs robot speed
- [x] Define clear verdict thresholds for "stable", "moderate drift", "clear drift"
- **Status:** complete

### Phase 2: Build controlled experiment matrix
- [x] Static robot + static obstacle baseline
- [x] Moving robot + static obstacle main test
- [x] Moving robot + perfect synthetic measurement control
- [ ] Optional holdout: moving robot + real UMAP but no rematch features
- [x] Keep obstacle count at 1 for first-pass isolation
- **Status:** in_progress

### Phase 3: Prepare data collection and diagnostics
- [x] Audit existing diagnostic launch/scripts only as candidates, not as constraints
- [x] Prefer correctness over reuse; write clean one-off diagnostics if existing tools encode stale assumptions
- [x] Identify gaps or hidden assumptions in current tooling before trusting outputs
- [x] Add one bag extraction script if current analysis is insufficient
- [x] Add one plotting/report script if current plots do not isolate Issue 1 cleanly
- [x] Standardize bag topic set and output artifact names
- **Status:** complete

### Phase 4: Execute staged tests
- [ ] Run baseline: robot static, verify all errors low
- [ ] Run main test: robot moving, static obstacle, measure `Tracker_odom` drift
- [ ] Run perfect-measurement control to isolate TF-only errors
- [ ] Compare runs under the same analysis pipeline
- [ ] Log anomalies and failed assumptions immediately
- **Status:** pending

### Phase 5: Decision logic and root-cause narrowing
- [ ] If `Tracker_odom` drifts while `UMAP_camera` stays good, blame camera->odom TF/stamp/tracker state chain
- [ ] If both `UMAP_camera` and `Tracker_odom` drift similarly, blame measurement/TF upstream
- [ ] If `Tracker_odom` is stable but `Tracker_base` is wrong, reject Issue 1 and shift to Issue 2
- [ ] Summarize evidence with plots, scalar metrics, and per-run verdicts
- **Status:** pending

## Planned Deliverables
- One short experiment runbook
- One reusable extraction/analysis script set
- Plot set:
  - timeseries of GT/tracker in `odom`
  - error vs time
  - error vs robot displacement
  - optional error vs speed
- Final verdict note with next-step recommendation

## Risks / Notes
- Existing planning files contain unrelated older work; keep this section append-only
- Do not modify planner code during Issue-1 investigation
- Prefer timestamped TF lookups in diagnostics when comparing frames
- Existing diagnostic scripts may contain historical frame assumptions from pre/post frame-honest refactors; validate before reuse

---

# Task Plan: Prediction Trust Geometry Investigation (2026-06-25)

## Goal
Read `prediction_trust_briefing.md` and investigate a principled replacement for the DOA -> T-MPC bridge's `cv_horizon` hard switch.

## Scope
- Verify the current bridge behavior and message fields.
- Verify whether T-MPC's Gaussian/probabilistic path is usable in the jackalsimulator setup.
- Verify whether DOA tracker covariance is available at `/rematch/tracker_details`.
- Compare candidate fixes and produce an implementation recommendation.
- Do not modify runtime behavior unless explicitly requested after the investigation.

## Phases

### Phase 1: Restore context and read briefing
- [x] Read existing planning files.
- [x] Read `prediction_trust_briefing.md`.
- [x] Run session catchup.
- **Status:** complete

### Phase 2: Trace code paths
- [x] Inspect `doa_to_tmpc.py` prediction/uncertainty output.
- [x] Inspect T-MPC obstacle parsing and deterministic/probabilistic constraint handling.
- [x] Inspect generated solver/build assumptions for probabilistic constraints.
- [x] Inspect DOA tracker message definitions and covariance publication.
- **Status:** complete

### Phase 3: Evaluate solution candidates
- [x] Assess Solution A: Gaussian/growing uncertainty.
- [x] Assess Solution B: smarter deterministic fallback.
- [x] Assess coupling with ID churn and billiard bounce.
- [x] Check for third-path alternatives.
- **Status:** complete

### Phase 4: Report
- [x] Append concrete findings to `findings.md`.
- [x] Provide ranked recommendation and next implementation steps.
- **Status:** complete

### Phase 5: T-MPC-input-first correction and bridge contract
- [x] Re-read current planner input consumer before using DOA covariance.
- [x] Separate active LMPCC/ellipsoid path from latent true T-MPC guidance path.
- [x] Compare DOA `TrackerDetail` output against the planner input contract.
- [x] Identify documentation/contract gaps caused by treating tracking covariance as prediction covariance.
- **Status:** complete
