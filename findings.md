# Findings

## Prediction Trust Geometry Investigation (2026-06-25)

### Correction after T-MPC-input-first reread
- Earlier notes over-interpreted DOA `sigma2_*` as if it were obstacle future-prediction covariance. That is wrong.
- `sigma2_*` in `TrackerDetail.msg` is the current post-update KF tracking covariance. It says how uncertain the tracker is about the current state estimate; it is not a behavior/model uncertainty forecast.
- Therefore, the bridge design must start from what the T-MPC input contract requires, then map DOA's current track state into that contract. It should not treat tracker covariance as a direct future obstacle distribution.

### T-MPC input contract actually observed in code
- `mpc_planner_jackalsimulator` subscribes to `/input/obstacles` as `mpc_planner_msgs/ObstacleArray`.
- `ObstacleArray` contains `ObstacleGMM[]`; each `ObstacleGMM` has:
  - `id`
  - current `pose`
  - `Gaussian[] gaussians`
  - `float64[] probabilities`
- Current ROS1 jackalsimulator wrapper supports only one Gaussian mode:
  - `probabilities.size()==0`: obstacle is kept but has no prediction, later fails readiness.
  - `probabilities.size()==1`: accepted.
  - multiple modes: hard assert.
- The wrapper ignores any obstacle radius in the message because `ObstacleGMM.msg` has no radius field. It always uses `CONFIG["obstacle_radius"]`.
- For each incoming mean pose, the wrapper copies:
  - predicted position
  - predicted yaw
  - `major_semiaxis[k]`
  - `minor_semiaxis[k]`
  into `PredictionStep`.
- If `mode.major_semiaxis.back()==0` or `probabilistic.enable=false`, the prediction type is forced to `DETERMINISTIC`; otherwise it is `GAUSSIAN`.
- Current generated jackalsimulator modules (`mpc_planner_modules/include/mpc_planner_modules/modules.h`) are **not** T-MPC guidance modules. They initialize:
  - `MPCBaseModule`
  - `GoalModule`
  - `PathReferenceVelocity`
  - `EllipsoidConstraints`
  There is no `GuidanceConstraints` in the active generated module list.
- This matches `generate_jackalsimulator_solver.py`, which currently selects `configuration_lmpcc(settings)` despite the nearby comment saying "T-MPC".

### Consumer semantics
- Active current path: `EllipsoidConstraints`.
  - Solver `setParameters()` runs for `k=0..N-1`.
  - At `k=0`, obstacle parameters are dummy values.
  - At `k=1..N-1`, it reads `prediction.modes[0][k-1]`.
  - Therefore the active ellipsoid constraint path needs at least `N-1` future prediction steps.
- If true `GuidanceConstraints` is regenerated/enabled later:
  - It loads dynamic obstacles into `GuidancePlanner::GlobalGuidance`.
  - It passes `positions = [current obstacle.position] + prediction positions`.
  - `GlobalGuidance::LoadObstacles()` requires at least `Config::N + 1`; if shorter, it extends using the last velocity between the final two positions.
  - Guidance collision checking treats obstacles as circular tubes with radius `obstacle.radius + robot_radius`; it does not use Gaussian semiaxes.
- `propagatePredictionUncertainty()` loops over `CONFIG["N"]` and accesses `prediction.modes[0][k]`. With the current bridge emitting `N-1` future poses, enabling this path can access past the end. It is currently masked because `probabilistic.enable=false` and `emit_uncertainty=false`, but it is a real gap before turning probabilistic on.
- `ensureObstacleSize()` also assumes `prediction.modes[0][k]` exists for `k=0..N-1` when sorting more than `max_obstacles` obstacles. Current scene may stay under `max_obstacles`, but this is another implicit array-length contract.

### DOA output contract
- `/rematch/tracker_details` provides current track state:
  - `tracker_id`
  - current position
  - current velocity
  - current box dimensions
  - `age`
  - `confidence`
  - current tracking covariance diagonal `sigma2_*`
- It does **not** provide:
  - future obstacle trajectory over T-MPC horizon
  - prediction model validity horizon
  - behavior/maneuver uncertainty
  - bounce/turning model
  - per-stage obstacle radius/shape for `mpc_planner_msgs`

### Gap to bridge
- T-MPC input needs a stage-indexed obstacle mean trajectory. DOA only gives a current state estimate.
- T-MPC current active safety constraint is hard deterministic/ellipsoid collision avoidance. It does not have a native "trust weight" for uncertain future predictions.
- T-MPC current message type cannot carry per-obstacle geometric radius/box dimensions to jackalsimulator; all obstacles use global `obstacle_radius`.
- DOA tracking covariance can be used as a quality signal for the current estimate, but it is not a complete future prediction covariance. If used at all, it should modulate trust conservatively rather than be directly rebranded as behavior uncertainty.
- The bridge should therefore explicitly own the prediction policy: how far and how strongly current velocity should influence the future mean trajectory.

### Generated-solver / T-MPC state
- The solver generator is not passive documentation. It rewrites:
  - `mpc_planner_modules/include/mpc_planner_modules/modules.h`
  - `mpc_planner_modules/include/mpc_planner_modules/definitions.h`
  - `mpc_planner_modules/modules.cmake`
  - `mpc_planner_solver/config/*`
  - generated parameter setters and solver code
- `mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py` currently has:
  - `configuration_tmpc(settings)` defined with `GuidanceConstraintModule(... EllipsoidConstraintModule)`
  - but the active selected line is `model, modules = configuration_lmpcc(settings)`
- To actually test T-MPC guidance behavior in jackalsimulator, the generator must select `configuration_tmpc(settings)`, then solver/modules must be regenerated and rebuilt. Launch/config flags alone will not enable guidance.
- Other packages show mixed intended patterns:
  - `mpc_planner_jackal` selects `configuration_tmpc(settings)` but its active T-MPC submodule is `GaussianConstraintModule`.
  - `mpc_planner_rosnavigation` selects `configuration_tmpc(settings)` with `EllipsoidConstraintModule`.
  This reinforces that the bridge should be designed against the actual generated module contract, not package names or comments.

### Current bridge fit against active generated solver
- `doa_to_tmpc.py` emits `N-1` future poses via `for k in range(1, cfg.N)`.
- That matches the active `EllipsoidConstraints::setParameters()` indexing for solver stages `k=1..N-1`.
- It does **not** match helper paths that assume `N` prediction entries:
  - `propagatePredictionUncertainty()`
  - `ensureObstacleSize()` sorting when obstacle count exceeds `max_obstacles`
- Since current launch has `emit_uncertainty=false` and settings have `probabilistic.enable=false`, propagation is not active. Since `max_obstacles=12` and scene 3 has 6 obstacles, the sorting overflow path is also likely inactive. This explains why the mismatch can remain latent.
- If we change bridge or settings, first decision should be: standardize prediction length and patch consumers or keep current assumptions explicitly documented.
- Current frame/timing contract is consistent:
  - DOA simulation config stamps `REMATCH_OUTPUT_FRAME: "odom"`.
  - `tmpc_doa.launch` sets bridge `target_frame=odom`.
  - bridge `N=30`, `dt=0.2` matches jackalsimulator `N=30`, `integrator_step=0.2`.
- Current launch comment already says `MPC Planner (LMPCC: GoalModule + EllipsoidConstraint)`, which matches generated modules. The briefing's "T-MPC" label is stale for this exact generated state.

### Revised design direction
- First fix the contract issues:
  1. Make prediction array length explicit and compatible with every enabled consumer path (`N-1` for current ellipsoid-only path; `N`/current+N if enabling propagation/guidance paths). Add checks rather than relying on implicit indexing.
  2. Keep `probabilistic.enable=false` until the propagation length/semantics are corrected.
  3. Treat `sigma2_*` as tracking-quality metadata, not future covariance.
- Bridge policy should be deterministic mean prediction with explicit velocity trust:
  - Inputs: current position, velocity, confidence, age, speed, optional tracking covariance quality.
  - Output: full stage-indexed mean trajectory.
  - Velocity contribution should decay smoothly with horizon or track quality, rather than binary freeze at `cv_horizon`.
  - This is a bridge-owned motion prior, not a KF covariance propagation.
- For bounce/turning:
  - Since DOA does not know the billiard boundary/reflection model, bridge should detect inconsistency (velocity flip, large innovation if exposed later, low confidence/new track) and reduce velocity trust quickly.
  - A principled bounce-aware predictor would need scene-boundary knowledge or a multi-model predictor upstream; it is outside the current DOA message contract.
- If the goal is truly "far uncertain predictions are less binding", the solver side must expose soft/stage-weighted obstacle treatment. The current hard ellipsoid path cannot express that by growing covariance.

### T-MPC-input-first follow-up
- The live jackalsimulator config has `solver_settings.solver: "acados"` and `SQP_RTI`, but the generator file still appends `~/forces_pro_client` and the repo supports both Acados and FORCES. For this bridge investigation, the important verified fact is not the backend name; it is the generated module list and solver parameter contract.
- `settings.yaml` has T-MPC parameters (`t-mpc/use_t-mpc++`, `enable_constraints`), but these are inert in the current generated module set because `GuidanceConstraints` is not initialized in `modules.h`.
- `ObstacleGMM.msg` cannot carry obstacle geometry beyond current `pose`; jackalsimulator overwrites radius from global `CONFIG["obstacle_radius"]`. DOA `box_dimensions` only affects bridge visualization markers today, not planner collision geometry.
- Current bridge tests intentionally assert `N-1` output length and the freeze behavior, so the existing unit tests encode the active ellipsoid-path contract rather than a general T-MPC contract.
- `TrackerDetail.msg` and `ObstacleTracker.h` comments correctly state that `sigma2_*` is current post-update KF covariance, but they also describe downstream propagation as if this were an established contract. The actual bridge does not implement the documented closed-form `sigma_pos(k)^2 = sigma_xx + (k*dt)^2*sigma_vxvx`; it uses only velocity variance to emit a constant rate, and this path is disabled by `emit_uncertainty=false`.
- Therefore there are two separate contract documents to fix before implementation:
  - Planner-facing bridge contract: exact prediction length, deterministic vs Gaussian activation, and whether semiaxes are per-stage radii or per-step rates.
  - DOA-facing output contract: `sigma2_*` means current tracking covariance only; any future model/trust semantics belong to the bridge or a dedicated predictor, not to the raw tracker message.

### Bridge design implication from the actual gap
- DOA output gives current `(p, v, box, age, confidence, P_track_current)`.
- Active planner input needs fixed-size obstacle list and a per-stage mean trajectory for stages `1..N-1`, plus optional semiaxes that become hard inflated ellipsoid radii if probabilistic mode is enabled.
- The missing object is not "prediction covariance"; it is a bridge-owned motion-prior function:
  - `mean_k = p + v_clamped * f(k*dt, track_quality)`
  - where `f(t, quality)` should be smooth and saturating, replacing `min(k, k_cv)*dt`.
- A concrete low-risk bridge baseline is:
  - keep deterministic semiaxes zero;
  - keep output length `N-1`;
  - replace hard freeze with a saturating displacement such as `tau * (1 - exp(-t/tau))`;
  - modulate `tau` downward for low confidence, high age, excessive speed clamp, or large current tracking covariance;
  - keep `max_age` filtering so stale tracks do not enter the planner.
- This does not pretend to be probabilistic. It simply makes the mean prediction continuous and makes far horizon motion less assertive without enlarging hard forbidden geometry.
- If future work enables true `GuidanceConstraints`, bridge output should be revisited as `[current + N future]` for guidance, or the guidance consumer should be patched to make its expected input length explicit instead of silently extending with last velocity.

### Initial verified facts
- `src/mpc_nav/scripts/doa_to_tmpc.py` currently builds one `mpc_planner_msgs/ObstacleGMM` per tracker and publishes a single Gaussian mode.
- Its prediction loop uses `k_eff = min(k, cfg.k_cv)`, so after `cv_horizon` it freezes the predicted obstacle center instead of continuing CV.
- `emit_uncertainty=false` in `src/mpc_nav/launch/tmpc_doa.launch`, so `_uncertainty_rate()` returns `0.0`; the planner receives zero semiaxes and classifies the obstacle as deterministic unless `probabilistic.enable` is also true and semiaxes are nonzero.
- `/rematch/tracker_details` does include KF covariance fields: `TrackerDetail.msg` has `sigma2_xx`, `sigma2_yy`, `sigma2_vxvx`, `sigma2_vyvy`.
- `src/DOA/src/umap/src/detailed_matcher.cpp` fills those fields from `tracker.getCovariance(0,0)`, `(1,1)`, `(3,3)`, `(4,4)`.
- `mpc_planner_jackalsimulator/config/settings.yaml` currently has `probabilistic.enable=false`, `risk=0.01`, `propagate_uncertainty=true`.
- `mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py` imports `GaussianConstraintModule` but the line adding it is commented; the active path uses `GuidanceConstraintModule(... EllipsoidConstraintModule)`.

### Additional code-path findings
- `src/mpc_planner/mpc_planner_jackalsimulator/src/ros1_jackalsimulator.cpp` parses every incoming `ObstacleGMM` as a `PredictionType::GAUSSIAN` first, then downgrades it to `DETERMINISTIC` when `mode.major_semiaxis.back()==0` or `probabilistic.enable=false`.
- `src/mpc_planner/mpc_planner_modules/src/ellipsoid_constraints.cpp` supports both deterministic and Gaussian predictions through the same generated solver parameters:
  - deterministic: `major=0`, `minor=0`, `chi=1`
  - Gaussian: `major/minor` from prediction and `chi = ExponentialQuantile(0.5, 1-risk)`
- The generated solver already contains `ellipsoid_obst_*_major`, `ellipsoid_obst_*_minor`, and `ellipsoid_obst_*_chi` parameters in `mpc_planner_solver/config/parameter_map.yaml` and generated setter functions in `mpc_planner_solver/include/mpc_planner_solver/mpc_planner_parameters.h`.
- Therefore, using the existing ellipsoid chance-constraint path should not require regenerating the solver. Regeneration would only be needed if switching to the separate `GaussianConstraintModule`, which is not the active jackalsimulator path.
- Current bridge uncertainty is only a velocity-derived per-step rate: `sqrt(max(sigma2_vxvx, sigma2_vyvy))`, clamped by `rate_floor/rate_ceil`. It ignores `sigma2_xx` and `sigma2_yy`.
- `propagatePredictionUncertainty()` in T-MPC treats incoming `major/minor` values as rates and integrates them as `sqrt(prev^2 + (rate*dt)^2)`. If the bridge sends already integrated per-stage sigma while `propagate_uncertainty=true`, uncertainty will be double-integrated.
- Other configs (`mpc_planner_jackal`, `mpc_planner_dingo`, `mpc_planner_rosnavigation`) default `probabilistic.enable=true`, which supports that the code path is intended to run, but jackalsimulator has it disabled locally.

### Solution assessment

#### A. "Enable Gaussian + growing uncertainty"
- Feasible mechanically in the current jackalsimulator code path:
  - Set `probabilistic.enable=true`.
  - Set bridge `emit_uncertainty=true`.
  - Keep using `EllipsoidConstraintModule`; do not switch to `GaussianConstraintModule`.
  - No solver regeneration is needed for this ellipsoid-based chance path.
- But the briefing's expected behavior is not what this solver implements. In `ellipsoid_constraints.py`, the constraint is:
  - outside ellipse: `(disc - obstacle)^T A (disc - obstacle) >= 1`
  - with `A` denominator `(major*sqrt(chi) + robot_radius + obstacle_radius)^2`.
  - Larger `major/minor` therefore enlarges the forbidden region and makes the hard constraint harder, not softer.
- `probabilistic.risk` is semantically meaningful but still operationally a tuning knob. Current values imply:
  - risk 0.05 -> `sqrt(chi) ~= 2.45`
  - risk 0.01 -> `sqrt(chi) ~= 3.03`
  Lower risk noticeably expands obstacles.
- Conclusion: A is useful for representing near-term covariance, but by itself it does **not** "downweight far uncertain predictions." Full-horizon CV plus growing hard ellipses is likely to worsen infeasibility unless uncertainty is capped or the solver gets soft/horizon-discounted constraints.

#### B. Smarter deterministic prediction
- This is safer as a bridge-only change because it does not enlarge hard chance constraints.
- However, simply replacing global `cv_horizon` with per-track thresholds still keeps a heuristic trust gate.
- A better bridge-only variant is a continuous velocity-trust decay:
  - Emit all `N-1` poses.
  - Replace `k_eff=min(k,k_cv)` with a smooth saturating displacement, e.g. `s(t)=tau*(1-exp(-t/tau))`, or a KF-derived decay where velocity influence decreases as predicted position variance grows.
  - Use tracker quality (`confidence`, age/update_count if exposed, covariance, innovation if exposed later) to shorten/lengthen the effective displacement per track.
- This removes the discontinuity but still leaves one or more model parameters. It is an incremental improvement, not a fundamental probabilistic solution.

#### C. Correct principled direction
- To actually model "lower trust at far horizon" in this solver, uncertainty must affect constraint weight/activation/softness, not only geometric size.
- Current generated jackalsimulator solver uses hard ellipsoid constraints and no per-stage obstacle slack in the active model. A true solution would require one of:
  - Add slack variables to ellipsoid obstacle constraints with stage-dependent and confidence-dependent penalty.
  - Add horizon/risk scheduling so far uncertain predictions can be less binding, not just larger.
  - Use a bounded reachable/occupancy tube with explicit finite support and soft penalties outside the near horizon.
- These are solver/module changes and likely require solver regeneration.

#### Coupling with ID churn and billiard bounce
- Downstream prediction cannot fully compensate for ID churn. A reborn track has reset velocity and covariance semantics; any CV, Gaussian, or deterministic scheme will receive a bad initial state.
- Prediction refactor can proceed independently only if it is robust to bad tracks by using confidence/age/covariance to avoid over-trusting newborn tracks.
- For billiard bounce, a CV-KF is structurally wrong at reflection. Growing hard uncertainty may cover the model mismatch but also tightens constraints. A practical bridge-side mitigation is maneuver detection: if velocity direction flips or innovation spikes, temporarily decay velocity trust/hold position and inflate only near-term uncertainty. A principled version is IMM/reachability, but that is larger than the bridge.

### Recommendation
- Do **not** ship "full CV + growing Gaussian uncertainty" as the main fix in the current hard-constraint T-MPC implementation.
- Recommended near-term path:
  1. Keep deterministic constraints for the main closed-loop test while removing the hard freeze discontinuity with a smooth, per-track trust decay.
  2. Gate velocity trust by track confidence/age/covariance so ID churn/new tracks cannot project confident far obstacles.
  3. Run an A/B/C test:
     - current `cv_horizon=1.5`, deterministic
     - smooth deterministic trust decay
     - Gaussian ellipsoid path with capped uncertainty, only as a measured variant
  4. In parallel, fix/verify upstream ID stability; otherwise planner metrics will conflate prediction geometry with bad velocities.
- Recommended long-term path:
  - Modify the T-MPC obstacle constraint to support soft/stage-weighted probabilistic constraints. Then the intended "continuous trust decay" can be represented honestly.




## Data Flow Trace Report (2026-05-12)

### Target Chain
```
Image → Perception Method → Tracking Results → Adaptor → MPC → Planned Path + Control
```

---

### Experiment 1: Tracking-to-Planning (`sup_tracking_to_planning.launch`)

**Runner**: `run_tracking_to_planning_sup.sh`
**MPC**: YES (active, publishes `/cmd_vel`)
**Post-process**: `process_bag.sh` → `calc_planning_metrics.py` → `visualize_keyframes.py` → `summarize_sup_results.py`

#### Method: `oracle_gt`
```
Gazebo GT (/obstacles, mpc_nav/ObstacleArray)
  → topic_to_obstacles.py (input_type=obstacle_array)
  → /sup_exp/planner_obstacles
  → MPC → /cmd_vel, /mpc/pred_path, /mpc/nav_path, /mpc/swept_capsules, /mpc/status
```
- **Adaptor**: `topic_to_obstacles.py:70-76` — relay passthrough, preserves header
- **MPC input**: `mpc_node.py:159` — `rospy.Subscriber(self.obstacles_topic, ObstacleArray, ...)` where `self.obstacles_topic = /sup_exp/planner_obstacles`
- **Verdict**: OK — direct passthrough, no conversion needed.

#### Method: `dodt`
```
/d435/depth/image_raw + /odom
  → onboard_detector/detector_node
  → /onboard_detector/tracked_bboxes (MarkerArray)
  → dodt_adapter.py
  → /sup_exp/planner_obstacles
  → MPC → /cmd_vel, /mpc/pred_path, /mpc/nav_path, /mpc/swept_capsules, /mpc/status
```
- **Perception**: DODT subscribes to `/d435/depth/image_raw` (depth-only detector, no RGB)
- **Adaptor**: `dodt_adapter.py:41` — `rospy.Subscriber(self.input_topic, MarkerArray, ...)`, input_topic = `/onboard_detector/tracked_bboxes`
- **Adaptor output**: `dodt_adapter.py:40` — `rospy.Publisher(self.output_topic, ObstacleArray, ...)`, output_topic = `/sup_exp/planner_obstacles`
- **Message conversion**: marker wireframe points → centroid + box dims; marker.text → velocity regex parse; marker.id → persistent track ID
- **Rosbag records**: `/onboard_detector/tracked_bboxes`, `/onboard_detector/dynamic_bboxes`, `/sup_exp/planner_obstacles`
- **Tracker extraction uses**: `/onboard_detector/tracked_bboxes` (line 92 of runner)
- **Verdict**: OK — chain complete.

#### Method: `fapp`
```
/d435/depth/image_raw + /d435/depth/camera_info
  → depth_to_fapp_cloud.py → /map_generator/obj_cloud (PointCloud2)
  → mot_mapping/mapping_node (odom remap: /drone_0_visual_slam/odom → /odom)
  → /states (obj_state_msgs/ObjectsStates)
  → topic_to_obstacles.py (input_type=fapp_states)
  → /sup_exp/planner_obstacles
  → MPC → /cmd_vel, /mpc/pred_path, /mpc/nav_path, /mpc/swept_capsules, /mpc/status
```
- **Bridge**: `depth_to_fapp_cloud.py` — depth image → CameraInfo unproject → TF to odom → filter ground/self → publish PointCloud2 to `/map_generator/obj_cloud`
- **FAPP**: `mapping_node` subscribes to `/map_generator/obj_cloud` (hardcoded), publishes `/states`
- **Adaptor**: `topic_to_obstacles.py:112-124` — `fapp_states_cb()` reads `state.position`, `state.velocity`, `state.size`, assigns incremental IDs
- **Verdict**: OK — chain complete. Note: FAPP ObjectsStates has no per-object persistent ID, so adaptor assigns `i` (index) as ID.

#### Method: `stage1_only` (UMAP coarse match only)
```
/d435/image_raw + /d435/depth/image_raw
  → umap_extract → /umap/3D_bounding_boxes + /umap/2D_bounding_boxes
  → coarse_match → /tracker/dynamic_obstacles (umap/Obstacles)
  → topic_to_obstacles.py (input_type=umap_obstacles)
  → /sup_exp/planner_obstacles
  → MPC → /cmd_vel, /mpc/pred_path, /mpc/nav_path, /mpc/swept_capsules, /mpc/status
```
- **Note**: NO detailed_match, NO para_samnode/clipnode/aggregator, NO doa_to_obstacles
- **Adaptor**: `topic_to_obstacles.py:96-110` — `umap_obstacles_cb()` reads `msg.box_ids`, `msg.centroid_x/y/z`, `msg.width/height`, velocity estimated from position delta
- **Verdict**: OK.

#### Method: `ours_full`
```
/d435/image_raw + /d435/depth/image_raw
  → umap_extract → /umap/3D_bounding_boxes + /umap/2D_bounding_boxes
  → coarse_match → /tracker/dynamic_obstacles
  → [fanout] para_samnode.py → /model_output/obstacles_sam
             para_clipnode.py → /model_output/obstacles_clip
  → para_aggregate.py (time-sync: SAM + CLIP) → /model_output/obstacles
  → detailed_match → /rematch/tracker_details (umap/TrackerDetailArray)
  → doa_to_obstacles.py (mpc_nav) → /sup_exp/planner_obstacles
  → MPC → /cmd_vel, /mpc/pred_path, /mpc/nav_path, /mpc/swept_capsules, /mpc/status
```
- **Aggregator**: `para_aggregate.py` time-synchronizes `/model_output/obstacles_sam` + `/model_output/obstacles_clip`, merges SAM spatial fields + CLIP features → `/model_output/obstacles`
- **Adaptor (doa)**: `doa_to_obstacles.py` subscribes to `/rematch/tracker_details` (default `~input_topic`), publishes to `/sup_exp/planner_obstacles` (set via `~output_topic`). Filters by age (`max_age=0.5`), transforms odom→odom (identity, no-op), builds ellipse footprint aligned with velocity.
- **Verdict**: OK — full chain complete.

#### Method: `ours_no_clip`
```
... (same as ours_full until coarse_match)
  → para_samnode.py → /model_output/obstacles_sam
  → no_clip_dummy_features.py → /model_output/obstacles (adds dummy CLIP features [1.0]*dim)
  → detailed_match → /rematch/tracker_details
  → doa_to_obstacles.py → /sup_exp/planner_obstacles
  → MPC → ...
```
- **SAM only**: para_clipnode NOT launched. para_aggregate NOT launched.
- **Bridge**: `no_clip_dummy_features.py` takes SAM output (no CLIP features), fills dummy `Float32MultiArray` per obstacle, publishes to `/model_output/obstacles`
- **Rematch config**: SIMILARITY_THRESHOLD=-1.0, SIMILARITY_WEIGHT=0.0 → CLIP similarity effectively disabled
- **Verdict**: OK.

#### Method: `ours_no_sam` / `ours_no_fastsam`
```
... (same as ours_full until coarse_match)
  → para_clipnode.py → /model_output/obstacles_clip (CLIP features, NO SAM spatial fields)
  → topic_tools relay → /model_output/obstacles
  → detailed_match → /rematch/tracker_details
  → doa_to_obstacles.py → /sup_exp/planner_obstacles
  → MPC → ...
```
- **CLIP only**: para_samnode NOT launched. para_aggregate NOT launched.
- **⚠️ POTENTIAL ISSUE**: `para_clipnode.py` publishes `Obstacles` with CLIP features but WITHOUT SAM spatial fields (binary_mask, contour_area, etc.). The `topic_tools relay` forwards this directly to `/model_output/obstacles`. If `detailed_match` requires SAM spatial fields for cost computation, this path would produce degraded (possibly broken) cost matrices. This is intentional for ablation, but should be verified that detailed_match handles missing SAM fields gracefully.
- **Verdict**: Chain is CORRECT for intended ablation, but verify detailed_match robustness.

---

### Experiment 2: Occlusion Re-ID (`sup_occlusion_reid.launch`)

**Runner**: `run_occlusion_reid_sup.sh`
**MPC**: NO (not launched)
**Post-process**: `process_bag.sh` → `calc_occlusion_reid.py` → `visualize_keyframes.py` → `summarize_sup_results.py`

This is a **perception-only** experiment. No MPC, no doa_to_obstacles. The trackers' raw output is recorded and analyzed offline for Re-ID accuracy.

#### Method: `dodt` (occlusion)
```
/d435/depth/image_raw + /odom
  → onboard_detector/detector_node
  → /onboard_detector/dynamic_bboxes + /onboard_detector/tracked_bboxes
```
- **NO adaptor** — no dodt_adapter.py, no MPC
- **Tracker extraction uses**: `/onboard_detector/dynamic_bboxes` (NOT tracked_bboxes)
- **⚠️ DISCREPANCY**: Exp 1 uses `/onboard_detector/tracked_bboxes` for tracker extraction; Exp 2 uses `/onboard_detector/dynamic_bboxes`. These are different DODT output topics. `extract_tracker.py` handles both identically (both are MarkerArray, centroid+velocity parsed same way, `marker.id` used as track ID). The difference is in what DODT publishes: `tracked_bboxes` likely contains Kalman-filtered tracks with persistent IDs across frames, while `dynamic_bboxes` may be raw per-frame detections with fresh IDs each frame. For Re-ID metrics (which need stable IDs across occlusion), `dynamic_bboxes` would NOT preserve IDs through occlusion — making Re-ID evaluation impossible for DODT.
- **Verdict**: Chain OK but **topic choice needs verification** — should Exp 2 also use `tracked_bboxes`?

#### Method: `fapp` (occlusion)
```
/d435/depth/image_raw + /d435/depth/camera_info
  → depth_to_fapp_cloud.py → /map_generator/obj_cloud
  → mot_mapping/mapping_node → /states
  → topic_to_obstacles.py (input_type=fapp_states) → /sup_exp/fapp_obstacles
```
- **Adaptor output**: `/sup_exp/fapp_obstacles` (NOT `/sup_exp/planner_obstacles` — no MPC to feed)
- **Tracker extraction uses**: `/states` directly (via `--topic /states --no-cluster`)
- **NOTE**: `/sup_exp/fapp_obstacles` is published but NOT recorded. Tracker extraction uses raw `/states` (ObjectsStates). Confirmed: `extract_tracker.py:165-176` handles this — iterates `msg.states`, uses index as ID.
- **Verdict**: OK for perception-only, but verify `extract_tracker.py` handles `ObjectsStates`.

#### Method: `ours_full` / `ours_no_clip` / `ours_no_sam` (occlusion)
```
... (same UMAP pipeline as Exp 1)
  → detailed_match → /rematch/tracker_details
```
- **NO doa_to_obstacles.py** (not launched)
- **Tracker extraction uses**: `/rematch/tracker_details` (TrackerDetailArray)
- **Rosbag records**: `/rematch/tracker_details`, `/tracker/dynamic_obstacles`, `/doa_obstacles` (but `/doa_obstacles` won't have data since doa_to_obstacles is not launched)
- **⚠️ NOTE**: `/doa_obstacles` is in the rosbag topics list but will be empty — `doa_to_obstacles.py` is not launched. This wastes bag space. Consider removing from occlusion topic list.
- **Verdict**: OK.

#### Method: `stage1_only` (occlusion)
```
... (umap_extract + coarse_match only)
  → coarse_match → /tracker/dynamic_obstacles
```
- **NO detailed_match, NO doa_to_obstacles**
- **Tracker extraction uses**: `/tracker/dynamic_obstacles` (umap/Obstacles)
- **Verdict**: OK.

---

### Experiment 3: Corridor Planning (`sup_corridor_planning.launch`)

**Runner**: `run_corridor_planning_table.sh`
**MPC**: YES (active)
**Scene**: `dync_scene1.py` (corridor mode)
**Post-process**: `calc_planning_metrics.py` + `calc_runtime_table.py` per run → `summarize_sup_results.py` → `make_corridor_tables.py`

The data flow for each method is **identical to Exp 1** (same launch groups, same adaptors, same MPC). The only differences are:
- Scene: `dync_scene1.py` with corridor params (bi-directional pedestrian flow)
- Goal: `fixed_goal_publisher.py` publishes fixed goal to `/move_base_simple/goal` after `start_delay`
- Extra topics recorded: `/rematch_statistics*`, `/mpc/status`, `/mpc/nav_path`, `/mpc/swept_capsules`
- No `process_bag.sh` — corrider uses `calc_planning_metrics.py` + `calc_runtime_table.py` directly from bag

All method data flows verified same as Exp 1 — no new issues.

---

### Cross-Cutting Issues Found

#### 1. ⚠️ DODT tracker topic mismatch (Exp 1 vs Exp 2)
| Experiment | Tracker extraction topic |
|------------|-------------------------|
| Exp 1 (tracking-to-planning) | `/onboard_detector/tracked_bboxes` |
| Exp 2 (occlusion Re-ID) | `/onboard_detector/dynamic_bboxes` |

`tracked_bboxes` = tracked output (MarkerArray with persistent IDs via marker.id)
`dynamic_bboxes` = likely raw per-frame detections (may not preserve IDs across frames)

For Re-ID evaluation, **tracked_bboxes** is likely the correct choice since it preserves track IDs through the occlusion. Using `dynamic_bboxes` would give each detection a new ID every frame, making Re-ID evaluation meaningless.

#### 2. ⚠️ `/doa_obstacles` in occlusion rosbag topics but never published
`run_occlusion_reid_sup.sh` records `/doa_obstacles` (line 114), but `sup_occlusion_reid.launch` does NOT launch `doa_to_obstacles.py`. This topic will be empty in every occlusion bag, wasting storage.

#### 3. ⚠️ `ours_no_sam` / `ours_no_fastsam` — missing SAM spatial fields
The `topic_tools relay` forwards `/model_output/obstacles_clip` (CLIP-only, no SAM fields) directly to `/model_output/obstacles`. If `detailed_match` expects SAM spatial fields (contour_area, binary_mask, etc.), this path may produce degraded matching. Verify `detailed_match` handles this gracefully.

#### 4. ✅ MPC interface convergence verified
ALL methods that involve MPC converge on the same topic: `/sup_exp/planner_obstacles` → MPC subscribes via `~obstacles_topic` param. Each adaptor outputs `mpc_nav/ObstacleArray` with consistent field population:
- `obs.id`: persistent track ID
- `obs.position`: geometry_msgs/Point (world frame)
- `obs.velocity`: geometry_msgs/Vector3 (world frame)
- `obs.radius`, `obs.semi_major`, `obs.semi_minor`: footprint ellipse
- `obs.confidence`: 0.0–1.0
- `obs.sigma2_xx/yy/vxvx/vyvy`: KF covariance diagonal

#### 5. ✅ Rosbag topics cover all data sources
Each runner records the union of all possible tracker outputs + MPC outputs, so no data is missing for any method. Verified:
- Exp 1 records: all 5 method tracker topics + MPC outputs + TF + odom + GT
- Exp 2 records: all 5 method tracker topics + TF + odom + GT (no MPC — by design)

---

## Issue-1 Diagnostic Audit (2026-06-19)

### Goal
Build a focused pipeline to answer only this question:

`/rematch/tracker_details` 是否在 `odom` 中已经随着机器人运动发生漂移？

### Audit Outcome
- `analyze_frame_errors.py` is useful as a reference, but it is too broad for Issue 1.
- Existing Stage-1/Stage-2 launches split the perfect-measurement and real-UMAP paths across different entrypoints.
- For Issue 1, we need one narrow experiment surface with explicit control over:
  - measurement source: `real_umap` vs `perfect`
  - robot motion: static vs moving
  - obstacle: single static obstacle
  - outputs: `GT_camera`, `GT_odom`, `GT_base`, `UMAP_camera`, `coarse_odom`, `tracker_odom`, `tracker_base`

### New Purpose-Built Pipeline Added
- Launch: `src/exp/launch/issue1_static_obstacle.launch`
- Runner: `src/exp/scripts/run_issue1_diag.sh`
- Analysis: `src/exp/scripts/analyze_issue1_tracking_drift.py`
- Config: `src/exp/configs/issue1.yaml`

### Why a New Analysis Script Was Added
The old analysis path was not rejected because it was wrong; it was rejected because it mixed multiple diagnostic questions together. The new script is narrower and adds:
- direct comparison of `coarse_odom` vs `GT_odom`
- explicit `Tracker_odom` drift verdict
- error vs robot displacement plots
- a bag-level `issue1_verdict` and `root_cause_hint`

### Current Intended Experiment Matrix
1. `real_umap`, `robot_speed=0.0`
   - static baseline
2. `real_umap`, `robot_speed>0`
   - reproduce motion-induced error
3. `perfect`, `robot_speed>0`
   - isolate TF/stamp/tracker-state effects from real measurement noise

### Coarse-Match Failure Mode (2026-06-19)

Using the repaired true-static bag plus the move-real bag:

- `UMAP` consistently contains a GT-near candidate
  - static bag: 187 GT-near UMAP frames
  - move-real bag: 122 GT-near UMAP frames
- but `/tracker/dynamic_obstacles` almost never publishes that GT-near candidate
  - static bag: only 1 GT-near coarse-published frame
  - move-real bag: only 16 GT-near coarse-published frames

This means the error is not primarily "TF transforms a good candidate into a bad candidate".
Instead, the GT-near candidate is present upstream and gets dropped before or during
coarse dynamic publication, while other wrong candidates are preserved.

#### Concrete evidence
- New script added:
  - `src/exp/scripts/analyze_coarse_candidate_drop.py`
- Outputs:
  - `issue1_static_coarse_candidate_drop.csv`
  - `issue1_move_real_coarse_candidate_drop.csv`

#### Specific logic hazards identified

1. **New trackers are initialized as dynamic immediately**
   - `src/DOA/src/umap/include/umap/ObstacleTracker.cpp:89`
   - `src/DOA/src/umap/include/umap/ObstacleTracker.cpp:110`
   - `isDynamic = true` on init means brand-new trackers can enter the
     "confirmed dynamic" publish path before sustained-velocity hysteresis
     has actually been satisfied.

2. **Confirmed-dynamic publish path emits any matched tracker with `isDynamic=true` as confidence 1.0**
   - `src/DOA/src/umap/src/coarse_match_visualization.cpp:279-337`
   - This is how false side fragments can seed downstream rematch as
     apparently confirmed dynamic tracks.

3. **Candidate early-publish path only keeps matched trackers with sufficiently large KF velocity**
   - `src/DOA/src/umap/src/coarse_match_visualization.cpp:410-450`
   - The GT-near static obstacle candidate tends to have low velocity and
     therefore disappears from coarse output, while side fragments with
     larger apparent motion survive as confidence 0.5/1.0.

4. **Dynamic status logic depends on sustained velocity + average movement**
   - `src/DOA/src/umap/src/coarse_matcher.cpp:11-100`
   - For a static world obstacle, the correct candidate is naturally weak
     under this gate, while ego-motion-induced side fragments can appear
     spuriously dynamic.

#### Implication
- Current evidence does **not** support TF-tree delay / odom-frame transform error
  as the main source of the large planner-facing bias.
- Current evidence **does** support a coarse-stage candidate selection /
  dynamic-gating problem that amplifies error between:

  `UMAP camera detections -> coarse /tracker/dynamic_obstacles -> rematch`
- Exp 3 records: all 5 method tracker topics + MPC outputs + TF + odom + runtime stats

---

### Summary of Action Items

| # | Severity | Issue | Recommendation |
|---|----------|-------|----------------|
| 1 | **HIGH** | DODT topic mismatch: Exp 2 uses `dynamic_bboxes` instead of `tracked_bboxes` | ✅ Fixed — changed to `/onboard_detector/tracked_bboxes` |
| 2 | **LOW** | `/doa_obstacles` recorded but never published in occlusion | ✅ Fixed — removed from occlusion TOPICS array |
| 3 | **LOW** | `ours_no_sam` relay may lack SAM fields | Verify detailed_match robustness (likely OK for ablation) |
| 4 | **INFO** | FAPP adaptor in occlusion publishes to unused topic | Expected — perception-only experiment, adaptor is for recording consistency |

---

## Fixes Applied Before Re-Run (2026-05-11)

1. **DODT sim overrides** — Added depth/aligned_depth topic + localization_mode + odom topic params to occlusion launch
2. **FAPP adapters** — Replaced sim_mapping.launch with direct mapping_node (odom remap) + depth_to_fapp_cloud.py bridge
3. **Occlusion timing** — STARTUP_WAIT: 8→14, START_DELAY: 12→28, before_window default: 1.0→3.0
4. **calc_occlusion_reid.py classification** — Decoupled missing_before/missing_after counters
5. **MOTA NaN fix** — calc_metrics.py returns NaN when total_gt==0 instead of misleading 1.0
6. **summarize_sup_results.py multi-row fix** — first_csv_row() → all_csv_rows(), added occlusion CSVs to DEFAULT_METRICS
7. **ours_no_sam placeholder** — Added to both launch files, both bash runners, figure script
8. **Random scene FOV bounds** — dync_obs.launch scene 0: x=[-2,2]→[0.5,5.0], y=[0,10]→[-3,3]
9. **visualize_keyframes.py** — New script for top-down keyframe rendering, auto-called from both runners

## FAPP Debug (2026-05-11) — CRITICAL

### Root Cause
`/d435/depth/points` PointCloud2 is **NOT published** by the `gazebo_ros_openni_kinect` plugin. Only depth/color images and camera_info are available. The original `depth_to_fapp_cloud.py` subscribed to a non-existent point cloud topic, so FAPP received no input.

### Fix
Rewrote `depth_to_fapp_cloud.py` to:
1. Subscribe to depth image (`/d435/depth/image_raw`, 32FC1) + camera info (`/d435/depth/camera_info`)
2. Unproject depth pixels → 3D points in camera optical frame via camera intrinsics
3. Transform to odom frame using TF (numpy-based quat→rot matrix, no tf2_sensor_msgs dependency)
4. Filter ground (z < -0.15m) and robot body (r < 0.35m)
5. Downsample by step=4 (160×120 = 19200 pts max per frame)
6. Publish XYZ PointCloud2 to `/map_generator/obj_cloud`

### Verification needed
- Confirm `/d435/depth/camera_info` is published (likely yes since Gazebo plugin provides it)
- Test FAPP with new bridge after Exp 1 completes
- FAPP already had data (empty tracker) in 2 Exp 1 runs — need to re-run after fix

## Experiment 1 Re-Run Progress (2026-05-11)
- pair_crossing: dodt ✓ fapp (empty) ours_no_clip ✓ ours_no_sam ✓ ours_full ✓ stage1_only (in progress)
- front_crossing: pending
- random_multi: pending

---

# Exp Scripts Investigation Report (2026-05-12)

## 1. Current State: 28 script files, ~5700 lines

### Script Inventory by Role

#### Scene Scripts (场景) — 1 file
| File | Lines | Purpose |
|------|-------|---------|
| `occlusion_reid_scene.py` | 233 | Controlled occlusion Re-ID scene (spawns Gazebo models, drives actors, publishes GT) |

Note: Other scene scripts (`dync_scene1.py`, `sim_exp_1.py`, etc.) live in `src/my_tb3_description/` or `src/mpc_nav/`, not in exp/.

#### Adaptor Scripts (适配器) — 5 files
| File | Lines | Purpose |
|------|-------|---------|
| `topic_to_obstacles.py` | 213 | **Primary** multi-format → ObstacleArray adapter (5 input types: obstacle_array, tracker_detail, umap_obstacles, fapp_states, marker_array) |
| `dodt_adapter.py` | 107 | DODT MarkerArray → ObstacleArray adapter (**subset of topic_to_obstacles.py**) |
| `no_clip_dummy_features.py` | 44 | Fills dummy CLIP features for "Ours w/o CLIP" ablation |
| `fixed_goal_publisher.py` | 72 | Publishes fixed PoseStamped goal for automated experiments |
| `depth_to_fapp_cloud.py` | 159 | Depth image → FAPP point cloud bridge |

#### Statistics & Visualization Scripts (统计与可视化) — 14 files
| File | Lines | Purpose |
|------|-------|---------|
| `extract_gt.py` | 40 | Extract GT from /obstacles topic in rosbag → CSV |
| `extract_tracker.py` | 344 | Extract tracker from rosbag (handles 5+ msg types, TF transform, clustering) → CSV |
| `calc_metrics.py` | 267 | Tracking metrics: MOTA, IDSW, FP, FN, epos (with FOV filtering) |
| `calc_prediction_error.py` | 126 | Prediction RMSE at multiple lookahead horizons |
| `calc_planning_metrics.py` | 477 | Planning safety metrics: clearance, goal success, MPC violations, BEV figure |
| `calc_occlusion_reid.py` | 295 | Occlusion Re-ID metrics: IDSW, Re-ID success rate, post-occlusion prediction |
| `calc_runtime_table.py` | 203 | Per-module runtime statistics from /rematch_statistics* topics |
| `summarize_sup_results.py` | 110 | Aggregate per-run CSVs → summary tables across methods/scenes |
| `make_corridor_tables.py` | 222 | Build final corridor planning + module runtime tables from per-run CSVs |
| `plot_results.py` | 219 | Single-run figures (prediction RMSE, MOTA decomp, BEV trajectory) |
| `gen_compare.py` | 338 | Multi-method IEEE comparison figures + markdown report |
| `gen_report.py` | 252 | **Near-duplicate** of gen_compare.py (same plots, different styling, has PLACEHOLDER markers) |
| `compare_bench.py` | 244 | **Hardcoded** Ours vs DODT for crossing scene only (subset of gen_compare.py) |
| `sup_exp_figures.py` | 566 | Supplemental experiment comprehensive figures + markdown report |
| `visualize_keyframes.py` | 391 | Top-down keyframe rendering from rosbag |

#### Runner Scripts (运行脚本) — 7 files
| File | Lines | Purpose |
|------|-------|---------|
| `run_tracking_to_planning_sup.sh` | 156 | **Current** Exp 1 runner (parameterized, multiple methods/scenes) |
| `run_occlusion_reid_sup.sh` | 166 | **Current** Exp 2 runner (parameterized, multiple durations/methods) |
| `run_corridor_planning_table.sh` | 256 | **Current** Exp 3 runner (parameterized, multiple methods) |
| `run_all_sup_exp.sh` | 52 | Master script: runs Exp 1 + Exp 2 + figures sequentially |
| `run_exp.sh` | 59 | **Legacy**: Launch single scene, record bag (hardcoded "ours" method) |
| `run_onboard_exp.sh` | 52 | **Legacy**: Launch DODT scene, record bag (hardcoded "dodt" method) |
| `process_bag.sh` | 46 | **Current**: Post-process pipeline (extract → calc metrics → plot) |

#### Documentation — 3 files
| File | Purpose |
|------|---------|
| `commands.md` | Original usage guide (references legacy run_exp.sh) |
| `SUPPLEMENTAL_EXPERIMENTS.md` | Current usage guide for all 3 experiments |
| `plan.md` | Original experiment design plan (paper planning) |

## 2. Redundancy Analysis

### HIGH REDUNDANCY — Strong candidates for deletion/merge

#### A. `gen_report.py` ≈ `gen_compare.py` (near-duplicate)
Both scripts:
- Accept `--scene`, `--methods name:path`, `--output` arguments
- Load metrics.csv, prediction_error.csv, tracker.csv, gt.csv
- Generate: prediction comparison line chart, MOTA decomposition bar chart, BEV comparison plot
- Generate markdown report with metrics tables

Differences:
- `gen_compare.py` (338 lines): Better IEEE styling, COLOR/MARKER dicts per-method, adds MOTA summary chart, cleaner code
- `gen_report.py` (252 lines): Has `# PLACEHOLDER_FUNCS` and `# PLACEHOLDER_MAIN` markers suggesting incomplete template state, simpler styling, no MOTA summary

**Verdict**: Keep `gen_compare.py`, delete `gen_report.py`. `gen_compare.py` is the more complete and polished version.

#### B. `dodt_adapter.py` ⊂ `topic_to_obstacles.py` (superset relationship)
- `dodt_adapter.py` (107 lines): DODT MarkerArray → ObstacleArray, dedicated node
- `topic_to_obstacles.py` (213 lines): Generic adapter handling 5 input types, including `marker_array` type which handles DODT's MarkerArray output identically

`topic_to_obstacles.py`'s `marker_array_cb()` does everything `dodt_adapter.py` does:
- Parse marker points for centroid
- Parse velocity from marker.text regex
- Estimate velocity from position delta if marker has no text
- Build Obstacle with footprint, confidence

**Verdict**: Can consolidate. However, `dodt_adapter.py` is referenced by name in two launch files (`sup_tracking_to_planning.launch:33`, `sup_corridor_planning.launch:67`). Options:
  1. Replace launch references with `topic_to_obstacles.py` + `input_type:=marker_array` params
  2. Keep `dodt_adapter.py` as a thin wrapper that imports from `topic_to_obstacles`

#### C. `compare_bench.py` ⊂ `gen_compare.py` (hardcoded subset)
- `compare_bench.py` (244 lines): Hardcodes Ours vs DODT for crossing scene at fixed paths (`exp/data/crossing/ours/run_1`), generates combined CSVs + comparison figure
- `gen_compare.py` (338 lines): Parameterized for any scene/methods via `--methods name:path`

**Verdict**: Delete `compare_bench.py`. All its functionality is covered by `gen_compare.py --scene crossing --methods "Ours:exp/data/crossing/ours/run_1" "DODT:exp/data/crossing/dodt/run_1"`.

### MEDIUM REDUNDANCY — Legacy + partial overlap

#### D. `run_exp.sh` / `run_onboard_exp.sh` → superseded by new runners
The new supplemental experiment runners cover all use cases:
- `run_tracking_to_planning_sup.sh` subsumes both `run_exp.sh` (run ours) and `run_onboard_exp.sh` (run DODT)
- New runners are parameterized, support multiple methods, and include post-processing

Key differences:
- Old scripts: hardcode topic lists, data paths (`data/<scene>/ours/run_<n>`)
- New scripts: flexible `--methods`/`--scenes`/`--repeats` parameters, auto post-processing

**Verdict**: Deprecate. Add a deprecation header and redirect users to use the new runners.

#### E. `plot_results.py` partially overlaps with `gen_compare.py`
- `plot_results.py`: Single-run figures (reads one data_dir, produces prediction RMSE + MOTA decomp + BEV for Ours only)
- `gen_compare.py`: Multi-method comparison (reads multiple method dirs, same plot types but with multiple lines/bars)

**Verdict**: Keep both. `plot_results.py` has a valid quick "plot one run" use case. Consider adding `--methods` support to `plot_results.py` later to make it a superset.

### LOW REDUNDANCY — Distinct purpose

- All metric computation scripts are distinct: `calc_metrics.py` (tracking), `calc_prediction_error.py` (prediction), `calc_planning_metrics.py` (planning), `calc_occlusion_reid.py` (Re-ID), `calc_runtime_table.py` (runtime)
- `summarize_sup_results.py` and `make_corridor_tables.py` serve different aggregation levels
- `sup_exp_figures.py` is specialized for the supplemental experiments' paper figures
- `visualize_keyframes.py` is unique (keyframe rendering)
- `process_bag.sh` is still used by Exp 1 and Exp 2 runners for offline post-processing

## 3. Proposed Three-Dimension Organization

```
exp/scripts/
├── scene/                          # 场景
│   └── occlusion_reid_scene.py     # Controlled occlusion scene
│
├── adaptor/                        # 适配器
│   ├── topic_to_obstacles.py       # Multi-format → ObstacleArray (primary)
│   ├── no_clip_dummy_features.py   # Dummy CLIP features for ablation
│   ├── fixed_goal_publisher.py     # Fixed goal for automated experiments
│   └── depth_to_fapp_cloud.py      # Depth → FAPP point cloud bridge
│
├── stats_viz/                      # 统计与可视化
│   ├── extract_gt.py               # GT extraction
│   ├── extract_tracker.py          # Tracker extraction
│   ├── calc_metrics.py             # Tracking metrics (MOTA, IDSW, etc.)
│   ├── calc_prediction_error.py    # Prediction RMSE
│   ├── calc_planning_metrics.py    # Planning safety metrics
│   ├── calc_occlusion_reid.py      # Occlusion Re-ID metrics
│   ├── calc_runtime_table.py       # Module runtime table
│   ├── summarize_sup_results.py    # Summary aggregation
│   ├── make_corridor_tables.py     # Corridor planning + runtime tables
│   ├── plot_results.py             # Single-run figures
│   ├── gen_compare.py              # Multi-method comparison figures (primary)
│   ├── sup_exp_figures.py          # Supplemental experiment paper figures
│   └── visualize_keyframes.py      # Keyframe rendering
│
├── run_tracking_to_planning_sup.sh # Exp 1 runner
├── run_occlusion_reid_sup.sh       # Exp 2 runner
├── run_corridor_planning_table.sh  # Exp 3 runner
├── run_all_sup_exp.sh              # Master runner
└── process_bag.sh                  # Post-processing pipeline

# TO DELETE:
#   dodt_adapter.py        → merged into topic_to_obstacles.py
#   gen_report.py          → duplicate of gen_compare.py
#   compare_bench.py       → subset of gen_compare.py
#   run_exp.sh             → legacy, superseded
#   run_onboard_exp.sh     → legacy, superseded
```

## 4. Impact Analysis — Files that need updating after reorganization

### Launch files (reference scripts by `type=` which resolves via ROS package):
- `sup_tracking_to_planning.launch`: references `dodt_adapter.py` and `topic_to_obstacles.py`
- `sup_corridor_planning.launch`: references `dodt_adapter.py` and `topic_to_obstacles.py`
- `sup_occlusion_reid.launch`: references `topic_to_obstacles.py`

### Shell runners (reference scripts by path):
- `run_tracking_to_planning_sup.sh`: calls `process_bag.sh`, `summarize_sup_results.py`, `calc_planning_metrics.py`, `visualize_keyframes.py`
- `run_occlusion_reid_sup.sh`: calls `process_bag.sh`, `calc_occlusion_reid.py`, `summarize_sup_results.py`, `visualize_keyframes.py`
- `run_corridor_planning_table.sh`: calls `calc_planning_metrics.py`, `calc_runtime_table.py`, `summarize_sup_results.py`, `make_corridor_tables.py`
- `run_all_sup_exp.sh`: calls `run_tracking_to_planning_sup.sh`, `run_occlusion_reid_sup.sh`, `sup_exp_figures.py`

### CMakeLists.txt:
- Currently installs all scripts flat to `lib/sup_exp/`. ROS `type=` resolution uses the devel space lib path, so subdirectories in scripts/ need corresponding install rules.

### SUPPLEMENTAL_EXPERIMENTS.md:
- References script paths like `./exp/scripts/run_tracking_to_planning_sup.sh`

## 5. Summary of Proposed Changes

| Action | Files | Lines affected |
|--------|-------|---------------|
| DELETE | `gen_report.py`, `compare_bench.py`, `run_exp.sh`, `run_onboard_exp.sh` | ~607 lines removed |
| MERGE | `dodt_adapter.py` → `topic_to_obstacles.py` (or keep as thin wrapper) | ~107 lines removed if fully merged |
| MOVE | 13 files into `scene/`, `adaptor/`, `stats_viz/` subdirs | Path updates in 7+ files |
| CLEAN | Remove `__pycache__/` directories | 4 .pyc files |
| UPDATE | `commands.md` → redirect to `SUPPLEMENTAL_EXPERIMENTS.md` | ~10 lines |
| UPDATE | Launch files if dodt_adapter merged | 2-3 lines each |
