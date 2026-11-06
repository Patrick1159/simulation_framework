# Supplemental Experiment Operation Guide

Run commands from the `simulation_framework` workspace root:

```bash
cd /home/patrick/sim_dev/simulation_framework
```

The new runners automatically set `ROS_PACKAGE_PATH` so the experiment-local
ROS package `sup_exp` can be found. All generated results are written under
`exp/data/`.

## Experiment 1: Tracking-to-Planning

Purpose: compare different tracking inputs with the same MPC and report
collision/near-collision rate, minimum dynamic-obstacle clearance, and
prediction RMSE/ADE-style statistics.

Smoke run:

```bash
./exp/scripts/run_tracking_to_planning_sup.sh \
  --methods "ours_full oracle_gt" \
  --scenes "front_crossing:3" \
  --duration 60 \
  --repeats 1
```

Paper-style run:

```bash
./exp/scripts/run_tracking_to_planning_sup.sh \
  --methods "dodt fapp ours_no_clip ours_full oracle_gt" \
  --scenes "pair_crossing:2 front_crossing:3 random_multi:0" \
  --duration 60 \
  --repeats 3
```

Outputs:

```text
exp/data/sup_tracking_to_planning/<scene>/<method>/run_<n>/
  raw.bag
  gt.csv
  tracker.csv
  metrics.csv
  prediction_error.csv
  planning_metrics.csv
  clearance_timeseries.csv

exp/data/sup_tracking_to_planning/
  summary_metrics.csv
  summary_prediction_error.csv
  summary_planning_metrics.csv
```

Direct launch for interactive debugging:

```bash
export ROS_PACKAGE_PATH="$PWD:${ROS_PACKAGE_PATH:-}"
roslaunch exp/launch/sup_tracking_to_planning.launch method:=ours_full scene:=3 use_rviz:=true
```

Method notes:

- `ours_full`: full DOA two-stage tracking, DOA adapter, same MPC.
- `ours_no_clip`: disables CLIP matching by setting `SIMILARITY_THRESHOLD=-1`
  and `SIMILARITY_WEIGHT=0`, then relays SAM output through
  `no_clip_dummy_features.py` so rematch still receives one neutral feature
  vector per obstacle.
- `stage1_only`: uses coarse `/tracker/dynamic_obstacles` through the
  experiment adapter.
- `oracle_gt`: relays Gazebo GT `/obstacles` into the same MPC.
- `dodt`: expects `onboard_detector` to publish marker output.
- `fapp`: expects FAPP to publish `/states`; the experiment adapter converts it
  to `ObstacleArray`. The runner checks whether `obj_state_msgs` is importable
  and runs `catkin build obj_state_msgs` when needed. If the FAPP stack is not
  already wired to the current Gazebo scene, launch it separately or treat this
  as the post-processing hook.

DODT uses the local `onboard_detector/cfg/detector_param.yaml` file. There is
no `detector_param_sim.yaml` in this workspace.

## Experiment 2: Controlled Occlusion + Re-ID

Purpose: vary occlusion duration and report ID switches, Re-ID success rate,
and prediction RMSE after reappearance.

Smoke run:

```bash
./exp/scripts/run_occlusion_reid_sup.sh \
  --methods "ours_full ours_no_clip stage1_only" \
  --durations "1.0 2.0" \
  --record-duration 35 \
  --repeats 1
```

Paper-style run:

```bash
./exp/scripts/run_occlusion_reid_sup.sh \
  --methods "stage1_only ours_no_clip ours_no_fastsam ours_full dodt fapp" \
  --durations "0.5 1.0 2.0 3.0 4.0" \
  --record-duration 35 \
  --repeats 3
```

Outputs:

```text
exp/data/sup_occlusion_reid/duration_<x>s/<method>/run_<n>/
  raw.bag
  gt.csv
  tracker.csv
  metrics.csv
  prediction_error.csv
  occlusion_reid_metrics.csv
  occlusion_identity_events.csv
  occlusion_prediction_error.csv

exp/data/sup_occlusion_reid/
  summary_metrics.csv
  summary_prediction_error.csv
  summary_occlusion_reid_metrics.csv
```

Direct launch for interactive debugging:

```bash
export ROS_PACKAGE_PATH="$PWD:${ROS_PACKAGE_PATH:-}"
roslaunch exp/launch/sup_occlusion_reid.launch method:=ours_full occlusion_duration:=2.0 use_rviz:=true
```

## Experiment 3: Corridor Planning-Level + Runtime Tables

Purpose: compare DODT/FAPP/DOA ablations under the same corridor MPC task and
produce the two requested tables:

```text
exp/data/sup_corridor_planning/planning_level_table.csv
exp/data/sup_corridor_planning/module_runtime_table.csv
```

Scene: 5 cylinders cross the corridor at linearly-spaced speeds
(0.2–0.9 m/s, wrap-around mode). The planning benchmark disables the physical
side walls by default because the UMAP coarse detector otherwise locks onto
the walls and stops publishing useful dynamic-obstacle input to MPC. The robot
navigates from (0, 0) to (0, 8).

Smoke run:

```bash
./exp/scripts/run_corridor_planning_table.sh \
  --methods "ours_no_clip ours_no_sam ours_full" \
  --duration 40 \
  --repeats 1
```

Paper-style run:

```bash
./exp/scripts/run_corridor_planning_table.sh \
  --methods "dodt fapp ours_no_clip ours_no_sam ours_full" \
  --duration 70 \
  --repeats 3
```

Outputs:

```text
exp/data/sup_corridor_planning/corridor/<method>/run_<n>/
  raw.bag
  planning_metrics.csv
  planning_metrics.json
  clearance_timeseries.csv
  runtime_table.csv
  fig_corridor_trajectory.pdf     (BEV path visualization)
  fig_corridor_trajectory.png
  raw.bag includes /mpc/nav_path   (full online nav-to-goal path)

exp/data/sup_corridor_planning/
  summary_planning_metrics.csv
  summary_runtime_table.csv
  planning_level_table.csv
  planning_level_table.md
  module_runtime_table.csv
  module_runtime_table.md
```

Direct launch for interactive debugging:

```bash
export ROS_PACKAGE_PATH="$PWD:${ROS_PACKAGE_PATH:-}"
roslaunch exp/launch/sup_corridor_planning.launch method:=ours_full use_rviz:=true
```

Scene parameters (passed through launch file, adjustable via shell script):

| Param | Default | Description |
|-------|---------|-------------|
| `corridor_spacing` | 1.6 | Meters between cylinder rows along Y (produces 5 cylinders) |
| `corridor_speed` | 0.35 | Nominal speed m/s |
| `corridor_min_speed` | 0.2 | Slowest cylinder speed |
| `corridor_max_speed` | 0.9 | Fastest cylinder speed |
| `corridor_speed_variance` | 0.05 | ±jitter added to each cylinder's assigned speed |
| `corridor_motion_mode` | `wrap` | `wrap` = one-way crossing with wrap-around; `bounce` = back-and-forth |
| `corridor_spawn_walls` | `false` | Enable only for interactive wall-debug runs; leave false for the table benchmark |

Use `--duration 40` or longer for smoke tests. A 5 s run is useful only for
launch debugging; it is too short for the robot to reach `(0, 8)`, so success,
time-to-goal, and path-length-to-goal columns are not meaningful.

The corridor runner delays the fixed goal until after `--startup-wait` by
default (`--goal-start-delay auto`). This keeps the recorded bag aligned with
the full nav-to-goal process instead of dropping the initial avoidance segment
during sensor warm-up.

Metric definitions:

- `Success`: percentage of runs where `/odom` enters the fixed goal tolerance
  around `(0, 8)`. **Each per-run value is 0 or 1; the table reports mean × 100
  across all repeats.**
- `Collision`: mean collision events per run from robot/GT obstacle clearance.
  **Averaged across repeats.**
- `Min Dist.`: mean per-run minimum signed clearance to GT dynamic obstacles.
  **Averaged across repeats.**
- `Time-to-goal`: mean time to first goal entry, successful runs only.
  **Averaged across repeats (NaN runs excluded).**
- `Path Len.`: mean path length to first goal entry, successful runs only.
  **Averaged across repeats (NaN runs excluded).**
- `MPC Violations`: mean count of `/mpc/status.success == false` samples per
  run. **Averaged across repeats.**
- All metrics are computed from per-run `planning_metrics.csv` files aggregated
  by `summarize_sup_results.py` then averaged by `make_corridor_tables.py`.
  The `Runs` column reports the number of runs used.
- Runtime rows are computed only for modules that publish timing topics in the
  bag: `/rematch_statistics*` and `/mpc/status`.
- The BEV trajectory figure (`fig_corridor_trajectory.pdf`) is generated for
  each run by `calc_planning_metrics.py`; it shows the robot path color-coded
  by clearance, GT obstacle trajectories, corridor walls, and the goal.

## Reprocessing Existing Bags

Tracking-to-planning run directory:

```bash
./exp/scripts/process_bag.sh exp/data/sup_tracking_to_planning/front_crossing/ours_full/run_1 \
  --topic /doa_obstacles --no-cluster

python3 exp/scripts/stats_viz/calc_planning_metrics.py \
  --bag exp/data/sup_tracking_to_planning/front_crossing/ours_full/run_1/raw.bag \
  --output-dir exp/data/sup_tracking_to_planning/front_crossing/ours_full/run_1
```

Occlusion run directory:

```bash
./exp/scripts/process_bag.sh exp/data/sup_occlusion_reid/duration_2.0s/ours_full/run_1 \
  --topic /rematch/tracker_details --no-cluster

python3 exp/scripts/stats_viz/calc_occlusion_reid.py \
  --gt exp/data/sup_occlusion_reid/duration_2.0s/ours_full/run_1/gt.csv \
  --tracker exp/data/sup_occlusion_reid/duration_2.0s/ours_full/run_1/tracker.csv \
  --output-dir exp/data/sup_occlusion_reid/duration_2.0s/ours_full/run_1 \
  --occlusion-start 5.0 \
  --occlusion-duration 2.0
```
