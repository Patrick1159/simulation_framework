# exp/scripts/

## Overview

Top-level experiment automation for the `sup_exp` package. This directory
provides two experiment families and an offline reprocessing helper:

- **Avoidance experiment** (`run_avoidance_exp.sh`) — robot navigates through
  a dynamic scene; tracking + MPC are exercised end-to-end.
- **Demo / occlusion experiment** (`run_demo_exp.sh`) — controlled occlusion
  + Re-ID demo where the robot can be held stationary by the `cmd_vel` gate
  while MPC still computes commands for analysis.
- **Offline bag reprocessing** (`process_bag.sh`) — re-runs the stats pipeline
  against a recorded `raw.bag` without re-running the experiment.

Each shell entrypoint wires together a ROS launch file, scene/method
configs, optional perception dispatch, and a `cmd_vel` gate. The
`stats_viz/` Python pipeline is the existing analysis stack and is reused
unchanged by every entrypoint and by `process_bag.sh`.

## Quick start

```bash
# avoidance: pair-crossing subscene, our full method, 60s, single run
./run_avoidance_exp.sh --subscene pair_crossing --method ours_full --duration 60 --repeats 1

# demo: ours_full, MPC computes but does not move the robot (cmd_vel gate closed)
./run_demo_exp.sh --method ours_full --apply-cmd-vel false --duration 20

# offline: rerun the stats pipeline on an already-recorded bag
./process_bag.sh ../data/avoidance/pair_crossing/ours_full/run_1
```

## Files

### `scripts/`

| File | Purpose |
|------|---------|
| `run_avoidance_exp.sh` | Orchestrates one or more avoidance runs (per subscene/method/repeat). |
| `run_demo_exp.sh` | Orchestrates a demo / occlusion run with optional cmd_vel gating. |
| `process_bag.sh` | Offline reprocessing: rerun stats_viz pipeline on a recorded `raw.bag`. |
| `cmd_vel_gate.py` | ROS node that gates `/mpc/cmd_vel_raw` to `/cmd_vel` based on a runtime flag. |
| `stats_viz/` | Existing analysis pipeline (extraction, metrics, plots). Reused as-is. |
| `adaptor/`, `scene/` | Reserved subpackages (currently empty). |

### `launch/`

| File | Purpose |
|------|---------|
| `avoidance_exp.launch` | Brings up Gazebo + robot + perception + MPC for an avoidance run. |
| `demo_exp.launch` | Like avoidance but remaps `/cmd_vel` -> `/mpc/cmd_vel_raw` and inserts `cmd_vel_gate.py`. |
| `perception_dispatch.launch` | Selects the perception stack to load based on the `method` arg. |
| `S1_static_corridor.launch` | Minimal environment-only viewer (Gazebo with corridor world). |
| `S1_dynamic_corridor.launch` | Robot + dual sine-wave pedestrians in the corridor. |

### `configs/`

| File | Purpose |
|------|---------|
| `avoidance_subscenes.yaml` | Subscene catalog for the avoidance experiment (pair_crossing, front_crossing, ...). |
| `demo_scenarios.yaml` | Scenario presets for the demo experiment (occlusion windows, starts). |
| `methods.yaml` | Method label -> perception stack mapping (consumed by `perception_dispatch.launch`). |
| `S1_static_corridor.yaml` | Human-readable spec for the S1 corridor world. |

## Method labels

The seven methods used by the experiment shells and by `stats_viz/`:

| Arg | Meaning |
|-----|---------|
| `ours_full` | Full DOA pipeline (SAM + CLIP + rematch). |
| `ours_no_clip` | DOA without CLIP features (dummy passthrough). |
| `ours_no_sam` / `ours_no_fastsam` | DOA without SAM segmentation. |
| `stage1_only` | Coarse Stage-I tracker only (no rematch). **NOT currently supported end-to-end:** the coarse tracker publishes `umap/Obstacles`, which `doa_to_obstacles.py` cannot consume (it expects `umap/TrackerDetailArray`). A dedicated bridge is required before this baseline produces non-empty `/doa_obstacles`. |
| `dodt` | Onboard detector + tracker baseline. |
| `fapp` | Depth-only mapping + DBSCAN clustering. |
| `oracle_gt` | Ground-truth obstacles from Gazebo (upper bound). |

## Output tree

Avoidance experiment:

```
exp/data/avoidance/
  <subscene>/                         # e.g. pair_crossing, front_crossing
    <method>/                         # e.g. ours_full
      run_<n>/
        raw.bag
        gt.csv
        tracker.csv
        metrics.csv
        prediction_error.csv
        planning_metrics.csv
        planning_metrics.json
        clearance_timeseries.csv
        runtime.csv
        keyframes/
        figures/
    summary_metrics.csv               # cross-run aggregate, written at the method root
  summary_planning_metrics.csv        # cross-method aggregate, written at the subscene root
```

Demo / occlusion experiment:

```
exp/data/demo_occlusion/
  <method>/
    run_<n>/
      raw.bag
      gt.csv
      tracker.csv
      metrics.csv
      prediction_error.csv
      planning_metrics.csv
      clearance_timeseries.csv
      runtime.csv
      occlusion_reid.csv              # only when --demo step ran
      keyframes/
      figures/
  summary_occlusion_reid.csv          # cross-method aggregate
```

Cross-experiment summaries land at each level's root via
`stats_viz/summarize_sup_results.py`.

## Offline reprocessing

Use `process_bag.sh` when a `raw.bag` already exists and only the stats
outputs need to be (re)built. Typical cases:

- A stats script was fixed and you want the new numbers without rerunning Gazebo.
- A run was recorded but the post-processing step failed.
- You received a bag from someone else and only have the bag, not the CSVs.

Behavior:

1. Validates `<run_dir>/raw.bag` exists.
2. Auto-detects the tracker topic from `rosbag info` (preferring
   `/rematch/tracker_details`, falling back through DOA / onboard /
   `/states` / `/obstacles`). Override with `--topic`.
3. Runs the stats_viz pipeline (extract_gt -> extract_tracker ->
   calc_metrics -> calc_prediction_error -> calc_planning_metrics ->
   calc_runtime_table -> visualize_keyframes -> plot_results).
4. With `--demo`, also runs `calc_occlusion_reid.py` (requires
   `--occlusion-start` and `--occlusion-duration`).

Per-step failures emit a `WARN:` line and the next step still runs. The
script exits 0 only when steps 1-3 (extract + core metrics) all succeed.

## Keyframe replay

`stats_viz/visualize_keyframes.py` selects critical frames from
`clearance_timeseries.csv` (the per-timestep clearance series written by
`calc_planning_metrics.py`) and renders top-down PDFs into `keyframes/`.
Because the selection is driven by the recorded clearance timeseries, the
same bag can be re-visualized at any time with `process_bag.sh` — the
keyframes do not depend on live state and are deterministic given the bag.

## cmd_vel gate

The demo experiment runs through `demo_exp.launch`, which remaps the MPC
node's `/cmd_vel` output to `/mpc/cmd_vel_raw` and inserts
`cmd_vel_gate.py` between MPC and the robot. When `--apply-cmd-vel false`
is passed to `run_demo_exp.sh`, the gate publishes zero twists to
`/cmd_vel` so the robot stays still, but MPC continues to plan: the bag
will contain `/mpc/cmd_vel_raw` and `/mpc/pred_path` for offline analysis
of what the controller *would* have done. With `--apply-cmd-vel true` the
gate is fully transparent and the robot moves normally.

## Goal publishing

`mpc_node.py` waits for a `PoseStamped` on `/move_base_simple/goal` before
it will start planning. The experiment launches include
`goal_publisher.py`, which reads `/scene/goal_pose` (loaded from the
active yaml config) and publishes one latched goal on
`/move_base_simple/goal` after a 3 second delay. Without this, MPC would
sit idle and the robot would not move.

## Troubleshooting

- **MPC node fails with `ModuleNotFoundError: casadi`** — the MPC node
  depends on CasADi. Run the experiment inside the `fastsam` conda
  environment (`casadi` 3.7.2 is installed there), or
  `pip install --user casadi` for the system python3.8.
- **`sim_exp_1.py: termios.error: (25, 'Inappropriate ioctl for device')`** —
  noise from the scene script's keyboard listener, which fails when launched
  from a non-tty process. Harmless; not produced by our scripts.
- **`topic types do not match: [umap/TrackerDetailArray] vs. [mpc_nav/ObstacleArray]`** —
  this used to appear for `--method oracle_gt` and indicates the dispatcher
  is using the wrong node for GT. Fixed by using `topic_tools/relay` for
  `oracle_gt`. If you see it again, confirm `perception_dispatch.launch`'s
  `oracle_gt` group has not regressed to using `doa_to_obstacles.py`.
- **Robot rotates ~180° and stops; `path_length_m ≈ 0` but `mpc_success_rate ≈ 1.0`** —
  `mpc_solver.py` uses `Wy * sin^2(yaw_err)` for yaw cost. `sin^2(π) == 0`,
  so the controller treats "back to the goal" as zero-cost on yaw and can
  get stuck rotating to the opposite heading. The fix is to keep
  `scene.goal_pose.yaw` close to the robot's initial heading. Our default
  TB3 spawn is at yaw = π/2 (facing +y), so the bundled configs use
  `goal_pose.yaw: 1.5708`. If you change the spawn or pick a different
  goal direction, set `yaw` to that heading (in radians) — not 0 by
  default.
- **All planning metrics NaN / `planner_obstacle_msgs: 0`** — the
  perception/relay chain published nothing into `/doa_obstacles`. Check
  with `rostopic hz /doa_obstacles` during a run.
- **`visualize_keyframes.py: error: unrecognized arguments: --run-dir`** —
  earlier versions of `process_bag.sh` used a non-existent flag. Pull the
  latest; the call is now positional (`visualize_keyframes.py .`).
