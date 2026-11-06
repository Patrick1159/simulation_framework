# exp/ (sup_exp ROS package) — Agent Index

Supplemental experiment automation: tracking, Re-ID, and MPC safety metrics.
Package: `sup_exp`, pure Python, built via catkin.

## Directory Map

```
src/exp/
  launch/       — 2 ROS launch files (S1 corridor)
  scripts/      — stats_viz/ + parse_tmpc_log.py
  configs/      — YAML scene spec (S1 only)
  worlds/       — Gazebo .world files (S1 only)
  data/         — Experiment outputs (bags, CSVs, figures)
  plan.md       — RAL paper experiment design doc
  SUPPLEMENTAL_EXPERIMENTS.md — CLI reference for running experiments
  commands.md   — Quick gazebo launch hint
```

## Launch Files (`launch/`)

| File | Gazebo | Robot | Obstacles | Perception | MPC | Use Case |
|------|--------|-------|-----------|------------|-----|----------|
| `S1_static_corridor.launch` | S1 corridor world | None | None | None | No | Corridor world viewer |
| `S1_dynamic_corridor.launch` | S1 corridor world | TB3 burger | Dual sine-wave peds (scene 4) | None | Straight-line @ 0.4 m/s | Automated robot-vs-pedestrian recordings |

### `S1_static_corridor.launch`
- Minimal environment-only launcher. Starts Gazebo with `worlds/S1_static_corridor.world`
- Args: `world_file`, `gui` (default true), `paused` (default false)
- No robot, no perception, no control.

### `S1_dynamic_corridor.launch` (NEW)
- Full S1 experiment runner. Spawns TB3 burger driving forward at constant speed through the corridor with two sine-wave pedestrians crossing from doorways.
- Args: `world_file`, `robot_speed` (default 0.4), `scenario_duration` (default 20.0), `gui` (default true)
- Nodes: `robot_state_publisher`, `gazebo_ros/spawn_model`, `tf2_ros/static_transform_publisher` (world→odom), `my_tb3_description/set_pose.py`, `mpc_nav/robot_straight_line.py`, `my_tb3_description/dync_obs.launch` (scene:=4)
- Pedestrians: ped_001 spawns at (5.5,-1.15) t=3s; ped_002 at (14.0,-1.15) t=5s; both sine-wave lateral motion (amplitude 0.15, omega 0.6, v_x=1.0)
- Recording: use external `record_S1.sh`, not included in launch.

## Scripts (`scripts/`)

### T-MPC Log Parser (`parse_tmpc_log.py`)

Parses ROS console logs from `mpc_planner_jackalsimulator` (T-MPC) and summarizes:
- Run lifecycle (start/stop, experiment count)
- Solve success rate (success/failure counts + %)
- Failure reasons (breakdown of each `MPC failed: ...` cause)
- Timing statistics (planning + optimization: mean/p50/p99/max/min/std)
- Data readiness warnings (missing data, obstacle overflow, export skips)
- Per-experiment success rate + timing
- Benchmark summary (`BENCHMARKERS.print()` output at shutdown)

Supports both terminal format (`roslaunch --screen` output) and log file format
(`~/.ros/log/.../mpc_planner-*.log`). Requires `debug_output: true` in T-MPC settings
(already set in `mpc_planner_jackalsimulator/config/settings.yaml`).

```bash
# From terminal capture:
roslaunch mpc_nav tmpc_doa.launch 2>&1 | tee /tmp/tmpc_run.log
python3 scripts/parse_tmpc_log.py /tmp/tmpc_run.log

# From ROS log file:
python3 scripts/parse_tmpc_log.py ~/.ros/log/latest/mpc_planner-*.log

# Export for further analysis:
python3 scripts/parse_tmpc_log.py /tmp/tmpc_run.log --csv loops.csv --json report.json
```

### Stats & Visualization (`scripts/stats_viz/`)

**Extraction from rosbags:**
- `extract_gt.py` — `/obstacles` (ObstacleArray) → `gt.csv` (timestamp, id, x, y, vx, vy, radius). 40 lines — simplest script.
- `extract_tracker.py` — tracker topic → `tracker.csv`. Supports DOA topics (`/rematch/tracker_details`, `/doa_obstacles`). `--legacy-camera-frame` flag for old bags needing TF-based odom→camera conversion. 344 lines.

**Core Metrics:**
- `calc_metrics.py` — MOTA, IDSW, FP, FN, epos (Hungarian matching, 1.0m threshold). Optional `--bag` for D435 FOV filtering (87° HFOV, 6m max depth). 269 lines.
- `calc_prediction_error.py` — Constant-velocity prediction RMSE at 5 horizons: [0.1, 0.2, 0.3, 0.5, 1.0]s. 126 lines.
- `calc_planning_metrics.py` — `/odom` + `/obstacles` + `/mpc/status` → collision (<0.3m), near-collision (<0.5m), min clearance, goal success. Outputs `planning_metrics.csv`, `planning_metrics.json`, `clearance_timeseries.csv`. 510 lines.
- `calc_occlusion_reid.py` — ID switches across occlusion interval, Re-ID success rate. Args: `--occlusion-start`, `--occlusion-duration`. 295 lines.
- `calc_runtime_table.py` — Per-module runtime stats (mean/std/min/max) from `/rematch_statistics_sam`, `_clip`, `_aggregator`. 203 lines.

**Visualization:**
- `plot_results.py` — Per-run IEEE figures: prediction RMSE line, IDSW/FP/FN bars, BEV trajectory. 219 lines.
- `gen_compare.py` — Cross-method comparison figures for a single scene (`--scene`). 338 lines.
- `compare_tracking_keyframes.py` — Multi-panel BEV grid (N keyframes × M methods). Imports from `visualize_keyframes.py`. 435 lines.
- `visualize_keyframes.py` — Critical-frame top-down snapshots from `clearance_timeseries.csv`. Outputs `keyframes/` PDFs. 391 lines.

**Aggregation:**
- `summarize_sup_results.py` — Merges per-run CSVs across `data/<exp>/<scenario>/<method>/run_<n>/` tree into `summary_*.csv`. 110 lines.
- `make_corridor_tables.py` — Builds `planning_level_table.csv` + `module_runtime_table.csv`. Maps method labels to display names. 228 lines.
- `sup_exp_figures.py` — Reads summary CSVs, generates publication-ready figures (collision-clearance, prediction RMSE, IDSW vs duration, Re-ID success rate) and `figures/sup_exp.md` LaTeX report. 566 lines — largest script.

## Configs & Worlds

- `configs/S1_static_corridor.yaml` — Human-readable spec for 20m×2.8m corridor, 2 doorways (x=5.5, x=14.0), 10 static obstacles (3 trash bins, 4 wall signs, 2 crates, 1 barrel). Blueprint for the world file.
- `worlds/S1_static_corridor.world` — SDF 1.6 Gazebo world (385 lines). Environment only — no robots, plugins, or spawn points.

## Data Directory (`data/`)

```
data/
  crossing/                              — Ad-hoc crossing experiment
    compare/                             — Cross-method figures (PDF/PNG) + report.md
    dodt/run_1/, ours/run_1/             — Per-method runs
  sup_tracking_to_planning/              — Exp 1 outputs
    front_crossing/oracle_gt/run_1/, ours_full/run_1/
    pair_crossing/dodt/run_1/, fapp/run_1/
    summary_metrics.csv, summary_planning_metrics.csv, summary_prediction_error.csv
  sup_corridor_planning/                 — Exp 3 outputs
    corridor/ours_no_sam/run_1/
```

**File type legend:** `raw.bag` = full recording, `gt.csv`/`tracker.csv` = extracted trajectories, `metrics.csv` = tracking quality (MOTA/IDSW/FP/FN/epos), `prediction_error.csv` = prediction RMSE, `planning_metrics.csv/.json` = safety metrics, `clearance_timeseries.csv` = per-timestep clearance, `keyframes/` = snapshot PDFs.

## Root-Level Docs

- `plan.md` — RAL paper experiment design: 3 scenes (S1-S3), 7 methods (M1-M7), 5 experiment matrices, metric definitions, paper figure inventory. Total plan: 6 phases, ~1 week.
- `SUPPLEMENTAL_EXPERIMENTS.md` — CLI operations guide for 3 experiments:
  1. Tracking-to-Planning (compares tracking inputs with same MPC)
  2. Controlled Occlusion + Re-ID (varies occlusion duration 0.5-4.0s)
  3. Corridor Planning-Level + Runtime Tables (5 cylinders, MPC navigation)
  Includes direct `roslaunch` debug commands and bag reprocessing instructions.
- `commands.md` — Quick hint: `gazebo worlds/S1_static_corridor.world`

## Key Workflows

### View S1 static corridor
```bash
gazebo worlds/S1_static_corridor.world
```

### Run S1 dynamic experiment (robot + dual sine-wave pedestrians)
```bash
roslaunch sup_exp S1_dynamic_corridor.launch
```

### Process a recorded bag end-to-end
```bash
python3 scripts/stats_viz/extract_gt.py --bag raw.bag --output gt.csv
python3 scripts/stats_viz/extract_tracker.py --bag raw.bag --output tracker.csv
python3 scripts/stats_viz/calc_metrics.py --gt gt.csv --tracker tracker.csv
python3 scripts/stats_viz/calc_prediction_error.py --gt gt.csv --tracker tracker.csv
python3 scripts/stats_viz/plot_results.py --data-dir .
```

### Compute planning safety metrics from a rosbag
```bash
python3 scripts/stats_viz/calc_planning_metrics.py --bag raw.bag
```

### Aggregate results across experiment runs
```bash
python3 scripts/stats_viz/summarize_sup_results.py --root data/sup_tracking_to_planning
```

### Generate paper figures from aggregated summaries
```bash
python3 scripts/stats_viz/sup_exp_figures.py
```

### Cross-method comparison for a scene
```bash
python3 scripts/stats_viz/gen_compare.py --scene pair_crossing \
  --methods "Ours:data/.../ours_full/run_1" "DODT:data/.../dodt/run_1"
```

### Analyze T-MPC planner performance
```bash
python3 scripts/parse_tmpc_log.py ~/.ros/log/latest/mpc_planner-*.log
```

## Method Abbreviations (used by stats_viz scripts)

| Arg | Meaning |
|-----|---------|
| `ours_full` | Full DOA pipeline (SAM + CLIP + rematch) |
| `ours_no_clip` | DOA without CLIP features (dummy passthrough) |
| `ours_no_sam` / `ours_no_fastsam` | DOA without SAM segmentation |
| `stage1_only` | Coarse Stage-I tracker only (no rematch) |
| `dodt` | Onboard detector + tracker baseline |
| `fapp` | Depth-only mapping + DBSCAN clustering |
| `oracle_gt` | Ground-truth obstacles from Gazebo (upper bound) |

## Build

Pure Python package, no compiled C++ nodes. `CMakeLists.txt` only installs `worlds/`, `launch/`, `configs/` to catkin share.

```bash
# From workspace root:
cd /home/patrick/sim_dev/simulation_framework
catkin build sup_exp
```

Dependencies (from package.xml): `rospy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `tf2_ros`, `visualization_msgs`, `cv_bridge`, `mpc_nav`, `umap`.
