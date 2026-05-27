# Experiment Workflow Guide

## Pipeline Overview

Two parallel MPC pipelines share the same DOA perception front-end:

```
DOA Perception → Topic Split → Pipeline A (Python CasADi, /doa_obstacles)
                              → Pipeline B (C++ T-MPC,  /doa_obstacles_tmpc)
```

---

## Pipeline A: DOA → Python MPC (Baseline)

**Bridge:** `doa_to_obstacles.py` converts `/rematch/tracker_details` → `/doa_obstacles`

```bash
# Full system
roslaunch mpc_nav mpc.launch
```

**Node structure:**
- `umap_extract` → `coarse_match` → `para_samnode` + `para_clipnode` + `para_aggregate` → `detailed_match`
- `doa_to_obstacles.py` listens on `/rematch/tracker_details`, publishes `mpc_nav/ObstacleArray` to `/doa_obstacles`
- `mpc_node.py` subscribes `/doa_obstacles`, solves CasADi NLP, publishes `/cmd_vel`

**Config:** `src/mpc_nav/config/mpc.yaml`

---

## Pipeline B: DOA → T-MPC (Current Focus)

**Bridge:** `doa_to_tmpc.py` converts `/rematch/tracker_details` → `/doa_obstacles_tmpc` (with GMM predictions)

```bash
# Full system
roslaunch mpc_nav tmpc_doa.launch
```

**Node structure:**
- Same DOA perception as Pipeline A
- `doa_to_tmpc.py` listens on `/rematch/tracker_details`, publishes `mpc_planner_msgs/ObstacleArray` with GMM
- `jackalsimulator_planner` node solves Acados SQP-RTI, publishes `/cmd_vel`

**Config:** `src/mpc_planner/mpc_planner_jackalsimulator/config/`

**Key parameters in doa_to_tmpc.py:**
- `N` / `dt` — prediction horizon for GMM extrapolation (default N=30, dt=0.2)
- `max_age` — obstacle timeout (default 0.5s)
- `velocity_limit` — outlier rejection (default 5.0 m/s)
- `target_frame` — transform target (default `odom`)

---

## Experiment Launcher

```bash
roslaunch exp avoidance_exp.launch method:=ours_full dynamic_scene:=2
```

**Available methods:** ours_full, ours_no_clip, ours_no_sam, oracle_gt (stage1_only/dodt/fapp are broken — see perception_dispatch.launch)

**Dynamic scenes:** 0=random walk, 1=corridor, 2=crossing pedestrians, 3=single crossing, 4=sine-wave

**Configs:** `src/exp/configs/avoidance_corridor.yaml`, `avoidance_crossing.yaml`, `demo_occlusion.yaml`

---

## S1 Corridor Experiment

```bash
roslaunch exp S1_dynamic_corridor.launch
```

Straight-line robot + dual sine-wave pedestrians from doorways. No perception — uses `robot_straight_line.py` for constant-velocity cmd_vel.

---

## Standalone Launches

```bash
# Perception only (no MPC)
roslaunch umap simulation.launch

# Dynamic obstacles only (no robot/perception)
roslaunch my_tb3_description dync_obs.launch scene:=2

# Static obstacles only
roslaunch my_tb3_description static_obs.launch
```

---

## Data Analysis

13 scripts in `src/exp/scripts/stats_viz/` process rosbags into metrics and figures.

**End-to-end bag processing:**
```bash
python3 extract_gt.py --bag raw.bag --output gt.csv
python3 extract_tracker.py --bag raw.bag --output tracker.csv
python3 calc_metrics.py --gt gt.csv --tracker tracker.csv
python3 calc_planning_metrics.py --bag raw.bag
python3 summarize_sup_results.py --root data/avoidance
```

**Active data directories:** `src/exp/data/avoidance/`, `sup_tracking_to_planning/`, `sup_corridor_planning/`
