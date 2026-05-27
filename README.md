# Simulation Framework

ROS Noetic / Gazebo 11 移动机器人动态障碍物避障系统。
Dynamic obstacle avoidance with DOA perception + dual-MPC planning (CasADi + T-MPC).

## Overview

This framework provides:
- **DOA perception pipeline** — RGB-D → 2D/3D bbox → coarse tracking → SAM/CLIP → detailed rematch
- **Dual MPC navigation** — Python CasADi (baseline) + C++ Acados T-MPC (current focus)
- **Dynamic obstacle simulation** — 5 scene scripts (random walk, corridor, crossing, sine-wave)
- **Experiment framework** — Multi-method comparison with 13 analysis scripts
- **Comparison baselines** — DODT, FAPP, oracle ground-truth

## Repository Structure — 19 Packages by Cluster

### 1. DOA Perception Pipeline

| Package | Role |
|---------|------|
| `src/DOA/umap/` | Core perception: `umap_extract` → `coarse_match` → [FastSAM+CLIP+aggregate] → `detailed_match` |
| `src/DOA/pointcloud_generator/` | Depth → PointCloud2 conversion |
| `src/DOA/path_planning/` | (Legacy) move_base + DWA integration with DynamicObstacleLayer |

### 2. MPC Planning (Dual Pipeline)

| Package | Role |
|---------|------|
| `src/mpc_nav/` | **Pipeline A** — Python CasADi/IPOPT MPC (comparison baseline) |
| `src/mpc_planner/` | **Pipeline B** — C++ Acados T-MPC (current focus, ~40000 lines) |
| `src/guidance_planner/` | Visibility-PRM topological global planner for T-MPC |
| `src/roadmap/` | XML/YAML → spline reference paths (Clothoid/cubic) |
| `src/ros_tools/` | Shared C++ utilities (vis, logging, math, YAML) |

### 3. Simulation Environment

| Package | Role |
|---------|------|
| `src/my_tb3_description/` | TurtleBot3 Burger URDF (D435) + 5 dynamic obstacle scene scripts |
| `src/pedestrian_simulator/` | ROS1/ROS2 Pedsim wrapper (social force model, 100+ XML scenarios) |
| `src/pedsim_original/` | Pedsim C++ social-force model library |
| `src/asr_rapidxml/` | RapidXML wrapper (dependency of pedestrian_simulator + roadmap) |

### 4. Experiment Framework

| Package | Role |
|---------|------|
| `src/exp/` | Experiment launch files, configs, data, and 13 analysis scripts |

### 5. Comparison Baselines

| Package | Role |
|---------|------|
| `src/onboard_detector/` | DODT baseline — DBSCAN + Kalman + YOLO |
| `src/FAPP/` | UAV perception-planning framework (mot_mapping used for comparison) |

### 6. Supporting / Build

| Package | Role |
|---------|------|
| `src/catkin_simple/` | Build tool (retained for compat) |
| `src/jackal_simulator/` | Jackal UGV sim stack (not currently used) |
| `src/DecompUtil/` | Convex decomposition (optional T-MPC dependency) |
| `src/dashgo_description/` | Dashgo robot URDF models (not currently used) |

## Data Flow

```
Camera RGB/Depth → DOA Perception (umap → coarse → SAM+CLIP+rematch)
                     │
                     │ /rematch/tracker_details (umap/TrackerDetailArray)
                     ▼
          ┌──────────────────────────┐
          │      Topic Split         │
          └──────┬──────────┬────────┘
                 │          │
    doa_to_obstacles.py   doa_to_tmpc.py
    (mpc_nav/ObstacleArray) (mpc_planner_msgs/ObstacleArray w/ GMM)
                 │          │
                 ▼          ▼
    ┌────────────────┐  ┌──────────────────┐
    │ mpc_node.py    │  │ jackalsimulator  │
    │ (CasADi/IPOPT) │  │ (Acados SQP-RTI) │
    │ Pipeline A     │  │ Pipeline B       │
    └───────┬────────┘  └───────┬──────────┘
            │                   │
            ▼                   ▼
                /cmd_vel → TurtleBot3
```

## Quick Start

```bash
# Full system: DOA → Python MPC + Gazebo
roslaunch mpc_nav mpc.launch

# Full system: DOA → T-MPC
roslaunch mpc_nav tmpc_doa.launch

# Main experiment (method + scene selectable)
roslaunch exp avoidance_exp.launch method:=ours_full dynamic_scene:=2

# DOA perception standalone (simulation mode)
roslaunch umap simulation.launch

# Dynamic obstacle scene standalone
roslaunch my_tb3_description dync_obs.launch scene:=2
```

## Key Comparisons

| | mpc_nav (Python) | mpc_planner (C++) |
|---|---|---|
| Solver | CasADi + IPOPT | Acados SQP-RTI / Forces Pro |
| Trajectories | Single | Multiple parallel (T-MPC++) |
| Obstacle model | Swept capsule + KF | Ellipsoid, GMM, Scenario-based |
| Max obstacles | 5 (fixed slots) | 12 (configurable) |
| Horizon | N=45, dt=0.1s | N=30, dt=0.2s |
| Global planner | None (point goal) | guidance_planner (PRM) |

## Experimental Methods

| Method | Meaning |
|--------|---------|
| `ours_full` | Full DOA pipeline (SAM + CLIP + rematch) |
| `ours_no_clip` | DOA w/o CLIP features |
| `ours_no_sam` | DOA w/o SAM segmentation |
| `stage1_only` | Coarse tracker only (no rematch) — **topic mismatch, not usable** |
| `dodt` | Onboard detector + tracker — **topic mismatch, not usable** |
| `fapp` | Depth-only mapping + DBSCAN — **topic mismatch, not usable** |
| `oracle_gt` | Ground-truth obstacles from Gazebo |

## Build

```bash
cd /home/patrick/sim_dev/simulation_framework
catkin build
```

> **Note:** Deactivate conda environments before building C++ packages.

## Prerequisites

- ROS Noetic (Ubuntu 20.04)
- Gazebo 11
- Python 3.8+
- CasADi (`python3-casadi`)

## Key Docs

- `CLAUDE.md` — Full module roadmap for coding agents
- `docs/code_audit/` — Comprehensive code audit (7 documents)
