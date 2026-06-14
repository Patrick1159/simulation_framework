# Simulation Framework — Project Map

ROS Noetic / Gazebo 11 移动机器人动态障碍物避障系统。
Dynamic obstacle avoidance with DOA perception + dual-MPC planning (CasADi + T-MPC).

---

## 目录结构 (src/ — 19 packages by cluster)

### 1. DOA 感知管线 (Perception Pipeline)

| Package | Role | Key Files |
|---------|------|-----------|
| **`DOA/umap/`** | 核心感知：RGB-D → 2D/3D bbox → coarse tracking → SAM/CLIP → detailed rematch | `clean_umap.cpp`, `clean_coarse_match.cpp`, `clean_detailed_match.cpp`, `para_samnode.py`, `para_clipnode.py`, `para_aggregate.py` |
| **`DOA/pointcloud_generator/`** | Depth→PointCloud2 转换 | `processor.cpp`, `camera_pose.py` |
| **`DOA/path_planning/`** | (Legacy) move_base + DWA 集成，含 DynamicObstacleLayer | — |

**Data flow:** `/d435/image_raw` + `/d435/depth/image_raw` → `umap_extract` → `/umap/2D_3D_bounding_boxes` → `coarse_match` → `/tracker/dynamic_obstacles` → [SAM+CLIP+aggregate] → `detailed_match` → `/rematch/tracker_details`

**Recommended launch:** `roslaunch umap simulation.launch`

---

### 2. MPC 规划器 (Dual Pipeline)

当前重心在 **T-MPC (C++ Acados)**，`mpc_nav` (Python CasADi) 作为对比基线保留。
Focus on T-MPC; mpc_nav kept as comparison baseline.

#### Pipeline A: mpc_nav (Python CasADi — baseline)

| File | Role |
|------|------|
| `mpc_nav/scripts/mpc_node.py` | MPC ROS node — subscribes odom/goal/obstacles, solves CasADi NLP, publishes cmd_vel |
| `mpc_nav/scripts/mpc_solver.py` | CasADi solver: unicycle model, swept-capsule obstacle constraints |
| `mpc_nav/scripts/doa_to_obstacles.py` | DOA → mpc_nav adapter: `/rematch/tracker_details` → `/doa_obstacles` (mpc_nav/ObstacleArray) |
| `mpc_nav/config/mpc.yaml` | Horizon N=45/dt=0.1, v_max=0.6, safety_margin=0.05, max_obstacles=5 |

**Launch:** `roslaunch mpc_nav mpc.launch`

#### Pipeline B: mpc_planner / T-MPC (C++ Acados — current focus)

| Package | Role |
|---------|------|
| **`mpc_planner/`** | T-MPC 框架 (tud-amr): 并行轨迹优化, GMM障碍物不确定性, Acados/ForcesPro 求解器. 含 mpc_planner, mpc_planner_types, mpc_planner_util, mpc_planner_solver, mpc_planner_modules, mpc_planner_jackalsimulator 等子包 |
| **`guidance_planner/`** | Visibility-PRM 拓扑全局规划器 — 提供 topology-distinct 轨迹给 T-MPC 的模块 |
| **`roadmap/`** | XML/YAML 地图 → spline 参考路径 (Clothoid/cubic) |
| **`ros_tools/`** | 共享 C++ 工具库 (visualization, logging, profiling, math, YAML config) — 被所有 tud-amr 包依赖 |

**Bridge (DOA → T-MPC):**
| File | Role |
|------|------|
| `mpc_nav/scripts/doa_to_tmpc.py` | DOA → T-MPC: `/rematch/tracker_details` → `/doa_obstacles_tmpc` (mpc_planner_msgs/ObstacleArray with GMM) |
| `mpc_nav/launch/tmpc_doa.launch` | 启动 bridge + mpc_planner_jackalsimulator 节点 |

**Launch:** `roslaunch mpc_nav tmpc_doa.launch`

**Key architectural difference:**
| | mpc_nav (Python) | mpc_planner (C++) |
|---|---|---|
| Solver | CasADi + IPOPT | Acados SQP-RTI / Forces Pro |
| Trajectories | Single | Multiple parallel (T-MPC++) |
| Obstacle model | Swept capsule + KF uncertainty | Ellipsoid, GMM, Scenario-based |
| Max obstacles | 5 (fixed slots) | 12 (configurable) |
| Global planner | None (point goal) | guidance_planner (PRM) |
| Horizon | N=45, dt=0.1s | N=30, dt=0.2s |

---

### 3. 仿真环境 (Simulation Environment)

| Package | Role |
|---------|------|
| **`my_tb3_description/`** | TurtleBot3 Burger URDF (D435 sensor) + 动态障碍物场景(`dync_obs.launch`, scenes 0-4) |
| `dashgo_description/` | Dashgo 机器人描述 (URDF/xacro) — **当前无引用** |
| `my_gazebo_test/` | 测试用 Gazebo world — **非ROS包, 无引用** |

**Dynamic scene scripts** (in `my_tb3_description/scripts/`):

| Scene | Script | Obstacles | Motion | Used by dync_obs scene# |
|-------|--------|-----------|--------|------------------------|
| Random walk | `random_dynamic_obstacle.py` | N cylinders | Random velocity/angular rate (**seed fixed** via `~random_seed`, default 42) | 0 |
| Corridor moving | `dync_scene1.py` | Columns | Bounce/wrap at X bounds | 1 |
| Crossing peds | `sim_exp_1.py` | 2 cylinders | Diagonal crossing + ID jitter sim | 2 |
| Single crossing | `crossing_pedestrian.py` | 1 cylinder | L-to-R constant y | 3 |
| Sine-wave peds | `pedestrian_sine_wave.py` | 2 cylinders | Sine wave from doorways | 4 |
| Static field | `static_scene1.py` | N cylinders | Static | (separate launch) |
| Shared lib | `sdf.py` | — | SDF model generator for all scenes | — |

**Launch:** `roslaunch my_tb3_description dync_obs.launch scene:=0`

---

### 4. 行人仿真 (Pedestrian Simulation — T-MPC 依赖)

| Package | Role |
|---------|------|
| **`pedestrian_simulator/`** | ROS1/ROS2 Pedsim wrapper: 社交力模型行人, 100+ XML场景, exec_depend of mpc_planner 部署包 |
| **`pedsim_original/`** | Pedsim C++ 库 (社交力模型) |
| **`asr_rapidxml/`** | RapidXML ROS 封装, 被 pedestrian_simulator + roadmap 依赖 |

---

### 5. 对比基线 (Comparison Baselines)

| Package | Role | Wired Into Pipeline? |
|---------|------|----------------------|
| **`onboard_detector/`** | DODT 基线: DBSCAN + Kalman + YOLO, C++ node | 通过 `perception_dispatch.launch method:=dodt` 接入，**但当前话题类型不匹配(需bridge)** |
| **`FAPP/`** | 无人机感知规划框架, 仅 `mot_mapping` 感知前端用于对比 | 通过 `perception_dispatch.launch method:=fapp` 接入, **同样话题不匹配** |

---

### 6. 实验框架 (Experiment Framework)

| File / Dir | Role |
|------------|------|
| **`exp/launch/avoidance_exp.launch`** | 主实验入口: Gazebo + 感知 + MPC + 导航目标 |
| **`exp/launch/demo_exp.launch`** | Occlusion demo: 带 cmd_vel 闸门 |
| **`exp/launch/perception_dispatch.launch`** | 方法选择器: ours_full / ours_no_clip / ours_no_sam / stage1_only / dodt / fapp / oracle_gt |
| **`exp/scripts/run_avoidance_exp.sh`** | 多方法实验编排器 |
| **`exp/configs/`** | 场景配置: `avoidance_corridor.yaml`, `avoidance_crossing.yaml`, `demo_occlusion.yaml` |
| **`exp/scripts/stats_viz/`** | 13 个离线分析脚本 (metrics, prediction, planning metrics, figures) |
| **`exp/data/avoidance/`** | 活跃实验数据 (May 24-27), 6 methods × pair_crossing |
| **`exp/SUPPLEMENTAL_EXPERIMENTS.md`** | CLI 操作指南 |

**Known broken methods in perception_dispatch:**
- `stage1_only`: coarse 输出 `umap/Obstacles`, adapter 期待 `TrackerDetailArray`
- `dodt`: `onboard_detector` 输出 `MarkerArray`, 同上
- `fapp`: `mot_mapping` 输出 `ObjectsStates`, 同上

---

## 核心数据流 (Data Flow)

```
                    ┌─────────────────────────────────────┐
Camera RGB/Depth ──→│  DOA Perceptio                     │
                    │  umap_extract → coarse_match        │
                    │  → [SAM+CLIP+aggregate]             │
                    │  → detailed_match                   │
                    └────────────┬────────────────────────┘
                                 │ /rematch/tracker_details
                                 │ (umap/TrackerDetailArray)
                                 ▼
                    ┌────────────────────────────┐
                    │      Topic Split           │
                    └────────┬────────┬──────────┘
                             │        │
              doa_to_obstacles.py    doa_to_tmpc.py  
              (mpc_nav/ObstacleArray) (mpc_planner_msgs/ObstacleArray)
                             │        │
                             ▼        ▼
                    ┌──────────┐  ┌──────────┐
                    │ mpc_node │  │ jackalsim │
                    │ (CasADi) │  │ (Acados) │
                    │ Pipeline│  │ Pipeline │
                    │ A (bl)  │  │ B (curr) │
                    └────┬─────┘  └────┬─────┘
                         │             │
                         ▼             ▼
                    ┌──────────────────────┐
                    │     /cmd_vel          │
                    │   → TurtleBot3       │
                    └──────────────────────┘
```

## 方法缩写 (Method Abbreviations for exp/)

| Arg | Meaning |
|-----|---------|
| `ours_full` | Full DOA pipeline (SAM + CLIP + rematch) |
| `ours_no_clip` | DOA w/o CLIP features |
| `ours_no_sam` / `ours_no_fastsam` | DOA w/o SAM segmentation |
| `stage1_only` | Coarse tracker only (no rematch) |
| `dodt` | Onboard detector + tracker baseline |
| `fapp` | Depth-only mapping + DBSCAN clustering baseline |
| `oracle_gt` | Ground-truth obstacles from Gazebo |

## Key Launch Commands

```bash
# Full system (DOA → Python MPC + Gazebo)
roslaunch mpc_nav mpc.launch

# Full system (DOA → T-MPC)
roslaunch mpc_nav tmpc_doa.launch

# Main experiment (method + scene selectable)
roslaunch sup_exp avoidance_exp.launch method:=ours_full dynamic_scene:=2

# Occlusion demo (cmd_vel gated)
roslaunch sup_exp demo_exp.launch method:=ours_full

# S1 corridor (straight-line + sine-wave peds, no perception)
roslaunch sup_exp S1_dynamic_corridor.launch

# DOA perception only (simulation mode)
roslaunch umap simulation.launch

# Dynamic obstacle scene standalone
roslaunch my_tb3_description dync_obs.launch scene:=2

# Multi-method experiment batch
bash exp/scripts/run_avoidance_exp.sh

# Parse T-MPC planner logs (after a tmpc_doa run)
python3 exp/scripts/parse_tmpc_log.py ~/.ros/log/latest/mpc_planner-*.log
```

## 快速索引 (Quick Index)

| 你想做什么 | 看什么文件 |
|-----------|-----------|
| 改感知参数 (DOA) | `DOA/umap/config/simulation.yaml` |
| 改MPC参数 (Python) | `mpc_nav/config/mpc.yaml` |
| 改MPC参数 (T-MPC) | `mpc_planner/.../mpc_planner_jackalsimulator/config/` |
| 加新场景 | `my_tb3_description/scripts/` + `dync_obs.launch` |
| 跑实验 | `exp/scripts/run_avoidance_exp.sh` 或 `exp/launch/avoidance_exp.launch` |
| 分析bag数据 | `exp/scripts/stats_viz/` (13个脚本) |
| 分析T-MPC日志 | `exp/scripts/parse_tmpc_log.py` — 解析 mpc_planner ROS log, 汇总 solve rate / timing / failures |
| 生成论文图 | `exp/scripts/stats_viz/sup_exp_figures.py` |
| T-MPC bridge | `mpc_nav/scripts/doa_to_tmpc.py` + `tmpc_doa.launch` |
| DOA内部架构 | `docs/code_audit/` (完整审计文档) |

## 仓库状态

- **git branch:** `develop`, 追踪 `origin/develop`
- **Tracked packages:** DOA(submodule), mpc_nav, my_tb3_description, dashgo_description, my_gazebo_test
- **Untracked additions:** exp/, mpc_planner/, guidance_planner/, roadmap/, ros_tools/, pedestrian_simulator/, pedsim_original/, onboard_detector/, FAPP/, jackal_simulator/, DecompUtil/, asr_rapidxml/, catkin_simple/
- **Deleted:** voxblox/ (submodule uninitialized, removed from disk)
- **Build:** `cd /home/patrick/sim_dev/simulation_framework && catkin build` (需退出 conda 环境)
