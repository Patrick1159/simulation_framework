# Repository Map — `simulation_framework`

**Root:** `/home/patrick/sim_dev/simulation_framework/`
**Type:** ROS Noetic catkin workspace (`.catkin_workspace` marker present)
**Git:** Branch `develop`, tracks `origin/develop`

---

## Top-Level Files

| File | Purpose | Importance |
|------|---------|------------|
| `README.md` | Project documentation (419 lines). Covers original 5 packages. **Stale** — 14+ packages undocumented. | Medium |
| `CLAUDE.md` | AI agent project overview (Chinese). Covers original packages + data flow. **Stale.** | Medium (agent context) |
| `install_voxblox_deps_no_rosdep.sh` | Script to clone & build voxblox + ETH-ASL deps. **Partially stale** (voxblox deleted). | Low (historical) |
| `kill_ros_gazebo.sh` | Kill stray ROS/Gazebo processes. Utility. | Low (dev utility) |
| `tmux_session.template.conf` | tmux config for real-robot launch (6 panes). Path references stale. | Low |
| `.gitmodules` | Declares 2 submodules: `src/DOA` (active), `src/voxblox` (deleted, uninitialized). | Medium |
| `task_plan.md` | Prior session task plan (exp scripts reorganization, complete). | Low (historical) |
| `findings.md` | Prior session findings (468 lines, May 12). Data flow traces, 5 cross-cutting issues. | High (experiment methodology) |
| `progress.md` | Prior session progress log (interrupted experiment run). | Low (historical) |
| `logfile.txt` | 1.2MB ROS console log (successful mpc_planner run). | Low (debug artifact) |
| `figures/` | 6 paper-quality experiment figures (PDF+PNG) + sup_exp.md report. | High (results output) |

---

## `src/` — Package Overview (19 directories)

### Tracked in git (original packages)

| Package | Role | Lines of Code | ROS Package? | Build Type |
|---------|------|---------------|--------------|------------|
| **DOA** (submodule) | Core perception pipeline: depth→bbox→coarse tracking→SAM/CLIP→detailed tracking | ~12000 | Yes (umap, pointcloud_generator, path_planning) | C++ + Python |
| **mpc_nav** | Python CasADi-based MPC navigation node | ~2500 | Yes | Python |
| **my_tb3_description** | TurtleBot3 Burger URDF + D435 sensor + 5 dynamic obstacle scene scripts | ~2000 | Yes | Data + Python |
| **my_gazebo_test** | Gazebo test world + model | ~10 | **No** | Data only |
| **dashgo_description** | Dashgo robot URDF/xacro description | ~100 | Yes | Data only |

### Untracked in git (new additions)

| Package | Role | Has own `.git`? | Lines of Code |
|---------|------|-----------------|---------------|
| **exp** (sup_exp) | Experiment automation: launch, analysis scripts, data | **No** | ~5000 |
| **FAPP** | Drone perception-planning comparison baseline | Yes | ~15000 |
| **mpc_planner** | C++ T-MPC framework (tud-amr) — parallel MPC, Acados solver | Yes | ~40000 |
| **guidance_planner** | Visibility-PRM topology-distinct global planner for T-MPC | Yes | ~6000 |
| **roadmap** | XML→spline reference path provider for MPC | Yes | ~4000 |
| **ros_tools** | Shared ROS1/ROS2 C++ utility library (used by all tud-amr packages) | Yes | ~3000 |
| **pedestrian_simulator** | ROS-wrapper for Pedsim social-force pedestrian simulation | Yes | ~4000 |
| **pedsim_original** | Original Pedsim C++ library (social force model) | Yes | ~2000 |
| **DecompUtil** | Convex decomposition library (IRIS, ellipsoid) | Yes | ~3000 |
| **onboard_detector** | DODT baseline: DBSCAN + Kalman + YOLO from depth | Yes | ~5000 |
| **jackal_simulator** | Jackal UGV simulation (7 sub-packages) | Yes | ~5000 |
| **asr_rapidxml** | RapidXML wrapper ROS package | Yes | ~500 (header) |
| **catkin_simple** | ETH-ASL CMake simplification build tool | Unknown | ~500 |

---

## Architecture Map: Four Software Clusters

### Cluster 1: DOA Perception Pipeline (Active Core)

```
umap_extract (C++) → coarse_match (C++) → [para_samnode|para_clipnode] (Python) → para_aggregate (Python) → detailed_match (C++)
                                                                                                                       ↓
                                                                                                          doa_to_obstacles.py → /doa_obstacles
```

- **Primary entry point:** `umap/launch/simulation.launch`
- **Primary integration:** `umap/launch/simulation_dync.launch` (Gazebo + DOA + MPC)
- **Messages:** `umap/msg/*.msg` (9 custom types), `path_planning/msg/*.msg` (2 types)
- **Configs:** `umap/config/*.yaml` (6 files — sim, exp, rosbag, test, etc.)

### Cluster 2: MPC Planning (Active Core, Dual Implementation)

| Component | Stack A (Tracked) | Stack B (Untracked) |
|-----------|------------------|---------------------|
| Solver | `mpc_nav` — Python CasADi/IPOPT | `mpc_planner` — C++ Acados/ForcesPro |
| Obstacle model | Swept capsule + KF uncertainty | GMM, Ellipsoid, Linearized, Scenario-based |
| Max obstacles | 5 (fixed) | 12 (configurable) |
| Trajectories | Single | Multiple parallel (T-MPC++) |
| Global planner | None (point goal) | guidance_planner (PRM topology-distinct) |
| Status | **Integrated** (DOA → adapter → mpc_node) | **Bridge exists** (DOA → doa_to_tmpc.py → jackalsimulator) |

**Integration bridge (untracked):** `mpc_nav/scripts/doa_to_tmpc.py` + `mpc_nav/launch/tmpc_doa.launch`

### Cluster 3: Experiment + Analysis (Active)

```
exp/launch/avoidance_exp.launch → Gazebo + DOA + MPC + goal
exp/launch/demo_exp.launch      → Occlusion demo with cmd_vel gate
exp/launch/perception_dispatch.launch → Method-selector (ours_*/dodt/fapp/oracle_gt)
exp/scripts/run_avoidance_exp.sh → Multi-method experiment orchestrator
exp/scripts/stats_viz/*.py       → 13 offline analysis scripts
```

**Data:** `exp/data/avoidance/` (active, May 24-27), `exp/data/sup_*/` (May 12, prior), `exp/data/crossing/` (April, stale)

### Cluster 4: Third-Party / Reference Baselines

| Component | Purpose | Status |
|-----------|---------|--------|
| **FAPP** | Drone perception-planning baseline. Only `mot_mapping` used for comparison | Reference only (own git) |
| **onboard_detector** | DODT baseline (DBSCAN + Kalman) | Wired via `perception_dispatch.launch` method:=dodt |
| **jackal_simulator** | Jackal UGV simulation (7 pkgs) | Standalone, unused by current pipeline |

---

## Git Status Summary

```
develop branch — 1 commit ahead?
  Modified:  DOA (submodule changes), mpc_nav/ (4 files), my_tb3_description/ (4 files)
  Deleted:   src/voxblox
  Untracked: ~15 new packages, .catkin_tools/, figures/, install_*.sh, logfile.txt, task_plan.md, findings.md, progress.md

  Total tracked: ~5 original packages
  Total new:     ~14 untracked packages
```
