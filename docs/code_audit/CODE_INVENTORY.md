# Code Inventory — Categorized, Scored

Scoring:
- **Usefulness** (0–5): 0=dead, 5=active core
- **Confidence** (low/medium/high)

---

## A. Core / Active — DOA Perception Pipeline

| File | Category | Usefulness | Confidence | Evidence |
|------|----------|------------|------------|----------|
| `DOA/src/umap/src/clean_umap.cpp` | **A - Core** | 5 | high | Main executable `umap_extract`. Entry point of all perception pipelines. |
| `DOA/src/umap/src/clean_coarse_match.cpp` | **A - Core** | 5 | high | Main executable `coarse_match`. Stage I coarse tracking. |
| `DOA/src/umap/src/clean_detailed_match.cpp` | **A - Core** | 5 | high | Main executable `detailed_match`. Stage II rematch tracking. |
| `DOA/src/umap/src/coarse_matcher.cpp` | **A - Core** | 5 | high | C++ library — trackerNode implementation. |
| `DOA/src/umap/src/detailed_matcher.cpp` | **A - Core** | 5 | high | C++ library — rematchNode implementation. |
| `DOA/src/umap/include/umap/KalmanFilter.cpp/h` | **A - Core** | 5 | high | Used by both coarse_match and detailed_match. |
| `DOA/src/umap/include/umap/ObstacleTracker.cpp/h` | **A - Core** | 5 | high | Used by both trackers. |
| `DOA/src/umap/include/umap/umapProcess.cpp/h` | **A - Core** | 5 | high | UMAP core processing (bbox extraction). |
| `DOA/src/umap/include/umap/umap_bbox_extraction.cpp` | **A - Core** | 5 | high | Bounding box extraction from depth. |
| `DOA/src/umap/include/umap/umap_visualization.cpp` | **A - Core** | 5 | high | RViz visualization markers. |
| `DOA/src/umap/include/umap/coarse_match.h` | **A - Core** | 5 | high | trackerNode class header. |
| `DOA/src/umap/include/umap/rematch.h` | **A - Core** | 5 | high | rematchNode class header. |
| `DOA/src/umap/include/umap/umapType.h` | **A - Core** | 5 | high | Core type definitions. |
| `DOA/src/umap/include/umap/utils.h` | **A - Core** | 5 | high | Shared utilities. |
| `DOA/src/umap/include/umap/params.h` | **A - Core** | 5 | high | Parameter definitions. |
| `DOA/src/umap/scripts/para_samnode.py` | **A - Core** | 5 | high | Parallel FastSAM node. Used by 6 launch files. |
| `DOA/src/umap/scripts/para_clipnode.py` | **A - Core** | 5 | high | Parallel CLIP node. Same usage. |
| `DOA/src/umap/scripts/para_aggregate.py` | **A - Core** | 5 | high | Parallel aggregate node. Same usage. |
| `DOA/src/umap/scripts/config_manager.py` | **A - Core** | 5 | high | Shared library imported by all para_* nodes. |
| `DOA/src/umap/scripts/model_manager.py` | **A - Core** | 5 | high | Model loading library. |
| `DOA/src/umap/scripts/utils.py` | **A - Core** | 4 | high | ImageProcessor, ColorGenerator utilities. |
| `DOA/src/umap/scripts/node_helpers.py` | **A - Core** | 4 | high | Shared helper functions. |
| `DOA/src/umap/config/simulation.yaml` | **A - Core** | 5 | high | Simulation config (CV default). |
| `DOA/src/umap/config/experiment.yaml` | **A - Core** | 5 | high | Real experiment config. |
| `DOA/src/pointcloud_generator/src/processor.cpp` | **A - Core** | 4 | high | C++ depth→pointcloud processor. Used but optional. |
| `DOA/src/pointcloud_generator/scripts/camera_pose.py` | **A - Core** | 4 | high | TF→PoseStamped, used by 3 launch files. |
| `DOA/src/pointcloud_generator/param/sim_param.yaml` | **A - Core** | 4 | high | Sim pointcloud params. |
| `DOA/src/pointcloud_generator/include/pointcloud_process.h/.cpp` | **A - Core** | 4 | high | Point cloud processing library. |

---

## B. Core / Active — MPC Navigation

| File | Category | Usefulness | Confidence | Evidence |
|------|----------|------------|------------|----------|
| `mpc_nav/scripts/mpc_node.py` | **A - Core** | 5 | high | Main MPC node. Entry point of mpc.launch + avoidance_exp.launch. |
| `mpc_nav/scripts/mpc_solver.py` | **A - Core** | 5 | high | CasADi MPC solver. Library imported by mpc_node.py. |
| `mpc_nav/scripts/doa_to_obstacles.py` | **A - Core** | 5 | high | DOA→mpc_nav ObstacleArray adapter. |
| `mpc_nav/config/mpc.yaml` | **A - Core** | 5 | high | MPC solver parameters. |
| `mpc_nav/msg/Obstacle.msg` | **A - Core** | 5 | high | Primary obstacle message format. |
| `mpc_nav/msg/ObstacleArray.msg` | **A - Core** | 5 | high | Primary obstacle array format. |
| `mpc_nav/launch/mpc.launch` | **A - Core** | 5 | high | Main MPC launch file. |
| `mpc_nav/scripts/doa_to_tmpc.py` | **B - Experiment** | 3 | medium | DOA→mpc_planner bridge (untracked). Depends on T-MPC integration direction. |
| `mpc_nav/launch/tmpc_doa.launch` | **B - Experiment** | 3 | medium | T-MPC integration launch (untracked). |

---

## C. Core / Active — Simulation Environment

| File | Category | Usefulness | Confidence | Evidence |
|------|----------|------------|------------|----------|
| `my_tb3_description/launch/my_tb3_gazebo.launch` | **A - Core** | 5 | high | Gazebo + TB3 spawn. Referenced by 8+ launch files. |
| `my_tb3_description/launch/dync_obs.launch` | **A - Core** | 5 | high | Dynamic obstacle dispatcher (scenes 0–4). |
| `my_tb3_description/scripts/dync_scene1.py` | **A - Core** | 4 | high | Scene 1: corridor moving cylinders. |
| `my_tb3_description/scripts/random_dynamic_obstacle.py` | **A - Core** | 4 | high | Scene 0: random walk obstacles. |
| `my_tb3_description/scripts/sim_exp_1.py` | **A - Core** | 4 | high | Scene 2: crossing pedestrians with ID jitter. |
| `my_tb3_description/scripts/crossing_pedestrian.py` | **A - Core** | 4 | high | Scene 3: single crossing pedestrian. |
| `my_tb3_description/scripts/pedestrian_sine_wave.py` | **A - Core** | 4 | high | Scene 4: dual sine-wave from doorways. |
| `my_tb3_description/scripts/sdf.py` | **A - Core** | 4 | high | SDF model generator library. Imported by all scene scripts. |
| `my_tb3_description/scripts/set_pose.py` | **A - Core** | 4 | high | Initial pose relay for Gazebo. |
| `my_tb3_description/scripts/teleop.py` | **C - Utility** | 2 | high | Keyboard teleop. Manual use only. |

---

## D. Core / Active — Experiment Package

| File | Category | Usefulness | Confidence | Evidence |
|------|----------|------------|------------|----------|
| `exp/launch/avoidance_exp.launch` | **A - Core** | 5 | high | Main experiment launch (May 2026). |
| `exp/launch/demo_exp.launch` | **A - Core** | 4 | high | Occlusion demo launch with cmd_vel gate. |
| `exp/launch/perception_dispatch.launch` | **A - Core** | 5 | high | Method dispatcher (ours_*/dodt/fapp/oracle_gt). |
| `exp/launch/S1_dynamic_corridor.launch` | **A - Core** | 3 | medium | S1 corridor experiment. Not yet part of main avoidance workflow. |
| `exp/launch/S1_static_corridor.launch` | **C - Utility** | 2 | medium | Environment viewer only. |
| `exp/scripts/run_avoidance_exp.sh` | **A - Core** | 5 | high | Multi-method experiment orchestrator. |
| `exp/scripts/run_demo_exp.sh` | **A - Core** | 4 | high | Demo experiment orchestrator. |
| `exp/scripts/process_bag.sh` | **C - Utility** | 4 | high | Bag reprocessing pipeline. |
| `exp/scripts/cmd_vel_gate.py` | **A - Core** | 4 | high | MPC cmd_vel gate for demo_exp. |
| `exp/scripts/goal_publisher.py` | **A - Core** | 4 | high | Fixed goal publisher from YAML. |
| `exp/configs/avoidance_corridor.yaml` | **A - Core** | 4 | high | Current experiment config. |
| `exp/configs/avoidance_crossing.yaml` | **A - Core** | 4 | high | Current experiment config. |
| `exp/configs/demo_occlusion.yaml` | **A - Core** | 4 | high | Current experiment config. |
| `exp/configs/S1_static_corridor.yaml` | **C - Utility** | 2 | medium | S1 blueprint (design doc, not runtime). |

### stats_viz Scripts (all 13)

| Script | Category | Usefulness | Confidence | Evidence |
|--------|----------|------------|------------|----------|
| `extract_gt.py` | **C - Utility** | 4 | high | Bag→GT CSV extraction. Required by metrics pipeline. |
| `extract_tracker.py` | **C - Utility** | 4 | high | Bag→tracker CSV. Multi-format support. |
| `calc_metrics.py` | **C - Utility** | 4 | high | MOTA/IDSW/FP/FN computation. |
| `calc_prediction_error.py` | **C - Utility** | 4 | high | Prediction RMSE at multiple horizons. |
| `calc_planning_metrics.py` | **C - Utility** | 4 | high | Collision/clearance metrics from bag. |
| `calc_occlusion_reid.py` | **C - Utility** | 3 | medium | Occlusion-specific metrics. Only useful if occlusion experiment runs. |
| `calc_runtime_table.py` | **C - Utility** | 3 | medium | Per-module runtime stats. |
| `plot_results.py` | **C - Utility** | 4 | high | Per-run figures (BEV, prediction RMSE, MOTA bars). |
| `gen_compare.py` | **C - Utility** | 4 | high | Cross-method comparison figures. |
| `compare_tracking_keyframes.py` | **C - Utility** | 3 | medium | Multi-panel keyframe comparison. |
| `visualize_keyframes.py` | **C - Utility** | 4 | high | Keyframe visualization from clearance timeseries. |
| `summarize_sup_results.py` | **C - Utility** | 4 | high | Per-run→summary aggregation. |
| `make_corridor_tables.py` | **C - Utility** | 3 | medium | Corridor planning + runtime tables. |
| `sup_exp_figures.py` | **C - Utility** | 4 | medium | Publication figures from summaries. Hardcoded paths limit reuse. |

---

## E. Experiment / Candidate

| File | Category | Usefulness | Confidence | Evidence |
|------|----------|------------|------------|----------|
| `mpc_planner/` (entire monorepo, ~40000 lines) | **B - Candidate** | 3 | medium | T-MPC C++ framework. Bridge exists but untracked. Depends on research direction. |
| `guidance_planner/` | **B - Candidate** | 3 | medium | Required by mpc_planner_modules if T-MPC proceeds. |
| `roadmap/` | **B - Candidate** | 3 | medium | exec_depend of mpc_planner deployment. |
| `onboard_detector/` | **B - Candidate** | 3 | medium | DODT baseline, wired but broken in perception_dispatch. |
| `FAPP/mot_mapping/` | **B - Candidate** | 3 | medium | Perception front-end used in exp comparisons. |
| `FAPP/obj_state_msgs/` | **B - Candidate** | 2 | medium | Message types for mot_mapping. |

---

## F. Utility / Reusable

| File | Category | Usefulness | Confidence | Evidence |
|------|----------|------------|------------|----------|
| `ros_tools/` | **C - Utility** | 4 | high | Shared C++ utilities (visualization, logging, profiling, math). Used by all tud-amr packages. |
| `pedestrian_simulator/` | **C - Utility** | 3 | medium | Pedsim wrapper with 100+ scenarios. Used as exec_depend by mpc_planner deployments. |
| `pedsim_original/` | **C - Utility** | 3 | medium | Social force model library. Required by pedestrian_simulator. |
| `asr_rapidxml/` | **C - Utility** | 3 | medium | XML parser. Required by pedestrian_simulator + roadmap. |
| `DecompUtil/` | **C - Utility** | 2 | medium | Convex decomposition library. Dependency commented out in local mpc_planner_solver. |

---

## G. Duplicate / Overlapping

| File | Category | Usefulness | Confidence | Evidence |
|------|----------|------------|------------|----------|
| `DOA/src/munkres-cpp/` (standalone) | **D - Duplicate** | 0 | high | Byte-for-byte copy of `umap/include/munkres/`. Never built. |
| `umap/scripts/feature_extract.py` | **D - Duplicate** | 2 | high | Superseded by para_samnode+para_clipnode+para_aggregate. Still used by main.launch. |
| `umap/launch/main.launch` | **D - Duplicate** | 2 | high | Legacy pipeline launcher. Uses feature_extract.py (monolithic). |

---

## H. Dead / Unreachable

| File | Category | Usefulness | Confidence | Evidence |
|------|----------|------------|------------|----------|
| `DOA/src/umap/src/convert_bbox_frame.cpp` | **E - Dead** | 1 | high | Compiled but never launched anywhere. |
| `DOA/src/umap/include/umap/dynamic_detector.h` | **E - Dead** | 0 | high | Class declared but never implemented (no .cpp). Not included by any file. |
| `DOA/src/umap/include/umap/visualize_utils.h` | **E - Dead** | 0 | high | Empty file (0 bytes). |
| `pointcloud_generator/scripts/fake_pose.py` | **E - Dead** | 1 | high | Not referenced by any launch file. |
| `dashgo_description/` | **E - Dead** | 0 | medium | No external references in any launch/script/config. |
| `my_gazebo_test/` | **E - Dead** | 1 | medium | Not a ROS package. Test world with Gazebo pedestrian actor. |
| `jackal_simulator/` (7 pkgs) | **E - Dead** | 1 | medium | Complete Jackal simulation stack. Zero external references in workspace. |
| `umap/config/umap_params.yaml` | **E - Dead** | 0 | high | Superseded by simulation.yaml. Not referenced in modern pipelines. |

---

## I. Generated / Messy

| File | Category | Usefulness | Confidence | Evidence |
|------|----------|------------|------------|----------|
| `logfile.txt` (1.2MB) | **F - Messy** | 1 | high | ROS console log with ANSI codes. Debug artifact. |
| `install_voxblox_deps_no_rosdep.sh` | **F - Messy** | 1 | high | Stale install script for deleted packages. |
| Simulated ID jitter in `sim_exp_1.py` | **F - Messy** | 2 | medium | Intentionally noisy perception simulation. Messy but purposeful code. |

---

## J. Unknown / Need Human Check

| Item | Category | Usefulness | Confidence | Why |
|------|----------|------------|------------|-----|
| `DOA/path_planning/` (entire package) | **G - Unknown** | 2 | medium | Alternative move_base+DWA integration path. Not wired into current experiments. Has dynamic obstacle costmap layer that could be valuable. |
| `tmux_session.template.conf` | **G - Unknown** | 2 | medium | References stale paths (`~/simulation_framework` not `~/sim_dev`). Was this for real robot demos? |
| `figures/sup_exp.md` report | **G - Unknown** | 3 | medium | Contains result tables from prior experiment run. Useful for paper writing but path-dependent. |
