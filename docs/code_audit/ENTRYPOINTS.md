# Entry Points — Everything Runnable

---

## A. Combined Launch (DOA + MPC + Gazebo)

| Command | File | What It Does |
|---------|------|-------------|
| `roslaunch umap simulation_dync.launch` | `umap/launch/simulation_dync.launch` | **Full system test**: Gazebo + TB3 + dynamic obstacles + DOA perception + MPC + RViz |
| `roslaunch umap simulation.launch` | `umap/launch/simulation.launch` | **Perception only** (simulation mode): DOA pipeline without MPC or Gazebo |
| `roslaunch umap experiment.launch` | `umap/launch/experiment.launch` | **Real experiment** (D400 camera): DOA pipeline for hardware |
| `roslaunch mpc_nav mpc.launch` | `mpc_nav/launch/mpc.launch` | **MPC + Gazebo + DOA adapter** — start MPC, spawn robot + obstacles |
| `roslaunch mpc_nav mpc_real.launch` | `mpc_nav/launch/mpc_real.launch` | **MPC real robot** — T265 odometry, real_baselink frame |
| `roslaunch mpc_nav tmpc_doa.launch` | `mpc_nav/launch/tmpc_doa.launch` | **DOA → T-MPC bridge** (using mpc_planner C++, untracked) |

---

## B. Experiment Launch (exp/sup_exp)

| Command | File | What It Does |
|---------|------|-------------|
| `roslaunch sup_exp avoidance_exp.launch method:=ours_full` | `exp/launch/avoidance_exp.launch` | **Main experiment**: Gazebo + method perception + MPC + goal. Methods: ours_full/no_clip/no_sam/stage1_only/dodt/fapp/oracle_gt |
| `roslaunch sup_exp avoidance_exp.launch method:=ours_full dynamic_scene:=2` | same | Same, with scene select (0=random, 1=corridor, 2=crossing, 3=occlusion) |
| `roslaunch sup_exp demo_exp.launch method:=ours_full` | `exp/launch/demo_exp.launch` | **Occlusion demo**: same as avoidance with cmd_vel gate (`apply_cmd_vel:=false` to suppress motion) |
| `roslaunch sup_exp S1_dynamic_corridor.launch` | `exp/launch/S1_dynamic_corridor.launch` | **S1 corridor**: TB3 straight line + dual sine-wave pedestrians. No perception/MPC. |
| `roslaunch sup_exp S1_static_corridor.launch` | `exp/launch/S1_static_corridor.launch` | **S1 corridor viewer**: Gazebo-only, no robot. |
| `roslaunch sup_exp perception_dispatch.launch method:=ours_full` | `exp/launch/perception_dispatch.launch` | **Method dispatch only** (included by avoidance_exp): starts perception pipeline by method name |

---

## C. DOA Perception (Standalone)

| Command | File | What It Does |
|---------|------|-------------|
| `roslaunch umap umap.launch` | `umap/launch/umap.launch` | Minimal: detection only `umap_extract`, no tracking |
| `roslaunch umap coarse_match.launch` | `umap/launch/coarse_match.launch` | Standalone coarse tracker |
| `roslaunch umap detailed_match.launch` | `umap/launch/detailed_match.launch` | Standalone detailed tracker (camera frame) |
| `roslaunch umap main.launch` | `umap/launch/main.launch` | **Legacy**: uses monolithic `feature_extract.py` (not para_* trio) |
| `roslaunch umap para_main.launch` | `umap/launch/para_main.launch` | Para variant: no Gazebo, static camera |
| `roslaunch umap real_rosbag.launch` | `umap/launch/real_rosbag.launch` | Rosbag replay: publish camera topics, run DOA |
| `roslaunch umap feature_extractor.launch` | `umap/launch/feature_extractor.launch` | SAM + CLIP only, no detection or tracking |
| `rosrun umap umap_extract` | `umap/src/clean_umap.cpp` | Core detection node |
| `rosrun umap coarse_match` | `umap/src/clean_coarse_match.cpp` | Coarse tracking node |
| `rosrun umap detailed_match` | `umap/src/clean_detailed_match.cpp` | Detailed (rematch) tracking node |
| `rosrun umap bbox_frame_converter` | `umap/src/convert_bbox_frame.cpp` | Camera→World frame converter (never launched) |

---

## D. Pointcloud Generator (Standalone)

| Command | File | What It Does |
|---------|------|-------------|
| `rosrun pointcloud_generator processor` | `pointcloud_generator/src/processor.cpp` | Depth→PointCloud2 processing |
| `roslaunch pointcloud_generator pointcloud_processor_node.launch` | pointcloud_generator/launch/... | Standalone pointcloud processor |
| `roslaunch pointcloud_generator sim_pointcloud_processor_node.launch` | same/... | Simulation variant with static_pose_tf |

---

## E. Simulation Environments (Standalone)

| Command | File | What It Does |
|---------|------|-------------|
| `roslaunch my_tb3_description my_tb3_gazebo.launch` | my_tb3_description/launch/... | Gazebo + TurtleBot3 Burger + D435 |
| `roslaunch my_tb3_description dync_obs.launch scene:=0` | same/launch/dync_obs.launch | Scene 0: Random walk obstacles |
| `roslaunch my_tb3_description dync_obs.launch scene:=1` | same | Scene 1: Corridor moving cylinders |
| `roslaunch my_tb3_description dync_obs.launch scene:=2` | same | Scene 2: Two crossing pedestrians (with ID jitter simulation) |
| `roslaunch my_tb3_description dync_obs.launch scene:=3` | same | Scene 3: Single crossing pedestrian |
| `roslaunch my_tb3_description dync_obs.launch scene:=4` | same | Scene 4: Dual sine-wave from doorways |
| `roslaunch my_tb3_description static_obs.launch` | same/launch/static_obs.launch | Static obstacle field |

---

## F. Walk-Through (move_base + DWA)

| Command | File | What It Does |
|---------|------|-------------|
| `roslaunch path_planning move_base.launch` | DOA/path_planning/launch/move_base.launch | move_base + DWA controller + costmap |
| `roslaunch path_planning run_dynamic_layer.launch` | same/launch/run_dynamic_layer.launch | move_base + DynamicObstacleLayer plugin |

---

## G. Python Data Analysis (exp/scripts/stats_viz/)

All have `if __name__ == "__main__"` — can run standalone.

| Script | Typical Command |
|--------|----------------|
| `extract_gt.py` | `python3 extract_gt.py --bag raw.bag --output gt.csv` |
| `extract_tracker.py` | `python3 extract_tracker.py --bag raw.bag --output tracker.csv` |
| `calc_metrics.py` | `python3 calc_metrics.py --gt gt.csv --tracker tracker.csv --bag raw.bag` |
| `calc_prediction_error.py` | `python3 calc_prediction_error.py --gt gt.csv --tracker tracker.csv` |
| `calc_planning_metrics.py` | `python3 calc_planning_metrics.py --bag raw.bag --output-dir .` |
| `calc_occlusion_reid.py` | `python3 calc_occlusion_reid.py --gt gt.csv --tracker tracker.csv --occlusion-start 8 --occlusion-duration 2` |
| `calc_runtime_table.py` | `python3 calc_runtime_table.py --bag raw.bag --output-dir .` |
| `plot_results.py` | `python3 plot_results.py --data-dir ./run_1` |
| `gen_compare.py` | `python3 gen_compare.py --scene pair_crossing --methods "Ours:/path" "DODT:/path" --output ./compare` |
| `visualize_keyframes.py` | `python3 visualize_keyframes.py ./run_1` |
| `compare_tracking_keyframes.py` | `python3 compare_tracking_keyframes.py --methods "Ours:/path" "DODT:/path" --output-dir ./kf` |
| `summarize_sup_results.py` | `python3 summarize_sup_results.py --root data/avoidance` |
| `make_corridor_tables.py` | `python3 make_corridor_tables.py --root data/avoidance/corridor` |
| `sup_exp_figures.py` | `python3 sup_exp_figures.py` (hardcoded paths) |

---

## H. Experiment Orchestration Shell Scripts

| Command | File | What It Does |
|---------|------|-------------|
| `bash exp/scripts/run_avoidance_exp.sh` | `exp/scripts/run_avoidance_exp.sh` | Multi-method avoidance experiment runner |
| `bash exp/scripts/run_demo_exp.sh` | `exp/scripts/run_demo_exp.sh` | Occlusion/demo experiment runner |
| `bash exp/scripts/process_bag.sh <dir>` | `exp/scripts/process_bag.sh` | One-stop bag → CSVs → metrics → figures |

---

## I. Standalone Python Utilities (DOA)

| Script | Purpose |
|--------|---------|
| `rosrun umap rematch_profiler.py` | Live profiler for `/rematch_statistics` (single topic) |
| `rosrun umap rematch_profiler_live.py` | Live profiler, 3 topic parallel version |
| `rosrun umap statistics.py` | Tracker statistics from `/rematch/tracker_details` |
| `rosrun umap full_pipe_statistics.py` | Full pipeline latency statistics |
| `rosrun umap teleop.py` | Keyboard teleop for TB3 |

---

## J. Known Broken Entry Points

| Entry Point | Problem |
|------------|---------|
| `avoidance_exp.launch method:=stage1_only` | Type mismatch: coarse publishes `umap/Obstacles`, adapter expects `umap/TrackerDetailArray` |
| `avoidance_exp.launch method:=dodt` | Type mismatch: `onboard_detector` publishes `MarkerArray`, adapter expects `TrackerDetailArray` |
| `avoidance_exp.launch method:=fapp` | Type mismatch: `mot_mapping` publishes `ObjectsStates`, adapter expects `TrackerDetailArray` |
| `sup_exp_figures.py` | Hardcoded paths to `data/sup_tracking_to_planning/` — won't use `data/avoidance/` |
| All SUPPLEMENTAL_EXPERIMENTS.md scripts | Referenced `.sh` files (`run_tracking_to_planning_sup.sh`, etc.) do not exist |
