# Garbage Triage — Suspected Dead/Unreachable Code

## Priority Scale

1. **Leave alone** — harmless, might be useful
2. **Archive** — package up and remove from active tree
3. **Can delete later** — not harmful now, but has no active use
4. **Confirm with human** — might be important, need judgment
5. **Extract fragments** — some reusable parts inside dead code

---

## Tier 1: Leave Alone (can be ignored)

| Item | Location | Why |
|------|----------|-----|
| DOA benchmark_utils/ | `umap/scripts/benchmark_utils/` (8 files) | Offline analysis tools. Not hurting anything, might be useful for future hardware evaluation. |
| `logfile.txt` (1.2MB) | Root directory | ROS console log. Leave alone — debug artifact. |
| `my_gazebo_test/` | `src/my_gazebo_test/` | Not a proper ROS package (no package.xml), but contains a box model and test world. Harmless. |
| `statistics.py`, `full_pipe_statistics.py` | `umap/scripts/` | Manual analysis scripts. Could be useful for debugging. |

---

## Tier 2: Archive (package up and remove from tree)

| Item | Location | Why |
|------|----------|-----|
| `munkres-cpp/` (standalone copy) | `DOA/src/munkres-cpp/` | ~800 lines, byte-for-byte identical to `umap/include/munkres/`. Never built. Third-party library with benchmarks, tests, examples — all dead. The embedded copy in `umap/include/` is the active one. |

**Recommendation:** Archive to `external/` if you want to preserve the git history, then delete from `DOA/src/`.

---

## Tier 3: Can Delete Later (safe to remove, not harming anything)

| Item | Location | Evidence |
|------|----------|----------|
| `bbox_frame_converter` | `umap/src/convert_bbox_frame.cpp` | Compiled and installed but never launched by any launch file anywhere in the workspace. |
| `visualize_utils.h` | `umap/include/umap/visualize_utils.h` | **Empty file** (0 bytes). |
| `dynamic_detector.h` | `umap/include/umap/dynamic_detector.h` | Class declared with private ctor/dtor, never implemented (no .cpp), never included by any other file. |
| `fake_pose.py` | `pointcloud_generator/scripts/fake_pose.py` | Not referenced by any launch file. Superseded by `camera_pose.py` and `static_pose_tf.py`. |
| `feature_extract.py` + `main.launch` (legacy pipeline) | `umap/scripts/` + `umap/launch/` | Superseded by modern `para_samnode/para_clipnode/para_aggregate` trio. Only used by `main.launch`. |
| `umap_params.yaml` (legacy) | `umap/config/umap_params.yaml` | Minimal params file, superseded by `simulation.yaml`. |

---

## Tier 4: Need Human Confirmation

| Item | Location | Question | Risk |
|------|----------|----------|------|
| `dashgo_description/` | `src/dashgo_description/` | Tracked in git, has package.xml, but zero external references. Is Dashgo still a target robot? | Low — data-only package |
| `FAPP/` (20+ sub-packages) | `src/FAPP/` | Only `mot_mapping` perception front-end is used as baseline. The full planning stack is drone-specific. Can the full FAPP tree be removed, keeping only `mot_mapping`? | Medium — git history lost if deleted |
| `jackal_simulator/` (7 sub-packages) | `src/jackal_simulator/` | Complete Jackal UGV simulation stack, untracked. Is Jackal still used? The `tmpc_doa.launch` file references `jackalsimulator_planner` from `mpc_planner_jackalsimulator` (inside mpc_planner/, separate from this). | Low — not referenced |
| `onboard_detector/` | `src/onboard_detector/` | C++ DODT baseline with YOLO learning module. Wired in `perception_dispatch.launch` but **broken** (type mismatch). Worth fixing or retire? | Low — has its own git |
| `guidance_planner/` | `src/guidance_planner/` | Required by `mpc_planner_modules`. If T-MPC integration proceeds, needed. If not, dead. | Medium — depends on T-MPC direction |
| `roadmap/` | `src/roadmap/` | Same as guidance_planner — exec_depend of mpc_planner deployment packages. | Medium — depends on T-MPC |
| `DecompUtil/` | `src/DecompUtil/` | mpc_planner_solver's dependency was commented out. Not used anywhere else. | Low — own git, not integrated |
| `catkin_simple/` | `src/catkin_simple/` | Remaining ETH-ASL build tool. Not used by any remaining package. Safe to remove. | Low — not needed |

---

## Tier 5: Extract Reusable Fragments

| Item | Location | What Can Be Saved |
|------|----------|-------------------|
| `FAPP` → `mot_mapping/` | `FAPP/mot_mapping/` | The DBSCAN clustering + Kalman tracking node is the only part used in experiments. Could be extracted as standalone package. |
| `FAPP` → `obj_state_msgs/` | `FAPP/obj_state_msgs/` | Only used by mot_mapping and the experiment adapter. |
| `onboard_detector` → `fake_detector` | `onboard_detector/.../fakeDetector.*` | Gazebo ground-truth obstacle detector. Could be useful as a testing tool independent of full DODT. |
| `pedsim_original/` | `pedsim_original/` | Social force model C++ library. The pedestrian_simulator depends on it, so it must stay with that package. |

---

## Summary: What to Do With What

| Action | Count | Contents |
|--------|-------|----------|
| Leave alone | ~5 items | benchmark_utils, logfile, my_gazebo_test, statistics scripts |
| Archive | 1 item | munkres-cpp/ (standalone copy) |
| Can delete later | 6 items | bbox_frame_converter, visualize_utils.h, dynamic_detector.h, fake_pose.py, feature_extract.py+main.launch, umap_params.yaml |
| Need confirmation | 9 items | dashgo_description, FAPP (full), jackal_simulator, onboard_detector (keep or fix?), guidance_planner, roadmap, DecompUtil, catkin_simple |
| Extract fragments | 4 items | mot_mapping, obj_state_msgs, fake_detector, pedsim_original (keep with dep) |
