# Duplicates and Overlaps

## Confirmed Duplicates (Code Reuse Opportunities)

### D1: Munkres Library Embedded Twice

**Files:**
- `DOA/src/umap/include/munkres/{munkres,matrix}.{h,cpp}` (4 files, ~799 lines)
- `DOA/src/munkres-cpp/src/{munkres,matrix}.{h,cpp}` (4 files, ~799 lines)

**Verdict:** BYTE-FOR-BYTE IDENTICAL. The `umap/include/munkres/` copy is the one actually used (included by `coarse_matcher.cpp` and `detailed_matcher.cpp`). The standalone `munkres-cpp/` directory is **never built** by any CMakeLists.txt in the workspace — no `add_subdirectory`, no `find_package`, nothing.

**Overlap score:** 5/5 (exact duplicate)
**Evidence:** Same file sizes, same content, same commit history in embedded form
**Confidence:** High

---

### D2: Parallel Feature Extraction vs Monolithic

**Files (parallel, 884 lines total):**
- `DOA/src/umap/scripts/para_samnode.py` (442 lines) — FastSAM in dedicated node
- `DOA/src/umap/scripts/para_clipnode.py` (268 lines) — CLIP in dedicated node
- `DOA/src/umap/scripts/para_aggregate.py` (174 lines) — Fusion node

**Monolithic (259 lines):**
- `DOA/src/umap/scripts/feature_extract.py` (259 lines) — Combined SAM+CLIP+aggregate in one node

**Overlap score:** 4/5 (functionally overlapping)
**Evidence:** Both implement the same SAM→CLIP→aggregate pipeline. The shared libraries (`config_manager.py`, `model_manager.py`, `utils.py`, `node_helpers.py`) are used by both.
**Status:** Parallel trio is active (used by 6 launch files); monolithic is legacy (only used by `main.launch`).
**Confidence:** High

---

### D3: mpc_nav/ObstacleArray vs mpc_planner_msgs/ObstacleArray

**Files:**
- `mpc_nav/msg/Obstacle.msg` + `ObstacleArray.msg` — flat list (position, velocity, radius, ellipse, KF covariance)
- `mpc_planner/.../mpc_planner_msgs/msg/ObstacleArray.msg` + `ObstacleGMM.msg` + `Gaussian.msg` — GMM probabilistic representation

**Overlap score:** 3/5 (semantically overlapping, structurally incompatible)
**Evidence:** Same semantic purpose (dynamic obstacle representation for MPC), completely incompatible message schemas. Bridge needed (`doa_to_tmpc.py`).
**Confidence:** High

---

## Overlapping Implementations

### O1: Camera Pose Scripts (3 scripts, similar purpose)

| Script | Lines | Used in | Purpose |
|--------|-------|---------|---------|
| `camera_pose.py` | 54 | simulation.launch, experiment.launch | TF→PoseStamped (dynamic) |
| `static_pose_tf.py` | 70 | para_main.launch, sim_pointcloud_processor | Static PoseStamped (fixed TF) |
| `fake_pose.py` | 67 | **None** (never launched) | Odometry→pose relay |

**Overlap score:** 2/5 (related but distinct)
**Verdict:** `fake_pose.py` is dead; the other two serve different use cases.
**Confidence:** High

---

### O2: Rematch Profilers (2 scripts)

| Script | Lines | Subscribes To |
|--------|-------|---------------|
| `rematch_profiler.py` | 266 | `/rematch_statistics` (single topic) |
| `rematch_profiler_live.py` | 264 | `/rematch_statistics_sam` + `_clip` + `_aggregator` (3 topics) |

**Overlap score:** 3/5 (overlapping, different pipeline versions)
**Verdict:** First is for legacy pipeline, second for parallel pipeline. Not exact duplicates but similar structure.
**Confidence:** High

---

### O3: Analysis Scripts with Similar Purposes

| Script | Input | What |
|--------|-------|------|
| `statistics.py` | `/rematch/tracker_details` | Tracker statistics (manual) |
| `full_pipe_statistics.py` | `/rematch/tracker_details` + more | Full pipeline latency (manual) |

**Overlap score:** 2/5
**Verdict:** Overlapping; `full_pipe_statistics.py` is the more feature-rich version.
**Confidence:** Medium

---

### O4: DOA vs path_planning Message Definitions

- `umap/msg/TrackerDetail.msg` / `TrackerDetailArray.msg` — the active DOA tracking output format
- `path_planning/msg/Obstacle.msg` / `ObstacleArray.msg` — legacy move_base obstacle format

**Overlap score:** 3/5
**Evidence:** Both represent detected obstacles with ID, position, velocity. `TrackerDetail` has KF covariance; `Obstacle` has size + radius. Different packages, different consumers (detailed_match vs move_base).
**Confidence:** High

---

### O5: mpc_nav Obstacle Model vs mpc_planner Obstacle Model

| Feature | mpc_nav (Python CasADi) | mpc_planner (C++ Acados) |
|---------|------------------------|-------------------------|
| Obstacle shape | Circular + swept capsule | Ellipsoid, bounding box |
| Prediction model | Constant velocity | GMM distribution, scenario-based |
| Uncertainty | KF covariance diagonal (sigma2) | Full Gaussian covariance |
| Interface | `mpc_nav/ObstacleArray` | `mpc_planner_msgs/ObstacleArray` |

**Overlap score:** 4/5 (same purpose, different sophistication levels)
**Verdict:** This is an architectural divergence, not a code duplicate. Both pipelines exist simultaneously.
**Confidence:** High
