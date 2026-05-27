# Questions for Human

## Priority: Need Decision to Unblock Next Steps

### Q1: T-MPC Integration Direction

The workspace now has **two parallel MPC pipelines**:

| Pipeline | Implementation | Status |
|----------|---------------|--------|
| A: DOA → `doa_to_obstacles.py` → `mpc_node.py` | Python CasADi/IPOPT | **Active, tracked** |
| B: DOA → `doa_to_tmpc.py` → `jackalsimulator_planner` | C++ Acados/ForcesPro | **Untracked, bridge exists** |

The bridge files (`doa_to_tmpc.py`, `tmpc_doa.launch`) are untracked. The entire `mpc_planner/` (40000 lines C++), `guidance_planner/`, `roadmap/` are untracked repos.

**Question:** Which direction do you want to go?
- (a) Keep both pipelines — commit the T-MPC bridge as a long-term candidate
- (b) Focus on Pipeline A (Python CasADi) — defer T-MPC, untracked packages are "just for reference"
- (c) Migrate to Pipeline B (C++ Acados) — make `mpc_planner/` the primary planner

---

### Q2: motion_capture_system Path References

Several files hardcode the workspace path as `~/simulation_framework`:
- `tmux_session.template.conf`
- `exp/scripts/stats_viz/sup_exp_figures.py`

Current actual path is `~/sim_dev/simulation_framework`.

**Question:** Is `~/sim_dev` the permanent location, and should I fix hardcoded paths? Or is this expected to change?

---

### Q3: DOA path_planning (move_base + DWA) — Legacy or Active?

The `DOA/path_planning/` package provides:
- Two costmap plugins (`DynamicObstacleLayer`, `DWAController`)
- Complete `move_base` configuration
- Custom `Obstacle.msg` / `ObstacleArray.msg` (duplicated from mpc_nav's types)

This appears to be an earlier attempt at dynamic obstacle avoidance using the standard ROS navigation stack (move_base + DWA), superseded by the direct MPC approach.

**Question:** Is this still under consideration, or should it be marked as "historical reference"? The dynamic obstacle costmap layer could be reusable.

---

### Q4: FAPP — Full Tree vs. mot_mapping Only

FAPP occupies `src/FAPP/` as a 22 sub-package monorepo (~15000 lines). Only its `mot_mapping` perception front-end (DBSCAN + Kalman) is used in experiments. The full FAPP planning stack (quadrotor simulation, polynomial trajectory optimization) is drone-specific and never integrated.

**Question:** Can we remove the full FAPP tree and keep only `mot_mapping/` + `obj_state_msgs/`? They're the only parts that `perception_dispatch.launch` and `extract_tracker.py` touch. (We have the full repo in git if we ever need the rest.)

---

## Priority: Medium

### Q5: DODT (onboard_detector) — Worth Fixing?

The `perception_dispatch.launch method:=dodt` pipeline is **broken** — the adapter expects `umap/TrackerDetailArray` but `onboard_detector` publishes `visualization_msgs/MarkerArray`. A proper `topic_to_obstacles.py` style bridge is needed.

**Question:** Is the DODT baseline comparison still important for your paper? If so, fixing this bridge is straightforward. If not, we can leave it.

---

### Q6: Perception Dispatch — 3 Broken Methods

Per the exp audit, `perception_dispatch.launch` has 3 methods that are explicitly broken (confirmed by launch-file comments and code inspection):
- `stage1_only` — coarse outputs `umap/Obstacles`, adapter expects `TrackerDetailArray`
- `dodt` — outputs `MarkerArray`, same mismatch
- `fapp` — outputs `ObjectsStates`, same mismatch

Currently only `ours_full`, `ours_no_clip`, `ours_no_sam`, and `oracle_gt` work end-to-end.

**Question:** Should I write proper adapter bridges for these 3 methods, or is the current set sufficient for your experiments?

---

### Q7: `sup_exp_figures.py` Hardcoded Paths

`scripts/stats_viz/sup_exp_figures.py` has hardcoded paths to `exp/data/sup_tracking_to_planning/` for its input data. Your current active experiment data is under `exp/data/avoidance/`. The paths point to a May 12 data snapshot, not the current May 24-27 results.

**Question:** Do you want the figure-generation script updated to point to `data/avoidance/`? Or is the `sup_tracking_to_planning/` tree the canonical one for the paper?

---

### Q8: dashgo_description — Is Dashgo Still a Target?

`dashgo_description/` has zero external references. It was listed in the original README but no launch file, script, or config anywhere references it.

**Question:** Are there plans to use the Dashgo robot? Or can this be archived?

---

## Priority: Informational

### Q9: stale_ros1_branch on asr_rapidxml

`asr_rapidxml` has two branches: `main` and `ros1` (local-only). Current checkout is on `main`. The `ros1` branch has unmerged changes.

**Question:** Anything to know about the `ros1` branch? Or is `main` the right one?

### Q10: mpc_planner Dingo Package Disabled

Inside `mpc_planner/mpc_planner_dingo/`, there are `CATKIN_IGNORE` and `COLCON_IGNORE` marker files, disabling the Dingo robot deployment. Intentional or accidental?

### Q11: DOA Submodule Modifications

`git status` shows `DOA` as a modified submodule (modified content + untracked content). These include the new scene scripts (`crossing_pedestrian.py`, `pedestrian_sine_wave.py`) and mpc_nav's new files.

**Question:** Should these changes be committed to the submodule, or are they untracked work-in-progress?
