# Experiment Index

All known experiments, comparison directions, and research branches identified in the repository.

---

## Exp A: Main DOA + MPC Avoidance (Active)

**Status:** Active (May 24-27 data)
**Config file:** `exp/configs/avoidance_crossing.yaml`, `exp/configs/avoidance_corridor.yaml`
**Launch:** `roslaunch sup_exp avoidance_exp.launch method:=ours_full dynamic_scene:=2`
**Orchestrator:** `exp/scripts/run_avoidance_exp.sh`
**Data:** `exp/data/avoidance/`

**Methods compared (6):**
| Method | Description | Compared |
|--------|-------------|----------|
| `ours_full` | Full DOA (SAM + CLIP + rematch) | ✅ Active |
| `ours_no_clip` | DOA w/o CLIP (ablation) | ✅ Active |
| `ours_no_sam` | DOA w/o SAM (ablation) | ✅ Active |
| `stage1_only` | Coarse tracker only | ✅ Active |
| `dodt` | Onboard DODT baseline | ✅ Planned |
| `fapp` | FAPP baseline | ✅ Planned |
| `oracle_gt` | Ground truth (upper bound) | ✅ Active |

**Sub-scenes:**
| Scene | dync_obs ID | Description | Data coverage |
|-------|-------------|-------------|---------------|
| `pair_crossing` | 2 | Two crossing pedestrians | 6 methods × run_1 complete |
| `random_multi` | 0 | Random multi-object | ours_no_sam, stage1_only, ours_no_clip done |
| `front_crossing` | 3 | Single crossing | Not in avoidance/ |
| `corridor` | 1 | Corridor moving cylinders | ours_no_clip bag only |

**Key results:** Summary CSVs exist in `exp/data/avoidance/summary_*.csv` (May 25)

---

## Exp B: Controlled Occlusion + Re-ID

**Status:** Partially complete (prior experiment branch)
**Config file:** `exp/configs/demo_occlusion.yaml`
**Launch:** `roslaunch sup_exp demo_exp.launch method:=ours_full`
**Orchestrator:** `exp/scripts/run_demo_exp.sh`

**Parameters:** occlusion_duration 0.5–4.0s, 7 methods
**Data:** Not found in `data/avoidance/`. Prior data exists only in `data/sup_tracking_to_planning/` (May 12) and `data/crossing/` (April).
**Special feature:** `cmd_vel_gate.py` gates MPC output (`/mpc/cmd_vel_raw` → `/cmd_vel`), allowing MPC analysis without robot motion.

**Status:** Likely paused after prior session was interrupted (`progress.md` mentions 21 runs not started).

---

## Exp C: Corridor Planning-Level + Runtime Benchmark

**Status:** Incomplete (prior experiment branch)
**Config:** Uses `dync_scene1.py` with 5 cylinders crossing corridor
**Launch:** Previously via `run_corridor_planning_table.sh` (now deleted)
**Data:** `exp/data/sup_corridor_planning/` (May 12) — single method `ours_no_sam`, 192MB bag, no post-processing.

**Status:** Not actively pursued in current `avoidance/` tree.

---

## Exp D: S1 Static Corridor

**Status:** Environment ready, experiment possible
**Config:** `configs/S1_static_corridor.yaml` (blueprint)
**World:** `worlds/S1_static_corridor.world` (Gazebo)
**Launch:** `roslaunch sup_exp S1_dynamic_corridor.launch` (TB3 + pedestrians)
**Data:** None collected yet under systematic experiment.

**Purpose:** Paper experiment S1 — single pedestrian crossing static corridor.

---

## Exp E: FAPP Baseline Comparison

**Status:** Reference only (not actively integrated)
**Files used:** `FAPP/mot_mapping/` only (perception front-end)
**Not used:** Full FAPP planning stack (quadrotor sim, polynomial optimization)
**Bridge:** `doa_to_tmpc.py` → converts DOA TrackerDetailArray → mpc_planner_msgs/ObstacleArray (untracked)
**Data:** `exp/data/avoidance/pair_crossing/fapp/run_1/` has pre-existing results
**Known issue:** perception_dispatch.launch method:=fapp is **broken** due to message type mismatch

**Status:** External reference baseline, no active development needed.

---

## Exp F: T-MPC (mpc_planner) Integration

**Status:** Experimental/Prototype (untracked code)
**Files:**
- `mpc_nav/scripts/doa_to_tmpc.py` — DOA→T-MPC bridge (untracked)
- `mpc_nav/launch/tmpc_doa.launch` — Launch integration (untracked)
**Components:** `mpc_planner/`, `guidance_planner/`, `roadmap/`, `ros_tools/` (all untracked)
**Solver:** Acados C++ (not the Python CasADi IPOPT used by mpc_nav)
**Feature:** Multiple parallel trajectory optimization + topology-distinct guidance

**Status:** Bridge exists but not officially integrated. Untracked, not committed.

---

## Exp G: Real-World Deployment

**Status:** Previously tested, not currently active
**Configs:**
- `mpc_nav/config/mpc.yaml` (has real-world params)
- `mpc_nav/launch/mpc_real.launch` (T265 odometry)
- `umap/config/experiment.yaml` (D400 camera parameters)
- `tmux_session.template.conf` (real-robot tmux layout)

**Artifacts:** `logfile.txt` (1.2MB) shows successful mpc_planner run with acados solver.

---

## Exp H: move_base + DWA Integration

**Status:** Legacy / abandoned
**Files:** `DOA/path_planning/` — costmap plugins, DynamicObstacleLayer, DWAController
**Launches:** `move_base.launch`, `run_dynamic_layer.launch`
**Messages:** Custom `path_planning/Obstacle.msg` (duplicated in spirit by mpc_nav/Obstacle.msg)
**Status:** Not wired into any active experiment. The dynamic obstacle costmap layer could still be useful but appears superseded by the MPC approach.
