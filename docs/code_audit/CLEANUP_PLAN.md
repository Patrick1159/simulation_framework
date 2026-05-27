# Cleanup Plan

**Based on:** `docs/code_audit/` (REPO_MAP, ENTRYPOINTS, EXPERIMENT_INDEX, CODE_INVENTORY, DUPLICATES_AND_OVERLAPS, GARBAGE_TRIAGE, QUESTIONS_FOR_HUMAN)
**User context:**
- FAPP, DODT = needed baselines
- T-MPC (C++ Acados) = current focus, mpc_nav (Python CasADi) = retained for comparison
- **Constraint: no moving/renaming/deleting/modifying any files yet**

---

## 1. Recommended Target Structure

*Not moving anything yet. Shown as a future ideal — dotted lines indicate optional components.*

```
simulation_framework/
├── README.md                          # Updated with full package map
├── CLAUDE.md                          # Updated
│
├── src/                               # Core ROS packages (active)
│   ├── DOA/                           # KEEP — perception pipeline
│   │   └── umap/                      #   Core: clean_umap, coarse_match, detailed_match
│   │   └── pointcloud_generator/      #   Support: depth→pointcloud
│   │   └── path_planning/             #   ┐ ← P2: legacy move_base, worth documenting
│   │
│   ├── mpc_nav/                       # KEEP — Python CasADi MPC (comparison baseline)
│   ├── my_tb3_description/            # KEEP — simulation environment
│   ├── dashgo_description/            # ?    ← Q8: Is Dashgo still used?
│   ├── my_gazebo_test/                # ┐ ← P3: not a ROS package, minimal value
│   │
│   ├── exp/                           # KEEP — experiment framework
│   │   ├── launch/                    #   avoidance_exp, demo_exp, perception_dispatch
│   │   ├── scripts/                   #   orchestrators + stats_viz/
│   │   ├── configs/                   #   avoidance_corridor, avoidance_crossing, demo_occlusion
│   │   ├── data/                      #   experiment outputs
│   │   └── worlds/                    #   S1_static_corridor.world
│   │
│   ├── mpc_planner/                   # KEEP — T-MPC (current focus)
│   ├── guidance_planner/              # KEEP — required by mpc_planner_modules
│   ├── roadmap/                       # KEEP — exec_depend of mpc_planner
│   ├── ros_tools/                     # KEEP — shared utility, required by all tud-amr pkgs
│   ├── pedestrian_simulator/          # KEEP — exec_depend of mpc_planner
│   ├── pedsim_original/               # KEEP — required by pedestrian_simulator
│   ├── asr_rapidxml/                  # KEEP — required by pedestrian_simulator + roadmap
│   │
│   ├── onboard_detector/              # KEEP — DODT baseline
│   ├── FAPP/                          # KEEP — FAPP baseline (perception front-end needed)
│   │
│   ├── jackal_simulator/              # ?    ← Q4: Jackal vs Dashgo robot?
│   ├── DecompUtil/                    # ?    ← unused (dep commented out)
│   └── catkin_simple/                 # ┐ ← P3: unused build tool after voxblox removal
│
├── docs/                              # Add documentation
│   └── code_audit/                    #   audit documents (already done)
│   └── experiment_workflow.md         #   ADD: how to run each experiment
│   └── MPC_comparison.md              #   ADD: mpc_nav vs mpc_planner differences
│
├── external/                          # RECOMMENDED: archive dir (not created yet)
│   └── munkres-cpp/                   #   move from DOA/src/munkres-cpp/ (byte-for-byte dup)
│   └── dashgo_description/            #   IF Dashgo is no longer a target
│   └── jackal_simulator/              #   IF Jackal is not used
│   └── DecompUtil/                    #   IF T-MPC's commented dep stays commented
│
└── tools/                             # RECOMMENDED: dev utilities
    └── kill_ros_gazebo.sh             #   move from root
    └── tmux_session.template.conf     #   move from root
```

---

## 2. Cleapriority (P0–P3)

### P0: Must Keep — Core Pipeline

| Package | Rationale |
|---------|-----------|
| `DOA/` | Primary perception pipeline (umap + pointcloud_generator) |
| `my_tb3_description/` | Simulation environment, dynamic scenes. Referenced by 8+ launch files. |
| `mpc_nav/` | Python MPC baseline for comparison. Also bridges to T-MPC (`doa_to_tmpc.py`). |
| `exp/` | Active experiment framework (May 24-27). Configs, orchestrators, analysis scripts. |
| `exp/launch/avoidance_exp.launch` | Main experiment entry point |
| `exp/launch/perception_dispatch.launch` | Method selector (6 methods) |
| `exp/scripts/stats_viz/*.py` | All 13 analysis scripts — core pipeline evaluation |

### P0: Must Keep — T-MPC Stack

| Package | Rationale |
|---------|-----------|
| `mpc_planner/` | Current focus. C++ Acados T-MPC framework. |
| `guidance_planner/` | Topology-distinct global planner for T-MPC. Required by mpc_planner_modules. |
| `roadmap/` | Reference path provider. exec_depend of all mpc_planner deployments. |
| `ros_tools/` | Shared utility. Required by ALL tud-amr packages. |
| `pedestrian_simulator/` | exec_depend of mpc_planner deployments. |
| `pedsim_original/` | Required by pedestrian_simulator. |
| `asr_rapidxml/` | Required by pedestrian_simulator + roadmap. |

### P0: Must Keep — Baselines

| Package | Rationale |
|---------|-----------|
| `onboard_detector/` | DODT baseline. Wired in perception_dispatch (though broken). |
| `FAPP/` | FAPP baseline. mot_mapping used as comparison method. |

---

### P1: Worth Integrating Into Mainline

| Item | Action | Rationale |
|------|--------|-----------|
| `mpc_nav/scripts/doa_to_tmpc.py` | **Document + commit** | Existing but untracked. Bridges DOA→T-MPC. Essential if you're running both pipelines. |
| `mpc_nav/launch/tmpc_doa.launch` | **Document + commit** | Untracked launcher for the bridge. |
| `mpc_nav/scripts/robot_straight_line.py` | **Document + commit** | Untracked. Test utility for straight-line S1 corridor experiments. |
| `perception_dispatch.launch` broken methods | **Document the 3 known breaks** | stage1_only, dodt, fapp all have type mismatches. At minimum document the workaround needed for each. |

---

### P2: Experimental Code — Archive or Document

| Item | Rationale |
|------|-----------|
| `DOA/path_planning/` | Legacy move_base + DWA approach. Altern dynamic obstacle costmap layer might be reusable. Document existence but not actively used. |
| `DOA/src/munkres-cpp/` | **Confirmed duplicate.** Byte-for-byte identical to `umap/include/munkres/`. The standalone copy is never built. Can be archived. |
| `umap/scripts/feature_extract.py` + `main.launch` | Superseded by parallel trio (para_samnode/clipnode/aggregate). Legacy pipeline. Document as deprecated. |
| `dashgo_description/` | No external references. If Dashgo not a target, archive. |
| `jackal_simulator/` | Complete Jackal UGV stack. Zero external references. If not using Jackal, archive. |
| `DecompUtil/` | Dependency commented out in mpc_planner_solver. Optional until T-MPC needs static obstacle decomp. |
| `DOA/src/voxblox/` | Already deleted from disk but submodule still in `.gitmodules`. Remove from git tracking. |

---

### P3: Suspected Garbage — Confirmation Needed

| Item | Action | Confidence |
|------|--------|------------|
| `my_gazebo_test/` | Not a ROS package (no package.xml). A box model + empty world. **Delete candidate.** | high |
| `catkin_simple/` | Build tool. Only used by deleted voxblox chain. Remaining packages use plain catkin. **Delete candidate.** | high |
| `umap/include/umap/dynamic_detector.h` | Declared but **never implemented** (no .cpp). Never included. **Delete candidate.** | high |
| `umap/include/umap/visualize_utils.h` | **Empty file** (0 bytes). **Delete candidate.** | high |
| `umap/src/convert_bbox_frame.cpp` | Compiled but never launched by any launch file. Dead code. **Delete candidate.** | high |
| `pointcloud_generator/scripts/fake_pose.py` | Not referenced by any launch file. Superseded. **Delete candidate.** | high |
| `umap/config/umap_params.yaml` | Superseded by simulation.yaml. **Delete candidate.** | high |
| `install_voxblox_deps_no_rosdep.sh` | Script for deleted packages. **Delete candidate.** | high |
| `logfile.txt` (1.2MB) | Debug artifact. **Delete candidate.** | high |
| `.catkin_tools/` | Cache directory. **Can be .gitignored.** | medium |
| `src/figures/` | Duplicate of root `figures/`. **Verify and delete.** | medium |

---

## 3. Per-File Action Matrix

| Path | Action | Priority | Evidence |
|------|--------|----------|----------|
| `src/DOA/` (all subpackages) | keep | P0 | Core perception pipeline |
| `src/DOA/src/munkres-cpp/` | archive later | P2 | Byte-for-byte duplicate of umap/include/munkres/ |
| `src/DOA/src/umap/include/umap/dynamic_detector.h` | delete candidate | P3 | Never implemented, never included |
| `src/DOA/src/umap/include/umap/visualize_utils.h` | delete candidate | P3 | 0-byte empty file |
| `src/DOA/src/umap/src/convert_bbox_frame.cpp` | delete candidate | P3 | Compiled but never launched |
| `src/DOA/src/umap/scripts/feature_extract.py` | document (deprecated) | P2 | Superseded by para_* trio, only main.launch uses it |
| `src/DOA/src/umap/launch/main.launch` | document (deprecated) | P2 | Legacy pipeline using feature_extract.py |
| `src/DOA/src/umap/config/umap_params.yaml` | delete candidate | P3 | Superseded by simulation.yaml |
| `src/DOA/src/path_planning/` | document | P2 | Legacy move_base approach |
| `src/DOA/src/pointcloud_generator/scripts/fake_pose.py` | delete candidate | P3 | Never launched |
| `src/mpc_nav/scripts/mpc_node.py` | keep | P0 | Active MPC pipeline (comparison baseline) |
| `src/mpc_nav/scripts/mpc_solver.py` | keep | P0 | Library for mpc_node |
| `src/mpc_nav/scripts/doa_to_obstacles.py` | keep | P0 | DOA→mpc_nav adapter |
| `src/mpc_nav/scripts/doa_to_tmpc.py` | keep, commit (untracked) | P1 | DOA→T-MPC bridge |
| `src/mpc_nav/launch/tmpc_doa.launch` | keep, commit (untracked) | P1 | T-MPC integration launch |
| `src/mpc_nav/scripts/robot_straight_line.py` | keep, document | P1 | S1 test utility |
| `src/mpc_planner/` | keep | P0 | T-MPC (current focus) |
| `src/guidance_planner/` | keep | P0 | Required by mpc_planner_modules |
| `src/roadmap/` | keep | P0 | exec_depend of mpc_planner |
| `src/ros_tools/` | keep | P0 | Shared utility, required everywhere |
| `src/pedestrian_simulator/` | keep | P0 | exec_depend of mpc_planner |
| `src/pedsim_original/` | keep | P0 | Required by pedestrian_simulator |
| `src/asr_rapidxml/` | keep | P0 | Required by pedestrian_simulator + roadmap |
| `src/onboard_detector/` | keep | P0 | DODT baseline |
| `src/FAPP/` (all) | keep | P0 | FAPP baseline |
| `src/my_tb3_description/` | keep | P0 | Simulation environment |
| `src/exp/` | keep | P0 | Active experiment framework |
| `src/exp/scripts/stats_viz/sup_exp_figures.py` | document (hardcoded paths) | P1 | Point to avoidance/ not sup_tracking_to_planning/ |
| `src/dashgo_description/` | need human decision | P2 | No external references |
| `src/jackal_simulator/` | need human decision | P2 | No external references |
| `src/DecompUtil/` | need human decision | P2 | Dep commented out in mpc_planner_solver |
| `src/catkin_simple/` | delete candidate | P3 | Unused build tool |
| `src/my_gazebo_test/` | delete candidate | P3 | Not a ROS package, zero references |
| `.gitmodules` (voxblox entry) | document (stale) | P1 | voxblox submodule line — should be removed |
| `install_voxblox_deps_no_rosdep.sh` | delete candidate | P3 | Script for deleted packages |
| `logfile.txt` | delete candidate | P3 | Debug artifact |
| `tmux_session.template.conf` | document (stale paths) | P2 | Paths outdated |

---

## 4. Risk Assessment — What Would Break If Deleted

### Critical (would break experiment pipeline):

| If deleted | Breaks |
|------------|--------|
| `DOA/umap/` | **Everything.** Core perception — no obstacle detection, no tracking. |
| `DOA/src/umap/config/simulation.yaml` | DOA simulation launch — wrong depth params, no detection. |
| `mpc_nav/scripts/mpc_node.py` | MPC avoidance fails. Both avoidance_exp.launch and mpc.launch break. |
| `mpc_nav/scripts/doa_to_obstacles.py` | DOA→mpc_nav bridge broken. No obstacles reach Python MPC. |
| `mpc_nav/scripts/doa_to_tmpc.py` | DOA→T-MPC bridge broken. Untracked but essential for T-MPC pipeline. |
| `exp/launch/avoidance_exp.launch` | Experiment framework broken. |
| `exp/launch/perception_dispatch.launch` | All method-based experiments fail. |
| `my_tb3_description/launch/my_tb3_gazebo.launch` | Gazebo simulation broken. Robot cannot spawn. |
| `my_tb3_description/scripts/` (any scene) | Dynamic obstacle scenes broken. |
| `my_tb3_description/scripts/sdf.py` | ALL scene scripts fail — shared model library. |
| `mpc_planner/` | T-MPC broken (current focus). |
| `guidance_planner/` | mpc_planner_modules compilation fails. |
| `ros_tools/` | ALL tud-amr packages (mpc_planner, guidance_planner, roadmap, etc.) fail to compile. |
| `pedestrian_simulator/` | mpc_planner jackalsimulator runtime fails. |
| `onboard_detector/` | DODT baseline unavailable for comparison. |

### Medium (experiment reproduction fails but pipeline still runs):

| If deleted | Breaks |
|------------|--------|
| `exp/scripts/stats_viz/*.py` | Offline analysis. Can't compute metrics or generate figures. |
| `exp/data/` | Can't reproduce prior experiment results. |
| `FAPP/mot_mapping/` | fapp method in perception_dispatch unavailable. |
| `roadmap/` | mpc_planner deployment packages fail. |

### Low (build or dev quality):

| If deleted | Breaks |
|------------|--------|
| `dashgo_description/` | Nothing. Zero external references. |
| `jackal_simulator/` | Nothing. Zero external references. |
| `catkin_simple/` | Nothing. No remaining package uses it. |
| `my_gazebo_test/` | Nothing. No package references it. |
| `DOA/src/munkres-cpp/` | Nothing. umap/include/munkres/ is the active copy. |
| `DecompUtil/` | mpc_planner_solver comment-out means nothing breaks now. |

---

## 5. Minimum Safe Cleanup — 3 Phases

### Phase 1: Documentation Only (add, never remove)

**Effort:** ~2 hours
**Risk:** None — only adds files, never touches existing code.

| # | Action | Files |
|---|--------|-------|
| 1.1 | Update `README.md` to reflect all 19 packages | `README.md` |
| 1.2 | Update `CLAUDE.md` with both MPC pipelines documented | `CLAUDE.md` |
| 1.3 | Fix hardcoded paths in `sup_exp_figures.py` | `exp/scripts/stats_viz/sup_exp_figures.py` |
| 1.4 | Document known breaks in perception_dispatch | comment or README |
| 1.5 | Document the T-MPC bridge (`doa_to_tmpc.py`) workflow | `docs/experiment_workflow.md` |
| 1.6 | Document mpc_nav vs mpc_planner differences | `docs/MPC_comparison.md` |
| 1.7 | Commit untracked T-MPC bridge files | `doa_to_tmpc.py`, `tmpc_doa.launch`, `robot_straight_line.py`, new scene scripts |
| 1.8 | Remove voxblox from `.gitmodules` | `.gitmodules` (edit one line only) |
| 1.9 | Add `.catkin_tools/` to `.gitignore` | `.gitignore` |

**Verification:** `catkin build` still works, all experiments launchable.

---

### Phase 2: Archive Low-Risk Items (wait for confirmation)

**Effort:** ~30 min
**Risk:** Low — these items have zero external references.

| # | Action | Files |
|---|--------|-------|
| 2.1 | **CONFIRM Q1:** mp_planner as default? | Keeps both pipelines, documents superiority. |
| 2.2 | **CONFIRM Q2:** Is Dashgo still a target? | If no, create `external/dashgo_description/`. |
| 2.3 | **CONFIRM Q3:** Is Jackal still a target? | If no, create `external/jackal_simulator/`. |
| 2.4 | **CONFIRM Q4:** DecompUtil — dead or needed? | If no T-MPC static decomp needed, archive. |
| 2.5 | **CONFIRM Q5:** Archive munkres-cpp/ | If yes, create `external/munkres-cpp/`. |

---

### Phase 3: Delete Candidates (only after confirmation)

**Effort:** ~15 min
**Risk:** Medium — ensure nothing depends on these.

| # | Action | Files |
|---|--------|-------|
| 3.1 | **CONFIRM Q6:** Delete my_gazebo_test? | Verify no external references. |
| 3.2 | **CONFIRM Q7:** Delete catkin_simple? | Verify `grep -r catkin_simple` only finds self-references. |
| 3.3 | **CONFIRM Q8:** Delete empty/broken files? | dynamic_detector.h, visualize_utils.h, fake_pose.py, umap_params.yaml |
| 3.4 | **CONFIRM Q9:** Delete convert_bbox_frame.cpp? | Verify not launched anywhere. |
| 3.5 | **CONFIRM Q10:** Delete install script + logfile? | `install_voxblox_deps_no_rosdep.sh`, `logfile.txt` |

---

## 6. Ten Decisions I Need From You

These are ordered by how much they affect what I recommend next.

| # | Question | Options | Impact |
|---|----------|---------|--------|
| **Q1** | Is `mpc_planner` (T-MPC, C++ Acados) now the **default** planner, with `mpc_nav` (Python CasADi) kept as comparison baseline? | Yes / Both equal / mpc_nav primary | Affects which bridge gets priority, documentation, and which launch file is "recommended" |
| **Q2** | Is `dashgo_description` still a target robot? | Yes / No, archive it | Affects cleanup scope |
| **Q3** | Is `jackal_simulator` still a target? | Yes / No, archive it | Affects cleanup scope |
| **Q4** | `DecompUtil` — do you anticipate needing static convex decomposition for T-MPC? | Yes, keep / No, archive | Affects archive scope |
| **Q5** | The `munkres-cpp/` duplicate — archive to `external/`? | Yes / No | Minor |
| **Q6** | `my_gazebo_test` — not a ROS package, zero references. Delete? | Yes / No | Minor |
| **Q7** | `catkin_simple` — unused build tool. Delete? | Yes / No | Minor |
| **Q8** | Dead DOA files: `dynamic_detector.h` (no impl), `visualize_utils.h` (empty), `convert_bbox_frame.cpp` (never launched), `fake_pose.py` (never launched), `umap_params.yaml` (superseded) — delete all? | Yes / Delete subset / Keep | Minor |
| **Q9** | Should I fix the 3 broken `perception_dispatch.launch` methods (stage1_only, dodt, fapp) with proper adapter bridges? | Yes, all 3 / Just dodt+fapp / None, document only | Affects whether comparison experiments can run |
| **Q10** | What is the **active scene list** for current experiments? | pair_crossing / random_multi / front_crossing / corridor / occlusion? | Determines which exp/config files are actively needed, and what broken methods affect you |

## Human answers
Q1: T-MPC是现在默认的planner, 两者都保留
Q2: 如果现在用了它的模型文件，就保留，如果没有就删除
Q3: 不是，但是留着，后续可能转进去
Q4: 保留
Q5: no
Q6: delete
Q7: Keep
Q8: 先不管
Q9: 标记下来我会安排做
Q10: 端到端避障测试只有pair crossing, random multi和corridor，另外几个做量化指标测试。都留着
---

## Quick Reference: What NOT to Delete (Ever)

```
DOA/umap/          → perception pipeline
mpc_nav/           → Python MPC (baseline) + T-MPC bridge files
exp/               → experiment framework + data
my_tb3_description/ → simulation environment
mpc_planner/       → T-MPC (current focus)
guidance_planner/  → required by T-MPC
ros_tools/         → required by ALL tud-amr packages
pedestrian_simulator/ → exec_depend of T-MPC
onboard_detector/  → DODT baseline
FAPP/              → FAPP baseline
```

Everything else is either archive-candidate or delete-candidate.
