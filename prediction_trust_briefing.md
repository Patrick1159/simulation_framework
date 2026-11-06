# 障碍物预测信任度问题 — 交叉验证简报

> 用途：交给多个 agent 独立评估「DOA→T-MPC 桥接层预测几何」的根本性重构方案。
> 每条事实标注了**证据等级**：[已验证]=读过代码/日志确认；[推断]=基于代码逻辑推理但未跑实验确认；[现象]=运行观察，主观。

---

## 1. 目的 (Objective)

把 bridge (`doa_to_tmpc.py`) 里的障碍物预测几何从**启发式硬切换**改成**有原则的、不需要反复手调单一阈值**的方案。

具体要同时满足两个目前互相打架的指标：
- **避障要早**：MPC 在远期 horizon stage 要"看见"障碍物未来会到的位置，避免贴近才打方向（晚避）。
- **求解要稳**：远期预测不能过度约束 NLP，导致 Acados/SQP 求解失败率飙升。

**反面教材（要摆脱的）**：单个 `cv_horizon` 标量在 1.0 / 1.5 / 2.0 之间来回试 —— 这是 whack-a-mole。

---

## 2. 系统背景 (System Context) [已验证]

- **Planner**：T-MPC (Topology-driven MPC)，C++ + Acados/SQP-RTI。
  - `N=30`, `dt=0.2s` → 预测 horizon = 6s。
  - `control_frequency=20Hz`。
  - `max_obstacles=12`, `obstacle_radius=0.3`, `robot_radius=0.18`。
  - `probabilistic.enable = false`（**当前走 DETERMINISTIC 路径**）。
  - 配置：`src/mpc_planner/mpc_planner_jackalsimulator/config/settings.yaml`。
- **障碍物约束**：`ellipsoid_constraints.cpp`
  - DETERMINISTIC 模式：`Major=0, Minor=0, Chi=1`，只用标量 `obstacle.radius`（即一个圆，不是椭球）。[已验证 lines 71-76]
  - GAUSSIAN 模式：用 `major_semiaxis / minor_semiaxis` + chi-square 置信度（chance constraint）。
- **入口解析**：`ros1_jackalsimulator.cpp` lines 264-291
  - line 291：当 `major_semiaxis.back()==0` 或 probabilistic 关闭 → 标记为 DETERMINISTIC。[已验证]
- **Bridge**：`src/mpc_nav/scripts/doa_to_tmpc.py`
  - 当前逻辑：**short-CV-then-freeze** —— `t < cv_horizon` 时用恒速 (CV) 外推障碍物位置；`t >= cv_horizon` 时**冻结**在 cv_horizon 处的位置不再前进。
  - `major_semiaxis = minor_semiaxis = rate`（per-step 不确定率，非预积分）。[已验证]
  - `emit_uncertainty=false`（当前）。
  - watchdog：感知静默时发空数组。

---

## 3. 上游感知背景 [已验证]

- 数据流：`umap_extract → coarse_match → detailed_match → bridge`。
- **coarse_match** (`coarse_matcher.cpp`)：Hungarian 关联 + 恒速 Kalman Filter。
  - 关联代价：`cost = iou_weight*iou_cost + dist_weight*distance`，带硬门限 `DYNAMIC_IOU_THRESHOLD` / `DYNAMIC_DIST_THRESHOLD`。
  - **KF 协方差已存在**（CV 模型自带 position covariance）—— 这是关键，方案 A 可直接复用，无需新建状态估计。[推断：需确认协方差是否暴露到 detailed_match → bridge 的消息里]
  - 关联时 **没有 predict-before-match**（line 146 直接用 `getState().position`）。
- **SAM 当前已关闭**（`use_sam:=false`），走 coarse + CLIP-passthrough。
- 测试场景 **scene 3**：开阔无墙、恒速、撞边界镜面反射（billiard）。**反射瞬间速度突变** → 对 CV-KF 是强压力测试（bounce 那一刻 CV 外推必然错）。

---

## 4. 已知现象与证据 (Phenomena + Evidence)

| # | 现象 | 证据等级 | 说明 |
|---|------|---------|------|
| P1 | `watchdog_timeout=0.01` 在健康 30Hz 感知下误触发，~5次/秒刷空障碍数组 → 规划抖动 | [已验证] haiku日志确认，改 0.5 后 missing-obstacle 事件降到 3 | **已修复**，与本问题正交 |
| P2 | `cv_horizon=1.0` → 远期 stage 看到冻结障碍物 → 晚避 | [现象]+[推断] | 冻结位置滞后于真实位置 |
| P3 | `cv_horizon=2.0` → solve 成功率 100%→63.5% (337/923 失败) | [已验证] parse_tmpc_log | 远期自信幻影过度约束 NLP |
| P4 | `cv_horizon=1.5` 为折中 | [现象] 待重跑确认 | 仍是猜一个 cliff-edge |
| P5 | 移动起来追踪 ID churn（31 IDs / 6 障碍物） | [现象] | IoU 硬门限在快速运动下杀近距匹配；已调 iou_weight 0.7→0.2, dist_weight→0.8, IOU门限→0.0，待重跑确认 |

**注意 P5 与本问题的耦合**：如果上游 ID churn 没解决，track 频繁重生 → 速度估计被重置 → 任何下游预测方案都吃到坏速度。需要 agent 判断：**预测信任度问题能否独立于 ID 稳定性单独解决**，还是必须先稳住 ID。

---

## 5. 当前 heuristic 的本质缺陷 (Root Cause)

`cv_horizon` 把"预测可信度"建模成**二值悬崖**：
- `t < cv_horizon`：100% 信任 CV 外推（硬圆约束钉在预测点）。
- `t >= cv_horizon`：0% 信任（冻结）。

但真实的预测置信度是**随时间连续衰减**的：0.4s 外的 CV 猜测很准；3s 外基本是噪声（障碍物可能转向/停/反弹）。用一个标量阈值去切这条连续曲线，必然在"早避"和"可解"之间无法同时满足 → 所以怎么调都是 whack-a-mole。

**待验证的核心命题**：把"信任度"建模成**随 horizon 增长的不确定性**（而非冻结），让 solver 自然地对远期不确定预测降权，就能消除 `cv_horizon` 这个旋钮。

---

## 6. 候选方案 (Candidate Directions) — 供交叉验证

### 方案 A：概率/增长不确定性 (Probabilistic, growing uncertainty)
- 启用 T-MPC 的 **GAUSSIAN chance-constraint 路径**（`probabilistic.enable=true`）。
- bridge 发**完整 CV 预测**（不冻结），椭球半轴 = KF 协方差沿 horizon 前向传播 → 近期 stage 紧约束（高信任），远期 stage 大软区域（低信任）。
- **优点**：原则性强，彻底删除 `cv_horizon`；复用已有 KF 协方差。
- **风险/待查**：
  - GAUSSIAN 路径在本 jackalsimulator 配置下是否真的能跑通、solve 率如何？
  - KF 协方差是否真的传到 bridge（消息字段）？还是要在 bridge 里重新积分 `Q`？
  - chance-constraint 的 chi-square 置信度怎么设，会不会又变成一个新旋钮？

### 方案 B：保持 DETERMINISTIC，更聪明的预测 (smarter deterministic)
- 不开概率路径。把二值冻结换成**按 track 置信度自适应的 per-track horizon**：用 track age、速度一致性、KF innovation 决定 CV 信任多远。
- **优点**：改动小，不碰 solver 路径。
- **缺点**：仍是"更软的启发式"，没从根上去掉旋钮，只是把一个全局旋钮换成一组自适应规则。

### 方案 C：A/B 都做，同场景实测对比
- 同一 scene 3 上比 solve rate + 避障时机，用数据选。

---

## 7. 给交叉验证 agent 的关键问题 (Questions to Cross-Validate)

1. **GAUSSIAN 路径可行性**：`mpc_planner_jackalsimulator` + Acados 当前编译配置是否支持 chance constraint？开 `probabilistic.enable=true` 需不需要重新生成 solver？（查 `mpc_planner_solver` / acados 生成代码）
2. **协方差来源**：coarse KF 的 position covariance 是否经 `detailed_match` 传到 `/rematch/tracker_details`？bridge 能直接读，还是要自己积分过程噪声 Q？
3. **是否引入新旋钮**：方案 A 的 chi-square 置信度 / Q 是不是只是把 `cv_horizon` 换成另一个要手调的量？哪个更有原则？
4. **耦合性**：P5（ID churn）是否必须先解决？坏速度喂给任何预测方案都会坏 —— 预测信任度重构能否独立推进？
5. **billiard 反弹**：恒速假设在 bounce 处必错。增长不确定性能否吸收这个 model mismatch，还是需要机动检测 / IMM 多模型？
6. **更根本的替代**：有没有第三条路被我漏了？（如 guidance_planner 拓扑层处理、reachability/可达集、occupancy 时空走廊，而非逐障碍物椭球）

---

## 8. 未提交的相关改动 (Uncommitted, for context)
- scene 3：`dync_scene3.py`, `dync_obs.launch`
- 关 SAM：`umap/launch/simulation.launch`
- IoU 关联：`umap/config/simulation.yaml`
- bridge 调参：`mpc_nav/launch/tmpc_doa.launch`
