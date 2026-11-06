# Simulation Experiment Plan — DOA Paper (RAL Submission)

> 目标：用最少的仿真实验补齐论文证据链，回应 TODO.md 中的关键审稿风险。
> 核心证据链：Two-stage tracking → 降低 IDSW → 提升长时域 prediction → 改善 planning safety

---

## 一、仿真场景定义

共 3 个场景，覆盖基础性能、核心失败模式、复杂交互：

| 场景 ID | 名称 | 描述 | 对应 launch | 回应的审稿问题 |
|---------|------|------|------------|---------------|
| S1 | Single Crossing | 1 个动态障碍物横穿机器人前方路径 | `dync_obs.launch` (单障碍物配置) | 基础 tracking 精度与稳定性 |
| S2 | Occlusion | 1 个动态障碍物被静态物体短时遮挡后重新出现 | `dync_obs.launch` (含静态遮挡物) | ID continuity / re-association 能力 |
| S3 | Multi-Object Interaction | 2-3 个动态障碍物近距离交汇/交叉行走 | `dync_obs.launch` (多障碍物配置) | IDSW / 误关联 / 多目标鲁棒性 |

### 场景设计要点

- S1 是 sanity check，所有方法都应该能跑，用于建立 baseline 参照
- S2 是论文核心卖点场景：遮挡后 ID 是否保持一致
- S3 是最难场景：多目标近距离时几何匹配容易混淆，语义关联的价值在此体现
- 每个场景跑 3-5 次取均值，报告 mean ± std
- 设计场景的时候，对于S2 S3实验场景设计需要考虑到不同障碍物的纹理、几何区别，使用gazebo中的object来设置场景，尽量拟真，多样化

---

## 二、方法矩阵

| 方法 ID | 名称 | 描述 | 实现方式 | 优先级 |
|---------|------|------|---------|--------|
| M1 | **Ours** | 完整 two-stage: coarse match + CLIP/FastSAM fine association | 现有系统，`simulation.launch` | 必做 |
| M2 | **DODT** | 同域 baseline，几何检测+特征关联+Kalman | 需复现核心逻辑或用其开源代码 | 必做 |
| M3 | **FAPP** | 感知规划耦合 baseline | 需复现核心 tracking 部分 | 必做 |
| M4 | **Stage I Only** | 只用 coarse match，不启动 rematch 节点 | 关闭 `clean_detailed_match` 节点 | 必做 |
| M5 | **DBSCAN + Kalman** | 纯几何聚类 + Kalman 跟踪，无语义 | 新写，约 200 行 Python | 推荐 |
| M6 | **w/o CLIP** | Ours 去掉 CLIP 特征，只用 IoU + distance + FastSAM | 修改 rematch 配置，`SIMILARITY_WEIGHT=0` | 推荐 |
| M7 | **w/o FastSAM** | Ours 去掉 FastSAM，只用 CLIP + IoU + distance | 关闭 FastSAM 节点 | 推荐 |

### 方法说明

- M4 是零额外工作量的 ablation，直接证明 two-stage 的价值
- M5 代表"最简几何方法"，拉开对比梯度
- M6/M7 是消融实验，证明 CLIP 和 FastSAM 解决不同的 failure mode

---

## 三、实验矩阵（场景 × 方法 × 指标）

### 3.1 主实验：Tracking 定量评估

| | S1 Single Crossing | S2 Occlusion | S3 Multi-Object |
|---|---|---|---|
| **M1 Ours** | MOTA, IDSW, FP, FN, epos, runtime | MOTA, IDSW, FP, FN, epos, runtime | MOTA, IDSW, FP, FN, epos, runtime |
| **M2 DODT** | MOTA, IDSW, FP, FN, epos, runtime | MOTA, IDSW, FP, FN, epos, runtime | MOTA, IDSW, FP, FN, epos, runtime |
| **M3 FAPP** | MOTA, IDSW, FP, FN, epos, runtime | MOTA, IDSW, FP, FN, epos, runtime | MOTA, IDSW, FP, FN, epos, runtime |
| **M4 Stage I Only** | MOTA, IDSW, FP, FN, epos, runtime | MOTA, IDSW, FP, FN, epos, runtime | MOTA, IDSW, FP, FN, epos, runtime |

> 每个 cell 报告 mean ± std (3-5 runs)

### 3.2 消融实验

| | S2 Occlusion | S3 Multi-Object |
|---|---|---|
| **M1 Ours** | MOTA, IDSW, epos | MOTA, IDSW, epos |
| **M6 w/o CLIP** | MOTA, IDSW, epos | MOTA, IDSW, epos |
| **M7 w/o FastSAM** | MOTA, IDSW, epos | MOTA, IDSW, epos |
| **M4 Stage I Only** | MOTA, IDSW, epos | MOTA, IDSW, epos |

> 消融只需在 S2/S3 上做，S1 太简单区分度不够

### 3.3 Prediction Error vs. Lookahead Time

| 方法 | 0.1s | 0.2s | 0.3s | 0.5s | 1.0s |
|------|------|------|------|------|------|
| M1 Ours | RMSE | RMSE | RMSE | RMSE | RMSE |
| M2 DODT | RMSE | RMSE | RMSE | RMSE | RMSE |
| M3 FAPP | RMSE | RMSE | RMSE | RMSE | RMSE |
| M4 Stage I Only | RMSE | RMSE | RMSE | RMSE | RMSE |

> 在 S2 (Occlusion) 和 S3 (Multi-Object) 上各做一组
> 预期结论：baseline 在长 horizon 误差增长更快，因为 IDSW 导致 Kalman velocity 重置

### 3.4 Runtime Breakdown

| 模块 | 耗时 (ms) |
|------|----------|
| Depth preprocessing (UMAP) | — |
| Coarse match (Stage I) | — |
| Feature extraction (CLIP + FastSAM) | — |
| Fine association (Stage II / Rematch) | — |
| Kalman update + publish | — |
| **Total** | — |

> 数据来源：`/rematch_statistics` topic + `rematch_profiler.py`
> 在 S3 (最多目标) 上测量，报告 mean ± std

### 3.5 Planning 层影响（如做 NB-1 confidence-aware）

| | Constant Safety Margin | Confidence-Adaptive Margin |
|---|---|---|
| Collision count | — | — |
| Min clearance distance (m) | — | — |
| Path length (m) | — | — |
| Arrival time (s) | — | — |
| Replanning count | — | — |
| Cmd_vel jitter (rad/s²) | — | — |

> 在 S2 和 S3 上各跑 5 次
> 这组实验是 NB-1 的验证，优先级低于 3.1-3.4

---

## 四、指标定义

| 指标 | 定义 | 用途 |
|------|------|------|
| MOTA | 1 - (FP + FN + IDSW) / GT_total | 综合 tracking 质量 |
| IDSW | ID switch 次数 | 身份一致性（论文核心） |
| FP | False positive 次数 | 虚警率 |
| FN | False negative 次数 | 漏检率 |
| epos | Position RMSE (m) | 位置估计精度 |
| Prediction RMSE | 预测位置与 GT 在 t+Δt 的欧氏距离 | 预测质量随时域的退化 |
| Runtime | 每帧总处理时间 (ms) | 实时性验证 |

### IDSW 统计方法

```
对每个 GT 目标 g：
  遍历时间序列，记录每帧匹配到的 tracker ID
  当 tracker ID 发生变化时，计 1 次 IDSW
IDSW_total = Σ_g IDSW(g)
```

### Prediction RMSE 计算方法

```
对每个时刻 t：
  用当前 Kalman state (pos, vel) 做恒速预测：pred(t+Δt) = pos(t) + vel(t) * Δt
  与 GT(t+Δt) 比较：error = ||pred(t+Δt) - GT(t+Δt)||
RMSE(Δt) = sqrt(mean(error²))
```

---

## 五、论文图表清单

### 表格

| 表格 ID | 内容 | 回应的审稿问题 | 数据来源 | TODO 对应项 |
|---------|------|---------------|---------|------------|
| Table I | 三场景 tracking 主结果 (MOTA, IDSW, epos, runtime) | "只有 2 个 baseline" / "实验不够" | 实验 3.1 | #5, #12 |
| Table II | 消融实验 (Ours vs w/o CLIP vs w/o FastSAM vs Stage I Only) | "CLIP/FastSAM 各自贡献是什么" | 实验 3.2 | 已有但需强化 |
| Table III | Runtime breakdown (各模块延迟) | "号称 real-time 但没数据" | 实验 3.4 | TODO 新增项 |

### 图

| 图 ID | 内容 | 类型 | 回应的审稿问题 | TODO 对应项 |
|-------|------|------|---------------|------------|
| Fig. 1 | 系统架构图 | 示意图 | — | 已有 |
| Fig. 2 | **Prediction RMSE vs. Lookahead Time** | 折线图 | "tracking 改善对 navigation 有什么意义？" | #6 |
| | x 轴: lookahead time (0.1-1.0s) | | | |
| | y 轴: prediction RMSE (m) | | | |
| | 4 条曲线: Ours / DODT / FAPP / Stage I Only | | | |
| | 在 S2 和 S3 上各一张子图 | | | |
| Fig. 3 | **IDSW 分解柱状图** | 柱状图 | "MOTA 提升到底来自哪里？" | #5 |
| | 每个方法一组柱: FP / FN / IDSW | | | |
| | 三个场景并排或分子图 | | | |
| Fig. 4 | **Occlusion 定性对比** | 时序截图 | "有没有直观证据？" | #7 |
| | 4-6 帧时序: 遮挡前 → 遮挡中 → 遮挡后 | | | |
| | 上排: Ours (ID 保持) | | | |
| | 下排: DODT 或 Stage I Only (ID 切换) | | | |
| | 用颜色编码 tracker ID | | | |
| Fig. 5 | **轨迹鸟瞰图** | 2D 轨迹图 | "tracking 连续性的直观证据" | #7 |
| | Bird's-eye-view: GT 轨迹 + 各方法跟踪轨迹 | | | |
| | Ours 轨迹连续，baseline 轨迹断裂/跳变 | | | |
| | 在 S3 (Multi-Object) 上最有说服力 | | | |

### 图表与审稿问题的映射

```
审稿问题                              → 用什么回应
─────────────────────────────────────────────────────────
"只有 2 个 baseline"                  → Table I (5 个方法)
"MOTA 提升来自哪里"                   → Fig. 3 (IDSW 分解)
"tracking 改善对 navigation 有什么用"  → Fig. 2 (prediction curve)
"有没有直观证据"                      → Fig. 4 + Fig. 5
"real-time 有没有数据支撑"            → Table III (runtime)
"CLIP/FastSAM 各自贡献"              → Table II (ablation)
"是不是调参调出来的"                  → Table II + prediction curve 的一致性
```

---

## 六、数据采集与处理流程

### 6.1 仿真数据采集

```bash
# Step 1: 启动仿真环境 + MPC + 场景
roslaunch mpc_nav mpc.launch scenario:=<S1|S2|S3>

# Step 2: 启动 DOA 感知系统
roslaunch umap simulation.launch

# Step 3: 录制 rosbag
rosbag record -O exp_<method>_<scenario>_<run>.bag \
  /rematch/tracker_details \
  /rematch_statistics \
  /odom \
  /cmd_vel \
  /mpc/pred_path \
  /d435/depth/image_raw \
  /d435/image_raw \
  /gazebo/model_states
```

### 6.2 Ground Truth 提取

```bash
# 从 Gazebo model_states 提取动态障碍物 GT 轨迹
# /gazebo/model_states 包含所有模型的 pose 和 twist
python3 exp/scripts/extract_gt.py --bag <bag_file> --output exp/data/<scenario>/gt.csv
```

### 6.3 离线指标计算

```bash
# Tracking 指标 (MOTA, IDSW, FP, FN, epos)
python3 exp/scripts/calc_tracking_metrics.py \
  --gt exp/data/<scenario>/gt.csv \
  --tracker exp/data/<scenario>/<method>/tracker.csv

# Prediction error vs. lookahead
python3 exp/scripts/calc_prediction_error.py \
  --gt exp/data/<scenario>/gt.csv \
  --tracker exp/data/<scenario>/<method>/tracker.csv \
  --horizons 0.1 0.2 0.3 0.5 1.0

# Runtime (直接从 rosbag 的 /rematch_statistics 提取)
python3 exp/scripts/calc_runtime.py --bag <bag_file>
```

---

## 七、执行优先级

| 阶段 | 内容 | 产出 | 预计工作量 |
|------|------|------|----------|
| **Phase 1** | 搭建 3 个仿真场景 + GT 提取脚本 | 可复现的实验环境 | 2-3 天 |
| **Phase 2** | 跑 M1/M2/M3/M4 × S1/S2/S3 | Table I 数据 + Fig. 2/3 数据 | 2-3 天 |
| **Phase 3** | 消融 M6/M7 × S2/S3 | Table II 数据 | 1 天 |
| **Phase 4** | 定性图 + runtime 表 | Fig. 4/5 + Table III | 1 天 |
| **Phase 5** | (可选) M5 DBSCAN+Kalman | 补强 Table I | 1-2 天 |
| **Phase 6** | (可选) NB-1 confidence-aware planning | Table IV (planning metrics) | 2-3 天 |

> Phase 1-4 是必做项，约 1 周完成
> Phase 5-6 是加分项，视时间决定
