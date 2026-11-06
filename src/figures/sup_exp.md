# 补充实验结果

## 实验 1：Tracking-to-Planning 端到端动态避障仿真

### 结果表格

| Method | Collision / Near-collision ↓ | Min Clearance (m) ↑ | Prediction RMSE @1.0s (m) ↓ |
|---|---:|---:|---:|
| DODT | nan% / nan% | N/A | N/A |
| FAPP | nan% / nan% | N/A | N/A |
| Ours w/o CLIP | nan% / nan% | N/A | N/A |
| Ours full | nan% / nan% | N/A | N/A |
| Oracle GT | nan% / nan% | N/A | N/A |

### 图表

![Collision Rate & Clearance](exp1_collision_clearance.png)

![Prediction RMSE vs Horizon](exp1_prediction_rmse.png)

### 各场景详细结果

---

## 实验 2：Controlled Occlusion + Re-ID 深度实验

### 结果表格 (Occlusion = 2.0 s)

| Method | IDSW ↓ | Re-ID Success ↑ | Prediction RMSE after reappearance (m) ↓ |
|---|---:|---:|---:|

### 图表

![ID Switch vs Occlusion Duration](exp2_idsw_vs_duration.png)

![Re-ID Success Rate](exp2_reid_success.png)

![Prediction RMSE after Reappearance](exp2_prediction_after_reappearance.png)

### Ours Full 在不同遮挡时长下的表现

| Occlusion Duration | IDSW ↓ | Re-ID Success ↑ | Prediction RMSE ↓ |
|---:|---:|---:|---:|

### 各遮挡时长完整结果

#### Occlusion = 0.5 s

| Method | IDSW | Re-ID Success | Pred RMSE |
|---|---:|---:|---:|

#### Occlusion = 1.0 s

| Method | IDSW | Re-ID Success | Pred RMSE |
|---|---:|---:|---:|

#### Occlusion = 2.0 s

| Method | IDSW | Re-ID Success | Pred RMSE |
|---|---:|---:|---:|

#### Occlusion = 3.0 s

| Method | IDSW | Re-ID Success | Pred RMSE |
|---|---:|---:|---:|

#### Occlusion = 4.0 s

| Method | IDSW | Re-ID Success | Pred RMSE |
|---|---:|---:|---:|
