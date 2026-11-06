# Realdata

## pure_transverse_1

| Baseline                | epos (m) (w/o pred) | RMSE   | epos (m) (with pred 0.5s) | RMSE   | time_coverage (%) | evel (m/s) | RMSE   |
|-------------------------|---------------------|--------|---------------------------|--------|-------------------|------------|--------|
| DODT(CMU)               | 0.2274              | 0.2533 | 0.2606                    | 0.2933 | 78.4%             | 0.4815     | 0.6504 |
| FAPP(HKU)               | 0.1237              | 0.1366 | 0.1676                    | 0.2067 | 77.0%             | 1.0415     | 1.2567 |
| Ours(w/o FastSAM)       | 0.0561              | 0.0651 | 0.0833                    | 0.1024 | 76.7%             | 0.5499     | 0.6336 |
| Ours(w/o CLIP, IOU only)| 0.0526              | 0.0648 | 0.0828                    | 0.1017 | 79.8%             | 0.6102     | 0.6921 |
| Ours                    | **0.0522** 🏆       | **0.0650** 🏆 | **0.0790** 🏆              | **0.0990** 🏆 | **80.8%** 🏆         | 0.5541     | **0.6337** 🏆 |

# occlusion

| Baseline                | epos (m) | RMSE   | MOTA (%) |
|-------------------------|----------|--------|----------|
| DODT(CMU)               | 0.3684   | 0.3999 | 91.04%   |
| FAPP(HKU)               | N/A (too many mismatches) | N/A    | 87.31%   |
| Ours(w/o FastSAM)       | 0.2483   | 0.2661 | 88.81%   |
| Ours(w/o CLIP)          | 0.2562   | 0.2731 | 89.55%   |
| Ours                    | **0.2296** 🏆 | **0.2461** 🏆 | **94.03%** 🏆 |

## Multiple DO (3 men run in loop multi_2) w & wo occlusion

| Baseline                | epos (m) | RMSE   | MOTA (%) |
|-------------------------|----------|--------|----------|
| DODT(CMU)               | 0.2227   | 0.2590 | 72.83%   |
| FAPP(HKU)               | **0.2066** 🏆 | 0.2149 | 41.46%   |
| Ours(w/o FastSAM)       | 0.2262   | 0.3545 | **79.20%** 🏆 |
| Ours(w/o CLIP)          | 0.2183   | 0.2719 | 73.02%   |
| Ours                    | 0.2023   | **0.2138** 🏆 | 78.08%   |



# Experiment Report — Crossing Scene

## Tracking Metrics

| Method | MOTA | IDSW | FP | FN | epos (m) |
|--------|-------:|-----:|----:|----:|---------:|
| Ours   | 0.6503268 |    7 |  33 |  67 |   0.0763 |
| DODT   | 0.0000000 |    1 |  84 |  45 |   0.1236 |

MOTA is computed as $1 - (FP + FN + IDSW) / total\_gt$ using each method's latest `metrics.csv`.

## Prediction RMSE (m)

| Method | 0.1 s | 0.2 s | 0.3 s | 0.5 s | 1.0 s |
|--------|------:|------:|------:|------:|------:|
| Ours   | 0.1130 | 0.1275 | 0.1449 | 0.1870 | 0.3352 |
| DODT   | 0.1852 | 0.1978 | 0.2120 | 0.2442 | 0.3439 |


## Figures

![MOTA & Position Error](fig_mota_summary.png)

![MOTA Decomposition](fig_mota_comparison.png)

![Prediction RMSE](fig_prediction_comparison.png)

![BEV Trajectory](fig_bev_comparison.png)

---

## Crossing Scene Visuals

### crossing1
![crossing1](crossing1.png)

### crossing2
![crossing2](crossing2.png)

### crossing3
![crossing3](crossing3.png)

---

## Reproduce

```bash
# ── 1. Record bags ──────────────────────────────────────────
# Ours (DOA + MPC)
./exp/scripts/run_exp.sh crossing 2 60 1

# DODT baseline (onboard_detector)
./exp/scripts/run_onboard_exp.sh crossing 2 60 1

# ── 2. Offline processing ──────────────────────────────────
# Ours: extract GT + tracker, compute metrics & prediction error, plot
./exp/scripts/process_bag.sh exp/data/crossing/ours/run_1

# DODT: use tracked_bboxes topic (all KF-tracked objects), skip clustering
./exp/scripts/process_bag.sh exp/data/crossing/dodt/run_1 \
    --topic /onboard_detector/tracked_bboxes --no-cluster

# ── 3. Generate comparison report + IEEE figures ───────────
python3 exp/scripts/gen_compare.py --scene crossing \
    --methods "Ours:exp/data/crossing/ours/run_1" \
              "DODT:exp/data/crossing/dodt/run_1" \
    --output exp/data/crossing/compare
```

### Notes

- DODT `/onboard_detector/dynamic_bboxes` may be empty if the detector classifies all obstacles as static. Use `/onboard_detector/tracked_bboxes` instead (contains all Kalman-filtered tracked objects regardless of dynamic classification).
- `--no-cluster` is required for DODT because the detector already maintains stable IDs via its internal Kalman filter; the spatial clustering step in `extract_tracker.py` is designed for DOA's fragmented coarse-match output and would incorrectly merge DODT's distinct obstacle tracks.
- FOV filtering (`--bag` in `calc_metrics.py`) removes GT detections outside the D435 camera FOV (87° horizontal, 0.3–6 m depth) to avoid penalizing methods for not detecting invisible obstacles.
