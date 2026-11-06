#!/usr/bin/env python3
"""
calc_prediction_error.py — Prediction RMSE at multiple lookahead horizons.

For each tracker observation at time t with (pos, vel), predict:
  pred(t+dt) = pos + vel * dt
Compare with GT at t+dt.

Usage: python3 calc_prediction_error.py --gt gt.csv --tracker tracker.csv
"""

import argparse
import csv
import math
import numpy as np
from collections import defaultdict

HORIZONS = [0.5, 1.0, 2.0, 3.0]


def load_by_id(path):
    """Load CSV grouped by ID, sorted by time."""
    by_id = defaultdict(list)
    with open(path) as f:
        for row in csv.DictReader(f):
            by_id[int(row["id"])].append({
                "t": float(row["timestamp"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "vx": float(row.get("vx", 0)),
                "vy": float(row.get("vy", 0)),
                "radius": float(row.get("radius", 0)),
            })
    for k in by_id:
        by_id[k].sort(key=lambda r: r["t"])
    return by_id


def interp_gt(gt_points, t):
    """Linear interpolation of GT at time t."""
    if not gt_points or t < gt_points[0]["t"] or t > gt_points[-1]["t"]:
        return None
    for i in range(len(gt_points) - 1):
        t0, t1 = gt_points[i]["t"], gt_points[i + 1]["t"]
        if t0 <= t <= t1:
            if t1 == t0:
                return gt_points[i]
            a = (t - t0) / (t1 - t0)
            return {
                "x": gt_points[i]["x"] + a * (gt_points[i + 1]["x"] - gt_points[i]["x"]),
                "y": gt_points[i]["y"] + a * (gt_points[i + 1]["y"] - gt_points[i]["y"]),
                "radius": gt_points[i].get("radius", 0),
            }
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--gt", required=True)
    p.add_argument("--tracker", required=True)
    p.add_argument("--output", default=None)
    p.add_argument("--horizons", nargs="+", type=float, default=HORIZONS)
    args = p.parse_args()

    gt_by_id = load_by_id(args.gt)
    trk_by_id = load_by_id(args.tracker)

    # Build per-frame tracker observations
    from collections import OrderedDict
    trk_frames = defaultdict(list)
    for trk_id, trk_pts in trk_by_id.items():
        for tp in trk_pts:
            trk_frames[round(tp["t"], 1)].append((trk_id, tp))

    errors = {h: [] for h in args.horizons}

    for t_bin, observations in trk_frames.items():
        for trk_id, tp in observations:
            # Per-frame nearest GT match by current position
            best_gt_id, best_dist = None, float("inf")
            for gt_id, gt_pts in gt_by_id.items():
                g = interp_gt(gt_pts, tp["t"])
                if g:
                    d = math.hypot(tp["x"] - g["x"], tp["y"] - g["y"])
                    if d < best_dist:
                        best_dist = d
                        best_gt_id = gt_id
            if best_gt_id is None or best_dist > 1.0:
                continue

            gt_pts = gt_by_id[best_gt_id]
            for h in args.horizons:
                t_pred = tp["t"] + h
                pred_x = tp["x"] + tp["vx"] * h
                pred_y = tp["y"] + tp["vy"] * h
                gt_at_h = interp_gt(gt_pts, t_pred)
                if gt_at_h:
                    raw_err = math.hypot(pred_x - gt_at_h["x"], pred_y - gt_at_h["y"])
                    r = gt_at_h.get("radius", 0)
                    err = max(0.0, raw_err - r)
                    errors[h].append(err)

    print(f"{'Horizon(s)':>10} {'RMSE(m)':>10} {'Mean(m)':>10} {'N':>8}")
    results = []
    for h in args.horizons:
        if errors[h]:
            arr = np.array(errors[h])
            rmse = math.sqrt(np.mean(arr ** 2))
            mean = np.mean(arr)
            n = len(arr)
        else:
            rmse = mean = float("nan")
            n = 0
        print(f"{h:>10.1f} {rmse:>10.4f} {mean:>10.4f} {n:>8}")
        results.append({"horizon": h, "rmse": rmse, "mean": mean, "n": n})

    if args.output:
        with open(args.output, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["horizon", "rmse", "mean", "n"])
            w.writeheader()
            w.writerows(results)
        print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
