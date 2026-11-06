#!/usr/bin/env python3
"""
calc_prediction_error_fair.py — GT-triggered prediction RMSE with fair coverage.

Evaluates prediction error at every GT sample time using one-to-one positional
matching at time t. Coverage is reported per horizon as:

  matched GT pairs at horizon h / valid GT pairs at horizon h

This removes the selection bias where sparse tracker output only gets evaluated
on easy cases.

Usage:
  python3 calc_prediction_error_fair.py --gt gt.csv --tracker tracker.csv [--bag raw.bag]
"""

import argparse
import csv
import math
import numpy as np
from collections import defaultdict

from tracking_eval_utils import build_fov_filter, match_detections

HORIZONS = [0.1, 0.2, 0.3, 0.5, 1.0]


def load_by_id(path):
    by_id = defaultdict(list)
    with open(path) as f:
        for row in csv.DictReader(f):
            by_id[int(row["id"])].append({
                "t": float(row["timestamp"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "vx": float(row.get("vx", 0)),
                "vy": float(row.get("vy", 0)),
                "radius": float(row.get("radius", 0.3)),
            })
    for k in by_id:
        by_id[k].sort(key=lambda r: r["t"])
    return by_id


def interp(points, t):
    if not points or t < points[0]["t"] or t > points[-1]["t"]:
        return None
    for i in range(len(points) - 1):
        t0, t1 = points[i]["t"], points[i + 1]["t"]
        if t0 <= t <= t1:
            if t1 == t0:
                return points[i]
            a = (t - t0) / (t1 - t0)
            return {
                "x": points[i]["x"] + a * (points[i + 1]["x"] - points[i]["x"]),
                "y": points[i]["y"] + a * (points[i + 1]["y"] - points[i]["y"]),
                "vx": points[i].get("vx", 0),
                "vy": points[i].get("vy", 0),
                "radius": points[i].get("radius", 0.3),
            }
    return None


def tracker_observations_at(trk_by_id, t, max_age, fov_filter=None):
    """Sample one tracker state per tracker ID near time t."""
    obs = []
    for trk_id, pts in trk_by_id.items():
        best = None
        best_dt = max_age
        for p in pts:
            dt = abs(p["t"] - t)
            if dt < best_dt:
                best = p
                best_dt = dt
        if best is None:
            continue
        if fov_filter is not None and not fov_filter(t, best["x"], best["y"]):
            continue
        obs.append({
            "id": trk_id,
            "x": best["x"],
            "y": best["y"],
            "vx": best.get("vx", 0),
            "vy": best.get("vy", 0),
            "radius": best.get("radius", 0.3),
            "timestamp": best["t"],
        })
    return obs


def main():
    p = argparse.ArgumentParser(description="GT-triggered prediction RMSE")
    p.add_argument("--gt", required=True)
    p.add_argument("--tracker", required=True)
    p.add_argument("--bag", default=None, help="Rosbag for FOV filtering")
    p.add_argument("--output", default=None)
    p.add_argument("--horizons", nargs="+", type=float, default=HORIZONS)
    p.add_argument("--sample-rate", type=float, default=10.0,
                   help="GT sampling rate in Hz (default 10)")
    p.add_argument("--match-dist", type=float, default=1.0,
                   help="Max GT-tracker match distance (default 1.0m)")
    p.add_argument("--max-age", type=float, default=0.15,
                   help="Max time diff for GT-tracker match (default 0.15s)")
    args = p.parse_args()

    gt_by_id = load_by_id(args.gt)
    trk_by_id = load_by_id(args.tracker)
    fov_filter = build_fov_filter(args.bag) if args.bag else None

    # Collect all GT times and find global range
    all_gt_times = set()
    for gt_id, pts in gt_by_id.items():
        for pt in pts:
            all_gt_times.add(round(pt["t"], 2))
    if not all_gt_times:
        print("No GT data found.")
        return

    t_min = min(all_gt_times)
    t_max = max(all_gt_times)
    dt_sample = 1.0 / args.sample_rate

    errors = {h: [] for h in args.horizons}
    matched_pairs = {h: 0 for h in args.horizons}
    valid_gt_pairs = {h: 0 for h in args.horizons}
    total_samples = 0

    # Sample at regular intervals
    t = t_min
    while t <= t_max:
        t = round(t, 2)
        gt_at_t = []
        for gt_id, pts in gt_by_id.items():
            g = interp(pts, t)
            if g:
                if fov_filter is not None and not fov_filter(t, g["x"], g["y"]):
                    continue
                g["id"] = gt_id
                gt_at_t.append(g)

        if not gt_at_t:
            t += dt_sample
            continue

        total_samples += 1
        tracker_at_t = tracker_observations_at(trk_by_id, t, args.max_age, fov_filter=fov_filter)
        matches, _, _ = match_detections(gt_at_t, tracker_at_t, args.match_dist)
        matched_by_gt = {int(g["id"]): trk for g, trk, _ in matches}

        for g in gt_at_t:
            gt_id = int(g["id"])
            trk = matched_by_gt.get(gt_id)
            for h in args.horizons:
                t_pred = t + h
                gt_at_h = interp(gt_by_id[gt_id], t_pred)
                if gt_at_h is None:
                    continue
                valid_gt_pairs[h] += 1
                if trk is None:
                    continue
                pred_x = trk["x"] + trk.get("vx", 0) * h
                pred_y = trk["y"] + trk.get("vy", 0) * h
                raw_err = math.hypot(pred_x - gt_at_h["x"], pred_y - gt_at_h["y"])
                r = gt_at_h.get("radius", 0.3)
                err = max(0.0, raw_err - r)
                errors[h].append(err)
                matched_pairs[h] += 1

        t += dt_sample

    print(f"{'Horizon(s)':>10} {'RMSE(m)':>10} {'Mean(m)':>10} {'N':>8} {'GT pairs':>10} {'Coverage':>10}")
    print(f"{'GT samples':>10} {'':>10} {'':>10} {total_samples:>8}")
    print()

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
        coverage = matched_pairs[h] / valid_gt_pairs[h] if valid_gt_pairs[h] > 0 else 0.0
        print(f"{h:>10.1f} {rmse:>10.4f} {mean:>10.4f} {n:>8} {valid_gt_pairs[h]:>10} {coverage:>10.4f}")
        results.append({
            "horizon": h,
            "rmse": rmse,
            "mean": mean,
            "n": n,
            "valid_gt_pairs": valid_gt_pairs[h],
            "matched_pairs": matched_pairs[h],
            "coverage": round(coverage, 4),
        })

    if args.output:
        with open(args.output, "w", newline="") as f:
            w = csv.DictWriter(
                f,
                fieldnames=[
                    "horizon", "rmse", "mean", "n",
                    "valid_gt_pairs", "matched_pairs",
                    "total_gt_samples", "coverage",
                ],
            )
            w.writeheader()
            for r in results:
                r["total_gt_samples"] = total_samples
                w.writerow(r)
        print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
