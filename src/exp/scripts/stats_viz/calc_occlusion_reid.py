#!/usr/bin/env python3
"""
Compute controlled-occlusion Re-ID metrics from GT/tracker CSV files.

The occlusion interval is expressed relative to the first GT timestamp by
default. For example, `--occlusion-start 8.0 --occlusion-duration 2.0`
evaluates the before window just before t0+8s and the after window starting at
t0+10s.
"""

import argparse
import csv
import json
import math
import os
from collections import Counter, defaultdict


def load_by_id(path):
    by_id = defaultdict(list)
    with open(path) as f:
        for row in csv.DictReader(f):
            by_id[int(row["id"])].append({
                "t": float(row["timestamp"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "vx": float(row.get("vx", 0.0) or 0.0),
                "vy": float(row.get("vy", 0.0) or 0.0),
            })
    for rows in by_id.values():
        rows.sort(key=lambda r: r["t"])
    return by_id


def interp(points, ts):
    if not points or ts < points[0]["t"] or ts > points[-1]["t"]:
        return None
    lo = 0
    hi = len(points) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if points[mid]["t"] < ts:
            lo = mid + 1
        else:
            hi = mid - 1
    i1 = min(lo, len(points) - 1)
    i0 = max(0, i1 - 1)
    p0 = points[i0]
    p1 = points[i1]
    if p1["t"] == p0["t"]:
        return p0
    a = (ts - p0["t"]) / (p1["t"] - p0["t"])
    return {
        "t": ts,
        "x": p0["x"] + a * (p1["x"] - p0["x"]),
        "y": p0["y"] + a * (p1["y"] - p0["y"]),
        "vx": p0["vx"] + a * (p1["vx"] - p0["vx"]),
        "vy": p0["vy"] + a * (p1["vy"] - p0["vy"]),
    }


def samples_in_window(points, start, end):
    return [p for p in points if start <= p["t"] <= end]


def nearest_tracker_at(tracker_by_id, ts, gx, gy, match_distance):
    best = None
    for tid, points in tracker_by_id.items():
        tr = interp(points, ts)
        if tr is None:
            continue
        dist = math.hypot(tr["x"] - gx, tr["y"] - gy)
        if dist <= match_distance and (best is None or dist < best["dist"]):
            best = {
                "tracker_id": tid,
                "tracker": tr,
                "dist": dist,
            }
    return best


def dominant_match(gt_points, tracker_by_id, start, end, match_distance):
    counts = Counter()
    samples = 0
    hits = 0
    for gt in samples_in_window(gt_points, start, end):
        samples += 1
        match = nearest_tracker_at(
            tracker_by_id, gt["t"], gt["x"], gt["y"], match_distance,
        )
        if match is None:
            continue
        hits += 1
        counts[match["tracker_id"]] += 1
    if not counts:
        return None, hits, samples
    tracker_id, _ = counts.most_common(1)[0]
    return tracker_id, hits, samples


def prediction_errors_after(gt_by_id, tracker_by_id, start, end, horizons, match_distance):
    errors = {h: [] for h in horizons}
    for tid, tr_points in tracker_by_id.items():
        for tr in samples_in_window(tr_points, start, end):
            best_gt_id = None
            best_dist = float("inf")
            for gid, gt_points in gt_by_id.items():
                gt = interp(gt_points, tr["t"])
                if gt is None:
                    continue
                dist = math.hypot(tr["x"] - gt["x"], tr["y"] - gt["y"])
                if dist < best_dist:
                    best_dist = dist
                    best_gt_id = gid
            if best_gt_id is None or best_dist > match_distance:
                continue
            gt_points = gt_by_id[best_gt_id]
            for h in horizons:
                gt_future = interp(gt_points, tr["t"] + h)
                if gt_future is None:
                    continue
                pred_x = tr["x"] + tr["vx"] * h
                pred_y = tr["y"] + tr["vy"] * h
                errors[h].append(math.hypot(pred_x - gt_future["x"],
                                            pred_y - gt_future["y"]))
    return errors


def rmse(values):
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def write_rows(path, rows, fields):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_single_row(path, row):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        writer.writeheader()
        writer.writerow(row)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gt", required=True)
    parser.add_argument("--tracker", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--occlusion-start", type=float, required=True)
    parser.add_argument("--occlusion-duration", type=float, required=True)
    parser.add_argument("--before-window", type=float, default=3.0)
    parser.add_argument("--after-window", type=float, default=2.0)
    parser.add_argument("--match-distance", type=float, default=1.0)
    parser.add_argument("--horizons", nargs="+", type=float,
                        default=[1.0, 2.0, 3.0])
    parser.add_argument("--absolute-time", action="store_true",
                        help="Treat occlusion-start as an absolute timestamp")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    gt_by_id = load_by_id(args.gt)
    tracker_by_id = load_by_id(args.tracker)

    all_gt_times = [p["t"] for points in gt_by_id.values() for p in points]
    if not all_gt_times:
        raise RuntimeError(f"No GT samples in {args.gt}")
    t0 = 0.0 if args.absolute_time else min(all_gt_times)

    occ_start = t0 + args.occlusion_start
    reappear = occ_start + args.occlusion_duration
    before_start = occ_start - args.before_window
    before_end = occ_start
    after_start = reappear
    after_end = reappear + args.after_window

    identity_rows = []
    considered = 0
    reid_success = 0
    idsw = 0
    missing_before = 0
    missing_after = 0

    for gid, gt_points in sorted(gt_by_id.items()):
        if not samples_in_window(gt_points, before_start, before_end):
            continue
        if not samples_in_window(gt_points, after_start, after_end):
            continue
        considered += 1
        before_id, before_hits, before_samples = dominant_match(
            gt_points, tracker_by_id, before_start, before_end,
            args.match_distance,
        )
        after_id, after_hits, after_samples = dominant_match(
            gt_points, tracker_by_id, after_start, after_end,
            args.match_distance,
        )

        status = "success"
        if before_id is None and after_id is None:
            missing_before += 1
            missing_after += 1
            status = "missing_both"
        elif before_id is None:
            missing_before += 1
            status = "missing_before"
        elif after_id is None:
            missing_after += 1
            status = "missing_after"
        elif before_id != after_id:
            idsw += 1
            status = "idsw"
        else:
            reid_success += 1

        identity_rows.append({
            "gt_id": gid,
            "before_tracker_id": "" if before_id is None else before_id,
            "after_tracker_id": "" if after_id is None else after_id,
            "before_hits": before_hits,
            "before_samples": before_samples,
            "after_hits": after_hits,
            "after_samples": after_samples,
            "status": status,
        })

    pred_errors = prediction_errors_after(
        gt_by_id, tracker_by_id, after_start, after_end,
        args.horizons, args.match_distance,
    )
    pred_rows = []
    for h in args.horizons:
        vals = pred_errors[h]
        pred_rows.append({
            "horizon": h,
            "rmse": rmse(vals),
            "mean": sum(vals) / len(vals) if vals else float("nan"),
            "n": len(vals),
        })

    pred_rmse_values = [row["rmse"] for row in pred_rows if math.isfinite(row["rmse"])]
    metrics = {
        "gt": os.path.abspath(args.gt),
        "tracker": os.path.abspath(args.tracker),
        "occlusion_start_s": args.occlusion_start,
        "occlusion_duration_s": args.occlusion_duration,
        "before_window_s": args.before_window,
        "after_window_s": args.after_window,
        "match_distance_m": args.match_distance,
        "gt_targets_considered": considered,
        "idsw_count": idsw,
        "missing_before": missing_before,
        "missing_after": missing_after,
        "reid_success_count": reid_success,
        "reid_success_rate": reid_success / considered if considered else float("nan"),
        "prediction_rmse_after_reappearance_m": (
            sum(pred_rmse_values) / len(pred_rmse_values)
            if pred_rmse_values else float("nan")
        ),
    }

    for row in pred_rows:
        key = f"prediction_rmse_h{row['horizon']:.1f}s_m"
        metrics[key] = row["rmse"]

    write_rows(
        os.path.join(args.output_dir, "occlusion_identity_events.csv"),
        identity_rows,
        ["gt_id", "before_tracker_id", "after_tracker_id", "before_hits",
         "before_samples", "after_hits", "after_samples", "status"],
    )
    write_rows(
        os.path.join(args.output_dir, "occlusion_prediction_error.csv"),
        pred_rows,
        ["horizon", "rmse", "mean", "n"],
    )
    write_single_row(
        os.path.join(args.output_dir, "occlusion_reid_metrics.csv"),
        metrics,
    )
    with open(os.path.join(args.output_dir, "occlusion_reid_metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2, sort_keys=True)

    print("Occlusion Re-ID metrics:")
    for key, value in metrics.items():
        print(f"  {key}: {value}")


if __name__ == "__main__":
    main()
