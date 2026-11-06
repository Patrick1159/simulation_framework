#!/usr/bin/env python3
"""
calc_metrics.py — Compute CLEAR MOT style tracking metrics from GT and tracker CSVs.

Metrics: MOTA, IDSW, FP, FN, epos (position RMSE)

Matching:
  1. Evaluate over the full union of GT/tracker timestamps.
  2. Preserve valid previous-frame associations when possible.
  3. Solve the remaining one-to-one assignment by minimum distance.

Optional: --bag to enable FOV filtering (only count GT visible to camera).

Usage: python3 calc_metrics.py --gt gt.csv --tracker tracker.csv [--bag raw.bag]
"""

import argparse
import csv
import math
from collections import defaultdict
import numpy as np

from tracking_eval_utils import build_fov_filter, match_detections

MATCH_DIST_THRESHOLD = 1.0  # meters
def load_csv(path, bin_ms=100):
    """Load CSV into list of dicts, grouped by timestamp bin."""
    frames = defaultdict(list)
    with open(path) as f:
        for row in csv.DictReader(f):
            t = round(float(row["timestamp"]) * (1000 / bin_ms)) * bin_ms / 1000
            frames[t].append(row)
    return frames


def deduplicate_frame(rows):
    """Keep only one row per ID within a time bin."""
    seen = {}
    for r in rows:
        rid = int(r["id"])
        if rid not in seen:
            seen[rid] = r
    return list(seen.values())


def compute_metrics(gt_frames, trk_frames, fov_filter=None):
    # CLEAR MOT style: evaluate over the full union of GT and tracker frames.
    eval_times = sorted(set(gt_frames.keys()) | set(trk_frames.keys()))

    total_gt = 0
    total_fp = 0
    total_fn = 0
    total_idsw = 0
    all_errors = []
    filtered_gt = 0

    # Track only previous-frame assignments so switches are counted on
    # consecutive associations, which is the standard CLEAR MOT convention.
    prev_matches = {}

    # Frag: count times a GT trajectory is lost then re-acquired
    gt_was_tracked = set()
    gt_state = {}  # gt_id -> 'tracked' | 'untracked'
    total_frag = 0

    for t in eval_times:
        gt_list = deduplicate_frame(gt_frames.get(t, []))
        trk_list = deduplicate_frame(trk_frames.get(t, []))

        # Filter to only include obstacles visible in camera FOV
        if fov_filter is not None:
            gt_visible = [g for g in gt_list
                          if fov_filter(t, float(g["x"]), float(g["y"]))]
            filtered_gt += len(gt_list) - len(gt_visible)
            gt_list = gt_visible
            trk_list = [tr for tr in trk_list
                       if fov_filter(t, float(tr["x"]), float(tr["y"]))]

        total_gt += len(gt_list)

        matches, unmatched_gt, unmatched_trk = match_detections(
            gt_list, trk_list, MATCH_DIST_THRESHOLD, preferred_pairs=prev_matches
        )
        total_fp += len(unmatched_trk)
        total_fn += len(unmatched_gt)

        # Frag: for each GT, check tracked/untracked state transitions
        tracked_gt_ids = set()
        for g, tr, dist in matches:
            tracked_gt_ids.add(int(g["id"]))

        for g in gt_list:
            gt_id = int(g["id"])
            is_tracked = gt_id in tracked_gt_ids
            if is_tracked:
                if gt_state.get(gt_id, 'untracked') == 'untracked' and gt_id in gt_was_tracked:
                    total_frag += 1
                gt_state[gt_id] = 'tracked'
                gt_was_tracked.add(gt_id)
            else:
                gt_state[gt_id] = 'untracked'

        curr_matches = {}
        for g, tr, dist in matches:
            gt_id = int(g["id"])
            trk_id = int(tr["id"])
            if gt_id in prev_matches and prev_matches[gt_id] != trk_id:
                total_idsw += 1
            curr_matches[gt_id] = trk_id
            r = float(g.get("radius", 0))
            all_errors.append(max(0.0, dist - r))

        prev_matches = curr_matches

    if total_gt == 0 or len(eval_times) == 0:
        mota = float("nan")
    else:
        mota = 1.0 - (total_fp + total_fn + total_idsw) / total_gt
    epos = math.sqrt(np.mean(np.array(all_errors) ** 2)) if all_errors else float("nan")

    return {
        "MOTA": mota,
        "IDSW": total_idsw,
        "Frag": total_frag,
        "FP": total_fp,
        "FN": total_fn,
        "epos": epos,
        "total_gt": total_gt,
        "total_frames": len(eval_times),
        "filtered_gt": filtered_gt,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--gt", required=True)
    p.add_argument("--tracker", required=True)
    p.add_argument("--bag", default=None, help="Rosbag for FOV filtering")
    p.add_argument("--output", default=None, help="Optional output CSV")
    args = p.parse_args()

    fov_filter = None
    if args.bag:
        print("Building FOV filter from bag TF...")
        fov_filter = build_fov_filter(args.bag)

    gt = load_csv(args.gt, bin_ms=100)
    trk = load_csv(args.tracker, bin_ms=100)
    m = compute_metrics(gt, trk, fov_filter=fov_filter)

    print(f"Frames: {m['total_frames']}  GT detections: {m['total_gt']}")
    if m['filtered_gt'] > 0:
        print(f"  (filtered {m['filtered_gt']} out-of-FOV GT detections)")
    print(f"MOTA:  {m['MOTA']:.4f}")
    print(f"IDSW:  {m['IDSW']}")
    print(f"Frag:  {m['Frag']}")
    print(f"FP:    {m['FP']}")
    print(f"FN:    {m['FN']}")
    print(f"epos:  {m['epos']:.4f} m")

    if args.output:
        with open(args.output, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=m.keys())
            w.writeheader()
            w.writerow(m)
        print(f"Saved to {args.output}")


if __name__ == "__main__":
    main()
