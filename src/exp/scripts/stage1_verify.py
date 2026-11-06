#!/usr/bin/env python3
"""
stage1_verify.py — Verification for Stage 1 perfect measurement test.

Checks whether the TF chain odom→camera→odom round-trip is correct
by comparing Tracker output against GT for a world-fixed obstacle.

Key metrics:
  Tracker_odom vs GT_odom: should be ≈0 (constant obstacle position)
  Tracker_base vs GT_base: should be ≈0 (correct relative distance)
  Tracker_odom std:       should be small (stable tracking)

Usage:
  python3 stage1_verify.py --bag stage1_run.bag --obstacle_x 2.0
"""

import argparse
import math
import sys
from collections import defaultdict

import rosbag
import numpy as np


def extract_tracker_positions(bag, topic):
    """Extract first tracker position from TrackerDetailArray messages."""
    positions = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        if msg.trackers:
            tr = msg.trackers[0]
            positions.append((t.to_sec(), tr.position.x, tr.position.y,
                              tr.position.z))
    return positions


def extract_gt_positions(bag, topic):
    """Extract first obstacle position from ObstacleArray messages."""
    positions = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        if msg.obstacles:
            obs = msg.obstacles[0]
            positions.append((t.to_sec(), obs.position.x, obs.position.y,
                              obs.position.z))
    return positions


def extract_odom(bag):
    """Extract robot position and speed from /odom."""
    positions = []
    for _, msg, t in bag.read_messages(topics=['/odom']):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        speed = math.sqrt(msg.twist.twist.linear.x ** 2 +
                          msg.twist.twist.linear.y ** 2)
        positions.append((t.to_sec(), px, py, speed))
    return positions


def associate_by_time(ref_data, tgt_data, tolerance=0.08):
    """Match ref and tgt data by nearest timestamp. Returns aligned pairs."""
    pairs = []
    tgt_idx = 0
    for ref_ts, *ref_vals in ref_data:
        while (tgt_idx < len(tgt_data) - 1 and
               tgt_data[tgt_idx + 1][0] < ref_ts - tolerance):
            tgt_idx += 1
        best_dt = float('inf')
        best_vals = None
        for j in range(max(0, tgt_idx - 3), min(len(tgt_data), tgt_idx + 4)):
            dt = abs(tgt_data[j][0] - ref_ts)
            if dt < tolerance and dt < best_dt:
                best_dt = dt
                best_vals = tgt_data[j][1:]
        if best_vals is not None:
            pairs.append((ref_ts, list(ref_vals), list(best_vals)))
    return pairs


def verify(bag_path, obs_x, obs_y, obs_z):
    with rosbag.Bag(bag_path, 'r') as bag:
        tracker_odom = extract_tracker_positions(bag, '/rematch/tracker_details')
        tracker_base = extract_tracker_positions(bag, '/tracker_details_base')
        gt_odom      = extract_gt_positions(bag, '/obstacles')
        gt_base      = extract_gt_positions(bag, '/gt_obstacles_base')
        odom         = extract_odom(bag)

    print(f"Tracker_odom:  {len(tracker_odom)} msgs")
    print(f"Tracker_base:  {len(tracker_base)} msgs")
    print(f"GT_odom:       {len(gt_odom)} msgs")
    print(f"GT_base:       {len(gt_base)} msgs")
    print(f"/odom:         {len(odom)} msgs")

    if not tracker_odom:
        print("\nERROR: No Tracker_odom messages!")
        return 1
    if not gt_odom:
        print("\nERROR: No GT_odom messages!")
        return 1

    # --- Tracker_odom analysis ---
    to_x = [p[1] for p in tracker_odom]
    to_y = [p[2] for p in tracker_odom]
    to_z = [p[3] for p in tracker_odom]
    to_ts = [p[0] for p in tracker_odom]

    print(f"\n── Tracker_odom (obstacle should be fixed at "
          f"({obs_x:.1f}, {obs_y:.1f}, {obs_z:.1f})) ──")
    print(f"  x: mean={np.mean(to_x):.3f}  std={np.std(to_x):.3f}  "
          f"min={np.min(to_x):.3f}  max={np.max(to_x):.3f}")
    print(f"  y: mean={np.mean(to_y):.3f}  std={np.std(to_y):.3f}")
    print(f"  z: mean={np.mean(to_z):.3f}  std={np.std(to_z):.3f}")

    odom_err_x = [x - obs_x for x in to_x]
    odom_rmse = math.sqrt(np.mean(np.array(odom_err_x) ** 2))
    print(f"  RMSE vs GT: {odom_rmse:.3f} m")

    # --- Tracker_base analysis ---
    if tracker_base:
        tb_x = [p[1] for p in tracker_base]
        tb_ts = [p[0] for p in tracker_base]
        print(f"\n── Tracker_base ──")
        print(f"  x: first={tb_x[0]:.3f}  last={tb_x[-1]:.3f}  "
              f"mean={np.mean(tb_x):.3f}  std={np.std(tb_x):.3f}")

    # --- Tracker_odom vs GT_odom direct comparison ---
    print(f"\n── Tracker_odom vs GT_odom ──")
    to_vs_gt = associate_by_time(tracker_odom, gt_odom)
    if to_vs_gt:
        errs = []
        for ts, tr_vals, gt_vals in to_vs_gt:
            ex = tr_vals[0] - gt_vals[0]
            ey = tr_vals[1] - gt_vals[1]
            errs.append((ts, math.sqrt(ex**2 + ey**2)))
        dists = [e for _, e in errs]
        print(f"  n_pairs: {len(to_vs_gt)}")
        print(f"  mean 2D error: {np.mean(dists):.3f} m")
        print(f"  max 2D error:  {np.max(dists):.3f} m")
        print(f"  std 2D error:  {np.std(dists):.3f} m")

    # --- Tracker_base vs GT_base ---
    if tracker_base and gt_base:
        print(f"\n── Tracker_base vs GT_base ──")
        tb_vs_gtb = associate_by_time(tracker_base, gt_base)
        if tb_vs_gtb:
            errs = []
            for ts, tr_vals, gt_vals in tb_vs_gtb:
                ex = tr_vals[0] - gt_vals[0]
                ey = tr_vals[1] - gt_vals[1]
                errs.append((ts, math.sqrt(ex**2 + ey**2)))
            dists = [e for _, e in errs]
            print(f"  n_pairs: {len(tb_vs_gtb)}")
            print(f"  mean 2D error: {np.mean(dists):.3f} m")
            print(f"  max 2D error:  {np.max(dists):.3f} m")
            print(f"  std 2D error:  {np.std(dists):.3f} m")

    # --- Tracker_odom constancy vs robot displacement ---
    if odom:
        robot_disp = []
        x0, y0 = odom[0][1], odom[0][2]
        for ts, px, py, _ in odom:
            robot_disp.append((ts, math.sqrt((px - x0)**2 + (py - y0)**2)))

        paired = associate_by_time(tracker_odom,
                                   [(ts, d) for ts, d in robot_disp])
        if paired and len(paired) > 10:
            disp = np.array([p[2][0] for p in paired])
            t_ox = np.array([p[1][0] for p in paired])
            if np.std(disp) > 0.01:
                slope = np.polyfit(disp, t_ox, 1)[0]
                corr = float(np.corrcoef(disp, t_ox)[0, 1])
                print(f"\n── Tracker_odom vs Robot Displacement ──")
                print(f"  slope: {slope:.4f} m/m  (0 = perfect constancy)")
                print(f"  correlation: {corr:.3f}")
                if abs(slope) > 0.1 and abs(corr) > 0.3:
                    print(f"  WARNING: Tracker_odom drifts with robot motion!")
                else:
                    print(f"  OK: Tracker_odom is stable.")

    # --- Verdict ---
    print("\n" + "=" * 60)
    if odom_rmse < 0.2:
        print("  VERDICT: TF round-trip (odom→camera→odom) is CORRECT")
        print(f"  Tracker_odom RMSE = {odom_rmse:.3f} m (stable at GT position)")
        print("  The pipeline correctly tracks a world-fixed obstacle.")
        print("  If real UMAP degrades, the problem is in depth extraction,")
        print("  not in the TF/frame/stamp/KF chain.")
    elif odom_rmse < 0.5:
        print("  VERDICT: TF round-trip has MODERATE error")
        print(f"  Tracker_odom RMSE = {odom_rmse:.3f} m")
        print("  Pipeline mostly works but has some instability.")
    else:
        print("  VERDICT: TF round-trip is WRONG")
        print(f"  Tracker_odom RMSE = {odom_rmse:.3f} m")
        print("  Problem exists even with perfect measurement.")
        print("  Root cause: TF/stamp/frame/KF, not UMAP/depth.")
    print("=" * 60)
    return 0


def main():
    parser = argparse.ArgumentParser(description="Stage 1 verification")
    parser.add_argument('--bag', required=True)
    parser.add_argument('--obstacle_x', type=float, default=2.0)
    parser.add_argument('--obstacle_y', type=float, default=0.0)
    parser.add_argument('--obstacle_z', type=float, default=0.0)
    args = parser.parse_args()
    sys.exit(verify(args.bag, args.obstacle_x, args.obstacle_y, args.obstacle_z))


if __name__ == '__main__':
    main()
