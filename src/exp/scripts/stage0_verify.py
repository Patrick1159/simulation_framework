#!/usr/bin/env python3
"""
stage0_verify.py — Quick verification for Stage 0 TF diagnostic.

Reads the recorded bag, extracts /tracker_details_base and /odom,
and checks whether the fake obstacle stays at ~0.5 m relative to base_link
as the robot moves.

Usage:
  python3 stage0_verify.py --bag stage0_run.bag
"""

import argparse
import math
import sys
from collections import defaultdict

import rosbag
import numpy as np


def verify(bag_path):
    tracker_positions = []  # (ts, x, y, z) of first tracker in base_link
    odom_positions = []     # (ts, x, y) of robot

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(
                topics=['/tracker_details_base', '/odom']):
            ts = t.to_sec()
            if topic == '/tracker_details_base':
                if msg.trackers:
                    tr = msg.trackers[0]
                    tracker_positions.append(
                        (ts, tr.position.x, tr.position.y, tr.position.z))
            elif topic == '/odom':
                odom_positions.append(
                    (ts, msg.pose.pose.position.x, msg.pose.pose.position.y))

    if not tracker_positions:
        print("ERROR: No /tracker_details_base messages found. "
              "The tracker may not have initialized. Check:")
        print("  1. Is /umap/3D_bounding_boxes being published?")
        print("  2. Is coarse_match running?")
        print("  3. Is the relay /tracker/dynamic_obstacles -> "
              "/model_output/obstacles active?")
        print("  4. Is detailed_match running?")
        sys.exit(1)

    if not odom_positions:
        print("WARNING: No /odom messages found. "
              "Cannot correlate with robot motion.")

    print(f"Tracker messages: {len(tracker_positions)}")
    print(f"Odom messages:    {len(odom_positions)}")

    xs = [p[1] for p in tracker_positions]
    print(f"\nTracker_base.x statistics:")
    print(f"  mean:  {np.mean(xs):.3f} m")
    print(f"  std:   {np.std(xs):.3f} m")
    print(f"  min:   {np.min(xs):.3f} m")
    print(f"  max:   {np.max(xs):.3f} m")
    print(f"  range: {np.max(xs) - np.min(xs):.3f} m")
    print(f"  first: {xs[0]:.3f} m")
    print(f"  last:  {xs[-1]:.3f} m")

    # Check for drift: correlate Tracker_base.x with robot displacement
    drift_detected = False
    if odom_positions:
        x0, y0 = odom_positions[0][1], odom_positions[0][2]
        paired = []
        for t_ts, tx, ty, tz in tracker_positions:
            best_dist = float('inf')
            best_odom_disp = 0.0
            for o_ts, ox, oy in odom_positions:
                dt = abs(o_ts - t_ts)
                if dt < 0.1 and dt < best_dist:
                    best_dist = dt
                    best_odom_disp = math.sqrt((ox - x0)**2 + (oy - y0)**2)
            if best_dist < 0.1:
                paired.append((best_odom_disp, tx))

        if len(paired) > 10:
            disp = np.array([d for d, _ in paired])
            t_x = np.array([x for _, x in paired])
            if np.std(disp) > 0.01:
                slope, intercept = np.polyfit(disp, t_x, 1)
                corr = float(np.corrcoef(disp, t_x)[0, 1])
                print(f"\nDrift analysis:")
                print(f"  slope vs displacement: {slope:.3f} m/m")
                print(f"  intercept (at origin): {intercept:.3f} m")
                print(f"  correlation: {corr:.3f}")
                if abs(slope) > 0.15 and abs(corr) > 0.3:
                    drift_detected = True
                    print(f"  >>> DRIFT DETECTED: Tracker moves with robot!")
                else:
                    print(f"  No significant drift detected.")
            else:
                print(f"\n  Robot didn't move enough to measure drift.")
        else:
            print(f"\n  Insufficient paired data for drift analysis.")

    # Verdict
    print("\n" + "=" * 60)
    mean_x = np.mean(xs)
    std_x = np.std(xs)
    if 0.3 < mean_x < 0.8 and std_x < 0.2 and not drift_detected:
        print("  VERDICT: TF chain looks CORRECT")
        print("  Tracker_base.x ≈ 0.5 m, stable during motion.")
        print("  Problem is NOT in TF/frame/stamp — investigate UMAP/depth.")
    else:
        print("  VERDICT: TF chain is WRONG")
        if mean_x < 0.2 or mean_x > 1.0:
            print(f"  Tracker_base.x = {mean_x:.2f} m (expected ~0.5)")
            print("  Camera extrinsic or frame semantic is incorrect.")
        if std_x > 0.2:
            print(f"  Tracker_base.x std = {std_x:.2f} m (unstable)")
        if drift_detected:
            print("  Tracker position correlates with robot displacement.")
            print("  Likely cause: static TF (camera_pose.py) not updating.")
            print("  Try: use_camera_pose_tf:=false to isolate.")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Stage 0 TF verification")
    parser.add_argument('--bag', required=True, help='Path to rosbag')
    args = parser.parse_args()
    verify(args.bag)


if __name__ == '__main__':
    main()
