#!/usr/bin/env python3
"""
stage2_analyze.py — Stage 2 analysis: compare real UMAP vs GT vs Tracker.

Reads the Stage 2 rosbag and produces:
  1. stage2_timeseries.png  — GT_base, UMAP_base, Tracker_base, Tracker_odom over time
  2. stage2_errors.png      — e_umap_base, e_tracker_base over time
  3. stage2_summary.json    — aggregate statistics

e_umap_base    = UMAP_base.x - GT_base.x   (depth extraction error)
e_tracker_base = Tracker_base.x - GT_base.x (pipeline error)

Usage:
  python3 stage2_analyze.py --bag stage2.bag
"""

import argparse
import json
import math
import os
import sys
from collections import defaultdict

import rosbag
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def extract_positions_obstacle_array(bag, topic):
    """Extract first obstacle position from ObstacleArray messages."""
    data = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        if msg.obstacles:
            obs = msg.obstacles[0]
            data.append((t.to_sec(), obs.position.x, obs.position.y,
                         obs.position.z))
    return data


def extract_positions_tracker(bag, topic):
    """Extract first tracker position from TrackerDetailArray messages."""
    data = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        if msg.trackers:
            tr = msg.trackers[0]
            data.append((t.to_sec(), tr.position.x, tr.position.y,
                         tr.position.z))
    return data


def extract_robot_odom(bag):
    """Extract robot x position from /odom."""
    data = []
    for _, msg, t in bag.read_messages(topics=['/odom']):
        data.append((t.to_sec(), msg.pose.pose.position.x))
    return data


def nearest_value(ref_ts, data, idx=1):
    """Get value from data nearest to ref_ts. Returns None if gap > 0.1s."""
    best_val = None
    best_dt = float('inf')
    for ts, *vals in data:
        dt = abs(ts - ref_ts)
        if dt < 0.1 and dt < best_dt:
            best_dt = dt
            best_val = vals[idx - 1]
    return best_val


def make_plot(output_prefix, gt_base, umap_base, tracker_base, tracker_odom,
              robot_odom):
    """Generate two diagnostic plots."""
    # ---- Figure 1: Position time series ----
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    if gt_base:
        tb, gx = zip(*[(t, x) for t, x, _, _ in gt_base])
        ax1.plot(tb, gx, 'k-', linewidth=2, label='GT_base.x')
    if umap_base:
        ub, ux = zip(*[(t, x) for t, x, _, _ in umap_base])
        ax1.plot(ub, ux, 'b--', linewidth=1.5, label='UMAP_base.x')
    if tracker_base:
        tb2, tx = zip(*[(t, x) for t, x, _, _ in tracker_base])
        ax1.plot(tb2, tx, 'r-', linewidth=1.5, label='Tracker_base.x')

    ax1.set_ylabel('Position X in base_link (m)')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Stage 2: Obstacle position relative to base_link')

    # Tracker_odom + robot odom
    if tracker_odom:
        to_t, to_x = zip(*[(t, x) for t, x, _, _ in tracker_odom])
        ax2.plot(to_t, to_x, 'r-', linewidth=1.5, label='Tracker_odom.x')
    if robot_odom:
        ro_t, ro_x = zip(*[(t, x) for t, x in robot_odom])
        ax2.plot(ro_t, ro_x, 'grey', linewidth=1, alpha=0.5,
                 label='Robot odom.x')

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position X in odom (m)')
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Obstacle (should be constant) and Robot in odom')

    fig.tight_layout()
    png1 = f'{output_prefix}_timeseries.png'
    fig.savefig(png1, dpi=150)
    plt.close(fig)
    print(f"Saved {png1}")

    # ---- Figure 2: Error time series ----
    fig, ax = plt.subplots(figsize=(12, 5))

    # Compute e_umap_base and e_tracker_base at each GT timestep
    e_umap_ts, e_umap_vals = [], []
    e_tracker_ts, e_tracker_vals = [], []

    for ts, gx, _, _ in gt_base:
        # UMAP error
        ux = nearest_value(ts, umap_base, 1)
        if ux is not None:
            e_umap_ts.append(ts)
            e_umap_vals.append(ux - gx)
        # Tracker error
        tx = nearest_value(ts, tracker_base, 1)
        if tx is not None:
            e_tracker_ts.append(ts)
            e_tracker_vals.append(tx - gx)

    if e_umap_vals:
        ax.plot(e_umap_ts, e_umap_vals, 'b-', linewidth=1.5, alpha=0.8,
                label='e_umap_base (UMAP - GT)')
    if e_tracker_vals:
        ax.plot(e_tracker_ts, e_tracker_vals, 'r-', linewidth=1.5, alpha=0.8,
                label='e_tracker_base (Tracker - GT)')

    ax.axhline(y=0, color='k', linewidth=0.5, linestyle=':')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error X (m)')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    ax.set_title('Stage 2: Position Error vs GT_base')

    # Add stats text
    stats_lines = []
    if e_umap_vals:
        stats_lines.append(
            f"e_umap: mean={np.mean(e_umap_vals):.3f} "
            f"std={np.std(e_umap_vals):.3f} "
            f"max={np.max(np.abs(e_umap_vals)):.3f}")
    if e_tracker_vals:
        stats_lines.append(
            f"e_tracker: mean={np.mean(e_tracker_vals):.3f} "
            f"std={np.std(e_tracker_vals):.3f} "
            f"max={np.max(np.abs(e_tracker_vals)):.3f}")

    ax.text(0.02, 0.98, '\n'.join(stats_lines), transform=ax.transAxes,
            fontsize=9, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    fig.tight_layout()
    png2 = f'{output_prefix}_errors.png'
    fig.savefig(png2, dpi=150)
    plt.close(fig)
    print(f"Saved {png2}")

    return e_umap_vals, e_tracker_vals


def analyze(bag_path, output_prefix):
    print(f"Loading: {bag_path}")
    with rosbag.Bag(bag_path, 'r') as bag:
        gt_base      = extract_positions_obstacle_array(bag, '/gt_obstacles_base')
        gt_camera    = extract_positions_obstacle_array(bag, '/gt_obstacles_camera')
        gt_odom      = extract_positions_obstacle_array(bag, '/obstacles')
        umap_base    = extract_positions_obstacle_array(bag, '/umap_base')
        tracker_base = extract_positions_tracker(bag, '/tracker_details_base')
        tracker_odom = extract_positions_tracker(bag, '/rematch/tracker_details')
        robot_odom   = extract_robot_odom(bag)

    print(f"  GT_base:      {len(gt_base)} msgs")
    print(f"  GT_camera:    {len(gt_camera)} msgs")
    print(f"  GT_odom:      {len(gt_odom)} msgs")
    print(f"  UMAP_base:    {len(umap_base)} msgs")
    print(f"  Tracker_base: {len(tracker_base)} msgs")
    print(f"  Tracker_odom: {len(tracker_odom)} msgs")
    print(f"  Robot odom:   {len(robot_odom)} msgs")

    if not gt_base:
        print("\nERROR: No GT_base data")
        return 1
    if not umap_base:
        print("\nERROR: No UMAP_base data. Check umap_to_base node.")
        return 1
    if not tracker_base:
        print("\nERROR: No Tracker_base data. Check pipeline.")
        return 1

    # Generate plots
    e_umap, e_tracker = make_plot(output_prefix, gt_base, umap_base,
                                   tracker_base, tracker_odom, robot_odom)

    # CLI summary
    print(f"\n── Error Summary ──")
    if e_umap:
        print(f"  e_umap_base:    mean={np.mean(e_umap):.3f} m  "
              f"std={np.std(e_umap):.3f} m  max_abs={np.max(np.abs(e_umap)):.3f} m")
    if e_tracker:
        print(f"  e_tracker_base: mean={np.mean(e_tracker):.3f} m  "
              f"std={np.std(e_tracker):.3f} m  max_abs={np.max(np.abs(e_tracker)):.3f} m")

    if e_umap and e_tracker:
        umap_rms = np.sqrt(np.mean(np.array(e_umap) ** 2))
        tracker_rms = np.sqrt(np.mean(np.array(e_tracker) ** 2))
        print(f"  UMAP RMS:       {umap_rms:.3f} m")
        print(f"  Tracker RMS:    {tracker_rms:.3f} m")

        if umap_rms > 0.3 and umap_rms > tracker_rms * 1.5:
            print("\n  >>> UMAP measurement error DOMINATES")
            print("      Real depth/UMAP extraction is the primary error source.")
        elif tracker_rms > 0.3 and tracker_rms > umap_rms * 1.5:
            print("\n  >>> Tracker error DOMINATES")
            print("      Tracker/KF/TF adds error beyond UMAP measurement.")
        elif umap_rms < 0.2 and tracker_rms < 0.2:
            print("\n  >>> Both errors are small — pipeline is working correctly.")
        else:
            print("\n  >>> Both UMAP and Tracker have comparable errors.")

    # JSON summary
    summary = {
        'n_gt_base': len(gt_base),
        'n_gt_camera': len(gt_camera),
        'n_umap_base': len(umap_base),
        'n_tracker_base': len(tracker_base),
        'n_tracker_odom': len(tracker_odom),
    }
    if e_umap:
        summary['e_umap_mean'] = float(np.mean(e_umap))
        summary['e_umap_std'] = float(np.std(e_umap))
        summary['e_umap_rms'] = float(np.sqrt(np.mean(np.array(e_umap) ** 2)))
    if e_tracker:
        summary['e_tracker_mean'] = float(np.mean(e_tracker))
        summary['e_tracker_std'] = float(np.std(e_tracker))
        summary['e_tracker_rms'] = float(np.sqrt(np.mean(np.array(e_tracker) ** 2)))

    json_path = f'{output_prefix}_summary.json'
    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"\nSaved {json_path}")
    return 0


def main():
    parser = argparse.ArgumentParser(description="Stage 2 analysis")
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', default='stage2')
    args = parser.parse_args()
    sys.exit(analyze(args.bag, args.output))


if __name__ == '__main__':
    main()
