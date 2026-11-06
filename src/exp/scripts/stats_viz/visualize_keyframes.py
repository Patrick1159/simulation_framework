#!/usr/bin/env python3
"""
Render top-down keyframe visualizations from experiment rosbags.

For each run directory, reads:
  - clearance_timeseries.csv  → identifies critical frames
  - raw.bag                   → robot pose, GT obstacles, tracker, MPC path
  - tracker.csv               → tracker outputs (already in odom frame)

Produces keyframes/ subdirectory with one PDF per key moment.

Usage:
  python3 visualize_keyframes.py /path/to/run_dir
  python3 visualize_keyframes.py /path/to/run_dir --method ours_full
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag
from nav_msgs.msg import Path


# ── styles ──────────────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family": "serif", "font.size": 8,
    "axes.labelsize": 9, "axes.titlesize": 9,
    "legend.fontsize": 7, "figure.dpi": 200,
    "savefig.dpi": 200, "savefig.bbox": "tight",
})

ROBOT_COLOR = "#2C3E50"
GT_COLOR = "#E74C3C"
TRACKER_COLOR = "#3498DB"
PATH_COLOR = "#2ECC71"
CAPSULE_COLOR = "#F39C12"
CLEARANCE_OK = "#27AE60"
CLEARANCE_NEAR = "#F39C12"
CLEARANCE_COL = "#E74C3C"


def load_csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def load_tracker_csv(path):
    """Load tracker.csv. Returns dict: timestamp_float → list of rows, and sorted timestamps."""
    frames = defaultdict(list)
    for row in load_csv(path):
        frames[float(row["timestamp"])].append(row)
    return frames


def tracker_at_ts(tracker_frames, ts, max_dt=0.15):
    """Return tracker rows closest to ts within max_dt seconds."""
    if not tracker_frames:
        return None
    keys = sorted(tracker_frames.keys())
    idx = min(range(len(keys)), key=lambda i: abs(keys[i] - ts))
    if abs(keys[idx] - ts) > max_dt:
        return None
    return tracker_frames[keys[idx]]


def load_bag_data(bag_path, topics):
    """Extract time-indexed data from a rosbag.

    Returns dict with keys: odom, gt_obstacles, pred_paths, swept_capsules
    """
    bag = rosbag.Bag(bag_path, "r")

    odom_frames = {}
    for _, msg, t in bag.read_messages(topics=["/odom"]):
        ts = t.to_sec()
        odom_frames[ts] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            2.0 * math.atan2(msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w),
        )

    gt_frames = defaultdict(list)
    for _, msg, t in bag.read_messages(topics=["/obstacles"]):
        ts = t.to_sec()
        for o in msg.obstacles:
            r = float(getattr(o, "radius", 0.3) or 0.3)
            gt_frames[ts].append({
                "id": int(o.id),
                "x": float(o.position.x),
                "y": float(o.position.y),
                "vx": float(o.velocity.x),
                "vy": float(o.velocity.y),
                "radius": r,
            })

    path_frames = {}
    if "/mpc/pred_path" in topics:
        for _, msg, t in bag.read_messages(topics=["/mpc/pred_path"]):
            ts = t.to_sec()
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if pts:
                path_frames[ts] = pts

    capsule_frames = {}
    if "/mpc/swept_capsules" in topics:
        for _, msg, t in bag.read_messages(topics=["/mpc/swept_capsules"]):
            ts = t.to_sec()
            capsules = []
            for m in msg.markers:
                if len(m.points) >= 2:
                    p0 = m.points[0]
                    p1 = m.points[1]
                    r = m.scale.x / 2.0 if m.scale.x > 0 else 0.15
                    capsules.append(((p0.x, p0.y), (p1.x, p1.y), r))
            if capsules:
                capsule_frames[ts] = capsules

    bag.close()
    return {
        "odom": odom_frames,
        "gt": gt_frames,
        "paths": path_frames,
        "capsules": capsule_frames,
    }


def nearest_data(data_dict, ts):
    """Return the value in data_dict with the closest timestamp to ts."""
    if not data_dict:
        return None
    keys = sorted(data_dict.keys())
    idx = min(range(len(keys)), key=lambda i: abs(keys[i] - ts))
    if abs(keys[idx] - ts) > 0.2:
        return None
    return data_dict[keys[idx]]


def find_keyframes(clearance_rows, max_keyframes=6):
    """Identify keyframes from clearance timeseries.

    Returns list of (timestamp, reason, row) tuples.
    """
    if not clearance_rows:
        return []

    keyframes = []
    events = []

    # Find collision events
    in_collision = False
    for row in clearance_rows:
        c = float(row["clearance"])
        ts = float(row["timestamp"])
        if c <= 0.0 and not in_collision:
            events.append((ts, "collision_start", row, -c))
            in_collision = True
        elif c > 0.1 and in_collision:
            in_collision = False

    # Find near-collision events
    in_near = False
    for row in clearance_rows:
        c = float(row["clearance"])
        ts = float(row["timestamp"])
        if c <= 0.15 and not in_near and c > 0.0:
            events.append((ts, "near_collision", row, -c))
            in_near = True
        elif c > 0.2 and in_near:
            in_near = False

    # Add worst-clearance frames (one per GT obstacle)
    by_obs = defaultdict(list)
    for row in clearance_rows:
        by_obs[int(row["gt_id"])].append((float(row["clearance"]), float(row["timestamp"]), row))
    for oid, pairs in by_obs.items():
        worst_c, worst_ts, worst_row = min(pairs, key=lambda x: x[0])
        keyframes.append((worst_ts, f"worst_clearance_id{oid}", worst_row))

    # Merge event frames
    for ts, reason, row, _ in events:
        keyframes.append((ts, reason, row))

    # Sort by time and deduplicate
    keyframes.sort(key=lambda x: x[0])
    seen_ts = set()
    unique = []
    for ts, reason, row in keyframes:
        t_key = round(ts, 2)
        if t_key not in seen_ts:
            seen_ts.add(t_key)
            unique.append((ts, reason, row))

    return unique[:max_keyframes]


def render_frame(ax, ts, bag_data, tracker_frames, clearance_row, title=""):
    """Render a single top-down frame."""
    odom = nearest_data(bag_data["odom"], ts)
    gt = nearest_data(bag_data["gt"], ts)
    path = nearest_data(bag_data["paths"], ts)
    capsules = nearest_data(bag_data["capsules"], ts)

    # Find nearby tracker data
    trk_rows = tracker_at_ts(tracker_frames, ts) if tracker_frames else None

    # ── plot robot ──
    if odom:
        rx, ry, ryaw = odom
        # Robot as triangle
        robot_len = 0.3
        dx = robot_len * math.cos(ryaw)
        dy = robot_len * math.sin(ryaw)
        ax.plot(rx, ry, "o", color=ROBOT_COLOR, markersize=6, zorder=5, label="Robot")
        ax.arrow(rx, ry, dx, dy, head_width=0.1, head_length=0.08,
                 fc=ROBOT_COLOR, ec=ROBOT_COLOR, zorder=5)
        # Robot body circle
        robot_circle = plt.Circle((rx, ry), 0.18, fill=False, color=ROBOT_COLOR,
                                  linewidth=1, linestyle="--", alpha=0.5)
        ax.add_patch(robot_circle)

    # ── plot MPC swept capsules ──
    if capsules:
        for (p0, p1), (q0, q1), r in capsules[:20]:
            ax.plot([p0, q0], [p1, q1], color=CAPSULE_COLOR, linewidth=r * 30,
                    alpha=0.25, solid_capstyle="round", zorder=2)
        # Dummy line for legend
        ax.plot([], [], color=CAPSULE_COLOR, linewidth=2, alpha=0.5,
                label="MPC swept capsules")

    # ── plot MPC pred path ──
    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, color=PATH_COLOR, linewidth=1.5, alpha=0.7,
                zorder=3, label="MPC pred path")

    # ── plot GT obstacles ──
    if gt:
        for obs in gt:
            o = obs  # noqa (shadow for brevity)
            ax.plot(o["x"], o["y"], "o", color=GT_COLOR, markersize=o["radius"] * 30,
                    alpha=0.3, zorder=1)
            ax.plot(o["x"], o["y"], "x", color=GT_COLOR, markersize=4, zorder=4)
            arrow_len = 0.5
            vmag = math.hypot(o["vx"], o["vy"])
            if vmag > 0.01:
                dx_v = o["vx"] / vmag * arrow_len
                dy_v = o["vy"] / vmag * arrow_len
                ax.arrow(o["x"], o["y"], dx_v, dy_v, head_width=0.08,
                         head_length=0.06, fc=GT_COLOR, ec=GT_COLOR,
                         alpha=0.7, zorder=4)
    # Dummy GT legend entry
    ax.plot([], [], "x", color=GT_COLOR, markersize=4, label="GT obstacles")

    # ── plot tracker outputs ──
    if trk_rows:
        for row in trk_rows:
            tx, ty = float(row["x"]), float(row["y"])
            vx, vy = float(row.get("vx", 0) or 0), float(row.get("vy", 0) or 0)
            ax.plot(tx, ty, "s", color=TRACKER_COLOR, markersize=5,
                    alpha=0.7, zorder=4)
            vmag = math.hypot(vx, vy)
            if vmag > 0.01:
                ax.arrow(tx, ty, vx / vmag * 0.4, vy / vmag * 0.4,
                         head_width=0.06, head_length=0.05,
                         fc=TRACKER_COLOR, ec=TRACKER_COLOR,
                         alpha=0.6, zorder=4)
        ax.plot([], [], "s", color=TRACKER_COLOR, markersize=5, label="Tracker")

    # ── clearance annotation ──
    if clearance_row:
        obs_x = float(clearance_row["obs_x"])
        obs_y = float(clearance_row["obs_y"])
        robot_x = float(clearance_row["robot_x"])
        robot_y = float(clearance_row["robot_y"])
        clr = float(clearance_row["clearance"])
        mid_x = (obs_x + robot_x) / 2
        mid_y = (obs_y + robot_y) / 2
        color = CLEARANCE_OK if clr > 0.15 else (CLEARANCE_NEAR if clr > 0 else CLEARANCE_COL)
        ax.annotate(f"{clr:.2f} m", (mid_x, mid_y),
                    fontsize=7, color=color, ha="center", va="bottom",
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="white",
                              alpha=0.8, edgecolor=color))

    # ── formatting ──
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title, fontsize=9)
    ax.legend(loc="upper right", fontsize=6, framealpha=0.8)
    ax.grid(alpha=0.3)
    ax.autoscale()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("run_dir", help="Path to run directory with raw.bag and CSVs")
    p.add_argument("--method", default=None, help="Method name for title")
    p.add_argument("--output-dir", default=None, help="Override output directory")
    p.add_argument("--max-keyframes", type=int, default=6,
                   help="Max keyframes to render")
    p.add_argument("--occlusion-start", type=float, default=None,
                   help="Occlusion start time offset for Experiment 2")
    p.add_argument("--occlusion-duration", type=float, default=None,
                   help="Occlusion duration for Experiment 2")
    p.add_argument("--xlim", type=float, nargs=2, default=None,
                   help="X-axis limits (e.g. -3 5)")
    p.add_argument("--ylim", type=float, nargs=2, default=None,
                   help="Y-axis limits (e.g. -3 5)")
    args = p.parse_args()

    run_dir = os.path.abspath(args.run_dir)
    bag_path = os.path.join(run_dir, "raw.bag")
    clearance_path = os.path.join(run_dir, "clearance_timeseries.csv")
    tracker_path = os.path.join(run_dir, "tracker.csv")

    if not os.path.exists(bag_path):
        sys.exit(f"Bag not found: {bag_path}")

    out_dir = args.output_dir or os.path.join(run_dir, "keyframes")
    os.makedirs(out_dir, exist_ok=True)

    # Load clearance data for keyframe detection
    clearance_rows = load_csv(clearance_path)
    keyframes = find_keyframes(clearance_rows, args.max_keyframes)

    # For occlusion experiment: add occlusion start / reappearance as keyframes
    if args.occlusion_start is not None:
        bag = rosbag.Bag(bag_path, "r")
        t0 = bag.get_start_time()
        bag.close()
        occ_start = t0 + args.occlusion_start
        keyframes.append((occ_start, "occlusion_start", None))
        if args.occlusion_duration:
            reappear = occ_start + args.occlusion_duration
            keyframes.append((reappear, "occlusion_reappear", None))

    if not keyframes:
        print("No keyframes identified — rendering frames at 1/3 and 2/3 of bag duration")
        bag = rosbag.Bag(bag_path, "r")
        start = bag.get_start_time()
        end = bag.get_end_time()
        bag.close()
        dur = end - start
        for i, frac in enumerate([0.33, 0.66]):
            ts = start + dur * frac
            keyframes.append((ts, f"snapshot_{i + 1}", None))

    # Load bag data
    print(f"Loading bag: {bag_path}")
    bag_info = rosbag.Bag(bag_path, "r")
    available_topics = set(bag_info.get_type_and_topic_info().topics.keys())
    bag_info.close()
    bag_data = load_bag_data(bag_path, available_topics)

    # Load tracker CSV (may be empty for methods with no output)
    tracker_frames = load_tracker_csv(tracker_path) if os.path.exists(tracker_path) else {}

    method_str = f" [{args.method}]" if args.method else ""

    for i, (ts, reason, clearance_row) in enumerate(keyframes):
        fig, ax = plt.subplots(figsize=(6, 5))
        reason_label = reason.replace("_", " ").title()
        title = f"t = {ts:.2f}s — {reason_label}{method_str}"
        render_frame(ax, ts, bag_data, tracker_frames, clearance_row, title=title)

        if args.xlim:
            ax.set_xlim(args.xlim)
        if args.ylim:
            ax.set_ylim(args.ylim)

        fname = f"keyframe_{i + 1:02d}_{reason}.pdf"
        fig.savefig(os.path.join(out_dir, fname))
        plt.close(fig)
        print(f"  Saved: {fname}")

    print(f"Keyframes saved to {out_dir}/")


if __name__ == "__main__":
    main()
