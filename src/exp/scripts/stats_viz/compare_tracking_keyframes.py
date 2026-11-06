#!/usr/bin/env python3
"""
Generate a multi-panel BEV figure comparing tracking methods (DODT, FAPP, Ours)
at aligned keyframe timestamps for a crossing scene.

Produces an N rows (keyframes) x M cols (methods) grid showing GT obstacles
(red), tracker outputs (blue with ID labels), and robot position at each
keyframe.  Saves both PDF and PNG.

Usage:
  python3 compare_tracking_keyframes.py \
    --methods "DODT:/path/to/dodt/run_1" "FAPP:/path/to/fapp/run_1" "Ours:/path/to/ours/run_1" \
    --output-dir /tmp/compare \
    --num-keyframes 4 \
    --title "Crossing Scene: Tracking Comparison"
"""

import argparse
import math
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag

# Import utilities from visualize_keyframes.py (same directory)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from visualize_keyframes import (  # noqa: E402
    load_csv,
    load_tracker_csv,
    tracker_at_ts,
    load_bag_data,
    nearest_data,
)

# ── Matplotlib style ────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 8,
    "axes.labelsize": 7,
    "axes.titlesize": 9,
    "legend.fontsize": 6,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "figure.dpi": 200,
    "savefig.dpi": 200,
    "savefig.bbox": "tight",
})

# Colors (reused from visualize_keyframes.py)
ROBOT_COLOR = "#2C3E50"
GT_COLOR = "#E74C3C"
TRACKER_COLOR = "#3498DB"


# ── CLI helpers ─────────────────────────────────────────────────────────────

def parse_method_specs(raw_specs):
    """Parse "Name:/path/to/run" specs into (name, abs_path) tuples.

    Args:
        raw_specs: list of strings like ["DODT:/path/1", "FAPP:/path/2"]

    Returns:
        list of (name, absolute_path) tuples
    """
    specs = []
    for spec in raw_specs:
        name, path = spec.split(":", 1)
        specs.append((name, os.path.abspath(path)))
    return specs


# ── Keyframe selection ──────────────────────────────────────────────────────

def get_shared_keyframes(bag_path, gt_csv_path, num_keyframes=4):
    """Determine keyframe timestamps shared across all methods.

    Loads GT from the first method's gt.csv, finds timestamps with >=2
    obstacles, and samples *num_keyframes* evenly within that range.  Falls
    back to the rosbag's start/end times when no suitable GT timestamps exist.

    Args:
        bag_path: path to raw.bag (fallback time source)
        gt_csv_path: path to gt.csv (primary time source)
        num_keyframes: how many keyframes to return

    Returns:
        sorted list of float timestamps
    """
    # ── primary: GT csv with >= 2 obstacles per frame ──
    gt_rows = load_csv(gt_csv_path)
    if gt_rows:
        by_ts = defaultdict(list)
        for row in gt_rows:
            by_ts[float(row["timestamp"])].append(row)

        multi_ts = sorted(ts for ts, obs in by_ts.items() if len(obs) >= 2)
        if multi_ts:
            if len(multi_ts) <= num_keyframes:
                return multi_ts
            indices = np.linspace(0, len(multi_ts) - 1, num_keyframes, dtype=int)
            return [multi_ts[i] for i in indices]

    # ── fallback: bag start / end ──
    if os.path.exists(bag_path):
        bag = rosbag.Bag(bag_path, "r")
        t0 = bag.get_start_time()
        t1 = bag.get_end_time()
        bag.close()
        dur = t1 - t0
        offset = dur / (num_keyframes + 1)
        return [t0 + offset * (i + 1) for i in range(num_keyframes)]

    # ── last resort ──
    return [float(i) for i in range(num_keyframes)]


# ── Data loading ────────────────────────────────────────────────────────────

def load_method_data(method_path):
    """Load all data for one method directory.

    Returns a dict with keys ``bag_data``, ``tracker_frames``, ``metrics``,
    ``name``, and ``path``, or **None** if ``raw.bag`` is missing.
    """
    bag_path = os.path.join(method_path, "raw.bag")
    tracker_path = os.path.join(method_path, "tracker.csv")
    metrics_path = os.path.join(method_path, "metrics.csv")

    if not os.path.exists(bag_path):
        print(f"WARNING: raw.bag not found at {bag_path} -- skipping method")
        return None

    # Discover available topics
    bag_info = rosbag.Bag(bag_path, "r")
    available_topics = set(bag_info.get_type_and_topic_info().topics.keys())
    bag_info.close()

    bag_data = load_bag_data(bag_path, available_topics)
    tracker_frames = (
        load_tracker_csv(tracker_path) if os.path.exists(tracker_path) else {}
    )
    metrics = load_csv(metrics_path) if os.path.exists(metrics_path) else []

    return {
        "bag_data": bag_data,
        "tracker_frames": tracker_frames,
        "metrics": metrics,
        "name": os.path.basename(os.path.normpath(method_path)),
        "path": method_path,
    }


# ── Layout helpers ──────────────────────────────────────────────────────────

def compute_shared_limits(all_bag_data, margin=1.0):
    """Compute (xlim, ylim) tuples covering all GT obstacle positions."""
    min_x = min_y = float("inf")
    max_x = max_y = float("-inf")
    found = False

    for bag_data in all_bag_data:
        for obstacles in bag_data.get("gt", {}).values():
            for obs in obstacles:
                x = float(obs["x"])
                y = float(obs["y"])
                min_x = min(min_x, x)
                max_x = max(max_x, x)
                min_y = min(min_y, y)
                max_y = max(max_y, y)
                found = True

    if not found:
        return (-5.0, 5.0), (-5.0, 5.0)

    return (min_x - margin, max_x + margin), (min_y - margin, max_y + margin)


def build_metrics_subtitle(method_metrics):
    """Format a metrics summary line for the figure suptitle.

    Args:
        method_metrics: dict mapping method name -> list of CSV rows

    Returns:
        e.g. "DODT: MOTA=0.12, epos=0.234 | FAPP: MOTA=0.34, epos=0.156 | ..."
    """
    parts = []
    for name, rows in method_metrics.items():
        if rows:
            r = rows[0]
            mota = r.get("MOTA", "?")
            epos = r.get("epos", "?")
            try:
                mota_s = f"{float(mota):.2f}"
            except (ValueError, TypeError):
                mota_s = str(mota)
            try:
                epos_s = f"{float(epos):.3f}"
            except (ValueError, TypeError):
                epos_s = str(epos)
            parts.append(f"{name}: MOTA={mota_s}, epos={epos_s}")
        else:
            parts.append(f"{name}: no metrics")
    return " | ".join(parts) if parts else ""


# ── Rendering ───────────────────────────────────────────────────────────────

def render_subplot_frame(ax, ts, bag_data, tracker_frames, title=""):
    """Render a single BEV frame on the given Axes.

    Shows the robot pose, GT obstacles (red), and tracker outputs (blue).
    This is a lighter variant of visualize_keyframes.render_frame() intended
    for embedding inside a multi-panel figure.

    Args:
        ax: matplotlib Axes
        ts: query timestamp (float)
        bag_data: dict returned by load_bag_data (keys: "odom", "gt", ...)
        tracker_frames: dict returned by load_tracker_csv
        title: optional subplot title
    """
    odom = nearest_data(bag_data.get("odom", {}), ts)
    gt = nearest_data(bag_data.get("gt", {}), ts)
    trk_rows = tracker_at_ts(tracker_frames, ts) if tracker_frames else None

    # ── robot ──
    if odom:
        rx, ry, ryaw = odom
        robot_len = 0.3
        dx = robot_len * math.cos(ryaw)
        dy = robot_len * math.sin(ryaw)
        ax.plot(rx, ry, "o", color=ROBOT_COLOR, markersize=6, zorder=5)
        ax.arrow(rx, ry, dx, dy,
                 head_width=0.1, head_length=0.08,
                 fc=ROBOT_COLOR, ec=ROBOT_COLOR, zorder=5)
        robot_circle = plt.Circle(
            (rx, ry), 0.18, fill=False, color=ROBOT_COLOR,
            linewidth=1, linestyle="--", alpha=0.5,
        )
        ax.add_patch(robot_circle)

    # ── GT obstacles ──
    if gt:
        for obs in gt:
            r = float(obs.get("radius", 0.3) or 0.3)
            obs_x = float(obs["x"])
            obs_y = float(obs["y"])
            ax.plot(obs_x, obs_y, "o", color=GT_COLOR,
                    markersize=r * 30, alpha=0.3, zorder=1)
            ax.plot(obs_x, obs_y, "x", color=GT_COLOR, markersize=4, zorder=4)
            vx = float(obs.get("vx", 0) or 0)
            vy = float(obs.get("vy", 0) or 0)
            vmag = math.hypot(vx, vy)
            if vmag > 0.01:
                ax.arrow(obs_x, obs_y,
                         vx / vmag * 0.5, vy / vmag * 0.5,
                         head_width=0.08, head_length=0.06,
                         fc=GT_COLOR, ec=GT_COLOR, alpha=0.7, zorder=4)

    # ── tracker outputs ──
    if trk_rows:
        n_trackers = len(trk_rows)
        show_labels = n_trackers <= 8
        for row in trk_rows:
            tx = float(row["x"])
            ty = float(row["y"])
            vx = float(row.get("vx", 0) or 0)
            vy = float(row.get("vy", 0) or 0)
            tid = row.get("id", "?")

            ax.plot(tx, ty, "s", color=TRACKER_COLOR, markersize=5,
                    alpha=0.7, zorder=4)
            vmag = math.hypot(vx, vy)
            if vmag > 0.01:
                ax.arrow(tx, ty,
                         vx / vmag * 0.4, vy / vmag * 0.4,
                         head_width=0.06, head_length=0.05,
                         fc=TRACKER_COLOR, ec=TRACKER_COLOR,
                         alpha=0.6, zorder=4)
            if show_labels:
                ax.annotate(str(tid), (tx + 0.05, ty + 0.05),
                            fontsize=4, color=TRACKER_COLOR, alpha=0.8)

    # ── formatting ──
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    if title:
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Multi-panel BEV figure comparing tracker methods at keyframes",
    )
    p.add_argument(
        "--methods", nargs="+", required=True,
        help='Method specs, e.g. "DODT:/path/to/run_1" "Ours:/path/to/run_2"',
    )
    p.add_argument("--output-dir", required=True,
                   help="Directory for output PDF and PNG")
    p.add_argument("--num-keyframes", type=int, default=4,
                   help="Number of keyframe rows (default: 4)")
    p.add_argument("--title", default="Crossing Scene: Tracking Comparison",
                   help="Figure suptitle")
    p.add_argument("--margin", type=float, default=1.0,
                   help="Axis-limit margin in metres (default: 1.0)")
    args = p.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    # ── parse & load all methods ──
    method_specs = parse_method_specs(args.methods)
    print("Loading method data ...")
    method_data_list = []
    for display_name, path in method_specs:
        print(f"  {display_name}: {path}")
        data = load_method_data(path)
        if data is None:
            print(f"  --> skipped (no raw.bag)")
            continue
        data["display_name"] = display_name
        method_data_list.append(data)

    if not method_data_list:
        sys.exit("ERROR: No methods could be loaded; nothing to plot.")

    # ── shared keyframes (from first method) ──
    first = method_data_list[0]
    first_bag = os.path.join(first["path"], "raw.bag")
    first_gt = os.path.join(first["path"], "gt.csv")
    keyframe_ts = get_shared_keyframes(first_bag, first_gt, args.num_keyframes)

    # ── shared axis limits ──
    all_bag = [m["bag_data"] for m in method_data_list]
    shared_xlim, shared_ylim = compute_shared_limits(all_bag, args.margin)

    # ── metrics subtitle ──
    metrics_dict = {m["display_name"]: m["metrics"] for m in method_data_list}
    metrics_subtitle = build_metrics_subtitle(metrics_dict)

    # ── build figure ──
    N = len(keyframe_ts)      # rows
    M = len(method_data_list)  # cols
    fig, axes = plt.subplots(
        nrows=N, ncols=M,
        figsize=(4.5 * M, 4.0 * N),
        squeeze=False,
    )

    # Suptitle: scene title + metrics line
    suptitle = args.title
    if metrics_subtitle:
        suptitle += f"\n{metrics_subtitle}"
    fig.suptitle(suptitle, fontsize=10, fontweight="bold", y=0.99)

    # Render every subplot
    for row_idx, ts in enumerate(keyframe_ts):
        for col_idx, md in enumerate(method_data_list):
            ax = axes[row_idx, col_idx]
            bag_data = md["bag_data"]
            trk = md["tracker_frames"]

            # Check whether *any* data exists near this timestamp
            has_data = (
                nearest_data(bag_data.get("odom", {}), ts) is not None or
                nearest_data(bag_data.get("gt", {}), ts) is not None or
                tracker_at_ts(trk, ts) is not None
            )

            if not has_data:
                ax.text(0.5, 0.5, "No data", transform=ax.transAxes,
                        ha="center", va="center", fontsize=8, color="gray")
                ax.set_aspect("equal")
                ax.grid(alpha=0.3)
            else:
                subplot_title = (
                    f"{md['display_name']}    t = {ts:.2f}s"
                )
                render_subplot_frame(ax, ts, bag_data, trk,
                                     title=subplot_title)

            ax.set_xlim(shared_xlim)
            ax.set_ylim(shared_ylim)

    # Column headers on first row (method names)
    for col_idx, md in enumerate(method_data_list):
        axes[0, col_idx].set_title(
            md["display_name"], fontsize=10, fontweight="bold", pad=10,
        )

    # Row labels on first column (timestamp)
    for row_idx, ts in enumerate(keyframe_ts):
        axes[row_idx, 0].set_ylabel(
            f"t = {ts:.2f} s", fontsize=8, fontweight="bold", labelpad=8,
        )

    # Single shared legend below the figure
    legend_handles = [
        plt.Line2D([], [], marker="o", color=ROBOT_COLOR, linestyle="",
                   markersize=6, label="Robot"),
        plt.Line2D([], [], marker="x", color=GT_COLOR, linestyle="",
                   markersize=6, markeredgewidth=1.5, label="GT obstacles"),
        plt.Line2D([], [], marker="s", color=TRACKER_COLOR, linestyle="",
                   markersize=6, label="Tracker"),
    ]
    fig.legend(
        handles=legend_handles, loc="lower center", ncol=3,
        fontsize=7, framealpha=0.8, bbox_to_anchor=(0.5, 0.005),
    )

    fig.tight_layout(rect=[0, 0.055, 1, 0.96])

    # Save outputs
    for fmt in ("pdf", "png"):
        out_path = os.path.join(args.output_dir, f"compare_keyframes.{fmt}")
        fig.savefig(out_path)
        print(f"Saved: {out_path}")

    plt.close(fig)
    print("Done.")


if __name__ == "__main__":
    main()
