#!/usr/bin/env python3
"""
Plot closed-loop trajectory comparison for random_multi (P2.4).
Two-panel side-by-side BEV figure: ours_full vs fapp baseline.

Usage:
    python3 plot_closed_loop_trajectory.py
Output:
    figures/fig_closed_loop_trajectory.pdf
"""

import csv
import json
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Paths
DATA_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "data", "avoidance", "random_multi",
)
FIG_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "figures",
)
os.makedirs(FIG_DIR, exist_ok=True)

RUNS = {
    "ours_full": os.path.join(DATA_DIR, "ours_full", "run_3"),
    "fapp": os.path.join(DATA_DIR, "fapp", "run_1"),
}

LABELS = {
    "ours_full": "Ours (Full)",
    "fapp": "FAPP",
}

# Collision / near-collision thresholds
COLLISION_THRESH = 0.2      # meters (body edge to obstacle edge)
NEAR_THRESH = 0.5           # meters

# Plot style
OBSTACLE_CMAP = plt.cm.tab10
ROBOT_COLOR = "#2166ac"
GOAL_COLOR = "#4daf4a"
COLLISION_COLOR = "#e41a1c"
NEAR_COLLISION_COLOR = "#ff7f00"

GOAL_X = 0.0
GOAL_Y = 8.0


def load_robot_trajectory(run_dir):
    """Load robot odometry from clearance_timeseries.csv.

    Returns (timestamps, xs, ys, clearances).
    """
    ts, xs, ys, cls = [], [], [], []
    with open(os.path.join(run_dir, "clearance_timeseries.csv")) as f:
        for row in csv.DictReader(f):
            ts.append(float(row["timestamp"]))
            xs.append(float(row["robot_x"]))
            ys.append(float(row["robot_y"]))
            cls.append(float(row["clearance"]))
    return (np.array(ts), np.array(xs), np.array(ys), np.array(cls))


def load_obstacles(run_dir):
    """Load GT obstacles from gt.csv grouped by obstacle ID.

    Returns dict: {obstacle_id: {"timestamps": [...], "xs": [...], "ys": [...]}}
    """
    obstacles = {}
    with open(os.path.join(run_dir, "gt.csv")) as f:
        for row in csv.DictReader(f):
            oid = row["id"]
            if oid not in obstacles:
                obstacles[oid] = {"timestamps": [], "xs": [], "ys": []}
            obstacles[oid]["timestamps"].append(float(row["timestamp"]))
            obstacles[oid]["xs"].append(float(row["x"]))
            obstacles[oid]["ys"].append(float(row["y"]))
    # convert to arrays
    for oid in obstacles:
        for k in ("timestamps", "xs", "ys"):
            obstacles[oid][k] = np.array(obstacles[oid][k])
    return obstacles


def find_collision_segments(clearances, threshold, inclusive=True):
    """Find contiguous segments where clearance <= threshold (or < threshold).

    Returns list of (start_idx, end_idx) for contiguous segments.
    """
    if inclusive:
        mask = clearances <= threshold
    else:
        mask = clearances < threshold
    # Pad to catch boundary segments
    padded = np.concatenate([[False], mask, [False]])
    starts = np.where(padded[1:] & ~padded[:-1])[0]
    ends = np.where(~padded[1:] & padded[:-1])[0] - 1
    return list(zip(starts, ends))


def downsample_obstacle(ts, xs, ys, max_samples=200):
    """Downsample obstacle trajectory for cleaner plotting."""
    if len(ts) <= max_samples:
        return ts, xs, ys
    idx = np.linspace(0, len(ts) - 1, max_samples, dtype=int)
    return ts[idx], xs[idx], ys[idx]


def plot_one_panel(ax, run_dir, title):
    """Plot one method's trajectory on a given axis."""
    # Load data
    robot_ts, robot_xs, robot_ys, clearances = load_robot_trajectory(run_dir)
    obstacles = load_obstacles(run_dir)

    # Load metrics for goal info
    with open(os.path.join(run_dir, "planning_metrics.json")) as f:
        metrics = json.load(f)

    # Plot obstacle GT trajectories as dashed lines
    for i, (oid, obs) in enumerate(sorted(obstacles.items())):
        t, xs, ys = downsample_obstacle(obs["timestamps"], obs["xs"], obs["ys"])
        color = OBSTACLE_CMAP(i % 10)
        ax.plot(xs, ys, linestyle="--", linewidth=1.2, color=color, alpha=0.7,
                label=f"Obstacle {int(oid)+1} GT")

    # Identify collision and near-collision segments
    collision_segments = find_collision_segments(clearances, COLLISION_THRESH)
    near_segments = find_collision_segments(clearances, NEAR_THRESH)
    # Exclude collision segments from near segments
    collision_idxs = set()
    for s, e in collision_segments:
        collision_idxs.update(range(s, e + 1))

    # Mark near-collision points
    near_scatter_x, near_scatter_y = [], []
    for s, e in near_segments:
        for j in range(s, e + 1):
            if j not in collision_idxs and clearances[j] <= NEAR_THRESH:
                near_scatter_x.append(robot_xs[j])
                near_scatter_y.append(robot_ys[j])

    # Mark collision points
    collision_scatter_x, collision_scatter_y = [], []
    for s, e in collision_segments:
        for j in range(s, e + 1):
            collision_scatter_x.append(robot_xs[j])
            collision_scatter_y.append(robot_ys[j])

    # Plot robot path
    ax.plot(robot_xs, robot_ys, color=ROBOT_COLOR, linewidth=1.8,
            label="Robot path", zorder=3)

    # Scatter collision and near-collision on top
    if near_scatter_x:
        ax.scatter(near_scatter_x, near_scatter_y, c=NEAR_COLLISION_COLOR,
                   s=4, alpha=0.6, label=f"Near-collision (<{NEAR_THRESH}m)", zorder=4)
    if collision_scatter_x:
        ax.scatter(collision_scatter_x, collision_scatter_y, c=COLLISION_COLOR,
                   s=12, alpha=0.9, label=f"Collision (<{COLLISION_THRESH}m)", zorder=5)

    # Goal marker
    ax.scatter([GOAL_X], [GOAL_Y], marker="*", c=GOAL_COLOR, s=200, edgecolors="black",
               linewidths=0.5, label="Goal", zorder=6)

    # Start marker
    ax.scatter([0], [0], marker="o", c="white", edgecolors=ROBOT_COLOR,
               s=80, linewidths=1.5, label="Start", zorder=6)

    # Add summary text
    if metrics["goal_success"]:
        status = f"Goal reached ({metrics['time_to_goal_s']:.1f}s)"
    else:
        status = f"Goal not reached\n(final dist: {metrics['final_goal_distance_m']:.2f}m)"

    collisions = metrics.get("collision_events", 0)
    near = metrics.get("near_collision_events", 0)

    summary = (
        f"Collisions: {collisions} | Near-collisions: {near}\n"
        f"Min clearance: {metrics['min_clearance_m']:.2f}m\n"
        f"{status}"
    )
    ax.text(0.03, 0.97, summary, transform=ax.transAxes, fontsize=8,
            verticalalignment="top", fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.85, edgecolor="gray"))

    # Styling
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title, fontweight="bold", fontsize=11)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3, linestyle=":")
    ax.legend(fontsize=7, loc="lower right", framealpha=0.9)


def main():
    matplotlib.rcParams.update({
        "font.size": 9,
        "axes.labelsize": 10,
        "legend.fontsize": 7,
        "figure.dpi": 150,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.05,
    })

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    for ax, (method, run_dir) in zip((ax1, ax2), RUNS.items()):
        plot_one_panel(ax, run_dir, LABELS[method])

    fig.suptitle("Closed-Loop Trajectory Comparison: random_multi", fontsize=13, fontweight="bold", y=1.01)

    out_path = os.path.join(FIG_DIR, "fig_closed_loop_trajectory.pdf")
    fig.savefig(out_path, dpi=150, bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
