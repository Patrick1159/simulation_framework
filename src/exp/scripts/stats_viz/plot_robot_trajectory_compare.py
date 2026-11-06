#!/usr/bin/env python3
"""
plot_robot_trajectory_compare.py -- BEV robot chassis trajectories for
random_multi, contrasting Ours (collision-free, smooth) against DODT/FAPP
baselines that collide.

Reads robot_x/robot_y (chassis path) and the paired closest-obstacle position
from each run's clearance_timeseries.csv. A single obstacle icon (0.3 m disc) is
drawn at the near-collision region of each baseline, with an X at the collision
point. Full obstacle trajectories are intentionally omitted.

Per-run choice reflects fixed seed=42 (identical obstacle motion):
  ours_full -> run_2 (clean, reaches goal)
  dodt      -> run_1 (penetrates, min_clr=-0.14 m)
  fapp      -> run_3 (penetrates, min_clr=-0.09 m)
"""

import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.lines import Line2D
import numpy as np

DATA_ROOT = "/data/rosbags/avoidance/random_multi"
OUTPUT = ("/home/patrick/sim_dev/simulation_framework/figures/"
          "fig_robot_trajectory_compare.pdf")

# method_key -> (run, label, color, collides)
SERIES = [
    ("ours_full", "run_2", "Ours (Full)", "#1b9e77", False),
    ("dodt",      "run_1", "DODT",        "#e7298a", True),
    ("fapp",      "run_3", "FAPP",        "#66a61e", True),
]

GOAL = (0.0, 8.0)
START = (0.0, 0.0)

# Obstacle motion-history overlay: fade-in dashed discs tracing the obstacle the
# robot interacts with, keyed by method. For Ours this is the proactively dodged
# obstacle (gt_id=2); for DODT/FAPP it is the obstacle they collide with
# (gt_id=0), so the collided obstacle is marked the same way as the left one.
# times are rosbag-relative seconds; collision histories end just before impact.
HISTORY = {
    "ours_full": {"gt_id": "2", "times": [4.5, 6.0, 7.5, 9.0, 10.5], "arrow": True},
    "dodt":      {"gt_id": "0", "times": [8.9, 10.2, 11.5, 12.8, 13.9], "arrow": False},
    "fapp":      {"gt_id": "0", "times": [8.97, 9.3, 9.7, 10.0, 10.3], "arrow": False},
}


def load_obstacle_history(path, gt_id, times):
    """Return [(t, ox, oy, r), ...] for gt_id at the requested timestamps."""
    rows = [r for r in csv.DictReader(open(path)) if r["gt_id"] == gt_id]
    if not rows:
        return []
    t = np.array([float(r["timestamp"]) for r in rows])
    out = []
    for tgt in times:
        r = rows[int(np.argmin(np.abs(t - tgt)))]
        out.append((float(r["timestamp"]), float(r["obs_x"]),
                    float(r["obs_y"]), float(r["obs_radius"])))
    return out


def load_clearance(path):
    """Return (robot_xy list, min-clearance row dict, obstacle (vx, vy))."""
    rows = list(csv.DictReader(open(path)))
    xy = [(float(r["robot_x"]), float(r["robot_y"])) for r in rows]
    crit = min(rows, key=lambda r: float(r["clearance"]))
    vel = obstacle_velocity(rows, crit)
    return xy, crit, vel


def obstacle_velocity(rows, crit):
    """Finite-difference velocity (vx, vy) of the critical obstacle (same gt_id),
    averaged over a +/-0.5 s window around the min-clearance frame."""
    gid = crit["gt_id"]
    same = [r for r in rows if r["gt_id"] == gid]
    if len(same) < 2:
        return (0.0, 0.0)
    t = np.array([float(r["timestamp"]) for r in same])
    ox = np.array([float(r["obs_x"]) for r in same])
    oy = np.array([float(r["obs_y"]) for r in same])
    tc = float(crit["timestamp"])
    win = (t >= tc - 0.5) & (t <= tc + 0.5)
    if win.sum() < 2:
        win = np.ones_like(t, dtype=bool)
    tw, xw, yw = t[win], ox[win], oy[win]
    dt = tw[-1] - tw[0]
    if dt <= 0:
        return (0.0, 0.0)
    return ((xw[-1] - xw[0]) / dt, (yw[-1] - yw[0]) / dt)


def main():
    fig, ax = plt.subplots(figsize=(5.5, 8))

    for key, run, label, color, collides in SERIES:
        path = os.path.join(DATA_ROOT, key, run, "clearance_timeseries.csv")
        xy, crit, vel = load_clearance(path)
        xs = [p[0] for p in xy]
        ys = [p[1] for p in xy]
        ax.plot(xs, ys, color=color, linewidth=2.2, alpha=0.9,
                label=label, zorder=3)
        ax.scatter(xs[-1], ys[-1], marker="s", s=55, color=color,
                   edgecolor="black", linewidth=0.6, zorder=4)

        # Obstacle motion-history overlay: dashed discs (fading from past to
        # present) tracing the obstacle this method interacts with. For Ours it
        # is the proactively dodged obstacle; for DODT/FAPP it is the obstacle
        # they collide with, so collided obstacles get the same trail treatment.
        if key in HISTORY:
            spec = HISTORY[key]
            hist = load_obstacle_history(path, spec["gt_id"], spec["times"])
            n = len(hist)
            hx = [h[1] for h in hist]
            hy = [h[2] for h in hist]
            ax.plot(hx, hy, linestyle="--", color=color, linewidth=1.4,
                    alpha=0.8, zorder=1)
            for i, (ht, ox, oy, r) in enumerate(hist):
                a = 0.22 + 0.55 * (i / max(1, n - 1))  # fade in toward present
                ax.add_patch(Circle((ox, oy), r, fill=False, linestyle="--",
                                    edgecolor=color, linewidth=1.4,
                                    alpha=a, zorder=2))
            # arrow on the last segment shows the obstacle's direction of travel
            ax.annotate("", xy=(hx[-1], hy[-1]), xytext=(hx[-2], hy[-2]),
                        arrowprops=dict(arrowstyle="-|>", color=color,
                                        lw=1.6, shrinkA=0, shrinkB=0),
                        zorder=3)

        if collides:
            ox, oy = float(crit["obs_x"]), float(crit["obs_y"])
            r = float(crit["obs_radius"])
            ax.add_patch(Circle((ox, oy), r, facecolor="#888888",
                                alpha=0.45, edgecolor="black",
                                linewidth=1.0, zorder=2))
            ax.scatter(ox, oy, marker="P", s=70, color="#444444", zorder=3)
            # collision X at robot position of min clearance
            ax.scatter(float(crit["robot_x"]), float(crit["robot_y"]),
                       marker="X", s=160, color=color,
                       edgecolor="black", linewidth=1.2, zorder=6)
            # obstacle velocity-direction arrow (fixed 0.9 m length) to show the
            # robot must predict where the obstacle is heading.
            vx, vy = vel
            speed = (vx * vx + vy * vy) ** 0.5
            if speed > 1e-3:
                ux, uy = vx / speed, vy / speed
                ax.annotate(
                    "", xy=(ox + 0.9 * ux, oy + 0.9 * uy), xytext=(ox, oy),
                    arrowprops=dict(arrowstyle="-|>", color="#222222",
                                    lw=2.0, shrinkA=0, shrinkB=0),
                    zorder=7)

    ax.scatter(*GOAL, marker="*", s=320, color="gold",
               edgecolor="black", linewidth=0.8, zorder=5)
    ax.annotate("Goal", GOAL, textcoords="offset points", xytext=(-10, 0),
                ha="right", fontsize=10, fontweight="bold")
    ax.scatter(*START, marker="^", s=90, color="black", zorder=5)
    ax.annotate("Start", START, textcoords="offset points", xytext=(10, -4),
                fontsize=10, fontweight="bold")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim(-1, 4)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_title("Robot Chassis Trajectory — random_multi\n"
                 "Ours avoids collision; DODT / FAPP penetrate an obstacle",
                 fontsize=12, fontweight="bold")

    handles = [
        Line2D([], [], color=c, linewidth=2.2, label=l)
        for _, _, l, c, _ in SERIES
    ]
    handles += [
        Line2D([], [], marker="X", color="gray", linestyle="None",
               markersize=10, markeredgecolor="black", label="Collision point"),
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=8.5,
              framealpha=0.9)

    os.makedirs(os.path.dirname(OUTPUT), exist_ok=True)
    fig.tight_layout()
    fig.savefig(OUTPUT, bbox_inches="tight")
    png_path = os.path.splitext(OUTPUT)[0] + ".png"
    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    print(f"Figure saved to {OUTPUT} and {png_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
