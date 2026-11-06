#!/usr/bin/env python3
"""
plot_cmd_vel_traj_combined.py -- Single composite figure for the closed-loop
random_multi trial: the three command/clearance time-series on the LEFT and the
bird's-eye robot chassis trajectory on the RIGHT.

The right BEV panel exists to tell the reader *what scenario* the left command
traces were collected in (robot driving to a goal through 3 crossing obstacles),
so the two halves are read together.

Left  (a) v      -- linear velocity (m/s)
      (b) omega  -- angular velocity (rad/s)
      (c) d_min  -- closest-obstacle clearance (m) + near-collision/collision refs
Right (d)        -- BEV chassis trajectories with obstacle motion trails

Colors are the colorblind-safe Okabe-Ito trio (blue / vermillion / bluish-green,
validated CVD dE ~37); identity is reinforced by linestyle (solid/dotted/dashed)
so the figure survives grayscale and red-green color vision deficiency.

Per-method run reflects fixed seed=42 (identical obstacle motion across methods):
  ours_full run_2 (collision-free) | dodt run_1, fapp run_3 (collide)

Usage:
  python3 plot_cmd_vel_traj_combined.py \
      --data-dir /data/rosbags/avoidance/random_multi \
      --output /home/patrick/sim_dev/paper/reframe_draft/images/fig_cmd_vel_traj_combined.pdf
"""

import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Patch
from matplotlib.lines import Line2D
from matplotlib.gridspec import GridSpec
import numpy as np

# key -> (label, color, linestyle, linewidth, cmd_run, traj_run, collides)
# Colorblind-safe Okabe-Ito trio; linestyle is a redundant (CVD-safe) encoding.
METHODS = {
    "ours_full": ("SCOT",        "#0072B2", "-",  2.6, "run_2", "run_2", False),
    "dodt":      ("DODT",        "#D55E00", ":",  2.4, "run_1", "run_1", True),
    "fapp":      ("FAPP",        "#009E73", "--", 2.2, "run_3", "run_3", True),
}

# Reference thresholds on the clearance panel -- neutral (amber/black) so they
# are never confused with the vermillion DODT series.
NEAR_COLLISION_THRESHOLD = 0.5   # metres (amber)
COLLISION_THRESHOLD = 0.2        # metres (black)
NEAR_COLLISION_COLOR = "#E69F00"
COLLISION_COLOR = "#000000"

XAXIS_MAX = 25.0                 # seconds
SMOOTH_WINDOW = 10               # samples (~0.5 s at 20 Hz)
TRUNCATE_AFTER_COLLISION = {"fapp"}  # colliders that stall (flatline uninformative)

GOAL = (0.0, 8.0)
START = (0.0, 0.0)

# BEV obstacle motion-history trails: the obstacle each method interacts with.
# Ours proactively dodges gt_id=2; DODT/FAPP collide with gt_id=0.
HISTORY = {
    "ours_full": {"gt_id": "2", "times": [4.5, 6.0, 7.5, 9.0, 10.5]},
    "dodt":      {"gt_id": "0", "times": [8.9, 10.2, 11.5, 12.8, 13.9]},
    "fapp":      {"gt_id": "0", "times": [8.97, 9.3, 9.7, 10.0, 10.3]},
}


# --------------------------------------------------------------------------- #
# Data loading
# --------------------------------------------------------------------------- #
def load_cmd(path):
    """cmd_vel.csv -> (t0, t, v, w)."""
    t, v, w = [], [], []
    with open(path) as f:
        t0 = None
        for row in csv.DictReader(f):
            ts = float(row["timestamp"])
            t0 = ts if t0 is None else t0
            t.append(ts - t0)
            v.append(float(row["v"]))
            w.append(float(row["w"]))
    return t0, np.array(t), np.array(v), np.array(w)


def load_clearance_series(path, t0):
    """clearance_timeseries.csv -> (t - t0, clearance)."""
    t, c = [], []
    for row in csv.DictReader(open(path)):
        t.append(float(row["timestamp"]) - t0)
        c.append(float(row["clearance"]))
    return np.array(t), np.array(c)


def load_traj(path):
    """clearance_timeseries.csv -> (robot xy list, min-clearance row, obs vel)."""
    rows = list(csv.DictReader(open(path)))
    xy = [(float(r["robot_x"]), float(r["robot_y"])) for r in rows]
    crit = min(rows, key=lambda r: float(r["clearance"]))
    return xy, crit, obstacle_velocity(rows, crit)


def load_obstacle_history(path, gt_id, times):
    """[(t, ox, oy, r), ...] for gt_id sampled at the requested timestamps."""
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


def obstacle_velocity(rows, crit):
    """Finite-difference (vx, vy) of the critical obstacle over +/-0.5 s."""
    same = [r for r in rows if r["gt_id"] == crit["gt_id"]]
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
    return (0.0, 0.0) if dt <= 0 else ((xw[-1] - xw[0]) / dt, (yw[-1] - yw[0]) / dt)


def near_collision_windows(t, c):
    """Contiguous (start, end) spans where clearance < NEAR_COLLISION_THRESHOLD."""
    below = c < NEAR_COLLISION_THRESHOLD
    windows, in_win, start = [], False, None
    for i, flag in enumerate(below):
        if flag and not in_win:
            in_win, start = True, t[max(0, i - 1)]
        elif not flag and in_win:
            in_win = False
            windows.append((start, t[i]))
    if in_win:
        windows.append((start, t[-1]))
    return windows


def collision_instant(t, c):
    """(t, clearance) at deepest penetration if it dips below COLLISION_THRESHOLD."""
    i = int(np.argmin(c))
    return (float(t[i]), float(c[i])) if c[i] < COLLISION_THRESHOLD else None


def moving_average(data, window):
    if len(data) < window or window < 2:
        return data
    return np.convolve(data, np.ones(window) / window, mode="same")


# --------------------------------------------------------------------------- #
# Left column: command / clearance time-series
# --------------------------------------------------------------------------- #
def draw_cmd_column(axes, data_dir, smooth=True, band=True):
    ax_v, ax_w, ax_d = axes
    avoid_spans, collisions = [], []

    for key, (label, color, ls, lw, cmd_run, _, _) in METHODS.items():
        cv_path = os.path.join(data_dir, key, cmd_run, "cmd_vel.csv")
        cl_path = os.path.join(data_dir, key, cmd_run, "clearance_timeseries.csv")
        if not os.path.exists(cv_path):
            print(f"WARNING: {cv_path} not found, skipping {label}")
            continue

        t0, t, v, w = load_cmd(cv_path)
        if smooth:
            v, w = moving_average(v, SMOOTH_WINDOW), moving_average(w, SMOOTH_WINDOW)

        ct = cc = cc_plot = hit = None
        if os.path.exists(cl_path):
            ct, cc = load_clearance_series(cl_path, t0)
            cc_plot = cc if not smooth else moving_average(cc, SMOOTH_WINDOW)
            hit = collision_instant(ct, cc)

        cutoff = hit[0] if (hit and key in TRUNCATE_AFTER_COLLISION) else np.inf
        if ct is not None:
            avoid_spans.append((label, color, [
                (s, min(e, cutoff)) for s, e in near_collision_windows(ct, cc)
                if s < cutoff]))

        vm = t <= cutoff
        ax_v.plot(t[vm], v[vm], color=color, ls=ls, lw=lw, label=label)
        ax_w.plot(t[vm], w[vm], color=color, ls=ls, lw=lw)
        if ct is not None:
            cm = ct <= cutoff
            ax_d.plot(ct[cm], cc_plot[cm], color=color, ls=ls, lw=lw)
            if hit is not None:
                ci = int(np.argmin(np.abs(t - hit[0])))
                collisions.append((label, color, hit[0], hit[1],
                                   float(v[ci]), float(w[ci])))

    if band:
        for _, color, intervals in avoid_spans:
            for ws, we in intervals:
                we = min(we, XAXIS_MAX)
                if ws >= XAXIS_MAX or we <= 0:
                    continue
                for ax in axes:
                    ax.axvspan(max(0, ws), we, alpha=0.10, facecolor=color, zorder=0)

    collisions.sort(key=lambda r: r[2])
    for i, (label, color, tc, cmin, v_tc, w_tc) in enumerate(collisions):
        if tc > XAXIS_MAX:
            continue
        for ax, yval in ((ax_v, v_tc), (ax_w, w_tc), (ax_d, cmin)):
            ax.plot(tc, yval, marker="X", markersize=9, color=color,
                    markeredgecolor="black", markeredgewidth=0.8, zorder=6)
        ax_d.annotate(f"{label} collision", xy=(tc, cmin),
                      xytext=(tc + 0.6, -0.28 if i == 0 else -0.05),
                      fontsize=8.5, color=color, fontweight="bold", va="center",
                      arrowprops=dict(arrowstyle="->", color=color, lw=1.0))

    # Reference thresholds (solid so they reach the right spine, neutral colors
    # so they are never confused with the vermillion DODT series).
    ax_d.axhline(NEAR_COLLISION_THRESHOLD, color=NEAR_COLLISION_COLOR,
                 ls="-", lw=1.1, alpha=0.9, zorder=1)
    ax_d.axhline(COLLISION_THRESHOLD, color=COLLISION_COLOR, ls="-", lw=1.1,
                 alpha=0.9, zorder=1)
    ax_d.text(0.4, NEAR_COLLISION_THRESHOLD + 0.06, "near-collision",
              fontsize=8, color=NEAR_COLLISION_COLOR)
    ax_d.text(0.4, COLLISION_THRESHOLD - 0.28, "collision",
              fontsize=8, color=COLLISION_COLOR)

    ax_v.set_ylabel("v (m/s)")
    ax_w.set_ylabel(r"$\omega$ (rad/s)")
    ax_d.set_ylabel(r"$d_{min}$ (m)")
    ax_d.set_xlabel("Time (s)")
    for tag, ax in zip(("(a)", "(b)", "(c)"), axes):
        ax.text(0.02, 0.86, tag, transform=ax.transAxes,
                fontsize=12, fontweight="bold")
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, XAXIS_MAX)
    ax_d.set_ylim(bottom=-0.4)


# --------------------------------------------------------------------------- #
# Right column: BEV trajectory (the scenario the commands were recorded in)
# --------------------------------------------------------------------------- #
def draw_traj_panel(ax, data_dir):
    for key, (label, color, ls, _, _, traj_run, collides) in METHODS.items():
        path = os.path.join(data_dir, key, traj_run, "clearance_timeseries.csv")
        if not os.path.exists(path):
            print(f"WARNING: {path} not found, skipping {label}")
            continue
        xy, crit, vel = load_traj(path)
        xs, ys = [p[0] for p in xy], [p[1] for p in xy]
        # Trajectory shares the method's color + linestyle with the left panels.
        ax.plot(xs, ys, color=color, ls=ls, lw=2.0, alpha=0.95, zorder=3)
        ax.scatter(xs[-1], ys[-1], marker="s", s=30, color=color,
                   edgecolor="white", linewidth=0.5, zorder=4)

        if key in HISTORY:
            spec = HISTORY[key]
            hist = load_obstacle_history(path, spec["gt_id"], spec["times"])
            n = len(hist)
            hx, hy = [h[1] for h in hist], [h[2] for h in hist]
            ax.plot(hx, hy, ls="--", color=color, lw=0.8, alpha=0.6, zorder=1)
            for i, (_, ox, oy, r) in enumerate(hist):
                a = 0.18 + 0.45 * (i / max(1, n - 1))  # fade toward present
                ax.add_patch(Circle((ox, oy), r, fill=False, ls="--",
                                    edgecolor=color, lw=0.9, alpha=a, zorder=2))
            ax.annotate("", xy=(hx[-1], hy[-1]), xytext=(hx[-2], hy[-2]),
                        arrowprops=dict(arrowstyle="-|>", color=color, lw=1.1,
                                        shrinkA=0, shrinkB=0, alpha=0.7), zorder=3)

        if collides:
            ox, oy, r = float(crit["obs_x"]), float(crit["obs_y"]), float(crit["obs_radius"])
            ax.add_patch(Circle((ox, oy), r, facecolor="#8a8a8a", alpha=0.28,
                                edgecolor="#666666", linewidth=0.6, zorder=2))
            ax.scatter(ox, oy, marker="P", s=34, color="#666666", zorder=3)
            ax.scatter(float(crit["robot_x"]), float(crit["robot_y"]), marker="X",
                       s=80, color=color, edgecolor="white", linewidth=0.8, zorder=6)
            vx, vy = vel
            speed = (vx * vx + vy * vy) ** 0.5
            if speed > 1e-3:
                ux, uy = vx / speed, vy / speed
                ax.annotate("", xy=(ox + 0.8 * ux, oy + 0.8 * uy), xytext=(ox, oy),
                            arrowprops=dict(arrowstyle="-|>", color="#333333",
                                            lw=1.2, shrinkA=0, shrinkB=0), zorder=7)

    # Minimal, semi-transparent start/goal markers.
    ax.scatter(*GOAL, marker="*", s=190, color="#3a3a3a", alpha=0.45,
               edgecolor="none", zorder=5)
    ax.annotate("Goal", GOAL, textcoords="offset points", xytext=(10, 5),
                ha="left", va="bottom", fontsize=9, color="#3a3a3a")
    ax.scatter(*START, marker="^", s=50, color="#3a3a3a", alpha=0.45,
               edgecolor="none", zorder=5)
    ax.annotate("Start", START, textcoords="offset points", xytext=(8, -2),
                fontsize=9, color="#3a3a3a")

    ax.text(0.05, 0.03, "(d)", transform=ax.transAxes,
            fontsize=12, fontweight="bold", va="bottom", ha="left")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim(-1, 4)
    ax.set_aspect("equal", adjustable="box")
    ax.set_anchor("NE")  # top-right: top aligns with (a), right edge at margin
    ax.grid(True, alpha=0.3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", default="/data/rosbags/avoidance/random_multi")
    parser.add_argument("--output",
                        default="/home/patrick/sim_dev/paper/reframe_draft/"
                                "images/fig_cmd_vel_traj_combined.pdf")
    parser.add_argument("--no-smooth", action="store_true")
    parser.add_argument("--no-band", action="store_true")
    args = parser.parse_args()

    # Single-column layout: wide command panels (left); the BEV (right) spans the
    # (a)+(b) rows and a dedicated legend cell aligns with (c) -> clean grid,
    # right and bottom panel borders line up.
    plt.rcParams.update({
        "font.size": 10,
        "axes.labelsize": 11,
        "axes.titlesize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 8,
    })
    fig = plt.figure(figsize=(7.4, 6.8))
    # 6-row grid: each command panel spans 2 rows; the BEV spans 5 rows (taller)
    # and the legend occupies the bottom row so its lower edge lines up with the
    # bottom of panel (c) / the command stack.
    gs = GridSpec(6, 2, figure=fig, width_ratios=[1.55, 1.0],
                  hspace=0.55, wspace=0.30)
    ax_v = fig.add_subplot(gs[0:2, 0])
    ax_w = fig.add_subplot(gs[2:4, 0], sharex=ax_v)
    ax_d = fig.add_subplot(gs[4:6, 0], sharex=ax_v)
    ax_t = fig.add_subplot(gs[0:5, 1])
    plt.setp(ax_v.get_xticklabels(), visible=False)
    plt.setp(ax_w.get_xticklabels(), visible=False)

    draw_cmd_column((ax_v, ax_w, ax_d), args.data_dir,
                    smooth=not args.no_smooth, band=not args.no_band)
    draw_traj_panel(ax_t, args.data_dir)

    handles = [Line2D([], [], color=c, ls=ls, lw=2.2, label=lbl)
               for lbl, c, ls, lw, *_ in METHODS.values()]
    handles.append(Line2D([], [], color="0.45", ls="--", lw=1.0, marker="o",
                          markerfacecolor="none", markeredgecolor="0.45",
                          markersize=7, label="moving obstacles"))
    handles.append(Patch(facecolor="0.5", alpha=0.18,
                         label=r"avoid ($d_{min}\!<\!0.5$ m)"))
    handles.append(Line2D([], [], marker="X", color="0.4", ls="None",
                          markersize=8, markeredgecolor="black",
                          label="collision point"))

    fig.tight_layout(rect=[0, 0, 1, 0.99])

    # Match the legend frame to the BEV: same width, left edge flush with the
    # BEV's left edge and lower edge flush with panel (c)'s bottom spine. Draw
    # once so the aspect-adjusted (visual) BEV box is available.
    fig.canvas.draw()
    pos_t = ax_t.get_position(original=False)
    pos_d = ax_d.get_position(original=False)
    fig.legend(handles=handles, loc="lower left",
               bbox_to_anchor=(pos_t.x0, pos_d.y0, pos_t.width, 0.2),
               mode="expand", ncol=1, fontsize=9.5, frameon=True,
               handlelength=1.9, handletextpad=0.6, labelspacing=0.5,
               borderpad=0.7, borderaxespad=0.0)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    fig.savefig(args.output, dpi=200, bbox_inches="tight")
    png_path = os.path.splitext(args.output)[0] + ".png"
    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    print(f"Figure saved to {args.output} and {png_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
