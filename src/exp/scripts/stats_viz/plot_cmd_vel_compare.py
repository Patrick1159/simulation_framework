#!/usr/bin/env python3
"""
plot_cmd_vel_compare.py -- Compare differential-drive command output and safety
clearance across perception methods feeding the same T-MPC planner.

Three stacked panels (random_multi, 3 dynamic obstacles):
  (a) v      -- linear velocity (m/s)
  (b) omega  -- angular velocity (rad/s)
  (c) d_min  -- closest-obstacle clearance (m), with near-collision (0.5 m) and
                collision (0.2 m) reference lines.

A shaded band marks the primary obstacle-interaction episode; vertical dashed
lines mark the collision instants of the colliding baselines (DODT / FAPP).

Per-method run is fixed (seed=42 => identical obstacle motion across runs):
  ours_full run_2, ours_no_clip run_1, ours_no_sam run_2  -- collision-free
  dodt run_1, fapp run_3                                   -- collide

Usage:
  python3 plot_cmd_vel_compare.py --data-dir /data/rosbags/avoidance/random_multi \
      --output figures/fig_planner_command_comparison.pdf
"""

import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# key -> (label, color, linestyle, linewidth, run)
# Same three methods/colors as fig_robot_trajectory_compare.pdf for consistency.
METHODS = {
    "ours_full":    ("Ours (Full)", "#1b9e77", "-",  2.6, "run_2"),
    "dodt":         ("DODT",        "#e7298a", ":",  2.2, "run_1"),
    "fapp":         ("FAPP",        "#66a61e", "--", 2.0, "run_3"),
}
RUN_OVERRIDE = {}
# Methods that stall on collision (never reach goal): truncate their curves at
# the collision instant, since the post-collision flatline is not informative.
TRUNCATE_AFTER_COLLISION = {"fapp"}

XAXIS_MAX = 25.0                 # seconds
NEAR_COLLISION_THRESHOLD = 0.5   # metres
COLLISION_THRESHOLD = 0.2        # metres (matches calc_planning_metrics default)
SMOOTH_WINDOW = 10               # samples (~0.5 s at 20 Hz)


def load_csv(path):
    """Load cmd_vel.csv -> (t0, timestamps, v, w, v_left, v_right)."""
    timestamps, v, w, v_left, v_right = [], [], [], [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        t0 = None
        for row in reader:
            ts = float(row["timestamp"])
            if t0 is None:
                t0 = ts
            timestamps.append(ts - t0)
            v.append(float(row["v"]))
            w.append(float(row["w"]))
            v_left.append(float(row["v_left"]))
            v_right.append(float(row["v_right"]))
    return (t0, np.array(timestamps), np.array(v), np.array(w),
            np.array(v_left), np.array(v_right))


def load_clearance_series(path, t0):
    """Load clearance_timeseries.csv -> (timestamps-t0, clearance)."""
    t, c = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["timestamp"]) - t0)
            c.append(float(row["clearance"]))
    return np.array(t), np.array(c)


def near_collision_windows(t, c):
    """Contiguous (start, end) windows where clearance < NEAR_COLLISION_THRESHOLD."""
    below = c < NEAR_COLLISION_THRESHOLD
    windows = []
    in_win = False
    start = None
    for i, flag in enumerate(below):
        if flag and not in_win:
            in_win = True
            start = t[max(0, i - 1)]
        elif not flag and in_win:
            in_win = False
            windows.append((start, t[i]))
    if in_win:
        windows.append((start, t[-1]))
    return windows


def collision_instant(t, c):
    """(t_collision, min_clearance) if clearance ever < COLLISION_THRESHOLD, else None."""
    i = int(np.argmin(c))
    if c[i] < COLLISION_THRESHOLD:
        return (float(t[i]), float(c[i]))
    return None


def moving_average(data, window):
    if len(data) < window or window < 2:
        return data
    kernel = np.ones(window) / window
    return np.convolve(data, kernel, mode="same")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", default=".",
                        help="Dir with per-method run_n/ subdirs "
                             "(cmd_vel.csv + clearance_timeseries.csv)")
    parser.add_argument("--output",
                        default="figures/fig_planner_command_comparison.pdf")
    parser.add_argument("--no-smooth", action="store_true")
    parser.add_argument("--no-band", action="store_true",
                        help="Disable interaction-band shading")
    args = parser.parse_args()

    fig, (ax_v, ax_w, ax_d) = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

    avoid_spans = []  # (label, color, [(start, end), ...]) per method
    collisions = []  # (label, color, t_collision, clearance, v_at_tc, w_at_tc)

    for key, (label, color, ls, lw, run) in METHODS.items():
        run = RUN_OVERRIDE.get(key, run)
        cv_path = os.path.join(args.data_dir, key, run, "cmd_vel.csv")
        cl_path = os.path.join(args.data_dir, key, run,
                               "clearance_timeseries.csv")
        if not os.path.exists(cv_path):
            print(f"WARNING: {cv_path} not found, skipping {label}")
            continue

        t0, t, v, w, v_left, v_right = load_csv(cv_path)
        if not args.no_smooth:
            v = moving_average(v, SMOOTH_WINDOW)
            w = moving_average(w, SMOOTH_WINDOW)

        ct = cc = cc_plot = None
        hit = None
        if os.path.exists(cl_path):
            ct, cc = load_clearance_series(cl_path, t0)
            cc_plot = cc if args.no_smooth else moving_average(cc, SMOOTH_WINDOW)
            hit = collision_instant(ct, cc)

        # Truncate stalling colliders at the collision instant.
        cutoff = hit[0] if (hit is not None and key in TRUNCATE_AFTER_COLLISION) \
            else np.inf
        if ct is not None:
            spans = [(s, min(e, cutoff)) for s, e in near_collision_windows(ct, cc)
                     if s < cutoff]
            avoid_spans.append((label, color, spans))
        vm = t <= cutoff
        ax_v.plot(t[vm], v[vm], color=color, linestyle=ls, linewidth=lw,
                  label=label)
        ax_w.plot(t[vm], w[vm], color=color, linestyle=ls, linewidth=lw)
        if ct is not None:
            cm = ct <= cutoff
            ax_d.plot(ct[cm], cc_plot[cm], color=color, linestyle=ls,
                      linewidth=lw)
            if hit is not None:
                ci = int(np.argmin(np.abs(t - hit[0])))
                collisions.append((label, color, hit[0], hit[1],
                                   float(v[ci]), float(w[ci])))

    axes = (ax_v, ax_w, ax_d)

    # Per-method avoidance highlighting: shade (in the method's colour) the
    # spans where its clearance < 0.5 m, on all three panels.
    if not args.no_band:
        for label, color, intervals in avoid_spans:
            for ws, we in intervals:
                we = min(we, XAXIS_MAX)
                if ws >= XAXIS_MAX or we <= 0:
                    continue
                for ax in axes:
                    ax.axvspan(max(0, ws), we, alpha=0.10, facecolor=color,
                               zorder=0)

    # Collision instants: marked only on the d_min panel (panel c), as a point
    # on the clearance curve at the moment of deepest penetration. No dashed
    # verticals across panels -- keeps (a)/(b) uncluttered.
    collisions.sort(key=lambda r: r[2])
    for i, (label, color, tc, cmin, v_tc, w_tc) in enumerate(collisions):
        if tc > XAXIS_MAX:
            continue
        for ax, yval in ((ax_v, v_tc), (ax_w, w_tc), (ax_d, cmin)):
            ax.plot(tc, yval, marker="X", markersize=11, color=color,
                    markeredgecolor="black", markeredgewidth=1.0, zorder=6)
        ax_d.annotate(
            f"{label} collision",
            xy=(tc, cmin), xytext=(tc + 0.6, -0.28 if i == 0 else -0.05),
            fontsize=8.5, color=color, fontweight="bold", va="center",
            arrowprops=dict(arrowstyle="->", color=color, lw=1.0))

    # Clearance reference thresholds.
    ax_d.axhline(NEAR_COLLISION_THRESHOLD, color="#f5a623", linestyle="-.",
                 linewidth=1.3, zorder=1)
    ax_d.axhline(COLLISION_THRESHOLD, color="#d62728", linestyle="-.",
                 linewidth=1.3, zorder=1)
    ax_d.text(0.4, NEAR_COLLISION_THRESHOLD + 0.06, "near-collision (0.5 m)",
              fontsize=8, color="#c8851a")
    ax_d.text(0.4, COLLISION_THRESHOLD - 0.28, "collision (0.2 m)",
              fontsize=8, color="#d62728")

    # Axis labels, panel tags, limits.
    ax_v.set_ylabel("v (m/s)")
    ax_w.set_ylabel(r"$\omega$ (rad/s)")
    ax_d.set_ylabel(r"$d_{min}$ (m)")
    ax_d.set_xlabel("Time (s)")
    for tag, ax in zip(("(a)", "(b)", "(c)"), axes):
        ax.text(0.015, 0.88, tag, transform=ax.transAxes,
                fontsize=12, fontweight="bold")
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, XAXIS_MAX)
    ax_d.set_ylim(bottom=-0.4)

    from matplotlib.patches import Patch
    handles, labels = ax_v.get_legend_handles_labels()
    handles.append(Patch(facecolor="0.5", alpha=0.18,
                         label="avoid (clearance < 0.5 m)"))
    fig.legend(handles, [*labels, "avoid (clearance < 0.5 m)"],
               loc="upper center", ncol=4, fontsize=9.5,
               frameon=True, bbox_to_anchor=(0.5, 1.0))
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    fig.savefig(args.output, dpi=200, bbox_inches="tight")
    png_path = os.path.splitext(args.output)[0] + ".png"
    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    print(f"Figure saved to {args.output} and {png_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()

