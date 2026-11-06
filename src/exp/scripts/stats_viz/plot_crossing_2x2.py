#!/usr/bin/env python3
"""
Aggregate per-run planning + control-smoothness metrics across perception
front-ends and render a 2x2 colorblind-safe grouped bar chart.

Input tree (one level of method dirs under --root):
  <root>/<method>/run_<n>/planning_metrics.json
  <root>/<method>/run_<n>/control_smoothness.json

Subplots (x-axis = front-end):
  (a) Success Rate (%, higher better)  = goal_success==1 AND collision-free
  (b) Collision Rate (%, lower better) = fraction of trials with a collision
  (c) Min Obstacle Clearance (m, higher better) = per-trial min_clearance_m, mean +/- std
  (d) Cmd-Vel Smoothness (lower better) = mean_abs_delta_v, mean +/- std

A trial is "collision-free" when collision_events == 0.

Usage:
  python3 plot_crossing_2x2.py --root /data/rosbags/avoidance/pair_crossing \
      --methods dodt fapp ours_full --labels DODT FAPP Ours \
      --out crossing_2x2.pdf --summary-csv crossing_summary.csv
"""

import argparse
import csv
import glob
import json
import math
import os
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Okabe-Ito colorblind-safe palette (one color per front-end column).
PALETTE = ["#0072B2", "#E69F00", "#009E73", "#CC79A7", "#56B4E9", "#D55E00"]


def _load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def collect_method(method_dir):
    """Return a list of per-run dicts for one method directory."""
    runs = []
    for run_dir in sorted(glob.glob(os.path.join(method_dir, "run_*"))):
        pm = _load_json(os.path.join(run_dir, "planning_metrics.json"))
        if pm is None:
            continue
        cs = _load_json(os.path.join(run_dir, "control_smoothness.json")) or {}

        goal_success = int(pm.get("goal_success", 0) or 0)
        collision_events = int(pm.get("collision_events", 0) or 0)
        collision_free = collision_events == 0
        runs.append(
            {
                "run": os.path.basename(run_dir),
                "goal_success": goal_success,
                "collision_events": collision_events,
                "success": 1 if (goal_success == 1 and collision_free) else 0,
                "collision": 1 if collision_events > 0 else 0,
                "min_clearance_m": pm.get("min_clearance_m"),
                "mean_abs_delta_v": cs.get("mean_abs_delta_v"),
                "mean_abs_delta_w": cs.get("mean_abs_delta_w"),
            }
        )
    return runs


def _clean(values):
    """Drop None/NaN."""
    out = []
    for v in values:
        if v is None:
            continue
        try:
            fv = float(v)
        except (TypeError, ValueError):
            continue
        if math.isnan(fv):
            continue
        out.append(fv)
    return out


def _mean_std(values):
    vals = _clean(values)
    if not vals:
        return float("nan"), 0.0
    mean = statistics.mean(vals)
    std = statistics.pstdev(vals) if len(vals) > 1 else 0.0
    return mean, std


def aggregate(runs):
    n = len(runs)
    success_rate = 100.0 * sum(r["success"] for r in runs) / n if n else float("nan")
    collision_rate = 100.0 * sum(r["collision"] for r in runs) / n if n else float("nan")
    clr_mean, clr_std = _mean_std([r["min_clearance_m"] for r in runs])
    sv_mean, sv_std = _mean_std([r["mean_abs_delta_v"] for r in runs])
    sw_mean, sw_std = _mean_std([r["mean_abs_delta_w"] for r in runs])
    return {
        "n": n,
        "success_rate_pct": success_rate,
        "collision_rate_pct": collision_rate,
        "min_clearance_mean_m": clr_mean,
        "min_clearance_std_m": clr_std,
        "smoothness_dv_mean": sv_mean,
        "smoothness_dv_std": sv_std,
        "smoothness_dw_mean": sw_mean,
        "smoothness_dw_std": sw_std,
    }


def make_figure(labels, aggs, out_path):
    fig, axes = plt.subplots(2, 2, figsize=(9, 7))
    x = list(range(len(labels)))
    colors = [PALETTE[i % len(PALETTE)] for i in x]

    def bars(ax, values, errs, title, ylabel):
        plotted = [0.0 if (v is None or math.isnan(v)) else v for v in values]
        kw = {}
        if errs is not None:
            kw["yerr"] = errs
            kw["capsize"] = 4
        ax.bar(x, plotted, color=colors, **kw)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.grid(axis="y", linestyle=":", alpha=0.5)

    bars(axes[0][0], [a["success_rate_pct"] for a in aggs], None,
         "(a) Success Rate (higher better)", "Success rate (%)")
    axes[0][0].set_ylim(0, 105)

    bars(axes[0][1], [a["collision_rate_pct"] for a in aggs], None,
         "(b) Collision Rate (lower better)", "Collision rate (%)")
    axes[0][1].set_ylim(0, 105)

    bars(axes[1][0], [a["min_clearance_mean_m"] for a in aggs],
         [a["min_clearance_std_m"] for a in aggs],
         "(c) Min Obstacle Clearance (higher better)", "Min clearance (m)")

    bars(axes[1][1], [a["smoothness_dv_mean"] for a in aggs],
         [a["smoothness_dv_std"] for a in aggs],
         "(d) Cmd-Vel Smoothness (lower better)", r"mean $|\Delta v|$ (m/s per step)")

    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    png = os.path.splitext(out_path)[0] + ".png"
    fig.savefig(png, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return png


def write_summary(labels, methods, aggs, csv_path):
    fields = [
        "method", "label", "n",
        "success_rate_pct", "collision_rate_pct",
        "min_clearance_mean_m", "min_clearance_std_m",
        "smoothness_dv_mean", "smoothness_dv_std",
        "smoothness_dw_mean", "smoothness_dw_std",
    ]
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for method, label, agg in zip(methods, labels, aggs):
            row = {"method": method, "label": label}
            row.update(agg)
            w.writerow(row)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--root", required=True,
                   help="Scenario dir containing <method>/run_<n>/ subdirs.")
    p.add_argument("--methods", nargs="+", default=["dodt", "fapp", "ours_full"],
                   help="Method dir names (x-axis order).")
    p.add_argument("--labels", nargs="+", default=None,
                   help="Display labels (default: title-cased method names).")
    p.add_argument("--out", default=None,
                   help="Figure path (default: <root>/crossing_2x2.pdf).")
    p.add_argument("--summary-csv", default=None,
                   help="Summary CSV path (default: <root>/crossing_summary.csv).")
    args = p.parse_args()

    labels = args.labels if args.labels else [m.replace("_", " ").title() for m in args.methods]
    if len(labels) != len(args.methods):
        p.error("--labels count must match --methods count")

    out = args.out or os.path.join(args.root, "crossing_2x2.pdf")
    summary_csv = args.summary_csv or os.path.join(args.root, "crossing_summary.csv")

    aggs = []
    for method in args.methods:
        method_dir = os.path.join(args.root, method)
        runs = collect_method(method_dir)
        agg = aggregate(runs)
        aggs.append(agg)
        print(f"[{method}] n={agg['n']} "
              f"success={agg['success_rate_pct']:.1f}% "
              f"collision={agg['collision_rate_pct']:.1f}% "
              f"min_clr={agg['min_clearance_mean_m']:.3f}+/-{agg['min_clearance_std_m']:.3f} "
              f"dv={agg['smoothness_dv_mean']:.5f}+/-{agg['smoothness_dv_std']:.5f}")

    if all(a["n"] == 0 for a in aggs):
        raise SystemExit(f"ERROR: no runs found under {args.root} for methods {args.methods}")

    png = make_figure(labels, aggs, out)
    write_summary(labels, args.methods, aggs, summary_csv)
    print(f"-> figure: {out}")
    print(f"-> figure: {png}")
    print(f"-> summary: {summary_csv}")


if __name__ == "__main__":
    main()
