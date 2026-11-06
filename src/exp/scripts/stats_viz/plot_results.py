#!/usr/bin/env python3
"""
plot_results.py — Generate paper figures from experiment data.

Figures:
  1. Prediction RMSE vs. Lookahead Time (line plot)
  2. IDSW / FP / FN decomposition (bar chart)
  3. Bird's-eye-view trajectory (GT + tracker)

Usage: python3 plot_results.py --data-dir exp/data/crossing/run_1
       python3 plot_results.py --data-dir exp/data/crossing/run_1 --methods Ours:exp/data/crossing/run_1
"""

import argparse
import csv
import math
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from collections import defaultdict

# IEEE-friendly style
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 9,
    "axes.labelsize": 10,
    "axes.titlesize": 10,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "figure.dpi": 300,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.05,
})

# PLACEHOLDER_MORE_PLOTS


def load_prediction_error(path):
    """Load prediction_error.csv."""
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({
                "horizon": float(r["horizon"]),
                "rmse": float(r["rmse"]),
                "mean": float(r["mean"]),
                "n": int(r["n"]),
            })
    return rows


def load_metrics(path):
    """Load metrics.csv."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    r = rows[0]
    out = {}
    for k, v in r.items():
        try:
            out[k] = float(v) if "." in v or v == "nan" else int(v)
        except ValueError:
            out[k] = float("nan")
    return out


def load_trajectory(path):
    """Load CSV trajectory grouped by ID."""
    by_id = defaultdict(lambda: {"t": [], "x": [], "y": []})
    with open(path) as f:
        for r in csv.DictReader(f):
            oid = int(r["id"])
            by_id[oid]["t"].append(float(r["timestamp"]))
            by_id[oid]["x"].append(float(r["x"]))
            by_id[oid]["y"].append(float(r["y"]))
    return by_id


def plot_prediction_rmse(methods, out_path):
    """Fig. 2: Prediction RMSE vs. Lookahead Time."""
    fig, ax = plt.subplots(figsize=(3.5, 2.5))
    markers = ["o", "s", "^", "D", "v"]
    for i, (name, data) in enumerate(methods.items()):
        horizons = [d["horizon"] for d in data]
        rmses = [d["rmse"] for d in data]
        ax.plot(horizons, rmses, marker=markers[i % len(markers)],
                label=name, markersize=4, linewidth=1.2)
    ax.set_xlabel("Lookahead Time (s)")
    ax.set_ylabel("Prediction RMSE (m)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, max(horizons) * 1.05)
    ax.set_ylim(0, None)
    fig.savefig(out_path)
    fig.savefig(out_path.replace(".pdf", ".png"))
    plt.close(fig)
    print(f"  Saved: {out_path}")


def plot_mota_decomposition(methods, out_path):
    """Fig. 3: IDSW / FP / FN decomposition bar chart."""
    fig, ax = plt.subplots(figsize=(3.5, 2.5))
    names = list(methods.keys())
    x = np.arange(len(names))
    width = 0.25

    fps = [methods[n].get("FP", 0) for n in names]
    fns = [methods[n].get("FN", 0) for n in names]
    idsws = [methods[n].get("IDSW", 0) for n in names]

    ax.bar(x - width, fps, width, label="FP", color="#4ECDC4")
    ax.bar(x, fns, width, label="FN", color="#FF6B6B")
    ax.bar(x + width, idsws, width, label="IDSW", color="#45B7D1")

    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=15, ha="right")
    ax.set_ylabel("Count")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")
    fig.savefig(out_path)
    fig.savefig(out_path.replace(".pdf", ".png"))
    plt.close(fig)
    print(f"  Saved: {out_path}")


# PLACEHOLDER_BEV


def plot_bev_trajectory(gt_path, tracker_paths, out_path):
    """Fig. 5: Bird's-eye-view trajectory plot."""
    fig, ax = plt.subplots(figsize=(4, 4))

    gt = load_trajectory(gt_path)
    colors_gt = ["#333333", "#666666", "#999999"]
    for i, (oid, traj) in enumerate(sorted(gt.items())):
        c = colors_gt[i % len(colors_gt)]
        ax.plot(traj["x"], traj["y"], color=c, linewidth=2, alpha=0.5,
                label=f"GT {oid}" if i < 3 else None)

    method_colors = ["#E74C3C", "#3498DB", "#2ECC71", "#F39C12"]
    for mi, (name, path) in enumerate(tracker_paths.items()):
        trk = load_trajectory(path)
        mc = method_colors[mi % len(method_colors)]
        for j, (oid, traj) in enumerate(sorted(trk.items())):
            label = name if j == 0 else None
            ax.plot(traj["x"], traj["y"], color=mc, linewidth=1,
                    alpha=0.7, linestyle="--", label=label)

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.legend(loc="best", fontsize=7)
    ax.grid(True, alpha=0.3)
    fig.savefig(out_path)
    fig.savefig(out_path.replace(".pdf", ".png"))
    plt.close(fig)
    print(f"  Saved: {out_path}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data-dir", required=True,
                   help="Directory with gt.csv, tracker.csv, metrics.csv, prediction_error.csv")
    p.add_argument("--methods", nargs="*", default=None,
                   help="name:data_dir pairs for multi-method comparison")
    p.add_argument("--out-dir", default=None,
                   help="Output directory for figures (default: data-dir)")
    args = p.parse_args()

    out_dir = args.out_dir or args.data_dir
    os.makedirs(out_dir, exist_ok=True)

    # Build method dict
    if args.methods:
        method_dirs = {}
        for m in args.methods:
            name, path = m.split(":", 1)
            method_dirs[name] = path
    else:
        method_dirs = {"Ours": args.data_dir}

    # Fig. 2: Prediction RMSE
    pred_data = {}
    for name, d in method_dirs.items():
        pe_path = os.path.join(d, "prediction_error.csv")
        if os.path.exists(pe_path):
            pred_data[name] = load_prediction_error(pe_path)
    if pred_data:
        plot_prediction_rmse(pred_data, os.path.join(out_dir, "fig_prediction_rmse.pdf"))

    # Fig. 3: MOTA decomposition
    metrics_data = {}
    for name, d in method_dirs.items():
        m_path = os.path.join(d, "metrics.csv")
        if os.path.exists(m_path):
            metrics_data[name] = load_metrics(m_path)
    if metrics_data:
        plot_mota_decomposition(metrics_data, os.path.join(out_dir, "fig_mota_decomposition.pdf"))

    # Fig. 5: BEV trajectory
    gt_path = os.path.join(args.data_dir, "gt.csv")
    tracker_paths = {}
    for name, d in method_dirs.items():
        tp = os.path.join(d, "tracker.csv")
        if os.path.exists(tp):
            tracker_paths[name] = tp
    if os.path.exists(gt_path) and tracker_paths:
        plot_bev_trajectory(gt_path, tracker_paths, os.path.join(out_dir, "fig_bev_trajectory.pdf"))

    print("Done.")


if __name__ == "__main__":
    main()
