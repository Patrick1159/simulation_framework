#!/usr/bin/env python3
"""
gen_compare.py — Generate IEEE-style comparison figures and report.

Usage:
  python3 gen_compare.py --scene crossing \
    --methods "Ours:exp/data/crossing/ours/run_1" "DODT:exp/data/crossing/dodt/run_1" \
    --output exp/data/crossing/compare
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

# IEEE-style plot settings
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "mathtext.fontset": "stix",
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
    "axes.linewidth": 0.6,
    "xtick.major.width": 0.6,
    "ytick.major.width": 0.6,
    "grid.linewidth": 0.4,
    "lines.linewidth": 1.2,
})

# Clean color palette
COLORS = {
    "Ours":  "#2166AC",   # deep blue
    "DODT":  "#B2182B",   # deep red
}
MARKERS = {
    "Ours": "o",
    "DODT": "s",
}
# Fallback for unknown methods
_FALLBACK_COLORS = ["#4DAF4A", "#FF7F00", "#984EA3"]
_FALLBACK_MARKERS = ["^", "D", "v"]


def get_style(name, idx):
    c = COLORS.get(name, _FALLBACK_COLORS[idx % len(_FALLBACK_COLORS)])
    m = MARKERS.get(name, _FALLBACK_MARKERS[idx % len(_FALLBACK_MARKERS)])
    return c, m


def load_metrics(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    r = rows[0]
    out = {}
    for k, v in r.items():
        try:
            out[k] = float(v)
        except ValueError:
            out[k] = v
    return out


def load_prediction_error(path):
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


def load_trajectory(path):
    by_id = defaultdict(lambda: {"t": [], "x": [], "y": []})
    with open(path) as f:
        for r in csv.DictReader(f):
            oid = int(r["id"])
            by_id[oid]["t"].append(float(r["timestamp"]))
            by_id[oid]["x"].append(float(r["x"]))
            by_id[oid]["y"].append(float(r["y"]))
    return by_id


def plot_prediction_comparison(methods_pred, out_dir):
    """Prediction RMSE vs lookahead horizon — line plot."""
    fig, ax = plt.subplots(figsize=(3.5, 2.5))
    for i, (name, data) in enumerate(methods_pred.items()):
        c, m = get_style(name, i)
        h = [d["horizon"] for d in data]
        r = [d["rmse"] for d in data]
        ax.plot(h, r, marker=m, color=c, label=name,
                markersize=4, markeredgewidth=0.6, markeredgecolor="white",
                linewidth=1.4, zorder=3)
    ax.set_xlabel("Prediction Horizon (s)")
    ax.set_ylabel("RMSE (m)")
    ax.legend(frameon=True, fancybox=False, edgecolor="#cccccc",
              framealpha=0.95, loc="upper left")
    ax.grid(True, alpha=0.25, linestyle="-")
    ax.set_xlim(0, None)
    ax.set_ylim(0, None)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    for fmt in ["pdf", "png"]:
        fig.savefig(os.path.join(out_dir, f"fig_prediction_comparison.{fmt}"))
    plt.close(fig)
    print(f"  Saved: fig_prediction_comparison.pdf/.png")


def plot_mota_decomposition(methods_metrics, out_dir):
    """MOTA decomposition — grouped bar chart."""
    fig, ax = plt.subplots(figsize=(3.5, 2.5))
    names = list(methods_metrics.keys())
    categories = ["FP", "FN", "IDSW"]
    cat_colors = ["#4ECDC4", "#FF6B6B", "#45B7D1"]
    x = np.arange(len(names))
    n_cat = len(categories)
    width = 0.22
    offsets = np.linspace(-(n_cat - 1) / 2 * width, (n_cat - 1) / 2 * width, n_cat)
    for j, (cat, color) in enumerate(zip(categories, cat_colors)):
        vals = [methods_metrics[n].get(cat, 0) for n in names]
        bars = ax.bar(x + offsets[j], vals, width, label=cat, color=color,
                      edgecolor="white", linewidth=0.4, zorder=3)
        for bar, val in zip(bars, vals):
            if val > 0:
                ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1,
                        f"{int(val)}", ha="center", va="bottom", fontsize=6.5)
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Count")
    ax.legend(frameon=True, fancybox=False, edgecolor="#cccccc",
              framealpha=0.95, ncol=3, loc="upper right")
    ax.grid(True, alpha=0.25, axis="y", linestyle="-")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.set_ylim(0, None)
    for fmt in ["pdf", "png"]:
        fig.savefig(os.path.join(out_dir, f"fig_mota_comparison.{fmt}"))
    plt.close(fig)
    print(f"  Saved: fig_mota_comparison.pdf/.png")


def plot_mota_summary(methods_metrics, out_dir):
    """MOTA + epos summary bar chart."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(3.5, 2.2))
    names = list(methods_metrics.keys())
    x = np.arange(len(names))

    # MOTA
    motas = [methods_metrics[n].get("MOTA", 0) for n in names]
    colors_mota = [get_style(n, i)[0] for i, n in enumerate(names)]
    bars = ax1.bar(x, motas, 0.5, color=colors_mota,
                   edgecolor="white", linewidth=0.4, zorder=3)
    for bar, val in zip(bars, motas):
        ax1.text(bar.get_x() + bar.get_width() / 2,
                 max(0, bar.get_height()) + 0.02,
                 f"{val:.2f}", ha="center", va="bottom", fontsize=7)
    ax1.set_xticks(x)
    ax1.set_xticklabels(names)
    ax1.set_ylabel("MOTA")
    ax1.set_ylim(min(0, min(motas) - 0.1), 1.0)
    ax1.axhline(0, color="#999999", linewidth=0.5, zorder=1)
    ax1.grid(True, alpha=0.25, axis="y", linestyle="-")
    ax1.spines["top"].set_visible(False)
    ax1.spines["right"].set_visible(False)

    # epos
    epos = [methods_metrics[n].get("epos", 0) for n in names]
    bars = ax2.bar(x, epos, 0.5, color=colors_mota,
                   edgecolor="white", linewidth=0.4, zorder=3)
    for bar, val in zip(bars, epos):
        ax2.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.005,
                 f"{val:.3f}", ha="center", va="bottom", fontsize=7)
    ax2.set_xticks(x)
    ax2.set_xticklabels(names)
    ax2.set_ylabel("Position Error (m)")
    ax2.set_ylim(0, None)
    ax2.grid(True, alpha=0.25, axis="y", linestyle="-")
    ax2.spines["top"].set_visible(False)
    ax2.spines["right"].set_visible(False)

    fig.tight_layout(w_pad=2.0)
    for fmt in ["pdf", "png"]:
        fig.savefig(os.path.join(out_dir, f"fig_mota_summary.{fmt}"))
    plt.close(fig)
    print(f"  Saved: fig_mota_summary.pdf/.png")


def plot_bev_comparison(gt_path, tracker_paths, out_dir):
    """BEV trajectory comparison — GT + all methods."""
    fig, ax = plt.subplots(figsize=(4, 3.5))
    gt = load_trajectory(gt_path)
    for i, (oid, traj) in enumerate(sorted(gt.items())):
        ax.plot(traj["x"], traj["y"], color="#888888", linewidth=2.5, alpha=0.35,
                label="Ground Truth" if i == 0 else None, zorder=1)

    for mi, (name, path) in enumerate(tracker_paths.items()):
        c, _ = get_style(name, mi)
        trk = load_trajectory(path)
        linestyles = ["--", "-.", ":", "-"]
        ls = linestyles[mi % len(linestyles)]
        for j, (oid, traj) in enumerate(sorted(trk.items())):
            ax.plot(traj["x"], traj["y"], color=c, linewidth=1.0, alpha=0.8,
                    linestyle=ls, label=name if j == 0 else None, zorder=2 + mi)

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.legend(loc="best", fontsize=7, frameon=True, fancybox=False,
              edgecolor="#cccccc", framealpha=0.95)
    ax.grid(True, alpha=0.25, linestyle="-")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    for fmt in ["pdf", "png"]:
        fig.savefig(os.path.join(out_dir, f"fig_bev_comparison.{fmt}"))
    plt.close(fig)
    print(f"  Saved: fig_bev_comparison.pdf/.png")


def generate_report(scene, methods, all_metrics, all_pred, out_dir):
    """Generate markdown comparison report."""
    lines = [f"# Experiment Report — {scene.title()} Scene\n"]

    # Metrics table
    if all_metrics:
        lines.append("## Tracking Metrics\n")
        cols = ["MOTA", "IDSW", "FP", "FN", "epos"]
        header = "| Method | " + " | ".join(cols) + " |"
        sep = "|--------|" + "|".join(["-------:"] * len(cols)) + "|"
        lines.append(header)
        lines.append(sep)
        for name, m in all_metrics.items():
            vals = []
            for c in cols:
                v = m.get(c, "-")
                if isinstance(v, float):
                    if c in ("MOTA", "epos"):
                        vals.append(f"{v:.4f}")
                    else:
                        vals.append(f"{int(v)}")
                else:
                    vals.append(str(v))
            lines.append(f"| {name} | " + " | ".join(vals) + " |")
        lines.append("")

    # Prediction error table
    if all_pred:
        lines.append("## Prediction RMSE (m)\n")
        horizons = [d["horizon"] for d in list(all_pred.values())[0]]
        header = "| Method | " + " | ".join(f"{h}s" for h in horizons) + " |"
        sep = "|--------|" + "|".join(["------:"] * len(horizons)) + "|"
        lines.append(header)
        lines.append(sep)
        for name, data in all_pred.items():
            vals = [f"{d['rmse']:.4f}" for d in data]
            lines.append(f"| {name} | " + " | ".join(vals) + " |")
        lines.append("")

    # Figures
    lines.append("## Figures\n")
    lines.append("![MOTA & Position Error](fig_mota_summary.png)\n")
    lines.append("![MOTA Decomposition](fig_mota_comparison.png)\n")
    lines.append("![Prediction RMSE](fig_prediction_comparison.png)\n")
    lines.append("![BEV Trajectory](fig_bev_comparison.png)\n")

    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--scene", required=True)
    p.add_argument("--methods", nargs="+", required=True,
                   help="name:data_dir pairs, e.g. Ours:exp/data/crossing/ours/run_1")
    p.add_argument("--output", required=True, help="Output directory for report + figures")
    args = p.parse_args()

    methods = {}
    for m in args.methods:
        name, path = m.split(":", 1)
        methods[name] = path

    os.makedirs(args.output, exist_ok=True)

    all_metrics = {}
    all_pred = {}
    tracker_paths = {}
    gt_path = None

    for name, d in methods.items():
        m_path = os.path.join(d, "metrics.csv")
        p_path = os.path.join(d, "prediction_error.csv")
        t_path = os.path.join(d, "tracker.csv")
        g_path = os.path.join(d, "gt.csv")

        if os.path.exists(m_path):
            all_metrics[name] = load_metrics(m_path)
        if os.path.exists(p_path):
            all_pred[name] = load_prediction_error(p_path)
        if os.path.exists(t_path):
            tracker_paths[name] = t_path
        if gt_path is None and os.path.exists(g_path):
            gt_path = g_path

    print("Generating comparison figures...")
    if all_pred:
        plot_prediction_comparison(all_pred, args.output)
    if all_metrics:
        plot_mota_decomposition(all_metrics, args.output)
        plot_mota_summary(all_metrics, args.output)
    if gt_path and tracker_paths:
        plot_bev_comparison(gt_path, tracker_paths, args.output)

    report = generate_report(args.scene, methods, all_metrics, all_pred, args.output)
    report_path = os.path.join(args.output, "report.md")
    with open(report_path, "w") as f:
        f.write(report)
    print(f"Report saved to {report_path}")


if __name__ == "__main__":
    main()
