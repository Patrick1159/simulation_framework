#!/usr/bin/env python3
"""
sup_exp_figures.py — Generate paper figures and result tables from supplemental experiments.

Reads summary CSVs from:
  - exp/data/sup_tracking_to_planning/summary_*.csv
  - exp/data/sup_occlusion_reid/summary_*.csv

Produces:
  - figures/exp1_collision_clearance.pdf/png
  - figures/exp1_prediction_rmse.pdf/png
  - figures/exp2_idsw_vs_duration.pdf/png
  - figures/exp2_reid_success.pdf/png
  - figures/exp2_prediction_after_reappearance.pdf/png
  - figures/sup_exp.md (updated with result tables)

Usage:
  python3 sup_exp_figures.py
"""

import csv
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from collections import defaultdict

# ── paths ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))  # simulation_framework
EXP_DATA = os.path.join(SIM_ROOT, "exp", "data")
TRACK_ROOT = os.path.join(EXP_DATA, "sup_tracking_to_planning")
OCC_ROOT = os.path.join(EXP_DATA, "sup_occlusion_reid")
FIG_DIR = os.path.join(os.path.dirname(SIM_ROOT), "figures")

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

METHOD_ORDER_EXP1 = ["dodt", "fapp", "ours_no_clip", "ours_full", "oracle_gt"]
METHOD_LABELS = {
    "dodt": "DODT",
    "fapp": "FAPP",
    "ours_no_clip": "Ours w/o CLIP",
    "ours_full": "Ours full",
    "oracle_gt": "Oracle GT",
    "stage1_only": "Stage I only",
    "ours_no_fastsam": "Ours w/o FastSAM",
    "ours_no_sam": "Ours w/o SAM",
}
SCENE_LABELS = {
    "pair_crossing": "Pair Crossing",
    "front_crossing": "Front Crossing",
    "random_multi": "Random Multi",
}

COLORS = {
    "dodt": "#E74C3C",
    "fapp": "#F39C12",
    "ours_no_clip": "#3498DB",
    "ours_full": "#2ECC71",
    "oracle_gt": "#9B59B6",
    "stage1_only": "#E67E22",
    "ours_no_fastsam": "#1ABC9C",
    "ours_no_sam": "#C0392B",
}


def load_csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def mean_std(values):
    """Return (mean, std) of non-nan values."""
    vals = [v for v in values if not (isinstance(v, float) and np.isnan(v))]
    if not vals:
        return float("nan"), float("nan")
    return np.mean(vals), np.std(vals, ddof=1)


# ══════════════════════════════════════════════════════════════════════════
# Experiment 1: Tracking-to-Planning
# ══════════════════════════════════════════════════════════════════════════

def process_exp1():
    """Aggregate Experiment 1 data across scenes, methods, and repeats."""
    planning = load_csv(os.path.join(TRACK_ROOT, "summary_planning_metrics.csv"))
    pred = load_csv(os.path.join(TRACK_ROOT, "summary_prediction_error.csv"))

    # Group planning metrics by (group=scene, method)
    plan_by_key = defaultdict(list)
    for row in planning:
        key = (row["group"], row["method"])
        plan_by_key[key].append(row)

    # Group prediction errors by (group=scene, method, horizon)
    pred_by_key = defaultdict(list)
    for row in pred:
        key = (row["group"], row["method"], row["horizon"])
        pred_by_key[key].append(row)

    # Build per-method aggregate (across all scenes)
    methods_agg = {}
    for method in METHOD_ORDER_EXP1:
        collisions = []
        near_collisions = []
        min_clearances = []
        mean_clearances = []

        for (scene, meth), rows in plan_by_key.items():
            if meth != method:
                continue
            for r in rows:
                collisions.append(float(r.get("collision_rate", 0)))
                near_collisions.append(float(r.get("near_collision_rate", 0)))
                min_clearances.append(float(r.get("min_clearance_m", float("nan"))))
                mean_clearances.append(float(r.get("mean_clearance_m", float("nan"))))

        pred_horizons = {}
        for horizon in ["0.1", "0.2", "0.3", "0.5", "1.0"]:
            rmses = []
            for (scene, meth, h), rows in pred_by_key.items():
                if meth != method or h != horizon:
                    continue
                for r in rows:
                    rmses.append(float(r.get("rmse", float("nan"))))
            if rmses:
                mu, sd = mean_std(rmses)
                pred_horizons[float(horizon)] = {"rmse_mean": mu, "rmse_std": sd, "n": len(rmses)}

        methods_agg[method] = {
            "collision_rate": mean_std(collisions),
            "near_collision_rate": mean_std(near_collisions),
            "min_clearance": mean_std(min_clearances),
            "mean_clearance": mean_std(mean_clearances),
            "prediction": pred_horizons,
        }

    return methods_agg


def plot_exp1_collision_clearance(methods_agg):
    """Bar chart: collision rate and min clearance per method."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7, 3))

    methods = [m for m in METHOD_ORDER_EXP1 if m in methods_agg]
    x = np.arange(len(methods))
    labels = [METHOD_LABELS.get(m, m) for m in methods]

    # Collision / near-collision rate
    near_rates = [methods_agg[m]["near_collision_rate"][0] * 100 for m in methods]
    col_rates = [methods_agg[m]["collision_rate"][0] * 100 for m in methods]

    w = 0.3
    ax1.bar(x - w/2, near_rates, w, label="Near-collision", color="#F39C12", alpha=0.8)
    ax1.bar(x + w/2, col_rates, w, label="Collision", color="#E74C3C", alpha=0.8)
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=20, ha="right")
    ax1.set_ylabel("Rate (%)")
    ax1.set_title("Collision / Near-collision Rate")
    ax1.legend(fontsize=7)
    ax1.grid(alpha=0.3, axis="y")

    # Min clearance
    clearances = [methods_agg[m]["min_clearance"][0] for m in methods]
    cle_std = [methods_agg[m]["min_clearance"][1] for m in methods]
    bars = ax2.bar(x, clearances, color=[COLORS.get(m, "#333") for m in methods], alpha=0.8)
    ax2.errorbar(x, clearances, yerr=cle_std, fmt="none", ecolor="black", capsize=3)
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, rotation=20, ha="right")
    ax2.set_ylabel("Min Clearance (m)")
    ax2.set_title("Minimum Dynamic-Obstacle Clearance")
    ax2.grid(alpha=0.3, axis="y")

    fig.tight_layout()
    for ext in [".pdf", ".png"]:
        fig.savefig(os.path.join(FIG_DIR, f"exp1_collision_clearance{ext}"))
    plt.close(fig)
    print("  Saved: exp1_collision_clearance.pdf/.png")


def plot_exp1_prediction_rmse(methods_agg):
    """Line chart: Prediction RMSE vs. Lookahead Horizon."""
    fig, ax = plt.subplots(figsize=(4.5, 3))
    markers = ["o", "s", "^", "D", "v"]

    for i, method in enumerate(METHOD_ORDER_EXP1):
        if method not in methods_agg:
            continue
        pred = methods_agg[method]["prediction"]
        if not pred:
            continue
        horizons = sorted(pred.keys())
        rmses = [pred[h]["rmse_mean"] for h in horizons]
        stds = [pred[h]["rmse_std"] for h in horizons]
        label = METHOD_LABELS.get(method, method)
        ax.errorbar(horizons, rmses, yerr=stds,
                    marker=markers[i % len(markers)], markersize=5,
                    linewidth=1.2, color=COLORS.get(method, "#333"),
                    label=label, capsize=3)

    ax.set_xlabel("Lookahead Horizon (s)")
    ax.set_ylabel("Prediction RMSE (m)")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
    ax.set_xlim(0, 1.05)
    ax.set_ylim(0, None)
    fig.tight_layout()
    for ext in [".pdf", ".png"]:
        fig.savefig(os.path.join(FIG_DIR, f"exp1_prediction_rmse{ext}"))
    plt.close(fig)
    print("  Saved: exp1_prediction_rmse.pdf/.png")


# ══════════════════════════════════════════════════════════════════════════
# Experiment 2: Controlled Occlusion + Re-ID
# ══════════════════════════════════════════════════════════════════════════

METHOD_ORDER_EXP2 = ["stage1_only", "ours_no_sam", "ours_no_clip", "ours_no_fastsam", "ours_full", "dodt", "fapp"]
DURATION_ORDER = [0.5, 1.0, 2.0, 3.0, 4.0]


def process_exp2():
    """Aggregate Experiment 2 data across durations, methods, repeats."""
    occ_metrics = load_csv(os.path.join(OCC_ROOT, "summary_occlusion_reid_metrics.csv"))

    # Group by (duration, method)
    by_key = defaultdict(list)
    for row in occ_metrics:
        dur_str = row.get("group", "").replace("duration_", "").replace("s", "")
        try:
            duration = float(dur_str)
        except ValueError:
            continue
        key = (duration, row["method"])
        by_key[key].append(row)

    methods_agg = {}
    for method in METHOD_ORDER_EXP2:
        dur_data = {}
        for dur in DURATION_ORDER:
            key = (dur, method)
            rows = by_key.get(key, [])
            if not rows:
                continue

            idsw_vals = [int(r.get("idsw_count", 0)) for r in rows]
            reid_vals = []
            pred_rmse_vals = []
            for r in rows:
                # Re-ID success rate
                reid_str = r.get("reid_success_rate", "")
                if reid_str:
                    try:
                        reid_vals.append(float(reid_str))
                    except ValueError:
                        pass
                # Prediction RMSE after reappearance
                pred_str = r.get("prediction_rmse_after_reappearance_m", "")
                if pred_str:
                    try:
                        pred_rmse_vals.append(float(pred_str))
                    except ValueError:
                        pass

            dur_data[dur] = {
                "idsw_mean": np.mean(idsw_vals) if idsw_vals else float("nan"),
                "idsw_std": np.std(idsw_vals, ddof=1) if len(idsw_vals) > 1 else 0.0,
                "reid_mean": np.mean(reid_vals) if reid_vals else float("nan"),
                "reid_std": np.std(reid_vals, ddof=1) if len(reid_vals) > 1 else 0.0,
                "pred_rmse_mean": np.mean(pred_rmse_vals) if pred_rmse_vals else float("nan"),
                "pred_rmse_std": np.std(pred_rmse_vals, ddof=1) if len(pred_rmse_vals) > 1 else 0.0,
                "n": len(rows),
            }

        methods_agg[method] = dur_data

    return methods_agg


def plot_exp2_idsw(methods_agg):
    """IDSW count vs. Occlusion Duration (line chart)."""
    fig, ax = plt.subplots(figsize=(4.5, 3))
    markers = ["o", "s", "^", "D", "v", "p"]

    for i, method in enumerate(METHOD_ORDER_EXP2):
        if method not in methods_agg:
            continue
        durs = sorted(methods_agg[method].keys())
        idsws = [methods_agg[method][d]["idsw_mean"] for d in durs]
        label = METHOD_LABELS.get(method, method)
        ax.plot(durs, idsws, marker=markers[i % len(markers)], markersize=5,
                linewidth=1.2, color=COLORS.get(method, "#333"), label=label)

    ax.set_xlabel("Occlusion Duration (s)")
    ax.set_ylabel("ID Switch Count")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
    ax.set_xlim(0, 4.5)
    ax.set_ylim(0, None)
    fig.tight_layout()
    for ext in [".pdf", ".png"]:
        fig.savefig(os.path.join(FIG_DIR, f"exp2_idsw_vs_duration{ext}"))
    plt.close(fig)
    print("  Saved: exp2_idsw_vs_duration.pdf/.png")


def plot_exp2_reid_success(methods_agg):
    """Re-ID success rate vs. Occlusion Duration."""
    fig, ax = plt.subplots(figsize=(4.5, 3))
    markers = ["o", "s", "^", "D", "v", "p"]

    for i, method in enumerate(METHOD_ORDER_EXP2):
        if method not in methods_agg:
            continue
        durs = sorted(methods_agg[method].keys())
        rates = [methods_agg[method][d]["reid_mean"] * 100 for d in durs]
        label = METHOD_LABELS.get(method, method)
        ax.plot(durs, rates, marker=markers[i % len(markers)], markersize=5,
                linewidth=1.2, color=COLORS.get(method, "#333"), label=label)

    ax.set_xlabel("Occlusion Duration (s)")
    ax.set_ylabel("Re-ID Success Rate (%)")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
    ax.set_xlim(0, 4.5)
    ax.set_ylim(0, 105)
    fig.tight_layout()
    for ext in [".pdf", ".png"]:
        fig.savefig(os.path.join(FIG_DIR, f"exp2_reid_success{ext}"))
    plt.close(fig)
    print("  Saved: exp2_reid_success.pdf/.png")


def plot_exp2_prediction_rmse(methods_agg):
    """Prediction RMSE after reappearance vs. Occlusion Duration."""
    fig, ax = plt.subplots(figsize=(4.5, 3))
    markers = ["o", "s", "^", "D", "v", "p"]

    for i, method in enumerate(METHOD_ORDER_EXP2):
        if method not in methods_agg:
            continue
        durs = sorted(methods_agg[method].keys())
        rmses = [methods_agg[method][d]["pred_rmse_mean"] for d in durs]
        label = METHOD_LABELS.get(method, method)
        ax.plot(durs, rmses, marker=markers[i % len(markers)], markersize=5,
                linewidth=1.2, color=COLORS.get(method, "#333"), label=label)

    ax.set_xlabel("Occlusion Duration (s)")
    ax.set_ylabel("Prediction RMSE after Reappearance (m)")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
    ax.set_xlim(0, 4.5)
    ax.set_ylim(0, None)
    fig.tight_layout()
    for ext in [".pdf", ".png"]:
        fig.savefig(os.path.join(FIG_DIR, f"exp2_prediction_after_reappearance{ext}"))
    plt.close(fig)
    print("  Saved: exp2_prediction_after_reappearance.pdf/.png")


# ══════════════════════════════════════════════════════════════════════════
# Markdown Report Generation
# ══════════════════════════════════════════════════════════════════════════

def fmt_val(mean, std, fmt=".3f", scale=1.0):
    """Format mean ± std."""
    if np.isnan(mean):
        return "N/A"
    if std == 0 or np.isnan(std):
        return f"{mean * scale:{fmt}}"
    return f"{mean * scale:{fmt}} ± {std * scale:{fmt}}"


def generate_markdown(exp1_agg, exp2_agg):
    """Generate the full sup_exp.md report."""
    lines = []

    lines.append("# 补充实验结果")
    lines.append("")
    lines.append("## 实验 1：Tracking-to-Planning 端到端动态避障仿真")
    lines.append("")
    lines.append("### 结果表格")
    lines.append("")
    lines.append("| Method | Collision / Near-collision ↓ | Min Clearance (m) ↑ | Prediction RMSE @1.0s (m) ↓ |")
    lines.append("|---|---:|---:|---:|")

    for method in METHOD_ORDER_EXP1:
        if method not in exp1_agg:
            continue
        agg = exp1_agg[method]
        label = METHOD_LABELS.get(method, method)
        col = agg["near_collision_rate"]
        near_str = f"{col[0]*100:.1f}%" if col[0] == 0 else f"{col[0]*100:.1f}% / {agg['collision_rate'][0]*100:.1f}%"
        clearance = fmt_val(*agg["min_clearance"], fmt=".2f")
        pred = agg["prediction"].get(1.0, {})
        pred_str = fmt_val(pred.get("rmse_mean", float("nan")), pred.get("rmse_std", float("nan")))
        lines.append(f"| {label} | {near_str} | {clearance} | {pred_str} |")

    lines.append("")
    lines.append("### 图表")
    lines.append("")
    lines.append("![Collision Rate & Clearance](exp1_collision_clearance.png)")
    lines.append("")
    lines.append("![Prediction RMSE vs Horizon](exp1_prediction_rmse.png)")
    lines.append("")

    # Per-scene breakdown
    lines.append("### 各场景详细结果")
    lines.append("")

    planning = load_csv(os.path.join(TRACK_ROOT, "summary_planning_metrics.csv"))
    scenes = sorted(set(r["group"] for r in planning))
    for scene in scenes:
        scene_label = SCENE_LABELS.get(scene, scene)
        lines.append(f"#### {scene_label}")
        lines.append("")
        lines.append("| Method | Collision Rate | Near-collision Rate | Min Clearance (m) | Mean Clearance (m) |")
        lines.append("|---|---:|---:|---:|")

        for method in METHOD_ORDER_EXP1:
            scene_rows = [r for r in planning if r["group"] == scene and r["method"] == method]
            if not scene_rows:
                continue
            label = METHOD_LABELS.get(method, method)
            col_rates = [float(r.get("collision_rate", 0)) for r in scene_rows]
            near_rates = [float(r.get("near_collision_rate", 0)) for r in scene_rows]
            min_clears = [float(r.get("min_clearance_m", float("nan"))) for r in scene_rows]
            mean_clears = [float(r.get("mean_clearance_m", float("nan"))) for r in scene_rows]

            col_m, col_s = mean_std(col_rates)
            near_m, near_s = mean_std(near_rates)
            minc_m, minc_s = mean_std(min_clears)
            meanc_m, meanc_s = mean_std(mean_clears)

            lines.append(f"| {label} | {fmt_val(col_m, col_s, scale=100, fmt='.1f')}% | {fmt_val(near_m, near_s, scale=100, fmt='.1f')}% | {fmt_val(minc_m, minc_s, fmt='.2f')} | {fmt_val(meanc_m, meanc_s, fmt='.2f')} |")

        lines.append("")

    lines.append("---")
    lines.append("")
    lines.append("## 实验 2：Controlled Occlusion + Re-ID 深度实验")
    lines.append("")

    # Summary table at 2.0s occlusion
    lines.append("### 结果表格 (Occlusion = 2.0 s)")
    lines.append("")
    lines.append("| Method | IDSW ↓ | Re-ID Success ↑ | Prediction RMSE after reappearance (m) ↓ |")
    lines.append("|---|---:|---:|---:|")

    for method in METHOD_ORDER_EXP2:
        if method not in exp2_agg or 2.0 not in exp2_agg[method]:
            continue
        d = exp2_agg[method][2.0]
        label = METHOD_LABELS.get(method, method)
        lines.append(f"| {label} | {fmt_val(d['idsw_mean'], d['idsw_std'], fmt='.1f')} | {fmt_val(d['reid_mean'], d['reid_std'], fmt='.1%', scale=100)}% | {fmt_val(d['pred_rmse_mean'], d['pred_rmse_std'], fmt='.3f')} |")

    lines.append("")
    lines.append("### 图表")
    lines.append("")
    lines.append("![ID Switch vs Occlusion Duration](exp2_idsw_vs_duration.png)")
    lines.append("")
    lines.append("![Re-ID Success Rate](exp2_reid_success.png)")
    lines.append("")
    lines.append("![Prediction RMSE after Reappearance](exp2_prediction_after_reappearance.png)")
    lines.append("")

    # Full table across all durations for our method
    lines.append("### Ours Full 在不同遮挡时长下的表现")
    lines.append("")
    lines.append("| Occlusion Duration | IDSW ↓ | Re-ID Success ↑ | Prediction RMSE ↓ |")
    lines.append("|---:|---:|---:|---:|")

    if "ours_full" in exp2_agg:
        for dur in DURATION_ORDER:
            if dur not in exp2_agg["ours_full"]:
                continue
            d = exp2_agg["ours_full"][dur]
            lines.append(f"| {dur:.1f} s | {fmt_val(d['idsw_mean'], d['idsw_std'], fmt='.1f')} | {fmt_val(d['reid_mean'], d['reid_std'], fmt='.1%', scale=100)}% | {fmt_val(d['pred_rmse_mean'], d['pred_rmse_std'], fmt='.3f')} |")

    lines.append("")

    # Full per-duration detail tables
    lines.append("### 各遮挡时长完整结果")
    lines.append("")

    for dur in DURATION_ORDER:
        lines.append(f"#### Occlusion = {dur:.1f} s")
        lines.append("")
        lines.append("| Method | IDSW | Re-ID Success | Pred RMSE |")
        lines.append("|---|---:|---:|---:|")

        for method in METHOD_ORDER_EXP2:
            if method not in exp2_agg or dur not in exp2_agg[method]:
                continue
            d = exp2_agg[method][dur]
            label = METHOD_LABELS.get(method, method)
            lines.append(f"| {label} | {fmt_val(d['idsw_mean'], d['idsw_std'], fmt='.1f')} | {fmt_val(d['reid_mean'], d['reid_std'], fmt='.1%', scale=100)}% | {fmt_val(d['pred_rmse_mean'], d['pred_rmse_std'], fmt='.3f')} |")

        lines.append("")

    return "\n".join(lines)


# ══════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════

def main():
    os.makedirs(FIG_DIR, exist_ok=True)

    # Experiment 1
    print("=" * 60)
    print("Experiment 1: Tracking-to-Planning")
    print("=" * 60)
    exp1_agg = process_exp1()

    if exp1_agg:
        plot_exp1_collision_clearance(exp1_agg)
        plot_exp1_prediction_rmse(exp1_agg)
        print("Experiment 1 figures done.")
    else:
        print("WARNING: No Experiment 1 data found. Run the experiment first.")

    # Experiment 2
    print("=" * 60)
    print("Experiment 2: Controlled Occlusion + Re-ID")
    print("=" * 60)
    exp2_agg = process_exp2()

    if exp2_agg:
        plot_exp2_idsw(exp2_agg)
        plot_exp2_reid_success(exp2_agg)
        plot_exp2_prediction_rmse(exp2_agg)
        print("Experiment 2 figures done.")
    else:
        print("WARNING: No Experiment 2 data found. Run the experiment first.")

    # Generate markdown
    if exp1_agg or exp2_agg:
        md = generate_markdown(exp1_agg, exp2_agg)
        md_path = os.path.join(FIG_DIR, "sup_exp.md")
        with open(md_path, "w") as f:
            f.write(md)
        print(f"Markdown report saved to {md_path}")

    print("Done.")


if __name__ == "__main__":
    main()