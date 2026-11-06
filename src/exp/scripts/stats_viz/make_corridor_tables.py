#!/usr/bin/env python3
"""Build final corridor planning and runtime tables from per-run CSVs."""

import argparse
import csv
import math
import os
from collections import defaultdict


METHOD_LABELS = {
    "dodt": "DODT + MPC",
    "fapp": "FAPP + MPC",
    "ours_no_clip": "Ours w/o CLIP + MPC",
    "ours_no_sam": "Ours w/o SAM + MPC",
    "ours_full": "Ours full + MPC",
}

DEFAULT_METHODS = ["dodt", "fapp", "ours_no_clip", "ours_no_sam", "ours_full"]


def read_single_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return rows[0] if rows else {}


def as_float(row, key):
    try:
        value = row.get(key, "")
        if value == "":
            return float("nan")
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def finite(values):
    return [v for v in values if math.isfinite(v)]


def mean(values):
    vals = finite(values)
    return sum(vals) / len(vals) if vals else float("nan")


def weighted_mean(pairs):
    usable = [(v, w) for v, w in pairs if math.isfinite(v) and w > 0]
    total_w = sum(w for _, w in usable)
    if total_w <= 0:
        return float("nan")
    return sum(v * w for v, w in usable) / total_w


def fmt(value, digits=2):
    if not math.isfinite(value):
        return ""
    return f"{value:.{digits}f}"


def discover_run_dirs(root):
    run_dirs = []
    for dirpath, _, filenames in os.walk(root):
        if "planning_metrics.csv" in filenames:
            run_dirs.append(dirpath)
    return sorted(run_dirs)


def method_from_run_dir(root, run_dir):
    rel = os.path.relpath(run_dir, root)
    parts = rel.split(os.sep)
    if len(parts) >= 3:
        return parts[1]
    return ""


def collect_planning(root, methods):
    rows_by_method = defaultdict(list)
    for run_dir in discover_run_dirs(root):
        method = method_from_run_dir(root, run_dir)
        if method not in methods:
            continue
        rows_by_method[method].append(read_single_csv(os.path.join(run_dir, "planning_metrics.csv")))
    return rows_by_method


def build_planning_table(root, methods):
    rows_by_method = collect_planning(root, methods)
    table = []
    for method in methods:
        rows = rows_by_method.get(method, [])
        success = [as_float(row, "goal_success") for row in rows]
        success_rate = 100.0 * mean(success)
        collision_events = [as_float(row, "collision_events") for row in rows]
        min_dist = [as_float(row, "min_clearance_m") for row in rows]
        time_to_goal = [as_float(row, "time_to_goal_s") for row in rows]
        path_len = [as_float(row, "path_length_to_goal_m") for row in rows]
        mpc_violations = [as_float(row, "mpc_violations") for row in rows]
        near_col_events = [as_float(row, "near_collision_events") for row in rows]
        near_col_rate = [as_float(row, "near_collision_rate") for row in rows]
        table.append({
            "Method": METHOD_LABELS.get(method, method),
            "Success ↑": fmt(success_rate, 1),
            "Collision ↓": fmt(mean(collision_events), 2),
            "Near Collisions ↓": fmt(mean(near_col_events), 2),
            "Near Coll. Rate ↓": fmt(mean(near_col_rate) * 100, 1),
            "Min Dist. ↑": fmt(mean(min_dist), 3),
            "Time-to-goal ↓": fmt(mean(time_to_goal), 2),
            "Path Len. ↓": fmt(mean(path_len), 2),
            "MPC Violations ↓": fmt(mean(mpc_violations), 2),
            "Runs": str(len(rows)),
        })
    return table


def collect_runtime(root, methods):
    grouped = defaultdict(list)
    for dirpath, _, filenames in os.walk(root):
        if "runtime_table.csv" not in filenames:
            continue
        method = method_from_run_dir(root, dirpath)
        if method not in methods:
            continue
        with open(os.path.join(dirpath, "runtime_table.csv")) as f:
            for row in csv.DictReader(f):
                module = row.get("module", "")
                if module:
                    grouped[(method, module)].append(row)
    return grouped


def build_runtime_table(root, methods):
    grouped = collect_runtime(root, methods)
    table = []
    for method in methods:
        modules = sorted(module for m, module in grouped if m == method)
        for module in modules:
            rows = grouped[(method, module)]
            sample_counts = [int(float(row.get("samples", 0) or 0)) for row in rows]
            means = [(as_float(row, "mean_ms"), samples) for row, samples in zip(rows, sample_counts)]
            p95s = [(as_float(row, "p95_ms"), samples) for row, samples in zip(rows, sample_counts)]
            table.append({
                "Method": METHOD_LABELS.get(method, method),
                "Module": module,
                "Mean (ms)": fmt(weighted_mean(means), 3),
                "P95 (ms)": fmt(weighted_mean(p95s), 3),
                "Samples": str(sum(sample_counts)),
            })
    return table


def write_csv(path, rows, fields=None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if fields is None:
        fields = []
        for row in rows:
            for key in row:
                if key not in fields:
                    fields.append(key)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_markdown(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not rows:
        with open(path, "w") as f:
            f.write("_No rows._\n")
        return
    fields = list(rows[0].keys())
    with open(path, "w") as f:
        f.write("| " + " | ".join(fields) + " |\n")
        f.write("| " + " | ".join(["---"] * len(fields)) + " |\n")
        for row in rows:
            f.write("| " + " | ".join(str(row.get(field, "")) for field in fields) + " |\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--methods", nargs="+", default=DEFAULT_METHODS)
    args = parser.parse_args()

    output_dir = args.output_dir or args.root
    planning = build_planning_table(args.root, args.methods)
    runtime = build_runtime_table(args.root, args.methods)

    planning_csv = os.path.join(output_dir, "planning_level_table.csv")
    planning_md = os.path.join(output_dir, "planning_level_table.md")
    runtime_csv = os.path.join(output_dir, "module_runtime_table.csv")
    runtime_md = os.path.join(output_dir, "module_runtime_table.md")
    write_csv(
        planning_csv,
        planning,
        fields=[
            "Method",
            "Success ↑",
            "Collision ↓",
            "Near Collisions ↓",
            "Near Coll. Rate ↓",
            "Min Dist. ↑",
            "Time-to-goal ↓",
            "Path Len. ↓",
            "MPC Violations ↓",
            "Runs",
        ],
    )
    write_markdown(planning_md, planning)
    write_csv(
        runtime_csv,
        runtime,
        fields=["Method", "Module", "Mean (ms)", "P95 (ms)", "Samples"],
    )
    write_markdown(runtime_md, runtime)

    print("Wrote corridor tables:")
    print(f"  {planning_csv}")
    print(f"  {planning_md}")
    print(f"  {runtime_csv}")
    print(f"  {runtime_md}")


if __name__ == "__main__":
    main()
