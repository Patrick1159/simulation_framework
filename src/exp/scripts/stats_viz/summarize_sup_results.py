#!/usr/bin/env python3
"""
Aggregate per-run supplemental experiment CSV metrics into summary tables.

The expected directory layout is:
  data/<experiment>/<scenario-or-duration>/<method>/run_<n>/<metric>.csv

This script is intentionally generic; it can aggregate planning, tracking,
prediction, or occlusion metric CSVs as long as each file has a header row.
"""

import argparse
import csv
import os


DEFAULT_METRICS = [
    "metrics.csv",
    "prediction_error.csv",
    "planning_metrics.csv",
    "occlusion_reid_metrics.csv",
    "occlusion_prediction_error.csv",
    "occlusion_identity_events.csv",
]


def all_csv_rows(path):
    """Return all rows from a CSV file (may be multi-row, e.g. prediction_error.csv)."""
    with open(path) as f:
        return list(csv.DictReader(f))


def discover(root, metric_name):
    matches = []
    for dirpath, _, filenames in os.walk(root):
        if metric_name in filenames:
            matches.append(os.path.join(dirpath, metric_name))
    return sorted(matches)


def metadata_from_path(root, path):
    run_dir = os.path.dirname(path)
    rel = os.path.relpath(run_dir, root)
    parts = rel.split(os.sep)
    meta = {
        "result_dir": run_dir,
        "group": parts[0] if len(parts) > 0 else "",
        "method": parts[1] if len(parts) > 1 else "",
        "run": parts[2] if len(parts) > 2 else "",
    }
    if len(parts) > 3:
        meta["extra"] = "/".join(parts[3:])
    return meta


def write_summary(rows, output):
    os.makedirs(os.path.dirname(output), exist_ok=True)
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with open(output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def aggregate_metric(root, metric_name, output_dir):
    rows = []
    for path in discover(root, metric_name):
        meta = metadata_from_path(root, path)
        csv_rows = all_csv_rows(path)
        for csv_row in csv_rows:
            merged = dict(meta)
            merged.update(csv_row)
            rows.append(merged)
    if not rows:
        return None
    stem = os.path.splitext(metric_name)[0]
    output = os.path.join(output_dir, f"summary_{stem}.csv")
    write_summary(rows, output)
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--metrics", nargs="+", default=DEFAULT_METRICS)
    args = parser.parse_args()

    output_dir = args.output_dir or args.root
    written = []
    for metric in args.metrics:
        out = aggregate_metric(args.root, metric, output_dir)
        if out:
            written.append(out)

    if written:
        print("Wrote summaries:")
        for path in written:
            print(f"  {path}")
    else:
        print(f"No metric CSVs found under {args.root}")


if __name__ == "__main__":
    main()
