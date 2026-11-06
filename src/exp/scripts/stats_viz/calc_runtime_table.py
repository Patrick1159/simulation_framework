#!/usr/bin/env python3
"""Aggregate per-module runtime statistics from an experiment rosbag."""

import argparse
import csv
import math
import os
from collections import defaultdict

import rosbag


DOA_STAT_TOPICS = {
    "/rematch_statistics_sam": "SAM",
    "/rematch_statistics_clip": "CLIP",
    "/rematch_statistics_aggregator": "Aggregator",
}


def finite(values):
    return [v for v in values if math.isfinite(v)]


def mean(values):
    vals = finite(values)
    return sum(vals) / len(vals) if vals else float("nan")


def std(values):
    vals = finite(values)
    if len(vals) < 2:
        return 0.0 if vals else float("nan")
    mu = mean(vals)
    return math.sqrt(sum((v - mu) ** 2 for v in vals) / (len(vals) - 1))


def percentile(values, q):
    vals = sorted(finite(values))
    if not vals:
        return float("nan")
    idx = (len(vals) - 1) * q
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return vals[lo]
    return vals[lo] + (vals[hi] - vals[lo]) * (idx - lo)


def sec_field(msg, name):
    return float(getattr(msg, name, float("nan"))) * 1000.0


def ms_field(msg, name):
    return float(getattr(msg, name, float("nan")))


def add_record(records, module, topic, total_ms, **parts):
    if not math.isfinite(total_ms):
        return
    row = {"module": module, "source_topic": topic, "total_ms": total_ms}
    row.update(parts)
    records[module].append(row)


def collect_records(bag_path, topics):
    records = defaultdict(list)
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, _ in bag.read_messages(topics=topics):
            if topic == "/mpc/status":
                add_record(
                    records,
                    "MPC solve",
                    topic,
                    ms_field(msg, "solve_time_ms"),
                    success=1.0 if bool(getattr(msg, "success", False)) else 0.0,
                )
            elif topic == "/rematch_statistics":
                add_record(
                    records,
                    "Rematch",
                    topic,
                    ms_field(msg, "t_total_ms"),
                    parse_ms=ms_field(msg, "t_parse_ms"),
                    cost_matrix_ms=ms_field(msg, "t_cost_matrix_ms"),
                    association_ms=ms_field(msg, "t_association_ms"),
                    update_ms=ms_field(msg, "t_update_ms"),
                    publish_vis_ms=ms_field(msg, "t_publish_vis_ms"),
                    publish_det_ms=ms_field(msg, "t_publish_det_ms"),
                )
            elif topic in DOA_STAT_TOPICS:
                module = DOA_STAT_TOPICS[topic]
                add_record(
                    records,
                    module,
                    topic,
                    sec_field(msg, "total_time"),
                    model_ms=sec_field(msg, "model_time"),
                    preprocess_ms=sec_field(msg, "preprocess_time"),
                    postprocess_ms=sec_field(msg, "postprocess_time"),
                    avg_model_per_obstacle_ms=sec_field(msg, "avg_model_time_per_obstacle"),
                    sync_wait_ms=sec_field(msg, "sync_wait_time"),
                    e2e_latency_ms=sec_field(msg, "e2e_latency"),
                    obstacle_count=float(getattr(msg, "obstacle_count", float("nan"))),
                )
    return records


def summarize_records(records):
    rows = []
    part_fields = [
        "model_ms",
        "preprocess_ms",
        "postprocess_ms",
        "avg_model_per_obstacle_ms",
        "sync_wait_ms",
        "e2e_latency_ms",
        "parse_ms",
        "cost_matrix_ms",
        "association_ms",
        "update_ms",
        "publish_vis_ms",
        "publish_det_ms",
        "success",
        "obstacle_count",
    ]
    for module in sorted(records):
        samples = records[module]
        totals = [row["total_ms"] for row in samples]
        out = {
            "module": module,
            "source_topic": samples[0]["source_topic"] if samples else "",
            "samples": len(samples),
            "mean_ms": mean(totals),
            "std_ms": std(totals),
            "p50_ms": percentile(totals, 0.50),
            "p95_ms": percentile(totals, 0.95),
            "max_ms": max(finite(totals)) if finite(totals) else float("nan"),
        }
        for field in part_fields:
            values = [row[field] for row in samples if field in row]
            if values:
                out[f"mean_{field}"] = mean(values)
        rows.append(out)
    return rows


def write_csv(path, rows):
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    if not fields:
        fields = [
            "module",
            "source_topic",
            "samples",
            "mean_ms",
            "std_ms",
            "p50_ms",
            "p95_ms",
            "max_ms",
        ]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--topics",
        nargs="+",
        default=[
            "/mpc/status",
            "/rematch_statistics",
            "/rematch_statistics_sam",
            "/rematch_statistics_clip",
            "/rematch_statistics_aggregator",
        ],
    )
    args = parser.parse_args()

    records = collect_records(args.bag, args.topics)
    rows = summarize_records(records)
    output = os.path.join(args.output_dir, "runtime_table.csv")
    write_csv(output, rows)
    print(f"Wrote runtime table: {output}")
    for row in rows:
        print(
            "  {module}: mean={mean_ms:.3f} ms p95={p95_ms:.3f} ms n={samples}".format(
                **row
            )
        )


if __name__ == "__main__":
    main()
