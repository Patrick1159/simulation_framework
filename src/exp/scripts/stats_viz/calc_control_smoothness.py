#!/usr/bin/env python3
"""
Compute control smoothness metrics from a cmd_vel CSV.

Input: cmd_vel.csv with columns timestamp,vx,vy,w

Outputs 5 metrics:
  1. mean|delta-v|   -- mean absolute frame-to-frame change in linear speed (m/s per step)
  2. mean|delta-w|   -- mean absolute frame-to-frame change in angular velocity
  3. max|w|          -- maximum absolute angular velocity (peak turning)
  4. stop_time       -- total time (seconds) where |v| < 0.01 m/s
  5. osc_count       -- number of sign changes in w (angular velocity zero-crossings)

Usage:
  python3 calc_control_smoothness.py --csv cmd_vel.csv
  python3 calc_control_smoothness.py --csv cmd_vel.csv --json out.json
"""

import argparse
import csv
import json
import math


def compute(csv_path):
    timestamps = []
    v, w = [], []

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamps.append(float(row["timestamp"]))
            # extract_cmd_vel.py writes a single linear-speed column "v";
            # older CSVs used separate vx/vy. Support both.
            if "v" in row and row["v"] != "":
                v.append(abs(float(row["v"])))
            else:
                vx = float(row["vx"])
                vy = float(row["vy"])
                v.append(math.sqrt(vx * vx + vy * vy))
            w.append(float(row["w"]))

    n = len(v)
    if n < 2:
        return {
            "mean_abs_delta_v": 0.0,
            "mean_abs_delta_w": 0.0,
            "max_abs_w": 0.0,
            "stop_time": 0.0,
            "osc_count": 0,
            "num_samples": n,
        }

    # 1. mean absolute delta-v (frame-to-frame change in linear speed)
    dv = [abs(v[i] - v[i - 1]) for i in range(1, n)]
    mean_abs_delta_v = sum(dv) / len(dv)

    # 2. mean absolute delta-w (frame-to-frame change in angular velocity)
    dw = [abs(w[i] - w[i - 1]) for i in range(1, n)]
    mean_abs_delta_w = sum(dw) / len(dw)

    # 3. max absolute w
    max_abs_w = max(abs(wi) for wi in w)

    # 4. stop time: total duration where |v| < 0.01 m/s
    #    Accumulate dt for each frame where robot is nearly stationary
    stop_time = 0.0
    for i in range(1, n):
        if v[i - 1] < 0.01:
            dt = timestamps[i] - timestamps[i - 1]
            if dt > 0:
                stop_time += dt
    # Check last frame
    if v[-1] < 0.01 and n >= 2:
        dt = timestamps[-1] - timestamps[-2]
        if dt > 0:
            stop_time += dt

    # 5. oscillation count: sign changes in w (ignoring zero)
    osc_count = 0
    prev_nonzero_w = None
    for wi in w:
        if abs(wi) < 1e-9:
            continue  # skip zero; doesn't count as a sign change
        if prev_nonzero_w is not None:
            if prev_nonzero_w * wi < 0:
                osc_count += 1
        prev_nonzero_w = wi

    return {
        "mean_abs_delta_v": mean_abs_delta_v,
        "mean_abs_delta_w": mean_abs_delta_w,
        "max_abs_w": max_abs_w,
        "stop_time": stop_time,
        "osc_count": osc_count,
        "num_samples": n,
        "duration": timestamps[-1] - timestamps[0] if timestamps else 0.0,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Compute control smoothness metrics from a cmd_vel CSV."
    )
    parser.add_argument("--csv", required=True, help="Path to cmd_vel.csv")
    parser.add_argument("--json", default=None, help="Optional JSON output path")
    parser.add_argument("--out-csv", default=None,
                        help="Optional single-row metrics CSV output path")
    args = parser.parse_args()

    metrics = compute(args.csv)

    print(f"File: {args.csv}")
    print(f"  Samples: {metrics['num_samples']}")
    print(f"  Duration: {metrics['duration']:.2f} s")
    print(f"  mean|delta-v|:   {metrics['mean_abs_delta_v']:.6f} m/s per step")
    print(f"  mean|delta-w|:   {metrics['mean_abs_delta_w']:.6f} rad/s per step")
    print(f"  max|w|:          {metrics['max_abs_w']:.6f} rad/s")
    print(f"  stop_time:       {metrics['stop_time']:.3f} s")
    print(f"  osc_count:       {metrics['osc_count']}")

    if args.json:
        with open(args.json, "w") as f:
            json.dump(metrics, f, indent=2)
        print(f"  -> written to {args.json}")

    if args.out_csv:
        with open(args.out_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(metrics.keys()))
            writer.writeheader()
            writer.writerow(metrics)
        print(f"  -> written to {args.out_csv}")


if __name__ == "__main__":
    main()
