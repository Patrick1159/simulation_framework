#!/usr/bin/env python3
"""
parse_tmpc_log.py — Parse T-MPC (mpc_planner_jackalsimulator) ROS console logs.

Summarizes planning statistics from a roslaunch log file or captured stdout.
Handles both the per-node log format (~/.ros/log/<uuid>/mpc_planner-*.log)
and the merged master.log.

Usage:
  python3 parse_tmpc_log.py <log_file> [--json] [--csv out.csv]

Output sections:
  1. Run overview        — node lifecycle, experiments, total iterations
  2. Solve success rate  — success/failure counts and ratio
  3. Failure reasons     — breakdown of MPC failure causes
  4. Timing statistics   — planning + optimization (mean, p50, p99, max)
  5. Data readiness      — missing-data warnings, obstacle counts
  6. Per-experiment      — per-experiment success rate + timing
  7. Benchmark summary   — final BENCHMARKERS.print() output
"""

import argparse
import json
import os
import re
import sys
from collections import defaultdict


# ── Patterns ──────────────────────────────────────────────────────────────

# ROS log formats (two variants):
#   Terminal (output="screen"):
#     [ INFO] [1234567890.123456]: message
#     [WARN]  [1234567890.123456]: message
#   Log file (~/.ros/log/.../*.log):
#     [/node_name][INFO] 2026-05-24 22:29:36,894: message
#     [ros.roscpp][WARN] 2026-05-24 22:29:36.894: message

ROS_LINE_TERMINAL = re.compile(
    r"^\[\s*(INFO|WARN|ERROR|DEBUG|FATAL)\s*\]\s*"
    r"\[(\d+\.?\d*)\]\s*:\s*"
    r"(.*)$"
)

ROS_LINE_FILE = re.compile(
    r"^\[[^\]]*\]\[(INFO|WARN|ERROR|DEBUG|FATAL)\]\s*"
    r"(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}[.,]\d+)\s*:\s*"
    r"(.*)$"
)

# LOG_MARK: file.cpp:123 message  (debug_output must be true)
# Strip ANSI escapes before matching
LOG_MARK_PAT = re.compile(r"^.*?\.(?:cpp|h):\d+\s+(.*)$")

# Key messages
STARTUP_PAT   = re.compile(r"Started (?:Jackal|Dingo)\s+Planner")
SHUTDOWN_PAT  = re.compile(r"Stopped (?:Jackal|Dingo)\s+Planner")
EXPERIMENT_PAT = re.compile(r"Starting experiment\s+(\d+)\s*/\s*(\d+)")
COMPLETED_PAT  = re.compile(r"Completed\s+(\d+)\s+experiments")
LOOP_PAT       = re.compile(r"=+\s*Loop\s*=+")
RESET_PAT      = re.compile(r"Resetting")

# Success / failure
SUCCESS_PAT = re.compile(r"Success:\s*(\d+)")
MPC_FAIL_PAT = re.compile(r"MPC failed:\s*(.+)$")

# Timing (from data_saver or LOG_WARN)
PLANNING_TOO_LONG_PAT = re.compile(
    r"Planning took too long:\s*([\d.]+)\s*ms"
)
# Data saver lines (when recording enabled):
#   runtime_control_loop: <value>
#   runtime_optimization: <value>
#   status: <value>
RUNTIME_CONTROL_PAT = re.compile(r"runtime_control_loop:\s*([\d.]+)")
RUNTIME_OPTIM_PAT   = re.compile(r"runtime_optimization:\s*([\d.]+)")
STATUS_PAT          = re.compile(r"status:\s*([\d.]+)")

# Data readiness
DATA_NOT_READY_PAT = re.compile(r"Data is not ready,\s*missing\s*(.+?)(?:\b\s*)?$")
OBSTACLE_COUNT_PAT = re.compile(
    r"Received\s+(\d+)\s*[>]\s*(\d+)"  # "Received N > M ..."
)
NOT_EXPORTING_PAT  = re.compile(r"Not exporting data:\s*(.+)")

# Benchmarker output (at shutdown)
BENCH_TIMING_PAT = re.compile(r"Timing\s+of:\s*(\S+)")
BENCH_AVG_PAT    = re.compile(r"Average\s+\(ms\):\s*([\d.]+)")
BENCH_MAX_PAT    = re.compile(r"Max\s+\(ms\):\s*([\d.]+)")

# Goal / path
GOAL_RECEIVED_PAT = re.compile(r"Goal callback")
PATH_RECEIVED_PAT  = re.compile(r"Path callback")

# ── ANSI stripping ────────────────────────────────────────────────────────

ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")


def strip_ansi(s: str) -> str:
    return ANSI_ESCAPE.sub("", s)


# ── Parser ────────────────────────────────────────────────────────────────

class TMPCLogParser:
    def __init__(self):
        # Lifecycle
        self.started = False
        self.stopped = False

        # Experiments
        self.total_experiments = 0
        self.current_experiment = 0
        self.experiment_boundaries = []  # [(exp_num, line_idx)]

        # Solve stats
        self.total_loops = 0
        self.success_count = 0
        self.failure_count = 0
        self.failure_reasons = defaultdict(int)

        # Timing (from runtime_control_loop / runtime_optimization in data saver,
        # or parsed from Planning took too long warnings)
        self.planning_times = []   # ms
        self.optim_times = []      # ms
        self.planning_timeouts = 0  # count of "Planning took too long"

        # Data readiness
        self.data_not_ready_count = 0
        self.missing_data_reasons = defaultdict(int)
        self.not_exporting_reasons = defaultdict(int)
        self.obstacle_count_warnings = []  # [(received, max)]

        # Benchmarks (from shutdown output)
        self.benchmarks = {}  # name -> {avg, max}

        # Raw log lines (for CSV export)
        self.raw_entries = []  # list of dicts for per-iteration export

        # Current state tracking
        self._current_loop_success = None
        self._current_loop_planning_time = None
        self._current_loop_optim_time = None

    def parse_file(self, filepath: str):
        with open(filepath, "r") as f:
            for line_idx, line in enumerate(f, 1):
                self._process_line(line.strip(), line_idx)

    def parse_string(self, text: str):
        for line_idx, line in enumerate(text.splitlines(), 1):
            self._process_line(line.strip(), line_idx)

    def _process_line(self, line: str, line_idx: int):
        # Try terminal format first, then log-file format
        m = ROS_LINE_TERMINAL.match(line)
        if m:
            level = m.group(1)
            timestamp = float(m.group(2))
            msg = m.group(3)
        else:
            m = ROS_LINE_FILE.match(line)
            if m:
                level = m.group(1)
                # Parse human-readable timestamp as epoch-like float for ordering
                ts_str = m.group(2).replace(",", ".")  # normalize decimal sep
                try:
                    from datetime import datetime
                    timestamp = datetime.strptime(
                        ts_str, "%Y-%m-%d %H:%M:%S.%f"
                    ).timestamp()
                except ValueError:
                    timestamp = float(line_idx)  # fallback: use line number
                msg = m.group(3)
            else:
                return  # not a ROS log line

        msg_clean = strip_ansi(msg).strip()

        # Lifecycle
        if STARTUP_PAT.search(msg_clean):
            self.started = True
            self._flush_loop_entry()
            return

        if SHUTDOWN_PAT.search(msg_clean):
            self.stopped = True
            self._flush_loop_entry()
            return

        if EXPERIMENT_PAT.search(msg_clean):
            match = EXPERIMENT_PAT.search(msg_clean)
            self.current_experiment = int(match.group(1))
            self.total_experiments = int(match.group(2))
            self.experiment_boundaries.append(
                (self.current_experiment, line_idx, timestamp)
            )
            self._flush_loop_entry()
            return

        if COMPLETED_PAT.search(msg_clean):
            self._flush_loop_entry()
            return

        # Loop boundaries (debug_output mode)
        if LOOP_PAT.search(msg_clean):
            self._flush_loop_entry()
            self.total_loops += 1
            self._current_loop_success = None
            self._current_loop_planning_time = None
            self._current_loop_optim_time = None
            return

        if RESET_PAT.search(msg_clean):
            self._flush_loop_entry()
            return

        # Success / failure
        if SUCCESS_PAT.search(msg_clean):
            val = int(SUCCESS_PAT.search(msg_clean).group(1))
            self._current_loop_success = val
            if val == 1:
                self.success_count += 1
            else:
                self.failure_count += 1
            return

        if MPC_FAIL_PAT.search(msg_clean):
            reason = MPC_FAIL_PAT.search(msg_clean).group(1).strip()
            self.failure_reasons[reason] += 1
            return

        # Timing
        if PLANNING_TOO_LONG_PAT.search(msg_clean):
            t = float(PLANNING_TOO_LONG_PAT.search(msg_clean).group(1))
            self.planning_timeouts += 1
            # Also record as a planning time sample
            self._current_loop_planning_time = t
            return

        if RUNTIME_CONTROL_PAT.search(msg_clean):
            t = float(RUNTIME_CONTROL_PAT.search(msg_clean).group(1)) * 1000.0
            self._current_loop_planning_time = t
            return

        if RUNTIME_OPTIM_PAT.search(msg_clean):
            t = float(RUNTIME_OPTIM_PAT.search(msg_clean).group(1)) * 1000.0
            self._current_loop_optim_time = t
            return

        if STATUS_PAT.search(msg_clean):
            # status: 2 = success, 3 = failure  (from saveData)
            # This is from data saver, not console
            return

        # Data readiness
        if DATA_NOT_READY_PAT.search(msg_clean):
            reason = DATA_NOT_READY_PAT.search(msg_clean).group(1).strip()
            self.data_not_ready_count += 1
            self.missing_data_reasons[reason] += 1
            return

        if OBSTACLE_COUNT_PAT.search(msg_clean):
            m2 = OBSTACLE_COUNT_PAT.search(msg_clean)
            self.obstacle_count_warnings.append(
                (int(m2.group(1)), int(m2.group(2)))
            )
            return

        if NOT_EXPORTING_PAT.search(msg_clean):
            reason = NOT_EXPORTING_PAT.search(msg_clean).group(1).strip()
            self.not_exporting_reasons[reason] += 1
            return

        # Benchmark output (shutdown)
        if BENCH_TIMING_PAT.search(msg_clean):
            self._current_bench_name = BENCH_TIMING_PAT.search(msg_clean).group(1)
            return

        if BENCH_AVG_PAT.search(msg_clean):
            avg = float(BENCH_AVG_PAT.search(msg_clean).group(1))
            name = getattr(self, "_current_bench_name", "unknown")
            self.benchmarks.setdefault(name, {})["avg"] = avg
            return

        if BENCH_MAX_PAT.search(msg_clean):
            bmax = float(BENCH_MAX_PAT.search(msg_clean).group(1))
            name = getattr(self, "_current_bench_name", "unknown")
            self.benchmarks.setdefault(name, {})["max"] = bmax
            return

    def _flush_loop_entry(self):
        if self._current_loop_success is not None:
            entry = {
                "loop": self.total_loops,
                "success": self._current_loop_success,
                "planning_time_ms": self._current_loop_planning_time,
                "optim_time_ms": self._current_loop_optim_time,
            }
            self.raw_entries.append(entry)

            if self._current_loop_planning_time is not None:
                self.planning_times.append(self._current_loop_planning_time)
            if self._current_loop_optim_time is not None:
                self.optim_times.append(self._current_loop_optim_time)

        self._current_loop_success = None
        self._current_loop_planning_time = None
        self._current_loop_optim_time = None


# ── Report ────────────────────────────────────────────────────────────────

def percentile(sorted_vals, p):
    if not sorted_vals:
        return None
    idx = int(len(sorted_vals) * p / 100.0)
    return sorted_vals[min(idx, len(sorted_vals) - 1)]


def format_timing(times):
    if not times:
        return {"mean": None, "p50": None, "p99": None, "max": None,
                "min": None, "std": None, "count": 0}
    s = sorted(times)
    mean = sum(s) / len(s)
    var = sum((x - mean) ** 2 for x in s) / len(s)
    return {
        "mean": mean,
        "p50":  percentile(s, 50),
        "p99":  percentile(s, 99),
        "max":  s[-1],
        "min":  s[0],
        "std":  var ** 0.5,
        "count": len(s),
    }


def print_report(p: TMPCLogParser):
    total = p.success_count + p.failure_count
    rate = (p.success_count / total * 100.0) if total > 0 else 0.0

    print("=" * 68)
    print("  T-MPC Log Analysis Report")
    print("=" * 68)

    # ── 1. Run overview ──
    print("\n── 1. Run Overview ──")
    print(f"  Started:          {'yes' if p.started else 'no'}")
    print(f"  Stopped:          {'yes' if p.stopped else 'no'}")
    print(f"  Total loops:      {p.total_loops}")
    print(f"  Total experiments:{'%d / %d' % (p.current_experiment, p.total_experiments) if p.total_experiments else 'N/A'}")

    # ── 2. Solve success rate ──
    print("\n── 2. Solve Success Rate ──")
    print(f"  Success: {p.success_count:>6d}")
    print(f"  Failure: {p.failure_count:>6d}")
    print(f"  Total:   {total:>6d}")
    print(f"  Rate:    {rate:>6.1f} %")

    if p.total_loops > 0 and total < p.total_loops:
        print(f"  (note: {p.total_loops - total} loops had no Success marker "
              f"— likely missing debug_output)")

    # ── 3. Failure reasons ──
    if p.failure_reasons:
        print("\n── 3. Failure Reasons ──")
        for reason, count in sorted(p.failure_reasons.items(),
                                    key=lambda x: -x[1]):
            pct = count / p.failure_count * 100.0 if p.failure_count else 0
            print(f"  [{count:>4d}  {pct:>5.1f}%]  {reason}")

    # ── 4. Timing ──
    pstats = format_timing(p.planning_times)
    ostats = format_timing(p.optim_times)

    print("\n── 4. Timing Statistics ──")
    if pstats["count"] > 0:
        print(f"  Planning time ({pstats['count']} samples):")
        print(f"    Mean: {pstats['mean']:>8.2f} ms")
        print(f"    P50:  {pstats['p50']:>8.2f} ms")
        print(f"    P99:  {pstats['p99']:>8.2f} ms")
        print(f"    Max:  {pstats['max']:>8.2f} ms")
        print(f"    Min:  {pstats['min']:>8.2f} ms")
        print(f"    Std:  {pstats['std']:>8.2f} ms")
        print(f"  Timeout warnings: {p.planning_timeouts}")
    else:
        print(f"  Planning time: (no samples — enable recording for per-loop data)")

    if ostats["count"] > 0:
        print(f"  Optimization time ({ostats['count']} samples):")
        print(f"    Mean: {ostats['mean']:>8.2f} ms")
        print(f"    Max:  {ostats['max']:>8.2f} ms")
    else:
        print(f"  Optimization time: (no samples)")

    # ── 5. Data readiness ──
    print("\n── 5. Data Readiness ──")
    print(f"  'Data not ready' warnings: {p.data_not_ready_count}")
    if p.missing_data_reasons:
        for reason, count in sorted(p.missing_data_reasons.items(),
                                    key=lambda x: -x[1]):
            print(f"    [{count:>4d}]  missing {reason}")

    if p.obstacle_count_warnings:
        print(f"  Obstacle overflow warnings: {len(p.obstacle_count_warnings)}")
        # Summarize: max received per warning
        max_recv = max(r for r, _ in p.obstacle_count_warnings)
        print(f"    Max obstacles received: {max_recv}")

    if p.not_exporting_reasons:
        print(f"  Export skipped: {sum(p.not_exporting_reasons.values())}")
        for reason, count in sorted(p.not_exporting_reasons.items(),
                                    key=lambda x: -x[1]):
            print(f"    [{count:>4d}]  {reason}")

    # ── 6. Per-experiment (requires experiment boundaries) ──
    if p.experiment_boundaries and p.raw_entries:
        print("\n── 6. Per-Experiment Summary ──")
        header = f"  {'Exp':>4s}  {'Success':>8s}  {'Rate':>7s}  {'Plan Mean':>10s}  {'Plan Max':>10s}"
        print(header)
        print("  " + "-" * (len(header) - 2))

        # Map raw entries to experiments by their loop number
        # (crude: split by experiment boundaries)
        boundaries = [0] + [b[0] for b in p.experiment_boundaries]
        for i in range(len(boundaries)):
            exp_num = i + 1
            lo_start = boundaries[i]
            # Find entries belonging to this experiment range
            exp_entries = [
                e for e in p.raw_entries
                if e["loop"] >= lo_start
                and (i + 1 >= len(boundaries) or e["loop"] < boundaries[i + 1])
            ]
            if not exp_entries:
                continue
            exp_success = sum(1 for e in exp_entries if e["success"] == 1)
            exp_total = len(exp_entries)
            exp_rate = exp_success / exp_total * 100.0 if exp_total else 0
            exp_times = [
                e["planning_time_ms"] for e in exp_entries
                if e["planning_time_ms"] is not None
            ]
            t_stats = format_timing(exp_times)
            mean_str = f"{t_stats['mean']:.1f}" if t_stats["mean"] else "N/A"
            max_str = f"{t_stats['max']:.1f}" if t_stats["max"] else "N/A"
            print(f"  {exp_num:>4d}  {exp_success:>4d}/{exp_total:<3d}  "
                  f"{exp_rate:>6.1f}%  {mean_str:>10s}  {max_str:>10s}")

    # ── 7. Benchmark summary ──
    if p.benchmarks:
        print("\n── 7. Benchmark Summary (from BENCHMARKERS.print()) ──")
        for name, stats in sorted(p.benchmarks.items()):
            avg = stats.get("avg", "N/A")
            bmax = stats.get("max", "N/A")
            avg_str = f"{avg:.2f} ms" if isinstance(avg, float) else str(avg)
            max_str = f"{bmax:.2f} ms" if isinstance(bmax, float) else str(bmax)
            print(f"  {name:<20s}  avg: {avg_str:>10s}  max: {max_str:>10s}")

    print("\n" + "=" * 68)


# ── CSV export ──

def export_csv(p: TMPCLogParser, csv_path: str):
    import csv
    if not p.raw_entries:
        print("[parse_tmpc_log] No per-loop data to export.", file=sys.stderr)
        return

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["loop", "success", "planning_time_ms", "optim_time_ms"]
        )
        writer.writeheader()
        writer.writerows(p.raw_entries)
    print(f"[parse_tmpc_log] Exported {len(p.raw_entries)} loop entries → {csv_path}")


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Parse T-MPC ROS console logs and summarize planning stats."
    )
    parser.add_argument(
        "log_file", help="Path to ROS log file (e.g., mpc_planner-*.log or master.log)"
    )
    parser.add_argument(
        "--csv", default=None,
        help="Export per-loop data to CSV"
    )
    parser.add_argument(
        "--json", dest="json_out", default=None,
        help="Export structured report to JSON file"
    )
    args = parser.parse_args()

    if not os.path.isfile(args.log_file):
        print(f"ERROR: file not found: {args.log_file}", file=sys.stderr)
        sys.exit(1)

    p = TMPCLogParser()
    p.parse_file(args.log_file)

    # Flush any remaining loop entry
    p._flush_loop_entry()

    print_report(p)

    if args.csv:
        export_csv(p, args.csv)

    if args.json_out:
        report = {
            "started": p.started,
            "stopped": p.stopped,
            "total_loops": p.total_loops,
            "total_experiments": p.total_experiments,
            "success_count": p.success_count,
            "failure_count": p.failure_count,
            "success_rate": (
                p.success_count / (p.success_count + p.failure_count) * 100.0
                if (p.success_count + p.failure_count) > 0 else 0.0
            ),
            "failure_reasons": dict(p.failure_reasons),
            "planning_time_ms": format_timing(p.planning_times),
            "optimization_time_ms": format_timing(p.optim_times),
            "planning_timeouts": p.planning_timeouts,
            "data_not_ready_count": p.data_not_ready_count,
            "missing_data_reasons": dict(p.missing_data_reasons),
            "obstacle_overflow_count": len(p.obstacle_count_warnings),
            "benchmarks": p.benchmarks,
        }
        with open(args.json_out, "w") as f:
            json.dump(report, f, indent=2, default=str)
        print(f"[parse_tmpc_log] JSON report → {args.json_out}")


if __name__ == "__main__":
    main()
