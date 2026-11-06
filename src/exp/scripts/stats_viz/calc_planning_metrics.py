#!/usr/bin/env python3
"""
Compute planning-safety metrics from an experiment rosbag.

Inputs:
  - /odom: robot pose
  - /obstacles: ground-truth dynamic obstacles as mpc_nav/ObstacleArray
  - /mpc/status: optional MPC solver status

Outputs in --output-dir:
  - planning_metrics.csv
  - planning_metrics.json
  - clearance_timeseries.csv
"""

import argparse
import csv
import json
import math
import os
import re
from collections import defaultdict

# rosbag / matplotlib / numpy are imported lazily (see main() and
# plot_corridor_trajectory) so the pure helpers can be imported and unit-tested
# without ROS installed.

MIN_MOTION_M = 0.5  # a run with less path than this is "static"


def stamp_to_sec(msg, bag_time):
    if hasattr(msg, "header") and msg.header.stamp.to_sec() > 0.0:
        return msg.header.stamp.to_sec()
    return bag_time.to_sec()


def obstacle_radius(obs):
    r = float(getattr(obs, "radius", 0.0) or 0.0)
    sm = float(getattr(obs, "semi_major", 0.0) or 0.0)
    sn = float(getattr(obs, "semi_minor", 0.0) or 0.0)
    return max(r, sm, sn)


def load_gt_obstacles(bag, topic):
    by_id = defaultdict(list)
    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = stamp_to_sec(msg, t)
        for obs in msg.obstacles:
            oid = int(obs.id)
            by_id[oid].append({
                "t": ts,
                "x": float(obs.position.x),
                "y": float(obs.position.y),
                "vx": float(obs.velocity.x),
                "vy": float(obs.velocity.y),
                "radius": obstacle_radius(obs),
            })
    for points in by_id.values():
        points.sort(key=lambda p: p["t"])
    return by_id


def load_odom(bag, topic):
    rows = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = stamp_to_sec(msg, t)
        pos = msg.pose.pose.position
        rows.append({"t": ts, "x": float(pos.x), "y": float(pos.y)})
    rows.sort(key=lambda p: p["t"])
    return rows


def load_mpc_status(bag, topic):
    rows = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = stamp_to_sec(msg, t)
        rows.append({
            "t": ts,
            "success": bool(getattr(msg, "success", False)),
            "solve_time_ms": float(getattr(msg, "solve_time_ms", float("nan"))),
            "cost": float(getattr(msg, "cost", float("nan"))),
        })
    return rows


def message_item_count(msg):
    for attr in ("obstacles", "trackers", "objects_states", "states", "box_ids"):
        if hasattr(msg, attr):
            try:
                return len(getattr(msg, attr))
            except TypeError:
                return 0
    return 0


def count_topic_items(bag, topic):
    msg_count = 0
    nonempty_count = 0
    total_items = 0
    max_items = 0
    for _, msg, _ in bag.read_messages(topics=[topic]):
        n_items = message_item_count(msg)
        msg_count += 1
        total_items += n_items
        max_items = max(max_items, n_items)
        if n_items > 0:
            nonempty_count += 1
    return {
        "msgs": msg_count,
        "nonempty_msgs": nonempty_count,
        "mean_count": total_items / msg_count if msg_count else float("nan"),
        "max_count": max_items,
    }


def interp_state(points, ts):
    if not points or ts < points[0]["t"] or ts > points[-1]["t"]:
        return None
    lo = 0
    hi = len(points) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if points[mid]["t"] < ts:
            lo = mid + 1
        else:
            hi = mid - 1
    i1 = min(lo, len(points) - 1)
    i0 = max(0, i1 - 1)
    p0 = points[i0]
    p1 = points[i1]
    if p1["t"] == p0["t"]:
        return p0
    a = (ts - p0["t"]) / (p1["t"] - p0["t"])
    out = {"t": ts}
    for key in ("x", "y", "vx", "vy", "radius"):
        out[key] = p0[key] + a * (p1[key] - p0[key])
    return out


def percentile(values, q):
    if not values:
        return float("nan")
    ordered = sorted(values)
    idx = (len(ordered) - 1) * q
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (idx - lo)


def count_events(values, threshold):
    events = 0
    active = False
    for value in values:
        inside = value <= threshold
        if inside and not active:
            events += 1
        active = inside
    return events


def compute_path_length(odom_rows, end_time=None):
    if len(odom_rows) < 2:
        return 0.0
    total = 0.0
    last = odom_rows[0]
    for row in odom_rows[1:]:
        if end_time is not None and row["t"] > end_time:
            break
        total += math.hypot(row["x"] - last["x"], row["y"] - last["y"])
        last = row
    return total


def compute_goal_metrics(odom_rows, goal_x, goal_y, tolerance,
                         min_motion=MIN_MOTION_M):
    """Goal-reach metrics.

    A goal hit only counts if the robot has actually travelled at least
    `min_motion` metres along its path up to that sample. Previously the first
    odom sample within `tolerance` of the goal counted, so a robot that never
    moved (or whose first odom sample was already near the assumed goal)
    scored goal_success=1 with path_length_m=0.
    """
    if not odom_rows:
        return {
            "goal_success": 0,
            "time_to_goal_s": float("nan"),
            "goal_reach_timestamp": float("nan"),
            "path_length_m": float("nan"),
            "path_length_to_goal_m": float("nan"),
            "min_goal_distance_m": float("nan"),
            "final_goal_distance_m": float("nan"),
            "start_goal_distance_m": float("nan"),
        }

    start_goal_dist = math.hypot(odom_rows[0]["x"] - goal_x, odom_rows[0]["y"] - goal_y)
    start_t = odom_rows[0]["t"]
    hit = None
    min_dist = float("inf")
    travelled = 0.0  # running path length up to and including `row`
    prev = odom_rows[0]
    for row in odom_rows:
        travelled += math.hypot(row["x"] - prev["x"], row["y"] - prev["y"])
        prev = row
        dist = math.hypot(row["x"] - goal_x, row["y"] - goal_y)
        min_dist = min(min_dist, dist)
        if hit is None and dist <= tolerance and travelled >= min_motion:
            hit = row

    full_path = compute_path_length(odom_rows)
    if hit is None:
        return {
            "goal_success": 0,
            "time_to_goal_s": float("nan"),
            "goal_reach_timestamp": float("nan"),
            "path_length_m": full_path,
            "path_length_to_goal_m": float("nan"),
            "min_goal_distance_m": min_dist,
            "final_goal_distance_m": math.hypot(
                odom_rows[-1]["x"] - goal_x, odom_rows[-1]["y"] - goal_y
            ),
            "start_goal_distance_m": start_goal_dist,
        }

    path_to_goal = compute_path_length(odom_rows, end_time=hit["t"])
    return {
        "goal_success": 1,
        "time_to_goal_s": max(0.0, hit["t"] - start_t),
        "goal_reach_timestamp": hit["t"],
        "path_length_m": path_to_goal,
        "path_length_to_goal_m": path_to_goal,
        "min_goal_distance_m": min_dist,
        "final_goal_distance_m": math.hypot(
            odom_rows[-1]["x"] - goal_x, odom_rows[-1]["y"] - goal_y
        ),
        "start_goal_distance_m": start_goal_dist,
    }


def infer_method(run_dir):
    """Method name = parent directory of the run dir (.../<method>/run_<i>/).

    Walks up from `run_dir` to the first component named run_<i> / run<i> and
    returns its parent. If no such component exists (different layout) the
    directory name itself is returned.
    """
    path = os.path.abspath(run_dir)
    parts = path.split(os.sep)
    for i in range(len(parts) - 1, 0, -1):
        if re.fullmatch(r"run_?\d+", parts[i]):
            return parts[i - 1]
    return os.path.basename(path)


def compute_validity(metrics, method=None):
    """Return (valid, reasons) for a metrics dict.

    Reasons: static, no-planner-obs, planner-obs-always-empty, no-rematch
    (ours_* only), goal-without-motion.
    """
    reasons = []
    path = metrics.get("path_length_m")
    static = (path is None or not math.isfinite(path)) or path < MIN_MOTION_M
    if static:
        reasons.append("static")
    n_planner = metrics.get("planner_obstacle_msgs", 0) or 0
    if n_planner == 0:
        reasons.append("no-planner-obs")
    elif metrics.get("planner_obstacle_nonempty_msgs", None) == 0:
        # Messages arrived (e.g. watchdog-published empty arrays) but the
        # planner never saw a single obstacle.
        reasons.append("planner-obs-always-empty")
    if (method and str(method).startswith("ours_")
            and (metrics.get("rematch_msgs", 0) or 0) == 0):
        reasons.append("no-rematch")
    if metrics.get("goal_success") == 1 and static:
        reasons.append("goal-without-motion")
    return (not reasons), reasons


def goal_from_messages(goal_msgs):
    """Last (x, y) from a list of PoseStamped-like messages, or None."""
    if not goal_msgs:
        return None
    p = goal_msgs[-1].pose.position
    return float(p.x), float(p.y)


def compute_clearance(odom_rows, gt_by_id, robot_radius):
    rows = []
    for robot in odom_rows:
        nearest = None
        for oid, points in gt_by_id.items():
            obs = interp_state(points, robot["t"])
            if obs is None:
                continue
            dist = math.hypot(robot["x"] - obs["x"], robot["y"] - obs["y"])
            clearance = dist - robot_radius - obs["radius"]
            candidate = {
                "timestamp": robot["t"],
                "robot_x": robot["x"],
                "robot_y": robot["y"],
                "gt_id": oid,
                "obs_x": obs["x"],
                "obs_y": obs["y"],
                "obs_radius": obs["radius"],
                "clearance": clearance,
            }
            if nearest is None or clearance < nearest["clearance"]:
                nearest = candidate
        if nearest is not None:
            rows.append(nearest)
    return rows


def finite_mean(values):
    vals = [v for v in values if math.isfinite(v)]
    return sum(vals) / len(vals) if vals else float("nan")


def plot_corridor_trajectory(odom_rows, gt_by_id, clearance_rows, goal_metrics, args,
                             output_path, tracker_by_ts=None):
    """Generate BEV trajectory plot of the nav-to-goal corridor run."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    fig, ax = plt.subplots(figsize=(5, 8))

    # ── corridor walls ──
    wall_xmin = getattr(args, "wall_xmin", -1.4)
    wall_xmax = getattr(args, "wall_xmax", 1.4)
    wall_ymin = getattr(args, "wall_ymin", -0.5)
    wall_ymax = getattr(args, "wall_ymax", 8.5)
    for x in (wall_xmin, wall_xmax):
        ax.axvline(x, color="gray", linewidth=1.2, linestyle="-", alpha=0.7)
    for y in (wall_ymin, wall_ymax):
        ax.axhline(y, color="gray", linewidth=1.2, linestyle="-", alpha=0.7)

    # ── GT obstacle trajectories ──
    for oid, points in sorted(gt_by_id.items()):
        xs = [p["x"] for p in points]
        ys = [p["y"] for p in points]
        ax.plot(xs, ys, color="#999999", linewidth=0.6, alpha=0.5, linestyle="-")

    # ── tracker output trajectories (what the MPC saw) ──
    if tracker_by_ts and clearance_rows:
        tracker_points_by_id = defaultdict(list)
        for row in clearance_rows:
            ts = row["timestamp"]
            keys = sorted(tracker_by_ts.keys())
            if not keys:
                continue
            idx = min(range(len(keys)), key=lambda i: abs(keys[i] - ts))
            if abs(keys[idx] - ts) <= 0.15:
                for trk in tracker_by_ts[keys[idx]]:
                    tid = int(trk["id"])
                    tx = float(trk["x"])
                    ty = float(trk["y"])
                    tracker_points_by_id[tid].append((tx, ty))
        for tid, pts in tracker_points_by_id.items():
            if len(pts) < 2:
                continue
            xs, ys = zip(*pts)
            ax.plot(xs, ys, color="#87CEEB", linewidth=0.5, alpha=0.35,
                    linestyle=":", zorder=2)

    # ── robot path with clearance coloring ──
    if clearance_rows and odom_rows:
        clearances = [r["clearance"] for r in clearance_rows]
        t_cl = [r["timestamp"] for r in clearance_rows]
        rx = [r["robot_x"] for r in clearance_rows]
        ry = [r["robot_y"] for r in clearance_rows]
        c_arr = np.array(clearances)
        near_thresh = args.near_clearance
        col_thresh = args.collision_clearance

        # segment by clearance: red (collision), orange (near), green (safe)
        colors = np.full(len(c_arr), "#2ECC71")  # safe = green
        colors[c_arr <= near_thresh] = "#F39C12"  # near = orange
        colors[c_arr <= col_thresh] = "#E74C3C"   # collision = red

        for i in range(len(rx) - 1):
            ax.plot(rx[i:i+2], ry[i:i+2], color=colors[i], linewidth=1.5, alpha=0.8)

    # ── start and goal markers ──
    if odom_rows:
        ax.scatter(odom_rows[0]["x"], odom_rows[0]["y"], marker="o", s=80,
                   color="#3498DB", zorder=5, label="Start", edgecolors="white",
                   linewidths=0.5)
    ax.scatter(args.goal_x, args.goal_y, marker="*", s=120,
               color="#E74C3C", zorder=5, label="Goal", edgecolors="white",
               linewidths=0.5)
    goal_circle = plt.Circle((args.goal_x, args.goal_y), args.goal_tolerance,
                              fill=False, color="#E74C3C", linewidth=1,
                              linestyle="--", alpha=0.6)
    ax.add_patch(goal_circle)

    # ── legend for clearance ──
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color="#2ECC71", linewidth=2, label="Safe"),
        Line2D([0], [0], color="#F39C12", linewidth=2,
               label=f"Near ({args.near_clearance}m)"),
        Line2D([0], [0], color="#E74C3C", linewidth=2,
               label=f"Collision ({args.collision_clearance}m)"),
        Line2D([0], [0], color="#999999", linewidth=0.6,
               label="GT obstacles"),
        Line2D([0], [0], color="#87CEEB", linewidth=0.5, linestyle=":",
               label="Tracker (MPC view)"),
    ]
    ax.legend(handles=legend_elements, loc="lower left", fontsize=6,
              framealpha=0.9)

    # ── labels and layout ──
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)
    margin = 0.5
    ax.set_xlim(wall_xmin - margin, wall_xmax + margin)
    ax.set_ylim(wall_ymin - margin, wall_ymax + margin)

    # ── title with key metrics ──
    success = "Reached" if goal_metrics.get("goal_success") else "Missed"
    t2g = goal_metrics.get("time_to_goal_s", float("nan"))
    title = f"{success} goal | Time: {t2g:.1f}s" if math.isfinite(t2g) else f"{success} goal"
    ax.set_title(title, fontsize=9)

    fig.tight_layout()
    fig.savefig(output_path)
    fig.savefig(output_path.replace(".pdf", ".png"))
    plt.close(fig)
    print(f"  Saved corridor trajectory: {output_path}")


def write_single_row_csv(path, row):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        writer.writeheader()
        writer.writerow(row)


def write_clearance_csv(path, rows):
    fields = [
        "timestamp", "robot_x", "robot_y", "gt_id", "obs_x", "obs_y",
        "obs_radius", "clearance",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def load_goal(bag, topic):
    msgs = [m for _, m, _ in bag.read_messages(topics=[topic])]
    return goal_from_messages(msgs)


def main():
    import rosbag
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--gt-topic", default="/obstacles")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--status-topic", default="/mpc/status")
    parser.add_argument("--planner-topic", default="/sup_exp/planner_obstacles")
    parser.add_argument("--coarse-topic", default="/tracker/dynamic_obstacles")
    parser.add_argument("--rematch-topic", default="/rematch/tracker_details")
    parser.add_argument("--goal-topic", default="/move_base_simple/goal",
                        help="Goal read from the bag when --goal-x/--goal-y are not given.")
    parser.add_argument("--method", default=None,
                        help="Method name; default inferred from the run dir layout.")
    parser.add_argument("--robot-radius", type=float, default=0.18)
    parser.add_argument("--near-clearance", type=float, default=0.50)
    parser.add_argument("--collision-clearance", type=float, default=0.20)
    parser.add_argument("--goal-x", type=float, default=None,
                        help="Explicit goal x (wins over the bag goal; 0.0 if no goal in bag)")
    parser.add_argument("--goal-y", type=float, default=None,
                        help="Explicit goal y (wins over the bag goal; 8.0 if no goal in bag)")
    parser.add_argument("--goal-tolerance", type=float, default=0.35)
    parser.add_argument("--collision-until-goal", dest="collision_until_goal",
                        action="store_true", default=True,
                        help="Count collisions/clearance only up to goal arrival (default).")
    parser.add_argument("--collision-full-run", dest="collision_until_goal",
                        action="store_false",
                        help="Count collisions over the entire recording, including post-arrival dwell.")
    parser.add_argument("--wall-xmin", type=float, default=-1.4)
    parser.add_argument("--wall-xmax", type=float, default=1.4)
    parser.add_argument("--wall-ymin", type=float, default=-0.5)
    parser.add_argument("--wall-ymax", type=float, default=8.5)
    parser.add_argument("--no-figure", action="store_true",
                        help="Skip corridor trajectory figure generation")
    parser.add_argument("--tracker-csv", default=None,
                        help="Optional path to tracker.csv for rendering tracker outputs on the figure")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    bag = rosbag.Bag(args.bag, "r")
    gt_by_id = load_gt_obstacles(bag, args.gt_topic)
    odom_rows = load_odom(bag, args.odom_topic)
    status_rows = load_mpc_status(bag, args.status_topic)
    bag_goal = load_goal(bag, args.goal_topic)
    if args.goal_x is not None and args.goal_y is not None:
        goal_source = "cli"
    elif bag_goal is not None:
        args.goal_x, args.goal_y = bag_goal
        goal_source = "bag"
    else:
        args.goal_x, args.goal_y = 0.0, 8.0
        goal_source = "default"
    planner_counts = count_topic_items(bag, args.planner_topic)
    coarse_counts = count_topic_items(bag, args.coarse_topic)
    rematch_counts = count_topic_items(bag, args.rematch_topic)
    bag.close()

    clearance_rows = compute_clearance(odom_rows, gt_by_id, args.robot_radius)
    duration = 0.0
    if odom_rows:
        duration = max(0.0, odom_rows[-1]["t"] - odom_rows[0]["t"])

    solve_times = [row["solve_time_ms"] for row in status_rows
                   if math.isfinite(row["solve_time_ms"])]
    status_count = len(status_rows)
    success_count = sum(1 for row in status_rows if row["success"])
    mpc_violations = status_count - success_count
    goal_metrics = compute_goal_metrics(
        odom_rows,
        args.goal_x,
        args.goal_y,
        args.goal_tolerance,
    )

    # Collision/clearance stats are evaluated only up to the moment the robot
    # reaches the goal. After arrival the robot dwells near the goal while the
    # recording continues, so pedestrians passing by would otherwise be counted
    # as collisions even though the navigation task already succeeded. The full
    # clearance_rows are still written to the timeseries CSV / figure.
    reach_ts = goal_metrics.get("goal_reach_timestamp")
    if (args.collision_until_goal and reach_ts is not None
            and math.isfinite(reach_ts)):
        eval_rows = [r for r in clearance_rows if r["timestamp"] <= reach_ts]
    else:
        eval_rows = clearance_rows
    clearances = [row["clearance"] for row in eval_rows]
    eval_samples = len(clearances)
    near_samples = sum(1 for c in clearances if c <= args.near_clearance)
    collision_samples = sum(1 for c in clearances if c <= args.collision_clearance)

    metrics = {
        "bag": os.path.abspath(args.bag),
        "duration_s": duration,
        "goal_x": args.goal_x,
        "goal_y": args.goal_y,
        "goal_tolerance_m": args.goal_tolerance,
        "gt_obstacle_count": len(gt_by_id),
        "odom_samples": len(odom_rows),
        "eval_samples": eval_samples,
        "min_clearance_m": min(clearances) if clearances else float("nan"),
        "mean_clearance_m": finite_mean(clearances),
        "p05_clearance_m": percentile(clearances, 0.05),
        "near_clearance_threshold_m": args.near_clearance,
        "near_collision_samples": near_samples,
        "near_collision_events": count_events(clearances, args.near_clearance),
        "near_collision_rate": near_samples / eval_samples if eval_samples else float("nan"),
        "collision_clearance_threshold_m": args.collision_clearance,
        "collision_samples": collision_samples,
        "collision_events": count_events(clearances, args.collision_clearance),
        "collision_rate": collision_samples / eval_samples if eval_samples else float("nan"),
        "mpc_status_samples": status_count,
        "mpc_success_rate": success_count / status_count if status_count else float("nan"),
        "mpc_violations": mpc_violations,
        "mpc_violation_rate": mpc_violations / status_count if status_count else float("nan"),
        "mpc_solve_time_mean_ms": finite_mean(solve_times),
        "mpc_solve_time_p95_ms": percentile(solve_times, 0.95),
        "planner_obstacle_msgs": planner_counts["msgs"],
        "planner_obstacle_nonempty_msgs": planner_counts["nonempty_msgs"],
        "planner_obstacle_mean_count": planner_counts["mean_count"],
        "planner_obstacle_max_count": planner_counts["max_count"],
        "coarse_obstacle_msgs": coarse_counts["msgs"],
        "coarse_obstacle_nonempty_msgs": coarse_counts["nonempty_msgs"],
        "coarse_obstacle_mean_count": coarse_counts["mean_count"],
        "coarse_obstacle_max_count": coarse_counts["max_count"],
        "rematch_msgs": rematch_counts["msgs"],
        "rematch_nonempty_msgs": rematch_counts["nonempty_msgs"],
        "rematch_mean_count": rematch_counts["mean_count"],
        "rematch_max_count": rematch_counts["max_count"],
    }
    metrics.update(goal_metrics)
    metrics["goal_source"] = goal_source
    method = args.method or infer_method(os.path.abspath(args.output_dir))
    valid, reasons = compute_validity(metrics, method)
    metrics["method"] = method
    metrics["valid"] = valid
    metrics["invalid_reasons"] = reasons

    write_clearance_csv(os.path.join(args.output_dir, "clearance_timeseries.csv"),
                        clearance_rows)
    csv_row = dict(metrics)
    csv_row["invalid_reasons"] = ";".join(metrics["invalid_reasons"])
    write_single_row_csv(os.path.join(args.output_dir, "planning_metrics.csv"),
                         csv_row)
    with open(os.path.join(args.output_dir, "planning_metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2, sort_keys=True)

    if not args.no_figure:
        tracker_by_ts = None
        if args.tracker_csv and os.path.exists(args.tracker_csv):
            tracker_by_ts = defaultdict(list)
            with open(args.tracker_csv) as f:
                for row in csv.DictReader(f):
                    tracker_by_ts[float(row["timestamp"])].append(row)
        plot_corridor_trajectory(
            odom_rows, gt_by_id, clearance_rows, goal_metrics, args,
            os.path.join(args.output_dir, "fig_corridor_trajectory.pdf"),
            tracker_by_ts=tracker_by_ts)

    print("Planning metrics:")
    for key, value in metrics.items():
        print(f"  {key}: {value}")


if __name__ == "__main__":
    main()
