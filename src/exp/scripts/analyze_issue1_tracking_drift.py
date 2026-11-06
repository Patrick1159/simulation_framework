#!/usr/bin/env python3
"""
Focused offline analysis for Issue 1:
  Is /rematch/tracker_details already wrong in odom once the robot moves?

Expected bag topics:
  /obstacles                GT in world/odom
  /gt_obstacles_camera      GT in camera frame
  /gt_obstacles_base        GT in base_link
  /umap/3D_bounding_boxes   camera-frame measurements
  /tracker/dynamic_obstacles
  /rematch/tracker_details
  /tracker_details_base
  /odom

Outputs:
  <prefix>_summary.json
  <prefix>_samples.csv
  <prefix>_timeseries.png
  <prefix>_errors.png
  <prefix>_displacement.png
"""

import argparse
import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


TIME_TOL = 0.10
ASSOC_THR = 1.50


def msg_time(msg, bag_time):
    try:
        stamp = msg.header.stamp.to_sec()
        if stamp > 0.0:
            return stamp
    except Exception:
        pass
    return bag_time.to_sec()


def load_obstacle_array(msg):
    return [(o.position.x, o.position.y, o.position.z) for o in msg.obstacles]


def load_bbox3d(msg):
    return [(x, y, z) for x, y, z in zip(msg.x, msg.y, msg.z)]


def load_umap_obstacles(msg):
    return [(x, y, z) for x, y, z in zip(msg.centroid_x, msg.centroid_y, msg.centroid_z)]


def load_tracker_details(msg):
    return [(t.position.x, t.position.y, t.position.z) for t in msg.trackers]


def load_bag(path):
    topics = {
        "/obstacles": [],
        "/gt_obstacles_camera": [],
        "/gt_obstacles_base": [],
        "/umap/3D_bounding_boxes": [],
        "/tracker/dynamic_obstacles": [],
        "/rematch/tracker_details": [],
        "/tracker_details_base": [],
        "/odom": [],
    }
    with rosbag.Bag(path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=list(topics.keys())):
            ts = msg_time(msg, t)
            if topic == "/odom":
                px = msg.pose.pose.position.x
                py = msg.pose.pose.position.y
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y
                topics[topic].append({
                    "t": ts,
                    "x": px,
                    "y": py,
                    "speed": math.hypot(vx, vy),
                })
            elif topic in ("/obstacles", "/gt_obstacles_camera", "/gt_obstacles_base"):
                topics[topic].append({"t": ts, "pts": load_obstacle_array(msg)})
            elif topic == "/umap/3D_bounding_boxes":
                topics[topic].append({"t": ts, "pts": load_bbox3d(msg)})
            elif topic == "/tracker/dynamic_obstacles":
                topics[topic].append({"t": ts, "pts": load_umap_obstacles(msg)})
            elif topic in ("/rematch/tracker_details", "/tracker_details_base"):
                topics[topic].append({"t": ts, "pts": load_tracker_details(msg)})
    return topics


def nearest_msg(ts, rows, tol=TIME_TOL):
    if not rows:
        return None
    best = None
    best_dt = float("inf")
    for row in rows:
        dt = abs(row["t"] - ts)
        if dt < best_dt and dt <= tol:
            best = row
            best_dt = dt
    return best


def nearest_point(ref_pt, pts):
    if not pts:
        return None, None
    best_pt = None
    best_d = float("inf")
    rx, ry, rz = ref_pt
    for pt in pts:
        d = math.sqrt((pt[0] - rx) ** 2 + (pt[1] - ry) ** 2 + (pt[2] - rz) ** 2)
        if d < best_d:
            best_d = d
            best_pt = pt
    return best_pt, best_d


def build_robot_interp(odom_rows):
    if not odom_rows:
        return None
    xs = np.array([r["t"] for r in odom_rows])
    ys_x = np.array([r["x"] for r in odom_rows])
    ys_y = np.array([r["y"] for r in odom_rows])
    ys_s = np.array([r["speed"] for r in odom_rows])
    x0 = ys_x[0]
    y0 = ys_y[0]

    def interp(ts):
        if ts < xs[0] or ts > xs[-1]:
            return None
        px = float(np.interp(ts, xs, ys_x))
        py = float(np.interp(ts, xs, ys_y))
        sp = float(np.interp(ts, xs, ys_s))
        disp = math.hypot(px - x0, py - y0)
        return {"x": px, "y": py, "speed": sp, "disp": disp}

    return interp


def compare_single_obstacle(name, ref_rows, tgt_rows, robot_at, association_thr=ASSOC_THR):
    samples = []
    for ref in ref_rows:
        if not ref["pts"]:
            continue
        tgt = nearest_msg(ref["t"], tgt_rows)
        robot = robot_at(ref["t"]) if robot_at is not None else None
        if tgt is None:
            samples.append({
                "name": name,
                "t": ref["t"],
                "matched": 0,
                "error": None,
                "ref_x": ref["pts"][0][0],
                "tgt_x": None,
                "robot_x": None if robot is None else robot["x"],
                "robot_disp": None if robot is None else robot["disp"],
                "robot_speed": None if robot is None else robot["speed"],
            })
            continue

        ref_pt = ref["pts"][0]
        tgt_pt, dist = nearest_point(ref_pt, tgt["pts"])
        matched = int(tgt_pt is not None and dist is not None and dist <= association_thr)
        samples.append({
            "name": name,
            "t": ref["t"],
            "matched": matched,
            "error": float(dist) if tgt_pt is not None else None,
            "ref_x": ref_pt[0],
            "tgt_x": None if tgt_pt is None else tgt_pt[0],
            "robot_x": None if robot is None else robot["x"],
            "robot_disp": None if robot is None else robot["disp"],
            "robot_speed": None if robot is None else robot["speed"],
        })
    return samples


def summarize_samples(samples):
    matched = [s for s in samples if s["matched"] and s["error"] is not None]
    out = {
        "frames": len(samples),
        "matched_frames": len(matched),
        "match_rate": 0.0 if not samples else len(matched) / float(len(samples)),
    }
    if not matched:
        return out

    errors = np.array([s["error"] for s in matched], dtype=float)
    out["mean_error_m"] = float(np.mean(errors))
    out["std_error_m"] = float(np.std(errors))
    out["max_error_m"] = float(np.max(errors))
    out["rmse_m"] = float(np.sqrt(np.mean(errors ** 2)))

    valid_disp = [s for s in matched if s["robot_disp"] is not None]
    if len(valid_disp) >= 3:
        disp = np.array([s["robot_disp"] for s in valid_disp], dtype=float)
        err = np.array([s["error"] for s in valid_disp], dtype=float)
        if np.std(disp) > 1e-6:
            slope = np.polyfit(disp, err, 1)[0]
            corr = np.corrcoef(disp, err)[0, 1]
            out["error_growth_m_per_m"] = float(slope)
            out["disp_corr"] = float(corr)
            out["disp_range_m"] = float(np.max(disp) - np.min(disp))

    valid_speed = [s for s in matched if s["robot_speed"] is not None]
    if len(valid_speed) >= 3:
        speed = np.array([s["robot_speed"] for s in valid_speed], dtype=float)
        err = np.array([s["error"] for s in valid_speed], dtype=float)
        if np.std(speed) > 1e-6:
            out["speed_corr"] = float(np.corrcoef(speed, err)[0, 1])
    return out


def drift_verdict(summary):
    mean_err = summary.get("mean_error_m")
    slope = summary.get("error_growth_m_per_m")
    corr = summary.get("disp_corr")
    disp_range = summary.get("disp_range_m", 0.0)
    if mean_err is None:
        return "no_data"
    if disp_range < 0.3:
        if mean_err < 0.2:
            return "stable_low_motion"
        return "unclear_low_motion"
    if slope is not None and corr is not None and slope > 0.15 and corr > 0.35:
        return "motion_coupled_drift"
    if mean_err < 0.20:
        return "stable"
    if mean_err < 0.40:
        return "moderate_bias"
    return "large_bias_no_clear_motion_coupling"


def render_timeseries(out_prefix, gt_odom, gt_camera, gt_base, umap_samples, coarse_samples,
                      tracker_samples, tracker_base_samples, odom_rows):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    if gt_odom:
        axes[0].plot([r["t"] for r in gt_odom], [r["pts"][0][0] for r in gt_odom],
                     "k-", linewidth=2, label="GT_odom.x")
    if coarse_samples:
        axes[0].plot([s["t"] for s in coarse_samples if s["tgt_x"] is not None],
                     [s["tgt_x"] for s in coarse_samples if s["tgt_x"] is not None],
                     color="tab:orange", linewidth=1.5, label="Coarse_odom.x")
    if tracker_samples:
        axes[0].plot([s["t"] for s in tracker_samples if s["tgt_x"] is not None],
                     [s["tgt_x"] for s in tracker_samples if s["tgt_x"] is not None],
                     color="tab:red", linewidth=1.5, label="Tracker_odom.x")
    if odom_rows:
        axes[0].plot([r["t"] for r in odom_rows], [r["x"] for r in odom_rows],
                     color="tab:gray", alpha=0.5, linewidth=1.0, label="Robot odom.x")
    axes[0].set_ylabel("X in odom (m)")
    axes[0].set_title("Issue 1 Diagnostic: odom-frame obstacle vs robot motion")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    if gt_base:
        axes[1].plot([r["t"] for r in gt_base], [r["pts"][0][0] for r in gt_base],
                     "k-", linewidth=2, label="GT_base.x")
    if tracker_base_samples:
        axes[1].plot([s["t"] for s in tracker_base_samples if s["tgt_x"] is not None],
                     [s["tgt_x"] for s in tracker_base_samples if s["tgt_x"] is not None],
                     color="tab:red", linewidth=1.5, label="Tracker_base.x")
    axes[1].set_ylabel("X in base_link (m)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    if umap_samples:
        axes[2].plot([s["t"] for s in umap_samples if s["tgt_x"] is not None],
                     [s["tgt_x"] for s in umap_samples if s["tgt_x"] is not None],
                     color="tab:blue", linewidth=1.5, label="UMAP_camera.x")
    if gt_camera:
        axes[2].plot([r["t"] for r in gt_camera], [r["pts"][0][0] for r in gt_camera],
                     "k-", linewidth=2, alpha=0.8, label="GT_camera.x")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("X in measurement frame")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")

    fig.tight_layout()
    fig.savefig(f"{out_prefix}_timeseries.png", dpi=150)
    plt.close(fig)


def render_error_plot(out_prefix, sample_sets):
    fig, ax = plt.subplots(figsize=(12, 5))
    for label, color, samples in sample_sets:
        vals = [(s["t"], s["error"]) for s in samples if s["matched"] and s["error"] is not None]
        if not vals:
            continue
        ax.plot([v[0] for v in vals], [v[1] for v in vals],
                linewidth=1.5, color=color, label=label)
    ax.axhline(0.0, color="k", linestyle=":", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.set_title("Error vs time")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(f"{out_prefix}_errors.png", dpi=150)
    plt.close(fig)


def render_displacement_plot(out_prefix, sample_sets):
    fig, ax = plt.subplots(figsize=(12, 5))
    for label, color, samples in sample_sets:
        vals = [(s["robot_disp"], s["error"]) for s in samples
                if s["matched"] and s["error"] is not None and s["robot_disp"] is not None]
        if not vals:
            continue
        vals = sorted(vals, key=lambda x: x[0])
        ax.plot([v[0] for v in vals], [v[1] for v in vals],
                linewidth=1.5, color=color, label=label)
    ax.axhline(0.0, color="k", linestyle=":", linewidth=0.8)
    ax.set_xlabel("Robot displacement from start (m)")
    ax.set_ylabel("Error (m)")
    ax.set_title("Error vs robot displacement")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(f"{out_prefix}_displacement.png", dpi=150)
    plt.close(fig)


def write_csv(path, all_samples):
    fieldnames = [
        "name", "t", "matched", "error", "ref_x", "tgt_x",
        "robot_x", "robot_disp", "robot_speed",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in all_samples:
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description="Issue-1 focused tracking drift analysis")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", default="issue1")
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        sys.exit(f"Bag not found: {args.bag}")

    data = load_bag(args.bag)
    robot_at = build_robot_interp(data["/odom"])

    umap_samples = compare_single_obstacle(
        "UMAP_camera_vs_GT_camera",
        data["/gt_obstacles_camera"],
        data["/umap/3D_bounding_boxes"],
        robot_at,
    )
    coarse_samples = compare_single_obstacle(
        "Coarse_odom_vs_GT_odom",
        data["/obstacles"],
        data["/tracker/dynamic_obstacles"],
        robot_at,
    )
    tracker_samples = compare_single_obstacle(
        "Tracker_odom_vs_GT_odom",
        data["/obstacles"],
        data["/rematch/tracker_details"],
        robot_at,
    )
    tracker_base_samples = compare_single_obstacle(
        "Tracker_base_vs_GT_base",
        data["/gt_obstacles_base"],
        data["/tracker_details_base"],
        robot_at,
    )

    summaries = {
        "UMAP_camera_vs_GT_camera": summarize_samples(umap_samples),
        "Coarse_odom_vs_GT_odom": summarize_samples(coarse_samples),
        "Tracker_odom_vs_GT_odom": summarize_samples(tracker_samples),
        "Tracker_base_vs_GT_base": summarize_samples(tracker_base_samples),
    }
    verdicts = {name: drift_verdict(summary) for name, summary in summaries.items()}

    issue1 = "inconclusive"
    tracker_verdict = verdicts["Tracker_odom_vs_GT_odom"]
    coarse_verdict = verdicts["Coarse_odom_vs_GT_odom"]
    umap_verdict = verdicts["UMAP_camera_vs_GT_camera"]
    if tracker_verdict == "motion_coupled_drift":
        issue1 = "supported"
    elif tracker_verdict in ("stable", "stable_low_motion") and coarse_verdict in ("stable", "stable_low_motion"):
        issue1 = "not_supported"

    root_cause_hint = "undetermined"
    if verdicts["Tracker_odom_vs_GT_odom"] == "motion_coupled_drift":
        if verdicts["UMAP_camera_vs_GT_camera"] in ("stable", "stable_low_motion", "moderate_bias"):
            root_cause_hint = "camera_to_odom_tf_or_tracker_state"
        elif verdicts["UMAP_camera_vs_GT_camera"] == "motion_coupled_drift":
            root_cause_hint = "measurement_or_upstream_tf"
    elif verdicts["Tracker_odom_vs_GT_odom"] in ("stable", "stable_low_motion") and \
            verdicts["Tracker_base_vs_GT_base"] == "motion_coupled_drift":
        root_cause_hint = "downstream_base_or_planner_frame"

    summary = {
        "issue1_verdict": issue1,
        "root_cause_hint": root_cause_hint,
        "per_comparison": summaries,
        "per_comparison_verdict": verdicts,
    }

    all_samples = umap_samples + coarse_samples + tracker_samples + tracker_base_samples
    write_csv(f"{args.output}_samples.csv", all_samples)
    with open(f"{args.output}_summary.json", "w") as f:
        json.dump(summary, f, indent=2)

    render_timeseries(
        args.output,
        data["/obstacles"],
        data["/gt_obstacles_camera"],
        data["/gt_obstacles_base"],
        umap_samples,
        coarse_samples,
        tracker_samples,
        tracker_base_samples,
        data["/odom"],
    )
    render_error_plot(
        args.output,
        [
            ("UMAP_camera", "tab:blue", umap_samples),
            ("Coarse_odom", "tab:orange", coarse_samples),
            ("Tracker_odom", "tab:red", tracker_samples),
            ("Tracker_base", "tab:green", tracker_base_samples),
        ],
    )
    render_displacement_plot(
        args.output,
        [
            ("UMAP_camera", "tab:blue", umap_samples),
            ("Coarse_odom", "tab:orange", coarse_samples),
            ("Tracker_odom", "tab:red", tracker_samples),
            ("Tracker_base", "tab:green", tracker_base_samples),
        ],
    )

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
