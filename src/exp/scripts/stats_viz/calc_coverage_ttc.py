#!/usr/bin/env python3
"""Tracking-time coverage and time-to-collision metrics for a single run.

Two metrics, computed from gt.csv + tracker.csv + raw.bag (for the camera
FOV filter):

1. Tracking time coverage
   For each GT obstacle, the time it spends inside the robot camera FOV is the
   denominator; the portion of that time during which the tracker has a matched
   detection (within --match-dist) is the numerator. Coverage = matched / in_fov,
   averaged over obstacles. Measures how persistently perception tracks an
   obstacle while it is observable, independent of whether the planner avoids it.

2. Time-to-collision (TTC) + minimum clearance
   At each odom sample, for each GT obstacle, extrapolate robot and obstacle at
   their current velocities and find the time at which their disc footprints
   (robot-radius + obstacle-radius) would touch. The per-frame TTC is the min
   over obstacles; the run TTC is the min over frames (smallest margin the robot
   ever allowed). Minimum clearance is the standard closest surface-to-surface
   distance. Both are evaluated only up to goal arrival (post-arrival dwell is
   not a navigation decision).

Outputs coverage_ttc.json (+ .csv) in the run dir.
"""

import argparse
import csv
import json
import math
import os

from tracking_eval_utils import build_fov_filter, match_detections


def load_csv_frames(path):
    """Load gt.csv / tracker.csv grouped by timestamp string -> list of dicts."""
    frames = {}
    if not os.path.exists(path):
        return frames
    with open(path) as f:
        for row in csv.DictReader(f):
            ts = row["timestamp"]
            frames.setdefault(ts, []).append(row)
    return frames


def load_odom(bag_path, topic="/odom"):
    import rosbag
    rows = []
    bag = rosbag.Bag(bag_path, "r")
    for _, m, _ in bag.read_messages(topics=[topic]):
        rows.append({
            "t": m.header.stamp.to_sec(),
            "x": m.pose.pose.position.x,
            "y": m.pose.pose.position.y,
        })
    bag.close()
    rows.sort(key=lambda r: r["t"])
    return rows


def finite_velocity(rows, i):
    """Central-difference velocity at index i of a time-sorted row list."""
    if len(rows) < 2:
        return 0.0, 0.0
    j0 = max(0, i - 1)
    j1 = min(len(rows) - 1, i + 1)
    dt = rows[j1]["t"] - rows[j0]["t"]
    if dt <= 1e-6:
        return 0.0, 0.0
    return (rows[j1]["x"] - rows[j0]["x"]) / dt, (rows[j1]["y"] - rows[j0]["y"]) / dt


def interp_gt(gt_rows_sorted, t):
    """Linear interp of a single obstacle's (x,y,vx,vy) at time t. None if outside."""
    if not gt_rows_sorted:
        return None
    if t < gt_rows_sorted[0][0] or t > gt_rows_sorted[-1][0]:
        return None
    lo, hi = 0, len(gt_rows_sorted) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if gt_rows_sorted[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0:
        return gt_rows_sorted[0][1:]
    a = gt_rows_sorted[lo - 1]
    b = gt_rows_sorted[lo]
    span = b[0] - a[0]
    f = 0.0 if span <= 1e-9 else (t - a[0]) / span
    return tuple(a[k] + f * (b[k] - a[k]) for k in range(1, 5))


def ttc_pair(rx, ry, rvx, rvy, ox, oy, ovx, ovy, r_sum):
    """Time until disc footprints (combined radius r_sum) touch, or inf."""
    dpx, dpy = ox - rx, oy - ry
    dvx, dvy = ovx - rvx, ovy - rvy
    a = dvx * dvx + dvy * dvy
    if a < 1e-9:
        return float("inf")  # no relative motion
    if dpx * dpx + dpy * dpy <= r_sum * r_sum:
        return 0.0  # already overlapping
    b = 2.0 * (dpx * dvx + dpy * dvy)
    c = dpx * dpx + dpy * dpy - r_sum * r_sum
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return float("inf")  # closest approach never reaches contact
    sq = math.sqrt(disc)
    t1 = (-b - sq) / (2.0 * a)
    t2 = (-b + sq) / (2.0 * a)
    cand = [t for t in (t1, t2) if t >= 0.0]
    return min(cand) if cand else float("inf")


def compute_coverage(gt_frames, trk_frames, fov_filter, match_dist,
                     time_tol=0.15):
    """Per-obstacle: matched_in_fov_time / in_fov_time, averaged.

    GT frames (dense, ~30Hz) are the time base. The tracker runs slower
    (~2-20Hz), so for each GT frame we take the nearest tracker frame within
    time_tol and ask whether that obstacle has a matched detection. This
    rewards persistent tracking without penalising the tracker for publishing
    at a lower rate than GT.
    """
    ts_sorted = sorted(gt_frames.keys(), key=lambda s: float(s))
    if len(ts_sorted) < 2:
        return {}, float("nan")

    trk_ts_sorted = sorted(trk_frames.keys(), key=lambda s: float(s))
    trk_t_vals = [float(s) for s in trk_ts_sorted]

    def nearest_trk(t):
        if not trk_t_vals:
            return []
        lo, hi = 0, len(trk_t_vals) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if trk_t_vals[mid] < t:
                lo = mid + 1
            else:
                hi = mid
        best = lo
        for cand in (lo - 1, lo):
            if 0 <= cand < len(trk_t_vals) and abs(trk_t_vals[cand] - t) < abs(trk_t_vals[best] - t):
                best = cand
        if abs(trk_t_vals[best] - t) > time_tol:
            return []
        return trk_frames[trk_ts_sorted[best]]

    in_fov = {}
    matched = {}
    prev_t = None
    for ts in ts_sorted:
        t = float(ts)
        dt = 0.0 if prev_t is None else (t - prev_t)
        prev_t = t
        gt_list = gt_frames[ts]
        trk_list = nearest_trk(t)
        # FOV-visible GT only
        vis = [g for g in gt_list
               if fov_filter is None or fov_filter(t, float(g["x"]), float(g["y"]))]
        for g in vis:
            oid = int(g["id"])
            in_fov[oid] = in_fov.get(oid, 0.0) + dt
        if not vis or not trk_list:
            continue
        pairs, _ugt, _utrk = match_detections(vis, trk_list, match_dist)
        for g, _trk, _d in pairs:
            oid = int(g["id"])
            matched[oid] = matched.get(oid, 0.0) + dt

    per_obs = {}
    for oid, tf in in_fov.items():
        if tf > 1e-6:
            per_obs[oid] = matched.get(oid, 0.0) / tf
    mean_cov = sum(per_obs.values()) / len(per_obs) if per_obs else float("nan")
    return per_obs, mean_cov


def compute_ttc_clearance(odom_rows, gt_by_id, robot_radius, obs_radius,
                          reach_ts, ttc_cap):
    min_ttc = float("inf")
    min_clear = float("inf")
    ttc_samples = []
    for i, rob in enumerate(odom_rows):
        t = rob["t"]
        if reach_ts is not None and math.isfinite(reach_ts) and t > reach_ts:
            break
        rvx, rvy = finite_velocity(odom_rows, i)
        frame_ttc = float("inf")
        for oid, rows_sorted in gt_by_id.items():
            st = interp_gt(rows_sorted, t)
            if st is None:
                continue
            ox, oy, ovx, ovy = st
            clr = math.hypot(rob["x"] - ox, rob["y"] - oy) - robot_radius - obs_radius
            min_clear = min(min_clear, clr)
            ttc = ttc_pair(rob["x"], rob["y"], rvx, rvy,
                           ox, oy, ovx, ovy, robot_radius + obs_radius)
            frame_ttc = min(frame_ttc, ttc)
        if math.isfinite(frame_ttc):
            ttc_samples.append(min(frame_ttc, ttc_cap))
            min_ttc = min(min_ttc, frame_ttc)
    danger = sum(1 for v in ttc_samples if v < 1.0)
    return {
        "min_ttc_s": min_ttc if math.isfinite(min_ttc) else float("nan"),
        "ttc_below_1s_frac": danger / len(ttc_samples) if ttc_samples else float("nan"),
        "min_clearance_m": min_clear if math.isfinite(min_clear) else float("nan"),
        "ttc_eval_samples": len(ttc_samples),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--gt", default=None)
    ap.add_argument("--tracker", default=None)
    ap.add_argument("--bag", default=None)
    ap.add_argument("--match-dist", type=float, default=1.0)
    ap.add_argument("--robot-radius", type=float, default=0.18)
    ap.add_argument("--obs-radius", type=float, default=0.30)
    ap.add_argument("--ttc-cap", type=float, default=10.0,
                    help="Cap TTC at this value for the danger-fraction stat.")
    ap.add_argument("--goal-x", type=float, default=0.0)
    ap.add_argument("--goal-y", type=float, default=8.0)
    ap.add_argument("--goal-tolerance", type=float, default=0.35)
    ap.add_argument("--no-fov", action="store_true",
                    help="Disable FOV filtering (coverage over whole episode).")
    args = ap.parse_args()

    gt_path = args.gt or os.path.join(args.run_dir, "gt.csv")
    trk_path = args.tracker or os.path.join(args.run_dir, "tracker.csv")
    bag_path = args.bag or os.path.join(args.run_dir, "raw.bag")

    gt_frames = load_csv_frames(gt_path)
    trk_frames = load_csv_frames(trk_path)

    fov_filter = None
    if not args.no_fov and os.path.exists(bag_path):
        fov_filter = build_fov_filter(bag_path)

    per_obs, mean_cov = compute_coverage(gt_frames, trk_frames, fov_filter,
                                         args.match_dist)

    # Build gt_by_id (sorted (t,x,y,vx,vy)) and odom for TTC.
    gt_by_id = {}
    for ts, rows in gt_frames.items():
        t = float(ts)
        for r in rows:
            gt_by_id.setdefault(int(r["id"]), []).append(
                (t, float(r["x"]), float(r["y"]), float(r["vx"]), float(r["vy"])))
    for oid in gt_by_id:
        gt_by_id[oid].sort(key=lambda x: x[0])

    odom_rows = load_odom(bag_path) if os.path.exists(bag_path) else []

    # goal arrival time
    reach_ts = None
    for rob in odom_rows:
        if math.hypot(rob["x"] - args.goal_x, rob["y"] - args.goal_y) <= args.goal_tolerance:
            reach_ts = rob["t"]
            break

    ttc = compute_ttc_clearance(odom_rows, gt_by_id, args.robot_radius,
                                args.obs_radius, reach_ts, args.ttc_cap)

    out = {
        "run_dir": os.path.abspath(args.run_dir),
        "match_dist_m": args.match_dist,
        "fov_filtered": fov_filter is not None,
        "tracking_coverage_mean": mean_cov,
        "tracking_coverage_per_obstacle": per_obs,
        "goal_reach_timestamp": reach_ts if reach_ts is not None else float("nan"),
    }
    out.update(ttc)

    with open(os.path.join(args.run_dir, "coverage_ttc.json"), "w") as f:
        json.dump(out, f, indent=2, sort_keys=True)

    flat = {k: v for k, v in out.items() if not isinstance(v, dict)}
    with open(os.path.join(args.run_dir, "coverage_ttc.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(list(flat.keys()))
        w.writerow(list(flat.values()))

    print(f"{os.path.basename(args.run_dir)}: coverage={mean_cov:.3f} "
          f"min_ttc={out['min_ttc_s']:.2f}s ttc<1s={out['ttc_below_1s_frac']:.3f} "
          f"min_clr={out['min_clearance_m']:+.3f} per_obs={ {k: round(v,2) for k,v in per_obs.items()} }")


if __name__ == "__main__":
    main()
