#!/usr/bin/env python3
"""
analyze_frame_errors.py — Offline diagnostic for frame-level error isolation.

Reads a rosbag with the 6 data streams from the frame diagnostic experiment:
  GT_camera:   /gt_obstacles_camera   (mpc_nav/ObstacleArray)
  GT_odom:     /obstacles             (mpc_nav/ObstacleArray, world=odom)
  GT_base:     /gt_obstacles_base     (mpc_nav/ObstacleArray)
  UMAP_camera: /umap/3D_bounding_boxes (umap/BoundingBoxes3D)
  Tracker_odom:/rematch/tracker_details (umap/TrackerDetailArray)
  Tracker_base:/tracker_details_base    (umap/TrackerDetailArray)

Produces:
  1. diagnostic_timeseries.csv  — per-frame errors for all 3 comparisons
  2. diagnostic_summary.json    — aggregate statistics
  3. Console diagnostic report  — root-cause isolation guidance

Usage:
  python3 analyze_frame_errors.py --bag diagnostic_run.bag
"""

import argparse
import json
import math
import os
import sys
from collections import defaultdict

import rosbag
import numpy as np


# ---------------------------------------------------------------------------
# Hungarian matching (min-cost assignment) — standalone, no scipy dependency
# ---------------------------------------------------------------------------

def hungarian(cost_matrix):
    """Min-cost assignment on square matrix. Returns (row_idx, col_idx) pairs.
    Implements the O(n^3) Hungarian algorithm (Munkres variant)."""
    n = len(cost_matrix)
    if n == 0:
        return []
    u = [0] * (n + 1)
    v = [0] * (n + 1)
    p = [0] * (n + 1)
    way = [0] * (n + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [float('inf')] * (n + 1)
        used = [False] * (n + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = float('inf')
            j1 = 0
            for j in range(1, n + 1):
                if not used[j]:
                    cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            for j in range(n + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    assignment = []
    for j in range(1, n + 1):
        if p[j] != 0:
            assignment.append((p[j] - 1, j - 1))
    return assignment


def match_obstacles(positions_a, positions_b, threshold=1.0):
    """Match two sets of 3D positions via Hungarian.
    Returns (errors, fp_count, fn_count).

    errors: list of (pos_a, pos_b, dist) for matched pairs within threshold
    fp_count: positions in A with no match in B (false positives)
    fn_count: positions in B with no match in A (false negatives)
    """
    na, nb = len(positions_a), len(positions_b)
    if na == 0 or nb == 0:
        return [], na, nb

    # Pad to square matrix
    size = max(na, nb)
    cost = np.full((size, size), 1e9)
    for i in range(na):
        a = np.array(positions_a[i])
        for j in range(nb):
            b = np.array(positions_b[j])
            cost[i, j] = np.linalg.norm(a - b)

    assignment = hungarian(cost.tolist())

    errors = []
    matched_a = set()
    matched_b = set()
    for i, j in assignment:
        if i < na and j < nb and cost[i, j] < threshold:
            errors.append((i, j, float(cost[i, j])))
            matched_a.add(i)
            matched_b.add(j)

    fp_count = na - len(matched_a)
    fn_count = nb - len(matched_b)
    return errors, fp_count, fn_count


# ---------------------------------------------------------------------------
# Bag extraction
# ---------------------------------------------------------------------------

TOPIC_MAP = {
    '/gt_obstacles_camera':   'gt_camera',
    '/obstacles':             'gt_odom',
    '/gt_obstacles_base':     'gt_base',
    '/umap/3D_bounding_boxes': 'umap_camera',
    '/rematch/tracker_details': 'tracker_odom',
    '/tracker_details_base':  'tracker_base',
    '/odom':                  'odom',
}


def extract_positions_obstacle_array(msg):
    """Extract (x, y, z) positions from mpc_nav/ObstacleArray."""
    return [(o.position.x, o.position.y, o.position.z) for o in msg.obstacles]


def extract_positions_bboxes3d(msg):
    """Extract (x, y, z) centroids from umap/BoundingBoxes3D."""
    return [(x, y, z) for x, y, z in zip(msg.x, msg.y, msg.z)]


def extract_positions_tracker_details(msg):
    """Extract (x, y, z) positions from umap/TrackerDetailArray."""
    return [(t.position.x, t.position.y, t.position.z) for t in msg.trackers]


EXTRACTORS = {
    '/gt_obstacles_camera':   extract_positions_obstacle_array,
    '/obstacles':             extract_positions_obstacle_array,
    '/gt_obstacles_base':     extract_positions_obstacle_array,
    '/umap/3D_bounding_boxes': extract_positions_bboxes3d,
    '/rematch/tracker_details': extract_positions_tracker_details,
    '/tracker_details_base':  extract_positions_tracker_details,
}


def load_bag(bag_path):
    """Read all relevant messages from the bag into per-topic lists.
    Returns dict: topic_name -> [(timestamp_sec, positions_list), ...]
    For /odom, stores (ts, speed, pos_x, pos_y)."""
    data = defaultdict(list)
    topics = set(TOPIC_MAP.keys())

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            ts = t.to_sec()
            if topic == '/odom':
                speed = math.sqrt(msg.twist.twist.linear.x ** 2 +
                                  msg.twist.twist.linear.y ** 2 +
                                  msg.twist.twist.linear.z ** 2)
                px = msg.pose.pose.position.x
                py = msg.pose.pose.position.y
                data[topic].append((ts, speed, px, py))
            elif topic in EXTRACTORS:
                positions = EXTRACTORS[topic](msg)
                data[topic].append((ts, positions))

    for topic in topics:
        print(f"  {topic}: {len(data[topic])} messages")
    return data


# ---------------------------------------------------------------------------
# Time association
# ---------------------------------------------------------------------------

def associate_by_time(ref_data, tgt_data, tolerance=0.08):
    """Match (ts, positions) pairs between ref and tgt by nearest timestamp.
    Returns list of (ref_ts, ref_positions, tgt_positions, dt) tuples."""
    if not ref_data or not tgt_data:
        return []

    pairs = []
    tgt_idx = 0
    for ref_ts, ref_pos in ref_data:
        # Advance tgt_idx until we bracket ref_ts
        while tgt_idx < len(tgt_data) - 1 and tgt_data[tgt_idx + 1][0] < ref_ts - tolerance:
            tgt_idx += 1
        # Find nearest within window
        best_dt = float('inf')
        best_pos = None
        for j in range(max(0, tgt_idx - 2), min(len(tgt_data), tgt_idx + 3)):
            dt = abs(tgt_data[j][0] - ref_ts)
            if dt < tolerance and dt < best_dt:
                best_dt = dt
                best_pos = tgt_data[j][1]
        if best_pos is not None:
            pairs.append((ref_ts, ref_pos, best_pos, best_dt))
    return pairs


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

def analyze_comparison(name, ref_data, tgt_data, match_threshold=1.0):
    """Run a single comparison: ref vs tgt."""
    if not ref_data:
        print(f"\n  [{name}] SKIP: no reference data")
        return None
    if not tgt_data:
        print(f"\n  [{name}] SKIP: no target data")
        return None

    # Diagnostic: show timestamp ranges
    ref_t0, ref_t1 = ref_data[0][0], ref_data[-1][0]
    tgt_t0, tgt_t1 = tgt_data[0][0], tgt_data[-1][0]
    overlap = min(ref_t1, tgt_t1) - max(ref_t0, tgt_t0)
    sys.stdout.write(f"  [{name}] ref=[{ref_t0:.1f}, {ref_t1:.1f}] tgt=[{tgt_t0:.1f}, {tgt_t1:.1f}] overlap={overlap:.1f}s\n")
    sys.stdout.flush()

    sys.stdout.write(f"  [{name}] Associating {len(ref_data)} ref × {len(tgt_data)} tgt messages ...")
    sys.stdout.flush()
    pairs = associate_by_time(ref_data, tgt_data)
    print(f" {len(pairs)} pairs")
    if not pairs:
        print(f"  [{name}] SKIP: no time-associated pairs")
        return None

    all_distances = []
    all_fp = []
    all_fn = []
    frames_with_matches = 0
    report_interval = max(1, len(pairs) // 10)

    for idx, (ref_ts, ref_pos, tgt_pos, dt) in enumerate(pairs):
        errors, fp, fn = match_obstacles(ref_pos, tgt_pos, match_threshold)
        if errors:
            frames_with_matches += 1
        for _, _, d in errors:
            all_distances.append((ref_ts, d))
        all_fp.append((ref_ts, fp))
        all_fn.append((ref_ts, fn))
        if (idx + 1) % report_interval == 0:
            sys.stdout.write(f"\r  [{name}] {idx + 1}/{len(pairs)} frames processed")
            sys.stdout.flush()

    if all_distances:
        print(f"\r  [{name}] {len(pairs)}/{len(pairs)} frames — {frames_with_matches} with matches, {len(all_distances)} obstacle-pairs")
    sys.stdout.flush()

    if not all_distances:
        print(f"\n  [{name}] NO MATCHED OBSTACLES in {len(pairs)} frame pairs")
        return None

    dists = [d for _, d in all_distances]
    fp_counts = [f for _, f in all_fp]
    fn_counts = [f for _, f in all_fn]

    result = {
        'name': name,
        'n_frame_pairs': len(pairs),
        'n_matched_frames': frames_with_matches,
        'n_matched_obstacles': len(dists),
        'mean_error_m': float(np.mean(dists)),
        'median_error_m': float(np.median(dists)),
        'p95_error_m': float(np.percentile(dists, 95)),
        'max_error_m': float(np.max(dists)),
        'rmse_m': float(np.sqrt(np.mean(np.array(dists) ** 2))),
        'mean_fp_per_frame': float(np.mean(fp_counts)),
        'mean_fn_per_frame': float(np.mean(fn_counts)),
        'timeseries': [(ts, d) for ts, d in all_distances],
    }
    return result


def robot_displacement(odom_data):
    """Compute cumulative robot displacement over time from odometry."""
    if not odom_data:
        return []
    disp = []
    x0, y0 = odom_data[0][2], odom_data[0][3]
    for ts, _, px, py in odom_data:
        d = math.sqrt((px - x0) ** 2 + (py - y0) ** 2)
        disp.append((ts, d))
    return disp


def _two_pointer_associate(ref_data, tgt_data, window=0.1):
    """Associate (ts, val) pairs between two sorted lists by nearest timestamp.
    Returns list of (tgt_val, ref_val) for matched pairs within window.
    Uses two-pointer O(n+m) scan, same pattern as associate_by_time."""
    if not ref_data or not tgt_data:
        return []
    pairs = []
    tgt_idx = 0
    for ref_ts, ref_val in ref_data:
        while tgt_idx < len(tgt_data) - 1 and tgt_data[tgt_idx + 1][0] < ref_ts - window:
            tgt_idx += 1
        best_dt = float('inf')
        best_val = None
        for j in range(max(0, tgt_idx - 2), min(len(tgt_data), tgt_idx + 3)):
            dt = abs(tgt_data[j][0] - ref_ts)
            if dt < window and dt < best_dt:
                best_dt = dt
                best_val = tgt_data[j][1]
        if best_val is not None:
            pairs.append((best_val, ref_val))
    return pairs


def correlate_with_signal(signal_data, error_timeseries, window=0.1):
    """Associate each error with a signal value (speed or displacement)
    at the nearest timestamp. Returns (correlation, paired_list)."""
    paired = _two_pointer_associate(error_timeseries, signal_data, window)
    if len(paired) < 10:
        return None, paired
    vals = np.array([v for v, _ in paired])
    errors = np.array([e for _, e in paired])
    corr = float(np.corrcoef(vals, errors)[0, 1])
    return corr, paired


def median_error_by_bin(signal_data, error_timeseries, bins, window=0.1):
    """Group errors by signal bins, return median error per bin."""
    paired = _two_pointer_associate(error_timeseries, signal_data, window)
    result = []
    for lo, hi, label in bins:
        bin_errors = [e for v, e in paired if lo <= v < hi]
        if bin_errors:
            result.append((label, float(np.median(bin_errors)), len(bin_errors)))
    return result


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def compute_error_growth(results, odom_data):
    """Estimate error growth per meter of robot displacement for each comparison.
    Returns dict: comparison_name -> (growth_rate_m_per_m, correlation_with_displacement)
    """
    growth = {}
    disp = robot_displacement(odom_data)
    if not disp:
        return growth
    # Build speed + displacement signals
    speed_signal = [(ts, s) for ts, s, _, _ in odom_data]
    for r in (r for r in results if r is not None):
        # Speed correlation
        vel_corr, _ = correlate_with_signal(speed_signal, r['timeseries'])
        # Displacement correlation
        disp_corr, paired = correlate_with_signal(disp, r['timeseries'])
        # Linear fit: error = a + b * displacement
        if paired and len(paired) > 10:
            d = np.array([v for v, _ in paired])
            e = np.array([err for _, err in paired])
            if np.std(d) > 0.01:
                b, a = np.polyfit(d, e, 1)
            else:
                b, a = 0.0, float(np.mean(e))
        else:
            b, a = 0.0, 0.0
        growth[r['name']] = {
            'vel_corr': vel_corr,
            'disp_corr': disp_corr,
            'growth_m_per_m': float(b),
            'intercept_m': float(a),
        }
    return growth


def print_report(results, odom_data):
    print("\n" + "=" * 72)
    print("  FRAME DIAGNOSTIC REPORT")
    print("=" * 72)

    # Compute growth rates for cross-validation
    growth = compute_error_growth(results, odom_data)
    disp = robot_displacement(odom_data)
    max_disp = max(d for _, d in disp) if disp else 0.0

    print(f"\n  Robot max displacement: {max_disp:.2f} m")
    print(f"  Odom messages: {len(odom_data)}")

    for r in results:
        if r is None:
            continue
        print(f"\n── {r['name']} ──")
        print(f"  Frame pairs:       {r['n_frame_pairs']}")
        print(f"  Matched obstacles: {r['n_matched_obstacles']}")
        print(f"  Mean error:        {r['mean_error_m']:.3f} m")
        print(f"  Median error:      {r['median_error_m']:.3f} m")
        print(f"  P95 error:         {r['p95_error_m']:.3f} m")
        print(f"  RMSE:              {r['rmse_m']:.3f} m")
        print(f"  Max error:         {r['max_error_m']:.3f} m")
        print(f"  Mean FP/frame:     {r['mean_fp_per_frame']:.2f}")
        print(f"  Mean FN/frame:     {r['mean_fn_per_frame']:.2f}")

        g = growth.get(r['name'])
        if g:
            if g['vel_corr'] is not None:
                tag = "STRONG" if abs(g['vel_corr']) > 0.3 else "weak"
                print(f"  Speed correlation:    {g['vel_corr']:.3f} ({tag})")
            if g['disp_corr'] is not None:
                tag = "STRONG" if abs(g['disp_corr']) > 0.3 else "weak"
                print(f"  Displacement corr:    {g['disp_corr']:.3f} ({tag})")
            print(f"  Error growth rate:    {g['growth_m_per_m']:.3f} m/m")

            # Show binned errors by displacement
            bins = [(0, 1.0, "0-1m"), (1.0, 3.0, "1-3m"), (3.0, 10.0, "3m+")]
            binned = median_error_by_bin(disp, r['timeseries'], bins)
            for label, med, n in binned:
                print(f"    median error @ {label}:  {med:.3f} m  (n={n})")

    # Diagnostic interpretation
    print("\n── DIAGNOSTIC INTERPRETATION ──")
    r_names = {r['name']: r for r in results if r is not None}

    def print_quality(label, err):
        if err is None:
            print(f"  [{label}] NO DATA")
        elif err < 0.3:
            print(f"  [{label}] GOOD (<0.3 m mean error)")
        elif err < 0.6:
            print(f"  [{label}] MODERATE (0.3-0.6 m mean error)")
        else:
            print(f"  [{label}] POOR (>0.6 m mean error)")

    u_err = r_names.get('UMAP_camera_vs_GT_camera', {}).get('mean_error_m', None) if 'UMAP_camera_vs_GT_camera' in r_names else None
    t_err = r_names.get('Tracker_odom_vs_GT_odom', {}).get('mean_error_m', None) if 'Tracker_odom_vs_GT_odom' in r_names else None
    b_err = r_names.get('Tracker_base_vs_GT_base', {}).get('mean_error_m', None) if 'Tracker_base_vs_GT_base' in r_names else None

    print_quality("UMAP ", u_err)
    print_quality("TF   ", t_err)
    print_quality("BASE ", b_err)

    # Root-cause isolation
    print("\n── ROOT-CAUSE ISOLATION ──")
    if u_err is not None and t_err is not None:
        u_growth = growth.get('UMAP_camera_vs_GT_camera', {}).get('growth_m_per_m', 0)
        t_growth = growth.get('Tracker_odom_vs_GT_odom', {}).get('growth_m_per_m', 0)
        u_disp_corr = growth.get('UMAP_camera_vs_GT_camera', {}).get('disp_corr', 0)
        t_disp_corr = growth.get('Tracker_odom_vs_GT_odom', {}).get('disp_corr', 0)

        # Pattern 1: Both errors grow with displacement, similar magnitude → frozen TF
        if (u_disp_corr is not None and t_disp_corr is not None and
                u_disp_corr > 0.3 and t_disp_corr > 0.3 and
                abs(u_growth - t_growth) < 0.5 * max(abs(u_growth), abs(t_growth), 0.01)):
            print("  >>> FROZEN TF detected: both UMAP_camera and Tracker_odom errors")
            print("      grow with robot displacement at similar rates.")
            print(f"      UMAP growth: {u_growth:.3f} m/m, Tracker growth: {t_growth:.3f} m/m")
            print("      Likely cause: camera_pose.py static transform not updating")
            print("      as robot moves, OR incorrect frame used in TF lookup.")
        # Pattern 2: UMAP error dominates and grows with speed (not displacement)
        elif u_err > 0.5 and (t_err is None or u_err > t_err * 1.5):
            u_vel_corr = growth.get('UMAP_camera_vs_GT_camera', {}).get('vel_corr', 0)
            if u_vel_corr and u_vel_corr > 0.3:
                print("  >>> UMAP/depth extraction errors are speed-correlated")
                print("      Motion blur or depth noise from robot movement likely.")
            else:
                print("  >>> UMAP/depth extraction is the PRIMARY error source")
                print("      (camera-frame error not explained by TF or motion)")
        # Pattern 3: Tracker odom error dominates → TF issue
        elif t_err is not None and t_err > 0.5 and (u_err is None or t_err > u_err * 1.5):
            print("  >>> camera→odom TF/stamp is the PRIMARY error source")
            print("      (odom-frame error much larger than camera-frame error)")
        # Pattern 4: Base error dominates
        elif b_err is not None and b_err > 0.5 and (t_err is None or b_err > t_err * 1.5):
            print("  >>> base_link transform or visualization frame is WRONG")
            print("      (base error grows significantly beyond odom error)")
        # Pattern 5: All low
        elif (u_err is None or u_err < 0.3) and (t_err is None or t_err < 0.3):
            print("  >>> All stages have low error — pipeline is frame-correct")
        else:
            print("  >>> MULTIPLE sources contribute — review detailed metrics above")
    else:
        print("  >>> Insufficient data for full isolation (check bag contents)")

    print("\n" + "=" * 72)


def save_outputs(results, odom_data, output_prefix):
    """Write timeseries CSV and summary JSON."""
    # CSV
    csv_path = f"{output_prefix}_timeseries.csv"
    with open(csv_path, 'w') as f:
        f.write("timestamp,comparison,distance_m\n")
        for r in results:
            if r is None:
                continue
            for ts, d in r['timeseries']:
                f.write(f"{ts:.6f},{r['name']},{d:.4f}\n")
    print(f"\nWrote {csv_path}")

    # JSON summary (strip timeseries for brevity)
    json_path = f"{output_prefix}_summary.json"
    growth = compute_error_growth(results, odom_data)
    summary = []
    for r in results:
        if r is None:
            continue
        s = dict(r)
        del s['timeseries']
        g = growth.get(r['name'], {})
        s['velocity_correlation'] = g.get('vel_corr')
        s['displacement_correlation'] = g.get('disp_corr')
        s['error_growth_m_per_m'] = g.get('growth_m_per_m')
        summary.append(s)

    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"Wrote {json_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Frame diagnostic analysis")
    parser.add_argument('--bag', required=True, help='Path to input rosbag')
    parser.add_argument('--output', default='diagnostic', help='Output file prefix')
    parser.add_argument('--match-threshold', type=float, default=1.0,
                        help='Max distance for obstacle association (m)')
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        sys.exit(f"Bag not found: {args.bag}")

    print(f"Loading bag: {args.bag}")
    data = load_bag(args.bag)

    # Map short names
    gt_camera   = data.get('/gt_obstacles_camera', [])
    gt_odom     = data.get('/obstacles', [])
    gt_base     = data.get('/gt_obstacles_base', [])
    umap_camera = data.get('/umap/3D_bounding_boxes', [])
    tracker_odom = data.get('/rematch/tracker_details', [])
    tracker_base = data.get('/tracker_details_base', [])
    odom_data   = data.get('/odom', [])

    results = []

    print("\nComparing UMAP_camera vs GT_camera ...")
    results.append(analyze_comparison(
        'UMAP_camera_vs_GT_camera', gt_camera, umap_camera, args.match_threshold))

    print("Comparing Tracker_odom vs GT_odom ...")
    results.append(analyze_comparison(
        'Tracker_odom_vs_GT_odom', gt_odom, tracker_odom, args.match_threshold))

    print("Comparing Tracker_base vs GT_base ...")
    results.append(analyze_comparison(
        'Tracker_base_vs_GT_base', gt_base, tracker_base, args.match_threshold))

    print_report(results, odom_data)
    save_outputs(results, odom_data, args.output)


if __name__ == '__main__':
    main()
