#!/usr/bin/env python3
"""Shared matching and FOV utilities for tracking evaluation scripts."""

import math

import numpy as np


def euclidean_distance(a, b):
    dx = float(a["x"]) - float(b["x"])
    dy = float(a["y"]) - float(b["y"])
    return math.hypot(dx, dy)


def linear_sum_assignment(cost_matrix):
    """Solve rectangular linear assignment with a pure-Python Hungarian method."""
    n_rows = len(cost_matrix)
    n_cols = len(cost_matrix[0]) if n_rows else 0
    if n_rows == 0 or n_cols == 0:
        return [], []

    transposed = False
    if n_rows > n_cols:
        transposed = True
        cost_matrix = [list(row) for row in zip(*cost_matrix)]
        n_rows, n_cols = n_cols, n_rows

    u = [0.0] * (n_rows + 1)
    v = [0.0] * (n_cols + 1)
    p = [0] * (n_cols + 1)
    way = [0] * (n_cols + 1)

    for i in range(1, n_rows + 1):
        p[0] = i
        j0 = 0
        minv = [float("inf")] * (n_cols + 1)
        used = [False] * (n_cols + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = float("inf")
            j1 = 0
            for j in range(1, n_cols + 1):
                if used[j]:
                    continue
                cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(n_cols + 1):
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

    row_ind = []
    col_ind = []
    for j in range(1, n_cols + 1):
        if p[j] == 0:
            continue
        r = p[j] - 1
        c = j - 1
        if transposed:
            row_ind.append(c)
            col_ind.append(r)
        else:
            row_ind.append(r)
            col_ind.append(c)
    return row_ind, col_ind


def match_detections(gt_list, trk_list, dist_threshold, preferred_pairs=None):
    """One-to-one matching with optional carry-over associations.

    Args:
        gt_list: list of GT dicts with id/x/y.
        trk_list: list of tracker dicts with id/x/y.
        dist_threshold: maximum admissible distance.
        preferred_pairs: optional dict {gt_id: tracker_id} from the previous
            frame. If the same pair is still within threshold, keep it before
            solving the remaining assignment.
    """
    matches = []
    used_gt = set()
    used_trk = set()
    gt_index = {int(g["id"]): i for i, g in enumerate(gt_list)}
    trk_index = {int(tr["id"]): j for j, tr in enumerate(trk_list)}

    if preferred_pairs:
        for gt_id, trk_id in preferred_pairs.items():
            i = gt_index.get(int(gt_id))
            j = trk_index.get(int(trk_id))
            if i is None or j is None:
                continue
            dist = euclidean_distance(gt_list[i], trk_list[j])
            if dist <= dist_threshold:
                matches.append((gt_list[i], trk_list[j], dist))
                used_gt.add(i)
                used_trk.add(j)

    rem_gt_idx = [i for i in range(len(gt_list)) if i not in used_gt]
    rem_trk_idx = [j for j in range(len(trk_list)) if j not in used_trk]

    if rem_gt_idx and rem_trk_idx:
        penalty = dist_threshold + 1e6
        cost_matrix = []
        for i in rem_gt_idx:
            row = []
            for j in rem_trk_idx:
                dist = euclidean_distance(gt_list[i], trk_list[j])
                row.append(dist if dist <= dist_threshold else penalty)
            cost_matrix.append(row)

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        for r, c in zip(row_ind, col_ind):
            i = rem_gt_idx[r]
            j = rem_trk_idx[c]
            dist = euclidean_distance(gt_list[i], trk_list[j])
            if dist <= dist_threshold:
                matches.append((gt_list[i], trk_list[j], dist))
                used_gt.add(i)
                used_trk.add(j)

    unmatched_gt = [gt_list[i] for i in range(len(gt_list)) if i not in used_gt]
    unmatched_trk = [trk_list[j] for j in range(len(trk_list)) if j not in used_trk]
    return matches, unmatched_gt, unmatched_trk


def build_fov_filter(
    bag_path,
    hfov_rad=math.radians(87.0),
    max_depth=6.0,
    min_depth=0.3,
):
    """Build a function that checks if an odom-frame point is in camera FOV."""
    import rosbag
    import tf.transformations as tft

    bag = rosbag.Bag(bag_path, "r")

    static_tfs = {}
    for _, msg, _ in bag.read_messages(topics=["/tf_static"]):
        for tr in msg.transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            q = tr.transform.rotation
            t = tr.transform.translation
            static_tfs[key] = (
                np.array([t.x, t.y, t.z]),
                np.array([q.x, q.y, q.z, q.w]),
            )

    odom_tfs = []
    for _, msg, _ in bag.read_messages(topics=["/tf"]):
        for tr in msg.transforms:
            if tr.header.frame_id == "odom" and tr.child_frame_id == "base_footprint":
                q = tr.transform.rotation
                t = tr.transform.translation
                odom_tfs.append((
                    tr.header.stamp.to_sec(),
                    np.array([t.x, t.y, t.z]),
                    np.array([q.x, q.y, q.z, q.w]),
                ))
    odom_tfs.sort(key=lambda x: x[0])
    bag.close()

    def compose(t1, q1, t2, q2):
        m1 = tft.quaternion_matrix(q1)
        m1[:3, 3] = t1
        m2 = tft.quaternion_matrix(q2)
        m2[:3, 3] = t2
        m = m1 @ m2
        return m[:3, 3], tft.quaternion_from_matrix(m)

    chain_keys = [
        ("base_footprint", "base_link"),
        ("base_link", "d435_bottom_screw_frame"),
        ("d435_bottom_screw_frame", "d435_link"),
        ("d435_link", "d435_color_frame"),
        ("d435_color_frame", "d435_color_optical_frame"),
    ]

    def get_odom_to_optical(ts):
        if not odom_tfs:
            return None
        idx = min(range(len(odom_tfs)), key=lambda i: abs(odom_tfs[i][0] - ts))
        _, t_acc, q_acc = odom_tfs[idx]
        for key in chain_keys:
            if key in static_tfs:
                t_s, q_s = static_tfs[key]
                t_acc, q_acc = compose(t_acc, q_acc, t_s, q_s)
        m = tft.quaternion_matrix(q_acc)
        m[:3, 3] = t_acc
        return m

    half_hfov = hfov_rad / 2.0

    def is_in_fov(ts, gx, gy):
        m = get_odom_to_optical(ts)
        if m is None:
            return True
        m_inv = np.linalg.inv(m)
        p_odom = np.array([gx, gy, 0.0, 1.0])
        p_cam = m_inv @ p_odom
        z = p_cam[2]
        if z < min_depth or z > max_depth:
            return False
        x = p_cam[0]
        if abs(math.atan2(x, z)) > half_hfov:
            return False
        return True

    return is_in_fov
