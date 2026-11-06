#!/usr/bin/env python3
"""
extract_tracker.py — Extract tracker output from rosbag into a common CSV.

Current DOA/rematch output is already in odom after the frame-honest refactor,
so `/rematch/tracker_details` and `/doa_obstacles` are treated as world-frame
topics by default. Use `--legacy-camera-frame` only for old bags recorded before
that refactor.

Usage: python3 extract_tracker.py --bag raw.bag --output tracker.csv
"""

import argparse
import csv
import math
import rosbag
import numpy as np
import tf.transformations as tft


def build_tf_cache(bag):
    """Build a time-sorted list of odom->camera transforms from /tf."""
    chain_frames = [
        ("odom", "base_footprint"),
        ("base_footprint", "base_link"),
        ("base_link", "d435_bottom_screw_frame"),
        ("d435_bottom_screw_frame", "d435_link"),
    ]
    # Collect static transforms first
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

    # Collect dynamic odom->base_footprint transforms
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
    return odom_tfs, static_tfs


def compose_transform(t1, q1, t2, q2):
    """Compose two transforms: T1 * T2."""
    M1 = tft.quaternion_matrix(q1)
    M1[:3, 3] = t1
    M2 = tft.quaternion_matrix(q2)
    M2[:3, 3] = t2
    M = M1 @ M2
    return M[:3, 3], tft.quaternion_from_matrix(M)


def get_odom_to_cam_at(ts, odom_tfs, static_tfs):
    """Get odom->d435_link transform at time ts."""
    # Find nearest odom->base_footprint
    if not odom_tfs:
        return None
    idx = min(range(len(odom_tfs)), key=lambda i: abs(odom_tfs[i][0] - ts))
    _, t_ob, q_ob = odom_tfs[idx]

    # Chain: odom->base_footprint * base_footprint->base_link * ... * ...->d435_link
    chain = [
        ("base_footprint", "base_link"),
        ("base_link", "d435_bottom_screw_frame"),
        ("d435_bottom_screw_frame", "d435_link"),
    ]
    t_acc, q_acc = t_ob, q_ob
    for key in chain:
        if key in static_tfs:
            t_s, q_s = static_tfs[key]
            t_acc, q_acc = compose_transform(t_acc, q_acc, t_s, q_s)

    return tft.quaternion_matrix(q_acc)[:3, :3], t_acc


def transform_point(rot, trans, px, py, pz):
    """Transform a point from camera body frame to odom frame."""
    p = np.array([px, py, pz])
    return rot @ p + trans


def extract_and_transform(bag, topic, odom_tfs, static_tfs, force_legacy_camera_frame=False):
    """Extract tracker data and transform to odom frame."""
    import re
    vel_pattern = re.compile(r"Vx\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*Vy\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)")

    rows = []
    prev = {}
    world_frame_topic = topic in ("/tracker/dynamic_obstacles", "/obstacles",
                                  "/doa_obstacles", "/rematch/tracker_details",
                                  "/onboard_detector/tracked_bboxes",
                                  "/onboard_detector/dynamic_bboxes",
                                  "/states")
    if force_legacy_camera_frame:
        world_frame_topic = False

    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = t.to_sec()
        # Keep the original project-specific TF pipeline for legacy topics.
        # For onboard baseline topic, data is already in world frame.
        if world_frame_topic:
            rot, trans = None, None
        else:
            tf_data = get_odom_to_cam_at(ts, odom_tfs, static_tfs)
            if tf_data is None:
                continue
            rot, trans = tf_data

        if hasattr(msg, 'obstacles'):
            # ObstacleArray: onboard world-frame vs legacy TF-based transform
            for o in msg.obstacles:
                if world_frame_topic:
                    p = np.array([o.position.x, o.position.y, o.position.z])
                    v = np.array([o.velocity.x, o.velocity.y, o.velocity.z])
                else:
                    p = transform_point(rot, trans, o.position.x, o.position.y, o.position.z)
                    v = rot @ np.array([o.velocity.x, o.velocity.y, o.velocity.z])
                rows.append([f"{ts:.6f}", o.id,
                             f"{p[0]:.4f}", f"{p[1]:.4f}",
                             f"{v[0]:.4f}", f"{v[1]:.4f}"])
        elif hasattr(msg, 'markers'):
            # MarkerArray (onboard_detector tracked/dynamic bboxes).
            # publish3dBox accumulates line.points across obstacles (never clears
            # between iterations), so marker i has 24*(i+1) points total.
            # The current box's 24 points are always the last 24.
            for marker in msg.markers:
                if len(marker.points) < 24:
                    continue
                pts = marker.points[-24:]
                cx = sum(pt.x for pt in pts) / 24
                cy = sum(pt.y for pt in pts) / 24
                mid = marker.id
                # Prefer Kalman velocity from marker.text if available;
                # fall back to finite difference for the frame when text is absent.
                vx, vy = 0.0, 0.0
                if marker.text:
                    m = vel_pattern.search(marker.text)
                    if m:
                        try:
                            vx, vy = float(m.group(1)), float(m.group(2))
                        except ValueError:
                            pass
                if vx == 0.0 and vy == 0.0 and mid in prev:
                    pt_prev, px_, py_ = prev[mid]
                    dt = ts - pt_prev
                    if dt > 0.001:
                        vx, vy = (cx - px_) / dt, (cy - py_) / dt
                prev[mid] = (ts, cx, cy)
                rows.append([f"{ts:.6f}", mid,
                             f"{cx:.4f}", f"{cy:.4f}",
                             f"{vx:.4f}", f"{vy:.4f}"])
        elif hasattr(msg, 'trackers'):
            # umap/TrackerDetailArray
            for td in msg.trackers:
                if world_frame_topic:
                    p = np.array([td.position.x, td.position.y, td.position.z])
                    v = np.array([td.velocity.x, td.velocity.y, td.velocity.z])
                else:
                    p = transform_point(rot, trans, td.position.x, td.position.y, td.position.z)
                    v = rot @ np.array([td.velocity.x, td.velocity.y, td.velocity.z])
                rows.append([f"{ts:.6f}", td.tracker_id,
                             f"{p[0]:.4f}", f"{p[1]:.4f}",
                             f"{v[0]:.4f}", f"{v[1]:.4f}"])
        elif hasattr(msg, 'states'):
            # FAPP obj_state_msgs/ObjectsStates has no explicit object ID.
            # Use per-message index as the best available stable identifier.
            for idx, state in enumerate(msg.states):
                if world_frame_topic:
                    p = np.array([state.position.x, state.position.y, state.position.z])
                    v = np.array([state.velocity.x, state.velocity.y, state.velocity.z])
                else:
                    p = transform_point(rot, trans, state.position.x, state.position.y, state.position.z)
                    v = rot @ np.array([state.velocity.x, state.velocity.y, state.velocity.z])
                rows.append([f"{ts:.6f}", idx,
                             f"{p[0]:.4f}", f"{p[1]:.4f}",
                             f"{v[0]:.4f}", f"{v[1]:.4f}"])
        elif hasattr(msg, 'box_ids'):
            # umap/Obstacles — current coarse tracker output is in odom;
            # legacy bags can still request camera-frame transform via CLI.
            for i, bid in enumerate(msg.box_ids):
                cx, cy, cz = msg.centroid_x[i], msg.centroid_y[i], msg.centroid_z[i]
                if world_frame_topic:
                    p = np.array([cx, cy, cz])
                else:
                    p = transform_point(rot, trans, cx, cy, cz)
                vx, vy = 0.0, 0.0
                if bid in prev:
                    pt, px_, py_ = prev[bid]
                    dt = ts - pt
                    if dt > 0.001:
                        vx, vy = (p[0] - px_) / dt, (p[1] - py_) / dt
                prev[bid] = (ts, p[0], p[1])
                rows.append([f"{ts:.6f}", bid, f"{p[0]:.4f}", f"{p[1]:.4f}",
                             f"{vx:.4f}", f"{vy:.4f}"])
    return rows


def cluster_per_frame(rows, merge_dist=0.5):
    """Cluster nearby detections within each frame, with temporal ID tracking.

    DOA coarse match fragments one physical obstacle into multiple box IDs.
    Merge detections within merge_dist, then track clusters across frames
    by nearest-neighbor matching to maintain consistent IDs.
    """
    from collections import defaultdict

    frames = defaultdict(list)
    for r in rows:
        frames[r[0]].append(r)  # group by timestamp string

    out = []
    prev_clusters = []  # list of (x, y, assigned_id)
    next_id = 0

    for ts in sorted(frames.keys()):
        pts = frames[ts]
        coords = [(float(p[2]), float(p[3])) for p in pts]
        vels = [(float(p[4]), float(p[5])) for p in pts]
        used = [False] * len(pts)
        clusters = []

        for i in range(len(pts)):
            if used[i]:
                continue
            cx, cy = [coords[i][0]], [coords[i][1]]
            cvx, cvy = [vels[i][0]], [vels[i][1]]
            used[i] = True
            for j in range(i + 1, len(pts)):
                if used[j]:
                    continue
                dx = coords[j][0] - np.mean(cx)
                dy = coords[j][1] - np.mean(cy)
                if math.sqrt(dx * dx + dy * dy) < merge_dist:
                    cx.append(coords[j][0])
                    cy.append(coords[j][1])
                    cvx.append(vels[j][0])
                    cvy.append(vels[j][1])
                    used[j] = True
            clusters.append((np.mean(cx), np.mean(cy),
                             np.mean(cvx), np.mean(cvy)))

        # Match to previous frame's clusters by nearest distance
        assigned = [None] * len(clusters)
        if prev_clusters:
            used_prev = set()
            pairs = []
            for ci, (x, y, _, _) in enumerate(clusters):
                for pi, (px, py, pid) in enumerate(prev_clusters):
                    d = math.sqrt((x - px) ** 2 + (y - py) ** 2)
                    pairs.append((d, ci, pi))
            pairs.sort()
            for d, ci, pi in pairs:
                if ci in [a[0] for a in [(i, assigned[i]) for i in range(len(assigned)) if assigned[i] is not None]]:
                    continue
                if pi in used_prev:
                    continue
                if assigned[ci] is not None:
                    continue
                if d < 1.0:  # max association distance
                    assigned[ci] = prev_clusters[pi][2]
                    used_prev.add(pi)

        # Assign new IDs to unmatched clusters
        for ci in range(len(clusters)):
            if assigned[ci] is None:
                assigned[ci] = next_id
                next_id += 1

        prev_clusters = [(clusters[ci][0], clusters[ci][1], assigned[ci])
                         for ci in range(len(clusters))]

        for ci, (x, y, vx, vy) in enumerate(clusters):
            out.append([ts, assigned[ci], f"{x:.4f}", f"{y:.4f}",
                        f"{vx:.4f}", f"{vy:.4f}"])
    return out


def has_msgs(topics, name):
    return name in topics and topics[name].message_count > 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--topic", default=None)
    p.add_argument("--no-cluster", action="store_true",
                   help="Skip spatial clustering (use when source already has stable IDs)")
    p.add_argument("--legacy-camera-frame", action="store_true",
                   help="Force legacy camera-body to odom transform for old DOA bags")
    args = p.parse_args()

    bag = rosbag.Bag(args.bag, "r")
    topics = bag.get_type_and_topic_info().topics

    topic = args.topic
    if topic is None:
        if has_msgs(topics, "/doa_obstacles"):
            topic = "/doa_obstacles"
        elif has_msgs(topics, "/rematch/tracker_details"):
            topic = "/rematch/tracker_details"
        elif has_msgs(topics, "/onboard_detector/tracked_bboxes"):
            topic = "/onboard_detector/tracked_bboxes"
        elif has_msgs(topics, "/onboard_detector/dynamic_bboxes"):
            topic = "/onboard_detector/dynamic_bboxes"
        elif has_msgs(topics, "/tracker/dynamic_obstacles"):
            topic = "/tracker/dynamic_obstacles"
        elif has_msgs(topics, "/states"):
            topic = "/states"
        elif has_msgs(topics, "/obstacles"):
            topic = "/obstacles"
        else:
            print("No tracker topic found in bag")
            bag.close()
            return

    print(f"Using topic: {topic}")
    if args.legacy_camera_frame:
        print("Forcing legacy camera-frame transform mode.")
    print("Building TF cache...")
    odom_tfs, static_tfs = build_tf_cache(bag)
    print(f"  {len(odom_tfs)} dynamic TFs, {len(static_tfs)} static TFs")

    rows = extract_and_transform(
        bag, topic, odom_tfs, static_tfs,
        force_legacy_camera_frame=args.legacy_camera_frame,
    )
    bag.close()

    print(f"  Raw detections: {len(rows)}")
    if not args.no_cluster:
        rows = cluster_per_frame(rows)
        print(f"  After clustering: {len(rows)}")
    else:
        print("  Skipping clustering (--no-cluster)")

    with open(args.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "id", "x", "y", "vx", "vy"])
        w.writerows(rows)
    print(f"Tracker extracted: {len(rows)} rows to {args.output}")


if __name__ == "__main__":
    main()
