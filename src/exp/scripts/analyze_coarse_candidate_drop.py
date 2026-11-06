#!/usr/bin/env python3
"""
Analyze where the GT-near candidate is lost between:
  UMAP camera detections -> coarse /tracker/dynamic_obstacles -> rematch

Focus:
  - Does a GT-near UMAP candidate exist?
  - If yes, does a GT-near coarse published obstacle exist?
  - If no, which IDs/confidences are being published instead?

Outputs:
  - console report
  - <prefix>_coarse_candidate_drop.csv
"""

import argparse
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag


TIME_TOL = 0.08


def msg_time(msg, bag_time):
    try:
        stamp = msg.header.stamp.to_sec()
        if stamp > 0.0:
            return stamp
    except Exception:
        pass
    return bag_time.to_sec()


def nearest(ts, rows):
    if not rows:
        return None
    best = min(rows, key=lambda r: abs(r["t"] - ts))
    if abs(best["t"] - ts) > TIME_TOL:
        return None
    return best


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def load_bag(path):
    data = {
        "gt_camera": [],
        "gt_odom": [],
        "umap": [],
        "coarse": [],
        "rematch": [],
    }
    with rosbag.Bag(path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[
            "/gt_obstacles_camera",
            "/obstacles",
            "/umap/3D_bounding_boxes",
            "/tracker/dynamic_obstacles",
            "/rematch/tracker_details",
        ]):
            ts = msg_time(msg, t)
            if topic == "/gt_obstacles_camera" and msg.obstacles:
                o = msg.obstacles[0]
                data["gt_camera"].append({"t": ts, "pt": (o.position.x, o.position.y, o.position.z)})
            elif topic == "/obstacles" and msg.obstacles:
                o = msg.obstacles[0]
                data["gt_odom"].append({"t": ts, "pt": (o.position.x, o.position.y, o.position.z)})
            elif topic == "/umap/3D_bounding_boxes":
                pts = [(msg.x[i], msg.y[i], msg.z[i]) for i in range(len(msg.x))]
                data["umap"].append({"t": ts, "pts": pts})
            elif topic == "/tracker/dynamic_obstacles":
                pts = []
                for i in range(len(msg.box_ids)):
                    pts.append({
                        "id": int(msg.box_ids[i]),
                        "pt": (msg.centroid_x[i], msg.centroid_y[i], msg.centroid_z[i]),
                        "confidence": float(msg.confidence[i]) if i < len(msg.confidence) else float("nan"),
                        "observed": bool(msg.observed[i]) if i < len(msg.observed) else False,
                    })
                data["coarse"].append({"t": ts, "pts": pts})
            elif topic == "/rematch/tracker_details":
                pts = []
                for tr in msg.trackers:
                    pts.append({
                        "id": int(tr.tracker_id),
                        "pt": (tr.position.x, tr.position.y, tr.position.z),
                        "age": float(tr.age),
                        "confidence": float(tr.confidence),
                    })
                data["rematch"].append({"t": ts, "pts": pts})
    return data


def nearest_point(ref, points):
    if not points:
        return None, None
    best = min(points, key=lambda p: dist(ref, p["pt"] if isinstance(p, dict) else p))
    pt = best["pt"] if isinstance(best, dict) else best
    return best, dist(ref, pt)


def analyze(data):
    rows = []
    for gt_cam in data["gt_camera"]:
        ts = gt_cam["t"]
        gt_odom = nearest(ts, data["gt_odom"])
        umap = nearest(ts, data["umap"])
        coarse = nearest(ts, data["coarse"])
        rematch = nearest(ts, data["rematch"])
        if gt_odom is None or umap is None:
            continue

        best_umap = None
        best_umap_err = None
        if umap["pts"]:
            best_umap, best_umap_err = nearest_point(gt_cam["pt"], umap["pts"])

        best_coarse = None
        best_coarse_err = None
        if coarse is not None and coarse["pts"]:
            best_coarse, best_coarse_err = nearest_point(gt_odom["pt"], coarse["pts"])

        best_rematch = None
        best_rematch_err = None
        if rematch is not None and rematch["pts"]:
            best_rematch, best_rematch_err = nearest_point(gt_odom["pt"], rematch["pts"])

        best_coarse_id = ""
        best_coarse_conf = ""
        best_coarse_observed = ""
        if best_coarse is not None:
            best_coarse_id = str(best_coarse["id"])
            best_coarse_conf = f"{best_coarse['confidence']:.3f}"
            best_coarse_observed = "1" if best_coarse["observed"] else "0"

        best_rematch_id = ""
        best_rematch_age = ""
        best_rematch_conf = ""
        if best_rematch is not None:
            best_rematch_id = str(best_rematch["id"])
            best_rematch_age = f"{best_rematch['age']:.3f}"
            best_rematch_conf = f"{best_rematch['confidence']:.3f}"

        rows.append({
            "t": ts,
            "gt_cam_x": gt_cam["pt"][0],
            "gt_odom_x": gt_odom["pt"][0],
            "umap_count": len(umap["pts"]),
            "best_umap_err": best_umap_err,
            "coarse_count": 0 if coarse is None else len(coarse["pts"]),
            "best_coarse_err": best_coarse_err,
            "coarse_ids": "" if coarse is None else ";".join(str(p["id"]) for p in coarse["pts"]),
            "coarse_conf": "" if coarse is None else ";".join(f"{p['confidence']:.1f}" for p in coarse["pts"]),
            "coarse_obs": "" if coarse is None else ";".join("1" if p["observed"] else "0" for p in coarse["pts"]),
            "best_coarse_id": best_coarse_id,
            "best_coarse_conf": best_coarse_conf,
            "best_coarse_observed": best_coarse_observed,
            "best_rematch_err": best_rematch_err,
            "rematch_ids": "" if rematch is None else ";".join(str(p["id"]) for p in rematch["pts"]),
            "rematch_age": "" if rematch is None else ";".join(f"{p['age']:.3f}" for p in rematch["pts"]),
            "best_rematch_id": best_rematch_id,
            "best_rematch_age": best_rematch_age,
            "best_rematch_conf": best_rematch_conf,
        })
    return rows


def summarize(rows):
    gt_near_umap = [r for r in rows if r["best_umap_err"] is not None and r["best_umap_err"] < 0.30]
    gt_near_coarse = [r for r in rows if r["best_coarse_err"] is not None and r["best_coarse_err"] < 0.30]
    gt_near_rematch = [r for r in rows if r["best_rematch_err"] is not None and r["best_rematch_err"] < 0.30]
    lost_after_umap = [
        r for r in rows
        if r["best_umap_err"] is not None and r["best_umap_err"] < 0.30
        and not (r["best_coarse_err"] is not None and r["best_coarse_err"] < 0.30)
    ]
    print(f"frames analyzed: {len(rows)}")
    print(f"GT-near UMAP candidate frames: {len(gt_near_umap)}")
    print(f"GT-near coarse published frames: {len(gt_near_coarse)}")
    print(f"GT-near rematch frames: {len(gt_near_rematch)}")
    print(f"GT-near UMAP but lost before coarse publish: {len(lost_after_umap)}")
    if lost_after_umap:
        print("\nFirst lost-after-UMAP frames:")
        for r in lost_after_umap[:10]:
            print(
                f"  t={r['t']:.3f} umap_err={r['best_umap_err']:.3f} "
                f"coarse_err={r['best_coarse_err'] if r['best_coarse_err'] is not None else 'NA'} "
                f"coarse_ids={r['coarse_ids']} conf={r['coarse_conf']} obs={r['coarse_obs']} "
                f"rematch_err={r['best_rematch_err'] if r['best_rematch_err'] is not None else 'NA'} "
                f"rematch_ids={r['rematch_ids']} age={r['rematch_age']}"
            )


def write_csv(path, rows):
    fieldnames = [
        "t", "gt_cam_x", "gt_odom_x", "umap_count", "best_umap_err",
        "coarse_count", "best_coarse_err", "coarse_ids", "coarse_conf",
        "coarse_obs", "best_coarse_id", "best_coarse_conf", "best_coarse_observed",
        "best_rematch_err", "rematch_ids", "rematch_age", "best_rematch_id",
        "best_rematch_age", "best_rematch_conf",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def render_presence_plot(prefix, rows):
    ts = [r["t"] for r in rows]
    umap_ok = [1 if r["best_umap_err"] is not None and r["best_umap_err"] < 0.30 else 0 for r in rows]
    coarse_ok = [1 if r["best_coarse_err"] is not None and r["best_coarse_err"] < 0.30 else 0 for r in rows]
    rematch_ok = [1 if r["best_rematch_err"] is not None and r["best_rematch_err"] < 0.30 else 0 for r in rows]

    fig, ax = plt.subplots(figsize=(12, 3.8))
    ax.step(ts, [v + 2 for v in umap_ok], where="post", linewidth=2, label="UMAP has GT-near candidate")
    ax.step(ts, [v + 1 for v in coarse_ok], where="post", linewidth=2, label="Coarse publishes GT-near candidate")
    ax.step(ts, rematch_ok, where="post", linewidth=2, label="Rematch has GT-near track")
    ax.set_yticks([0, 1, 2, 3])
    ax.set_yticklabels(["Rematch: no/yes", "", "Coarse: no/yes", "UMAP: no/yes"])
    ax.set_xlabel("Time (s)")
    ax.set_title("GT-near candidate survival timeline")
    ax.grid(True, axis="x", alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(f"{prefix}_coarse_candidate_timeline.png", dpi=150)
    plt.close(fig)


def render_error_plot(prefix, rows):
    ts = [r["t"] for r in rows]
    umap_err = [r["best_umap_err"] if r["best_umap_err"] is not None else float("nan") for r in rows]
    coarse_err = [r["best_coarse_err"] if r["best_coarse_err"] is not None else float("nan") for r in rows]
    rematch_err = [r["best_rematch_err"] if r["best_rematch_err"] is not None else float("nan") for r in rows]
    coarse_conf = [float(r["best_coarse_conf"]) if r["best_coarse_conf"] != "" else float("nan") for r in rows]
    coarse_id = [float(r["best_coarse_id"]) if r["best_coarse_id"] != "" else float("nan") for r in rows]

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(ts, umap_err, linewidth=1.8, label="best UMAP err")
    axes[0].plot(ts, coarse_err, linewidth=1.8, label="best coarse err")
    axes[0].plot(ts, rematch_err, linewidth=1.8, label="best rematch err")
    axes[0].axhline(0.30, color="k", linestyle="--", linewidth=1.0, alpha=0.7)
    axes[0].set_ylabel("3D error (m)")
    axes[0].set_title("Nearest-to-GT candidate error by stage")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(ts, coarse_conf, linewidth=1.8, color="tab:orange")
    axes[1].set_ylabel("best coarse conf")
    axes[1].set_ylim(-0.05, 1.05)
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(ts, coarse_id, linewidth=1.8, color="tab:red")
    axes[2].set_ylabel("best coarse id")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(f"{prefix}_coarse_candidate_errors.png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Analyze coarse candidate drop between UMAP and coarse/rematch")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", default="coarse_drop")
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        sys.exit(f"Bag not found: {args.bag}")

    data = load_bag(args.bag)
    rows = analyze(data)
    summarize(rows)
    write_csv(f"{args.output}_coarse_candidate_drop.csv", rows)
    render_presence_plot(args.output, rows)
    render_error_plot(args.output, rows)
    print(f"\nWrote {args.output}_coarse_candidate_drop.csv")
    print(f"Wrote {args.output}_coarse_candidate_timeline.png")
    print(f"Wrote {args.output}_coarse_candidate_errors.png")


if __name__ == "__main__":
    main()
