#!/usr/bin/env python3
"""
extract_gt.py — Extract ground truth obstacle trajectories from rosbag.

Reads /obstacles (mpc_nav/ObstacleArray) and writes CSV:
  timestamp, id, x, y, vx, vy

Usage: python3 extract_gt.py --bag raw.bag --output gt.csv
"""

import argparse
import csv
import rosbag


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--topic", default="/obstacles")
    args = p.parse_args()

    bag = rosbag.Bag(args.bag, "r")
    with open(args.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "id", "x", "y", "vx", "vy", "radius"])
        for _, msg, t in bag.read_messages(topics=[args.topic]):
            ts = t.to_sec()
            for o in msg.obstacles:
                r = o.radius if hasattr(o, 'radius') else 0.0
                w.writerow([f"{ts:.6f}", o.id,
                            f"{o.position.x:.4f}", f"{o.position.y:.4f}",
                            f"{o.velocity.x:.4f}", f"{o.velocity.y:.4f}",
                            f"{r:.4f}"])
    bag.close()
    print(f"GT extracted to {args.output}")


if __name__ == "__main__":
    main()
