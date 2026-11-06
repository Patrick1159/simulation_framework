#!/usr/bin/env python3
"""
extract_cmd_vel.py -- Extract /cmd_vel from a rosbag into a CSV.

Outputs: timestamp, v, w, v_left, v_right
  v       = linear.x (forward velocity)
  w       = angular.z
  v_left  = left wheel velocity for differential drive
  v_right = right wheel velocity for differential drive

Wheel separation L = 0.16 m (TurtleBot3 Burger).
  v_left  = v - w * L/2
  v_right = v + w * L/2

Usage: python3 extract_cmd_vel.py raw.bag cmd_vel.csv
"""

import argparse
import csv
import rosbag

L = 0.16  # TurtleBot3 Burger wheel separation (m)


def extract(bag_path, output_path):
    bag = rosbag.Bag(bag_path)
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "v", "w", "v_left", "v_right"])
        for _, msg, t in bag.read_messages(topics=["/cmd_vel"]):
            ts = t.to_sec()
            v = msg.linear.x
            w = msg.angular.z
            half_L = L / 2.0
            v_left = v - w * half_L
            v_right = v + w * half_L
            writer.writerow([ts, v, w, v_left, v_right])
    bag.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="Path to input rosbag")
    parser.add_argument("output", help="Path to output CSV")
    args = parser.parse_args()
    extract(args.bag, args.output)
