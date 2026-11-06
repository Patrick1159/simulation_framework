#!/usr/bin/env python3
"""Publish a straight-line reference path start -> goal for contouring-based MPC.

Reads /scene/start_pose/{x,y} and /scene/goal_pose/{x,y} (loaded from the
experiment yaml) and publishes nav_msgs/Path on /input/reference_path
(latched, republished at ~1 Hz). The planner ignores a republished path when
the point count and first two points match (ros1_jackalsimulator.cpp
isPathTheSame), so the repeat is harmless.

The planner only uses pose.position.x/y of each pose (pathCallback) and never
checks frame_id; "odom" is used because the state comes from /odom and
world->odom is identity in this setup.
"""
import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def build_path(sx, sy, gx, gy, spacing, extension, frame):
    dx, dy = gx - sx, gy - sy
    length = math.hypot(dx, dy)
    if length < 1e-6:
        raise ValueError("start and goal coincide")
    ux, uy = dx / length, dy / length
    total = length + extension
    n = int(math.ceil(total / spacing))
    yaw = math.atan2(dy, dx)
    path = Path()
    path.header.frame_id = frame
    for i in range(n + 1):
        s = min(i * spacing, total)
        p = PoseStamped()
        p.header.frame_id = frame
        p.pose.position.x = sx + ux * s
        p.pose.position.y = sy + uy * s
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(p)
    return path


def main():
    rospy.init_node("reference_path_publisher")
    frame = rospy.get_param("~frame_id", "odom")
    spacing = rospy.get_param("~spacing", 0.5)
    extension = rospy.get_param("~extension", 1.0)  # metres past the goal

    rospy.loginfo("waiting for /scene/start_pose and /scene/goal_pose")
    while not rospy.is_shutdown() and not (
            rospy.has_param("/scene/start_pose/x") and rospy.has_param("/scene/goal_pose/x")):
        rospy.sleep(0.2)
    if rospy.is_shutdown():
        return
    sx = float(rospy.get_param("/scene/start_pose/x"))
    sy = float(rospy.get_param("/scene/start_pose/y"))
    gx = float(rospy.get_param("/scene/goal_pose/x"))
    gy = float(rospy.get_param("/scene/goal_pose/y"))

    path = build_path(sx, sy, gx, gy, spacing, extension, frame)
    pub = rospy.Publisher("/input/reference_path", Path, queue_size=1, latch=True)
    rospy.loginfo("reference path: (%.2f,%.2f)->(%.2f,%.2f) +%.1fm, %d pts, frame=%s",
                  sx, sy, gx, gy, extension, len(path.poses), frame)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        pub.publish(path)
        rate.sleep()


if __name__ == "__main__":
    main()
