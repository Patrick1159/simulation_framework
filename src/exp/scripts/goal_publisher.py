#!/usr/bin/env python3
"""One-shot goal publisher for sup_exp experiments.

Reads goal pose from rosparam (under /scene/goal_pose, loaded by the
experiment launch from the active yaml config) and publishes a single
PoseStamped to /move_base_simple/goal after ~delay seconds.

This is needed because mpc_node.py subscribes to /move_base_simple/goal,
but the experiment launches do not include rviz "2D Nav Goal" by default.
"""

import math
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("goal_publisher")

    # Read params (with sensible fallbacks)
    delay = float(rospy.get_param("~delay", 3.0))
    topic = rospy.get_param("~topic", "/move_base_simple/goal")
    frame_id = rospy.get_param("~frame_id", "odom")

    gx = float(rospy.get_param("/scene/goal_pose/x", rospy.get_param("~x", 0.0)))
    gy = float(rospy.get_param("/scene/goal_pose/y", rospy.get_param("~y", 0.0)))
    gyaw = float(rospy.get_param("/scene/goal_pose/yaw",
                                 rospy.get_param("~yaw", 0.0)))

    pub = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=True)

    rospy.loginfo(
        "goal_publisher: will publish (x=%.2f y=%.2f yaw=%.2f) to %s after %.1fs",
        gx, gy, gyaw, topic, delay)

    # Sleep, then build & publish
    rospy.sleep(delay)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.pose.position.x = gx
    msg.pose.position.y = gy
    msg.pose.position.z = 0.0
    q = quaternion_from_euler(0.0, 0.0, gyaw)
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    pub.publish(msg)
    rospy.loginfo("goal_publisher: published goal; latching for late subscribers")

    # Keep node alive so the latched publisher stays available
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
