#!/usr/bin/env python3
"""
relay_with_features.py — Relay umap::Obstacles adding dummy features.

The detailed_match node requires one features[] entry per box_id.
coarse_match doesn't populate features, so the raw topic_tools relay
causes [IdxMismatch] errors in detailed_match.

This node subscribes to /tracker/dynamic_obstacles, adds an empty
Float32MultiArray per box_id, and publishes to /model_output/obstacles.
"""

import rospy
from std_msgs.msg import Float32MultiArray
from umap.msg import Obstacles


class RelayWithFeatures:
    def __init__(self):
        self.pub = rospy.Publisher('/model_output/obstacles', Obstacles,
                                    queue_size=1)
        self.sub = rospy.Subscriber('/tracker/dynamic_obstacles', Obstacles,
                                     self._callback, queue_size=1)
        rospy.loginfo("RelayWithFeatures: /tracker/dynamic_obstacles -> "
                       "/model_output/obstacles (adding features)")

    def _callback(self, msg):
        # Ensure features array matches box_ids size
        n = len(msg.box_ids)
        if len(msg.features) < n:
            msg.features = list(msg.features) + [Float32MultiArray()] * (n - len(msg.features))
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('relay_with_features')
    RelayWithFeatures()
    rospy.spin()
