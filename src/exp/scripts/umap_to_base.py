#!/usr/bin/env python3
"""
umap_to_base.py — Transform UMAP BoundingBoxes3D centroids from camera
optical frame to base_link via TF lookup.

Publishes a lightweight topic /umap_base with x,y,z positions in base_link,
one per detected box. Used in Stage 2 analysis to compare UMAP output
directly against GT_base.
"""

import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from umap.msg import BoundingBoxes3D
from mpc_nav.msg import Obstacle, ObstacleArray


class UmapToBase:
    def __init__(self):
        self.target_frame = rospy.get_param('~target_frame', 'base_link')
        self.tf_timeout = rospy.get_param('~tf_timeout', 0.2)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher('/umap_base', ObstacleArray, queue_size=1)
        self.sub = rospy.Subscriber('/umap/3D_bounding_boxes', BoundingBoxes3D,
                                     self._callback, queue_size=1)
        rospy.loginfo("UmapToBase: /umap/3D_bounding_boxes -> /umap_base (%s)",
                       self.target_frame)

    def _callback(self, msg):
        source_frame = msg.header.frame_id
        stamp = msg.header.stamp

        try:
            xf = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, stamp,
                rospy.Duration(self.tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(5.0, "TF %s->%s failed: %s",
                                   source_frame, self.target_frame, str(e))
            return

        q = xf.transform.rotation
        t = xf.transform.translation
        rot = tft.quaternion_matrix([q.x, q.y, q.z, q.w])

        out = ObstacleArray()
        out.header.stamp = stamp
        out.header.frame_id = self.target_frame

        for i in range(len(msg.box_ids)):
            # Apply transform to centroid
            v = [msg.x[i], msg.y[i], msg.z[i], 1.0]
            r = rot.dot(v)
            obs = Obstacle()
            obs.id = msg.box_ids[i]
            obs.position.x = r[0] + t.x
            obs.position.y = r[1] + t.y
            obs.position.z = r[2] + t.z
            out.obstacles.append(obs)

        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('umap_to_base')
    UmapToBase()
    rospy.spin()
