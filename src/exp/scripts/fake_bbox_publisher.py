#!/usr/bin/env python3
"""
fake_bbox_publisher.py — Synthetic bounding-box publisher for TF diagnostics.

Publishes a single constant 3D point in a configurable camera frame,
bypassing UMAP/depth entirely. Used in Stage 0–5 diagnostic tests.

Publishes:
  /umap/3D_bounding_boxes  (umap/BoundingBoxes3D)
  /umap/2D_bounding_boxes  (umap/BoundingBoxes2D, dummy values)

Parameters:
  ~frame_id       Frame for header (default: d435_color_optical_frame)
  ~x, ~y, ~z      3D centroid in that frame (default: 0, 0, 0.5)
  ~rate           Publish rate Hz (default: 15)
  ~stamp_offset   Artificial timestamp lag in seconds (default: 0.0)
                  Used in Stage 4 to inject controlled timestamp errors.
"""

import rospy
import std_msgs.msg
from sensor_msgs.msg import Image
from umap.msg import BoundingBoxes3D, BoundingBoxes2D


class FakeBBoxPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'd435_color_optical_frame')
        self.x = rospy.get_param('~x', 0.0)
        self.y = rospy.get_param('~y', 0.0)
        self.z = rospy.get_param('~z', 0.5)
        self.rate_hz = rospy.get_param('~rate', 15.0)
        self.stamp_offset = rospy.get_param('~stamp_offset', 0.0)

        self.pub_3d = rospy.Publisher('/umap/3D_bounding_boxes', BoundingBoxes3D,
                                       queue_size=1)
        self.pub_2d = rospy.Publisher('/umap/2D_bounding_boxes', BoundingBoxes2D,
                                       queue_size=1)

        rospy.loginfo("FakeBBox: frame=%s pos=(%.2f, %.2f, %.2f) rate=%.1f Hz "
                       "stamp_offset=%.3fs",
                       self.frame_id, self.x, self.y, self.z,
                       self.rate_hz, self.stamp_offset)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._publish)

    def _publish(self, _event):
        now = rospy.Time.now()
        if self.stamp_offset != 0.0:
            stamp = now - rospy.Duration(self.stamp_offset)
        else:
            stamp = now

        # 3D bounding box
        bbox3d = BoundingBoxes3D()
        bbox3d.header.frame_id = self.frame_id
        bbox3d.header.stamp = stamp
        bbox3d.image_width = 640
        bbox3d.image_height = 480
        bbox3d.box_ids = [0]
        bbox3d.x = [self.x]
        bbox3d.y = [self.y]
        bbox3d.z = [self.z]
        bbox3d.width = [0.2]
        bbox3d.height = [0.2]
        bbox3d.thickness = [0.2]
        # coarse_match accesses mask[i] for each box_id; must match array size
        bbox3d.mask = [Image()]
        self.pub_3d.publish(bbox3d)

        # 2D bounding box (dummy — coarse_match needs both topics synced)
        bbox2d = BoundingBoxes2D()
        bbox2d.header.frame_id = self.frame_id
        bbox2d.header.stamp = stamp
        bbox2d.image_width = 640
        bbox2d.image_height = 480
        bbox2d.box_ids = [0]
        bbox2d.x = [300]
        bbox2d.y = [220]
        bbox2d.width = [40]
        bbox2d.height = [40]
        bbox2d.confidence = [1.0]
        self.pub_2d.publish(bbox2d)


if __name__ == '__main__':
    rospy.init_node('fake_bbox_publisher')
    FakeBBoxPublisher()
    rospy.spin()
