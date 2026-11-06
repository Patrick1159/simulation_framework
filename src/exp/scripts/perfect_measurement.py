#!/usr/bin/env python3
"""
perfect_measurement.py — Stage 1: TF-computed perfect camera measurement.

Computes p_cam_gt(t) = T_cam_odom(t) * p_odom_gt at each frame using TF,
and publishes it as fake /umap/3D_bounding_boxes. This simulates a
PERFECT depth extractor — zero measurement noise, exact TF geometry.

Also publishes /obstacles (GT in odom) for the frame bridges.

The test: if Tracker_odom stays constant at p_odom_gt and Tracker_base
correctly decreases as the robot approaches, the TF chain odom→camera→odom
is correct. If not, the problem is in the pipeline's TF/stamp/frame handling.

Parameters:
  ~obstacle_x/y/z    Fixed obstacle center position in odom frame
                     (default: 0.0, 2.0, 0.5)
  ~rate              Publish rate Hz (default: 15)
  ~stamp_offset      Artificial timestamp lag for Stage 4 tests (default: 0.0)
"""

import math
import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from mpc_nav.msg import Obstacle, ObstacleArray
from sensor_msgs.msg import Image
from umap.msg import BoundingBoxes3D, BoundingBoxes2D


class PerfectMeasurement:
    def __init__(self):
        self.obs_x = rospy.get_param('~obstacle_x', 0.0)
        self.obs_y = rospy.get_param('~obstacle_y', 2.0)
        self.obs_z = rospy.get_param('~obstacle_z', 0.5)
        self.rate_hz = rospy.get_param('~rate', 15.0)
        self.stamp_offset = rospy.get_param('~stamp_offset', 0.0)
        self.tf_timeout = rospy.get_param('~tf_timeout', 0.2)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Fake UMAP output (replaces umap_extract)
        self.pub_3d = rospy.Publisher('/umap/3D_bounding_boxes', BoundingBoxes3D,
                                       queue_size=1)
        self.pub_2d = rospy.Publisher('/umap/2D_bounding_boxes', BoundingBoxes2D,
                                       queue_size=1)

        # GT obstacle in odom (for frame bridges)
        self.pub_gt = rospy.Publisher('/obstacles', ObstacleArray, queue_size=1)

        rospy.loginfo("PerfectMeasurement: obstacle_odom=(%.2f, %.2f, %.2f) "
                       "rate=%.1f Hz", self.obs_x, self.obs_y, self.obs_z,
                       self.rate_hz)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz),
                                  self._publish)

    def _lookup_camera_tf(self, stamp):
        """Look up transform from odom to camera optical frame.
        Returns None on failure."""
        try:
            t = self.tf_buffer.lookup_transform(
                'd435_color_optical_frame',  # target
                'odom',                       # source
                stamp,
                rospy.Duration(self.tf_timeout))
            return t
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(5.0, "TF odom->camera failed: %s", str(e))
            return None

    @staticmethod
    def _apply_transform(pt_x, pt_y, pt_z, transform):
        """Apply TransformStamped to a point. Returns (x, y, z) in target frame."""
        q = transform.transform.rotation
        t = transform.transform.translation
        rot = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        v = [pt_x, pt_y, pt_z, 1.0]
        r = rot.dot(v)
        return (r[0] + t.x, r[1] + t.y, r[2] + t.z)

    def _publish(self, _event):
        now = rospy.Time.now()
        if self.stamp_offset != 0.0:
            stamp = now - rospy.Duration(self.stamp_offset)
        else:
            stamp = now

        xf = self._lookup_camera_tf(stamp)
        if xf is None:
            return

        # p_cam_gt = T_cam_odom * p_odom_gt
        cx, cy, cz = self._apply_transform(self.obs_x, self.obs_y, self.obs_z, xf)

        # 3D bbox
        bbox3d = BoundingBoxes3D()
        bbox3d.header.frame_id = 'd435_color_optical_frame'
        bbox3d.header.stamp = stamp
        bbox3d.image_width = 640
        bbox3d.image_height = 480
        bbox3d.box_ids = [0]
        bbox3d.x = [cx]
        bbox3d.y = [cy]
        bbox3d.z = [cz]
        bbox3d.width = [0.3]
        bbox3d.height = [0.3]
        bbox3d.thickness = [0.3]
        bbox3d.mask = [Image()]
        self.pub_3d.publish(bbox3d)

        # 2D bbox (dummy)
        bbox2d = BoundingBoxes2D()
        bbox2d.header.frame_id = 'd435_color_optical_frame'
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

        # GT obstacle in odom (same timestamp, for verification)
        gt = ObstacleArray()
        gt.header.frame_id = 'world'
        gt.header.stamp = stamp
        obs = Obstacle()
        obs.id = 0
        obs.position = Point(self.obs_x, self.obs_y, self.obs_z)
        obs.velocity = Vector3(0.0, 0.0, 0.0)
        obs.radius = 0.3
        obs.type = 1
        obs.confidence = 1.0
        gt.obstacles.append(obs)
        self.pub_gt.publish(gt)


if __name__ == '__main__':
    rospy.init_node('perfect_measurement')
    PerfectMeasurement()
    rospy.spin()
