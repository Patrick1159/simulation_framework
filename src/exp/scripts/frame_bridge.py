#!/usr/bin/env python3
"""
frame_bridge.py — TF-based message republisher for frame diagnostics.

Bridges a source topic from one frame to another via TF lookup.
Supports mpc_nav/ObstacleArray and umap/TrackerDetailArray.

Three instances cover the diagnostic experiment:
  GT -> camera:  /obstacles (world)           -> /gt_obstacles_camera (d435_color_optical_frame)
  GT -> base:    /obstacles (world)           -> /gt_obstacles_base   (base_link)
  Tracker->base: /rematch/tracker_details (odom) -> /tracker_details_base (base_link)
"""

import math
import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import Point, Vector3
from mpc_nav.msg import ObstacleArray
from umap.msg import TrackerDetailArray


class FrameBridge:
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic')
        self.output_topic = rospy.get_param('~output_topic')
        self.target_frame = rospy.get_param('~target_frame')
        self.tf_timeout = rospy.get_param('~tf_timeout', 0.2)
        self.drop_on_fail = rospy.get_param('~drop_on_fail', True)
        self.msg_type = rospy.get_param('~msg_type', 'ObstacleArray')

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        if self.msg_type == 'ObstacleArray':
            self.pub = rospy.Publisher(self.output_topic, ObstacleArray, queue_size=1)
            self.sub = rospy.Subscriber(self.input_topic, ObstacleArray,
                                        self._obstacle_cb, queue_size=1)
        elif self.msg_type == 'TrackerDetailArray':
            self.pub = rospy.Publisher(self.output_topic, TrackerDetailArray, queue_size=1)
            self.sub = rospy.Subscriber(self.input_topic, TrackerDetailArray,
                                        self._tracker_cb, queue_size=1)
        else:
            raise ValueError(f"Unknown msg_type: {self.msg_type}")

        rospy.loginfo("FrameBridge: %s (%s) -> %s (%s)",
                       self.input_topic, self.msg_type, self.output_topic, self.target_frame)

    def _get_transform(self, source_frame, stamp):
        """Look up TF source_frame -> target_frame. Returns None on failure."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, stamp,
                rospy.Duration(self.tf_timeout))
            return t
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(2.0, "TF lookup %s->%s at %.3f failed: %s",
                                   source_frame, self.target_frame,
                                   stamp.to_sec(), str(e))
            return None

    @staticmethod
    def _transform_point(pt, transform):
        q = transform.transform.rotation
        t = transform.transform.translation
        rot = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        v = [pt.x, pt.y, pt.z, 1.0]
        r = rot.dot(v)
        return Point(x=r[0] + t.x, y=r[1] + t.y, z=r[2] + t.z)

    @staticmethod
    def _transform_vector(vec, transform):
        q = transform.transform.rotation
        rot = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        v = [vec.x, vec.y, vec.z, 0.0]
        r = rot.dot(v)
        return Vector3(x=r[0], y=r[1], z=r[2])

    def _obstacle_cb(self, msg):
        source_frame = msg.header.frame_id
        stamp = msg.header.stamp
        xf = self._get_transform(source_frame, stamp)
        if xf is None:
            if self.drop_on_fail:
                return
            xf = None  # pass-through below

        out = ObstacleArray()
        out.header.stamp = stamp
        out.header.frame_id = self.target_frame

        for obs in msg.obstacles:
            if xf is not None:
                obs.position = self._transform_point(obs.position, xf)
                obs.velocity = self._transform_vector(obs.velocity, xf)
            out.obstacles.append(obs)

        self.pub.publish(out)

    def _tracker_cb(self, msg):
        source_frame = msg.header.frame_id
        stamp = msg.header.stamp
        xf = self._get_transform(source_frame, stamp)
        if xf is None:
            if self.drop_on_fail:
                return
            xf = None

        out = TrackerDetailArray()
        out.header.stamp = stamp
        out.header.frame_id = self.target_frame

        for td in msg.trackers:
            if xf is not None:
                td.position = self._transform_point(td.position, xf)
                td.velocity = self._transform_vector(td.velocity, xf)
            out.trackers.append(td)

        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('frame_bridge', anonymous=True)
    FrameBridge()
    rospy.spin()
