#!/usr/bin/env python3
"""
dodt_to_tmpc.py
===============
Bridge: onboard_detector (DODT baseline) → T-MPC planner.

Subscribes to /onboard_detector/tracked_bboxes (visualization_msgs/MarkerArray).
Using tracked_bboxes (all tracked objects) rather than dynamic_bboxes because:
  1. dynamic_bboxes requires ~10 frames of voting to classify an obstacle as
     dynamic — causes a multi-second blind spot at experiment start.
  2. dynamic_bboxes additionally applies constrain_size filtering which may
     reject cylinders whose detected dimensions deviate from target_object_size.
  tracked_bboxes contains all DBSCAN clusters with stable Kalman-filter tracking,
  which is the appropriate input for an MPC planner operating in simulation.

Each Marker is a LINE_LIST encoding a 3-D bounding box (24 points = 12 edges):
  - Position: last 24 points' centroid (upstream accumulation bug workaround)
  - Velocity:  parsed from marker.text " Vx <float> Vy <float>"
  - Frame:     "map" (identity with odom in simulation)

Publishes /doa_obstacles_tmpc (mpc_planner_msgs/ObstacleArray) with
constant-velocity predictions — same format as doa_to_tmpc.py.
"""

import math
import re
import rospy
import tf.transformations as tft
from visualization_msgs.msg import MarkerArray
from mpc_planner_msgs.msg import ObstacleArray, ObstacleGMM, Gaussian
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion


# Regex compiled once: matches " Vx <float> Vy <float>" (leading space, 6 decimals)
_VEL_RE = re.compile(r'Vx\s+([-\d.]+)\s+Vy\s+([-\d.]+)')


class DODTToMPCAdapter:
    def __init__(self):
        self.N = rospy.get_param('~N', 30)
        self.dt = rospy.get_param('~dt', 0.2)
        self.velocity_limit = rospy.get_param('~velocity_limit', 5.0)
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        # NOTE: onboard_detector publishes markers in frame "map" (hardcoded in
        # publish3dBox). We label the output ObstacleArray as target_frame="odom"
        # without a TF transform. This is safe in simulation because a static
        # identity TF map→odom (parent=map, child=odom) makes the two frames
        # numerically equivalent. For real deployment a proper TF lookup would
        # be needed.
        self.output_topic = rospy.get_param('~output_topic', '/doa_obstacles_tmpc')
        self.input_topic = rospy.get_param('~input_topic',
                                           '/onboard_detector/tracked_bboxes')
        self.watchdog_timeout = rospy.get_param('~watchdog_timeout', 0.5)
        self.watchdog_rate = rospy.get_param('~watchdog_rate', 5.0)

        self.pub = rospy.Publisher(self.output_topic, ObstacleArray, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, MarkerArray,
                                    self.callback, queue_size=1)

        self._last_callback_time = rospy.Time.now()
        self._received_first_msg = False
        self._watchdog_timer = rospy.Timer(
            rospy.Duration(1.0 / self.watchdog_rate),
            self._watchdog_callback)

        rospy.loginfo("DODTToMPCAdapter started")
        rospy.loginfo("  Subscribing  : %s", self.input_topic)
        rospy.loginfo("  Publishing   : %s", self.output_topic)
        rospy.loginfo("  Horizon      : N=%d, dt=%.2f s", self.N, self.dt)

    # ------------------------------------------------------------------

    def callback(self, msg):
        self._last_callback_time = rospy.Time.now()
        self._received_first_msg = True

        out_msg = ObstacleArray()
        # Use wall-clock time rather than msg.markers[0].header.stamp:
        # publish3dBox() never sets line.header.stamp, so upstream stamp is
        # always ros::Time(0). Using now() gives the ObstacleArray a valid
        # timestamp and keeps the watchdog elapsed-time calculation correct.
        out_msg.header.stamp = rospy.Time.now()
        out_msg.header.frame_id = self.target_frame

        for marker in msg.markers:
            # onboard_detector bug: line.points accumulates across obstacles in the
            # same publish cycle (never cleared between iterations of the for-loop in
            # publish3dBox). Marker i contains 24*(i+1) points: i boxes worth of
            # edges concatenated. The current box's 24 points are always the LAST 24.
            if len(marker.points) < 24:
                continue
            pts = marker.points[-24:]   # last 24 points = this box only
            cx = sum(p.x for p in pts) / 24
            cy = sum(p.y for p in pts) / 24
            cz = sum(p.z for p in pts) / 24

            # Parse velocity from marker.text: " Vx <float> Vy <float>"
            vx, vy = 0.0, 0.0
            m = _VEL_RE.search(marker.text)
            if m:
                try:
                    vx = float(m.group(1))
                    vy = float(m.group(2))
                except ValueError:
                    pass

            # Clamp speed
            v_mag = math.sqrt(vx * vx + vy * vy)
            if v_mag > self.velocity_limit:
                scale = self.velocity_limit / v_mag
                vx *= scale
                vy *= scale
                v_mag = self.velocity_limit

            # Build ObstacleGMM
            obs = ObstacleGMM()
            obs.id = marker.id

            obs.pose.position = Point(x=cx, y=cy, z=cz)
            if v_mag < 0.01:
                obs.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            else:
                yaw = math.atan2(vy, vx)
                q = tft.quaternion_from_euler(0, 0, yaw)
                obs.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            # Constant-velocity prediction (no uncertainty data available)
            gaussian = Gaussian()
            gaussian.mean.header.stamp = out_msg.header.stamp
            gaussian.mean.header.frame_id = self.target_frame
            for k in range(1, self.N):
                dt_k = self.dt * k
                ps = PoseStamped()
                ps.header.stamp = out_msg.header.stamp
                ps.header.frame_id = self.target_frame
                ps.pose.position = Point(x=cx + vx * dt_k,
                                         y=cy + vy * dt_k,
                                         z=cz)
                ps.pose.orientation = obs.pose.orientation
                gaussian.mean.poses.append(ps)
                gaussian.major_semiaxis.append(0.0)
                gaussian.minor_semiaxis.append(0.0)

            obs.gaussians.append(gaussian)
            obs.probabilities = [1.0]
            out_msg.obstacles.append(obs)

        self.pub.publish(out_msg)

    def _watchdog_callback(self, event):
        if not self._received_first_msg:
            return
        elapsed = rospy.Time.now() - self._last_callback_time
        if elapsed > rospy.Duration(self.watchdog_timeout):
            out_msg = ObstacleArray()
            out_msg.header.stamp = rospy.Time.now()
            out_msg.header.frame_id = self.target_frame
            self.pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('dodt_to_tmpc')
    adapter = DODTToMPCAdapter()
    rospy.spin()
