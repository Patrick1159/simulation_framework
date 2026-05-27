#!/usr/bin/env python3

import math
import rospy
import tf.transformations as tft
from umap.msg import TrackerDetailArray
from mpc_planner_msgs.msg import ObstacleArray, ObstacleGMM, Gaussian
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray


class DOAToMPCAdapter:
    """Converts DOA perception output (TrackerDetailArray) to MPC planner
    input format (ObstacleArray with GMM predictions)."""

    def __init__(self):
        # --- ROS parameters ---
        self.N = rospy.get_param('~N', 30)
        self.dt = rospy.get_param('~dt', 0.2)
        self.max_age = rospy.get_param('~max_age', 0.5)
        self.velocity_limit = rospy.get_param('~velocity_limit', 5.0)
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        self.output_topic = rospy.get_param('~output_topic', '/doa_obstacles_tmpc')
        self.input_topic = rospy.get_param('~input_topic', '/rematch/tracker_details')
        self.publish_markers = rospy.get_param('~publish_markers', True)
        self.marker_topic = rospy.get_param('~marker_topic', '/doa_obstacles_tmpc_markers')
        self.watchdog_timeout = rospy.get_param('~watchdog_timeout', 0.5)
        self.watchdog_rate = rospy.get_param('~watchdog_rate', 5.0)
        assert self.watchdog_timeout > 0.0, "watchdog_timeout must be positive"
        assert self.watchdog_rate > 0.0, "watchdog_rate must be positive"

        # --- Publishers and subscriber ---
        self.pub = rospy.Publisher(self.output_topic, ObstacleArray, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, TrackerDetailArray,
                                     self.callback, queue_size=1)

        if self.publish_markers:
            self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray,
                                              queue_size=1)

        # Watchdog timer: publishes empty ObstacleArray when DOA goes silent,
        # so MPC's ensureObstacleSize can pad to max_obstacles and pass isDataReady.
        self._last_callback_time = rospy.Time.now()
        self._received_first_msg = False
        self._watchdog_timer = rospy.Timer(
            rospy.Duration(1.0 / self.watchdog_rate),
            self._watchdog_callback)

        rospy.loginfo("DOAToMPCAdapter started")
        rospy.loginfo("  Subscribing  : %s", self.input_topic)
        rospy.loginfo("  Publishing   : %s", self.output_topic)
        if self.publish_markers:
            rospy.loginfo("  Markers      : %s", self.marker_topic)
        rospy.loginfo("  Horizon      : N=%d, dt=%.2f s", self.N, self.dt)
        rospy.loginfo("  Filters      : max_age=%.2f s, velocity_limit=%.1f m/s",
                       self.max_age, self.velocity_limit)
        rospy.loginfo("  Watchdog     : timeout=%.2f s, rate=%.1f Hz",
                       self.watchdog_timeout, self.watchdog_rate)

    def callback(self, msg):
        self._last_callback_time = rospy.Time.now()
        self._received_first_msg = True
        out_msg = ObstacleArray()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = self.target_frame

        marker_array = MarkerArray() if self.publish_markers else None

        for idx, tracker in enumerate(msg.trackers):
            # 1. Skip trackers older than max_age
            if tracker.age > self.max_age:
                continue

            # 2. Create obstacle with id
            obs = ObstacleGMM()
            obs.id = tracker.tracker_id

            # 3. Set current position
            obs.pose.position = tracker.position

            # 4. Set orientation from velocity direction
            vx = tracker.velocity.x
            vy = tracker.velocity.y
            v_mag = math.sqrt(vx * vx + vy * vy)

            if v_mag < 0.01:
                obs.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                yaw = 0.0
            else:
                yaw = math.atan2(vy, vx)
                q = tft.quaternion_from_euler(0, 0, yaw)
                obs.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            # 5. Clamp velocity magnitude
            if v_mag > self.velocity_limit:
                scale = self.velocity_limit / v_mag
                cvx = vx * scale
                cvy = vy * scale
            else:
                cvx = vx
                cvy = vy

            # 6. Build prediction for k = 1 .. N-1
            gaussian = Gaussian()
            gaussian.mean.header.stamp = msg.header.stamp
            gaussian.mean.header.frame_id = self.target_frame

            has_uncertainty = (tracker.sigma2_xx > 0.0)

            for k in range(1, self.N):
                dt_k = self.dt * k

                px_k = tracker.position.x + cvx * dt_k
                py_k = tracker.position.y + cvy * dt_k

                ps = PoseStamped()
                ps.header.stamp = msg.header.stamp
                ps.header.frame_id = self.target_frame
                ps.pose.position = Point(x=px_k, y=py_k, z=tracker.position.z)
                ps.pose.orientation = obs.pose.orientation

                gaussian.mean.poses.append(ps)

                if has_uncertainty:
                    sigma_x = math.sqrt(tracker.sigma2_xx +
                                        (dt_k * dt_k) * tracker.sigma2_vxvx)
                    sigma_y = math.sqrt(tracker.sigma2_yy +
                                        (dt_k * dt_k) * tracker.sigma2_vyvy)
                    gaussian.major_semiaxis.append(max(sigma_x, sigma_y))
                    gaussian.minor_semiaxis.append(min(sigma_x, sigma_y))
                else:
                    gaussian.major_semiaxis.append(0.0)
                    gaussian.minor_semiaxis.append(0.0)

            obs.gaussians.append(gaussian)
            obs.probabilities = [1.0]

            out_msg.obstacles.append(obs)

            # --- Visualization markers ---
            if self.publish_markers:
                # SPHERE marker at current position, colored by confidence
                sphere = Marker()
                sphere.header = out_msg.header
                sphere.ns = "doa_tmpc"
                sphere.id = idx * 2
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose = obs.pose
                box_x = max(float(tracker.box_dimensions.x), 0.1)
                box_y = max(float(tracker.box_dimensions.y), 0.1)
                box_z = max(float(tracker.box_dimensions.z), 0.1)
                sphere.scale.x = box_x
                sphere.scale.y = box_y
                sphere.scale.z = box_z
                conf = tracker.confidence
                sphere.color.r = 1.0 - conf
                sphere.color.g = conf
                sphere.color.b = 0.0
                sphere.color.a = 0.6
                sphere.lifetime = rospy.Duration(0.2)
                marker_array.markers.append(sphere)

                # ARROW marker showing velocity direction
                arrow = Marker()
                arrow.header = out_msg.header
                arrow.ns = "doa_tmpc"
                arrow.id = idx * 2 + 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.scale.x = 0.05   # shaft diameter
                arrow.scale.y = 0.1    # head diameter
                arrow.scale.z = 0.1    # head length
                arrow.color.r = 1.0
                arrow.color.g = 0.4
                arrow.color.b = 0.1
                arrow.color.a = 0.9
                arrow.lifetime = rospy.Duration(0.2)

                arrow_start = Point(x=tracker.position.x,
                                    y=tracker.position.y,
                                    z=tracker.position.z)
                arrow_len = max(0.2, v_mag)
                arrow_end = Point(x=tracker.position.x + cvx * arrow_len,
                                  y=tracker.position.y + cvy * arrow_len,
                                  z=tracker.position.z)
                arrow.points.append(arrow_start)
                arrow.points.append(arrow_end)
                marker_array.markers.append(arrow)

        self.pub.publish(out_msg)
        if self.publish_markers:
            self.marker_pub.publish(marker_array)

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
    rospy.init_node('doa_to_tmpc')
    adapter = DOAToMPCAdapter()
    rospy.spin()
