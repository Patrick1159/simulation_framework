#!/usr/bin/env python3

import math
from dataclasses import dataclass
import rospy
import tf.transformations as tft
from umap.msg import TrackerDetailArray
from mpc_planner_msgs.msg import ObstacleArray, ObstacleGMM, Gaussian
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class BridgeConfig:
    N: int
    dt: float
    k_cv: int
    v_min: float
    velocity_limit: float
    emit_uncertainty: bool
    rate_floor: float
    rate_ceil: float
    target_frame: str
    stamp: object  # rospy.Time, passed in (keeps this function ROS-call-free)


def _uncertainty_rate(tracker, cfg):
    """Constant per-step uncertainty rate. Returns 0.0 in deterministic mode.
    Never integrated here — the planner integrates once via
    propagatePredictionUncertainty."""
    if not cfg.emit_uncertainty:
        return 0.0
    var = max(tracker.sigma2_vxvx, tracker.sigma2_vyvy)
    rate = math.sqrt(var) if var > 0.0 else 0.0
    return min(max(rate, cfg.rate_floor), cfg.rate_ceil)


def tracker_to_obstacle(tracker, cfg):
    """Convert one umap TrackerDetail into one mpc_planner_msgs ObstacleGMM.
    Geometry: constant velocity for k<=k_cv, frozen afterward. Pure (no rospy
    calls other than message construction; stamp comes from cfg)."""
    obs = ObstacleGMM()
    obs.id = tracker.tracker_id
    obs.pose.position = tracker.position

    vx = tracker.velocity.x
    vy = tracker.velocity.y
    v_mag = math.sqrt(vx * vx + vy * vy)

    if v_mag < cfg.v_min:
        obs.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        cvx = cvy = 0.0
    else:
        yaw = math.atan2(vy, vx)
        q = tft.quaternion_from_euler(0, 0, yaw)
        obs.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        if v_mag > cfg.velocity_limit:
            scale = cfg.velocity_limit / v_mag
            cvx, cvy = vx * scale, vy * scale
        else:
            cvx, cvy = vx, vy

    rate = _uncertainty_rate(tracker, cfg)

    gaussian = Gaussian()
    gaussian.mean.header.stamp = cfg.stamp
    gaussian.mean.header.frame_id = cfg.target_frame

    for k in range(1, cfg.N):
        k_eff = min(k, cfg.k_cv)  # freeze after k_cv
        dt_eff = cfg.dt * k_eff
        px_k = tracker.position.x + cvx * dt_eff
        py_k = tracker.position.y + cvy * dt_eff

        ps = PoseStamped()
        ps.header.stamp = cfg.stamp
        ps.header.frame_id = cfg.target_frame
        ps.pose.position = Point(x=px_k, y=py_k, z=tracker.position.z)
        ps.pose.orientation = obs.pose.orientation
        gaussian.mean.poses.append(ps)

        gaussian.major_semiaxis.append(rate)
        gaussian.minor_semiaxis.append(rate)

    obs.gaussians.append(gaussian)
    obs.probabilities = [1.0]
    return obs


class DOAToMPCAdapter:
    """Converts DOA perception output (TrackerDetailArray) to MPC planner
    input format (ObstacleArray with GMM predictions)."""

    def __init__(self):
        # --- ROS parameters ---
        self.N = rospy.get_param('~N', 30)
        self.dt = rospy.get_param('~dt', 0.2)
        self.cv_horizon = rospy.get_param('~cv_horizon', 1.0)
        self.v_min = rospy.get_param('~v_min', 0.1)
        self.emit_uncertainty = rospy.get_param('~emit_uncertainty', False)
        self.rate_floor = rospy.get_param('~rate_floor', 0.05)
        self.rate_ceil = rospy.get_param('~rate_ceil', 0.5)
        self.k_cv = max(1, int(round(self.cv_horizon / self.dt)))
        assert self.cv_horizon > 0.0, "cv_horizon must be positive"
        assert self.v_min >= 0.0, "v_min must be non-negative"
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

        cfg = BridgeConfig(
            N=self.N, dt=self.dt, k_cv=self.k_cv, v_min=self.v_min,
            velocity_limit=self.velocity_limit,
            emit_uncertainty=self.emit_uncertainty,
            rate_floor=self.rate_floor, rate_ceil=self.rate_ceil,
            target_frame=self.target_frame, stamp=msg.header.stamp,
        )

        for idx, tracker in enumerate(msg.trackers):
            if tracker.age > self.max_age:
                continue

            obs = tracker_to_obstacle(tracker, cfg)
            out_msg.obstacles.append(obs)

            if self.publish_markers:
                self._append_markers(marker_array, idx, tracker, obs)

        self.pub.publish(out_msg)
        if self.publish_markers:
            self.marker_pub.publish(marker_array)

    def _append_markers(self, marker_array, idx, tracker, obs):
        sphere = Marker()
        sphere.header.frame_id = self.target_frame
        sphere.header.stamp = obs.gaussians[0].mean.header.stamp
        sphere.ns = "doa_tmpc"
        sphere.id = idx * 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = obs.pose
        sphere.scale.x = max(float(tracker.box_dimensions.x), 0.1)
        sphere.scale.y = max(float(tracker.box_dimensions.y), 0.1)
        sphere.scale.z = max(float(tracker.box_dimensions.z), 0.1)
        conf = tracker.confidence
        sphere.color.r = 1.0 - conf
        sphere.color.g = conf
        sphere.color.b = 0.0
        sphere.color.a = 0.6
        sphere.lifetime = rospy.Duration(0.2)
        marker_array.markers.append(sphere)

        arrow = Marker()
        arrow.header = sphere.header
        arrow.ns = "doa_tmpc"
        arrow.id = idx * 2 + 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05, 0.1, 0.1
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.4, 0.1, 0.9
        arrow.lifetime = rospy.Duration(0.2)
        vx, vy = tracker.velocity.x, tracker.velocity.y
        v_mag = math.sqrt(vx * vx + vy * vy)
        if v_mag > self.velocity_limit and v_mag > 0.0:
            s = self.velocity_limit / v_mag
            vx, vy = vx * s, vy * s
        arrow_len = max(0.2, v_mag)
        start = Point(x=tracker.position.x, y=tracker.position.y, z=tracker.position.z)
        end = Point(x=tracker.position.x + vx * arrow_len,
                    y=tracker.position.y + vy * arrow_len,
                    z=tracker.position.z)
        arrow.points.append(start)
        arrow.points.append(end)
        marker_array.markers.append(arrow)

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
