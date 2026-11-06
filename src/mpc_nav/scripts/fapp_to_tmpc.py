#!/usr/bin/env python3
"""
fapp_to_tmpc.py
===============
Bridge: FAPP mot_mapping → T-MPC planner.

Subscribes to /states (obj_state_msgs/ObjectsStates).
mot_mapping (FAPP baseline) publishes DBSCAN-clustered object states on /states
using the obj_state_msgs/ObjectsStates message type, which is incompatible with
doa_to_obstacles.py (expects umap/TrackerDetailArray).

Each State contains:
  - position  (geometry_msgs/Vector3): centroid
  - velocity  (geometry_msgs/Vector3): estimated velocity
  - size      (geometry_msgs/Vector3): bounding-box dimensions (may be zero)
  - poscov    (geometry_msgs/Vector3): position covariance (diagonal, may be zero)

Publishes /doa_obstacles_tmpc (mpc_planner_msgs/ObstacleArray) with
constant-velocity predictions — same format as dodt_to_tmpc.py.

Notes:
  - obj_state_msgs/State has no tracker_id field; array index is used as id.
  - mot_mapping requires robot motion to build its ikd-tree background model
    before it can separate dynamic from static points. During the initial
    static phase /states is published empty. To avoid sending an empty obstacle
    array to T-MPC (which would make the planner run blind), this node keeps
    the last non-empty ObstacleArray and re-publishes it with positions
    extrapolated forward using the stored velocities. Extrapolation is capped
    at `keep_alive_timeout` seconds; after that an empty array is published.
"""

import math
import rospy
from obj_state_msgs.msg import ObjectsStates
from mpc_planner_msgs.msg import ObstacleArray, ObstacleGMM, Gaussian
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf.transformations as tft


class FAPPToMPCAdapter:
    def __init__(self):
        self.N = rospy.get_param('~N', 30)
        self.dt = rospy.get_param('~dt', 0.2)
        self.velocity_limit = rospy.get_param('~velocity_limit', 1.5)
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        self.output_topic = rospy.get_param('~output_topic', '/doa_obstacles_tmpc')
        self.input_topic = rospy.get_param('~input_topic', '/states')
        self.watchdog_timeout = rospy.get_param('~watchdog_timeout', 0.5)
        self.watchdog_rate = rospy.get_param('~watchdog_rate', 5.0)
        # How long (s) to keep extrapolating the last non-empty detection before
        # giving up and publishing an empty array. mot_mapping may output empty
        # /states while the robot is initializing or the scene is fully static.
        self.keep_alive_timeout = rospy.get_param('~keep_alive_timeout', 2.0)
        # mot_mapping computes velocity as (pos_cur - pos_prev) / 0.02, where 0.02 s
        # is the EKF predict timer period. The actual point-cloud callback fires at
        # the depth-image rate (D435 Gazebo: 30 Hz → dt_actual ≈ 0.033 s), so the
        # raw vel_detect is scaled up by 0.033/0.02 ≈ 1.67×. On top of that the EKF
        # smoothing introduces additional bias. Empirically (pair_crossing corridor,
        # GT speed = 0.50 m/s, tracker output p50 = 1.50 m/s) the net overestimate
        # is ~3×, so the correction factor is 0.50/1.50 ≈ 0.33.
        # Set to 1.0 to disable correction.
        self.velocity_scale = rospy.get_param('~velocity_scale', 0.33)

        self.pub = rospy.Publisher(self.output_topic, ObstacleArray, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, ObjectsStates,
                                    self.callback, queue_size=1)

        self._last_callback_time = rospy.Time.now()

        # Keep-alive state: stores (stamp, [(cx,cy,cz,vx,vy,yaw), ...]) of the
        # last callback that produced at least one obstacle.
        self._last_valid_stamp = None   # rospy.Time when last non-empty msg arrived
        self._last_valid_obs = []       # list of (cx,cy,cz,vx,vy,orientation_q)

        self._watchdog_timer = rospy.Timer(
            rospy.Duration(1.0 / self.watchdog_rate),
            self._watchdog_callback)

        rospy.loginfo("FAPPToMPCAdapter started")
        rospy.loginfo("  Subscribing      : %s", self.input_topic)
        rospy.loginfo("  Publishing       : %s", self.output_topic)
        rospy.loginfo("  Horizon          : N=%d, dt=%.2f s", self.N, self.dt)
        rospy.loginfo("  Keep-alive       : %.1f s", self.keep_alive_timeout)
        rospy.loginfo("  Velocity scale   : %.3f", self.velocity_scale)

    # ------------------------------------------------------------------

    def _build_obs_gmm(self, cx, cy, cz, vx, vy, orientation_q, stamp):
        """Build one ObstacleGMM from current-position (cx,cy,cz) and velocity."""
        obs = ObstacleGMM()
        obs.pose.position = Point(x=cx, y=cy, z=cz)
        obs.pose.orientation = Quaternion(
            x=orientation_q[0], y=orientation_q[1],
            z=orientation_q[2], w=orientation_q[3])

        gaussian = Gaussian()
        gaussian.mean.header.stamp = stamp
        gaussian.mean.header.frame_id = self.target_frame
        for k in range(1, self.N):
            dt_k = self.dt * k
            ps = PoseStamped()
            ps.header.stamp = stamp
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
        return obs

    def callback(self, msg):
        self._last_callback_time = rospy.Time.now()

        now = rospy.Time.now()
        out_msg = ObstacleArray()
        out_msg.header.stamp = now
        out_msg.header.frame_id = self.target_frame

        new_valid_obs = []
        for idx, state in enumerate(msg.states):
            cx = state.position.x
            cy = state.position.y
            cz = state.position.z
            vx = state.velocity.x * self.velocity_scale
            vy = state.velocity.y * self.velocity_scale

            # Clamp speed
            v_mag = math.sqrt(vx * vx + vy * vy)
            if v_mag > self.velocity_limit:
                scale = self.velocity_limit / v_mag
                vx *= scale
                vy *= scale
                v_mag = self.velocity_limit

            if v_mag < 0.01:
                q = [0.0, 0.0, 0.0, 1.0]
            else:
                yaw = math.atan2(vy, vx)
                q = list(tft.quaternion_from_euler(0, 0, yaw))

            obs = self._build_obs_gmm(cx, cy, cz, vx, vy, q, now)
            obs.id = idx
            out_msg.obstacles.append(obs)
            new_valid_obs.append((cx, cy, cz, vx, vy, q))

        if new_valid_obs:
            # Fresh detections: update keep-alive state
            self._last_valid_stamp = now
            self._last_valid_obs = new_valid_obs
        elif self._last_valid_obs:
            # /states is empty (mot_mapping still initializing or brief gap).
            # Extrapolate the last known obstacles forward if within timeout.
            age = (now - self._last_valid_stamp).to_sec() if self._last_valid_stamp else 999.0
            if age <= self.keep_alive_timeout:
                for idx, (cx, cy, cz, vx, vy, q) in enumerate(self._last_valid_obs):
                    # Advance position by elapsed time
                    ecx = cx + vx * age
                    ecy = cy + vy * age
                    obs = self._build_obs_gmm(ecx, ecy, cz, vx, vy, q, now)
                    obs.id = idx
                    out_msg.obstacles.append(obs)
                rospy.logdebug_throttle(1.0,
                    "fapp_to_tmpc: /states empty, extrapolating %d obstacles (age=%.2fs)",
                    len(self._last_valid_obs), age)
            else:
                # Keep-alive expired: clear stored state, publish empty
                rospy.logdebug_throttle(2.0,
                    "fapp_to_tmpc: keep-alive expired (age=%.2fs), publishing empty", age)
                self._last_valid_obs = []

        self.pub.publish(out_msg)

    def _watchdog_callback(self, event):
        """Publish empty array if /states topic is silent (including before the
        first message ever arrives).

        mot_mapping does not publish /states until the robot has moved enough to
        build its ikd-tree background model. Without this, /doa_obstacles_tmpc is
        never published, T-MPC gets no obstacle input and the robot stays still —
        which means mot_mapping never bootstraps (a self-sustaining deadlock).
        Publishing an empty ObstacleArray lets T-MPC drive toward the goal under a
        no-obstacle assumption; that motion seeds mot_mapping's background model
        and breaks the deadlock.
        """
        elapsed = rospy.Time.now() - self._last_callback_time
        if elapsed > rospy.Duration(self.watchdog_timeout):
            out_msg = ObstacleArray()
            out_msg.header.stamp = rospy.Time.now()
            out_msg.header.frame_id = self.target_frame
            self.pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('fapp_to_tmpc')
    adapter = FAPPToMPCAdapter()
    rospy.spin()
