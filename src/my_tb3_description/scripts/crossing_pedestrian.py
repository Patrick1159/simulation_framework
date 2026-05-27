#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
crossing_pedestrian.py — Scene 3: Single Crossing Pedestrian

Purpose
=======
A *minimal*, *fully deterministic* reproduction of the "front-crossing
pedestrian" pain point we keep hitting in real experiments:

  * Robot spawns at (0, 0) facing +y (handled by my_tb3_gazebo.launch).
  * Goal at (0, 8), straight ahead — the planner's natural shortest path
    is a straight line.
  * One cylinder pedestrian sweeps left → right at constant speed at a
    fixed y, crossing the robot's intended path.

This script INTENTIONALLY does NOT simulate any of the perception-layer
artefacts that sim_exp_1.py models (ID jitter, TrackEstimator velocity
noise, etc.). It publishes ground-truth position / velocity directly,
because the goal here is to isolate *planner / obstacle-modelling*
behaviour, not perception robustness.

Boundary conditions (deliberately rigid):

  * 1 obstacle (no multi-pedestrian noise)
  * Constant speed, constant trajectory (perfectly predictable)
  * Loops back to start after crossing — infinite repetition, easy to
    watch many planner reactions per run
  * No KF covariance fields populated (downstream falls back to
    geometric capsule, no uncertainty inflation — exactly what we want
    when validating "is the swept-capsule cost itself enough?")

Interface
=========
Same as my_tb3_description/scripts/dync_scene1.py and sim_exp_1.py:
  * publishes ObstacleArray on   ~obstacles
  * publishes MarkerArray on     ~obstacles_marker
  * uses /gazebo/spawn_sdf_model and /gazebo/set_model_state

Wired into dync_obs.launch under scene=3. The mpc.launch arg
``dynamic_scene`` selects it as usual.

Tunable params (all rospy ~private):
  ~radius           default 0.30
  ~height           default 1.00
  ~mass             default  5.0
  ~y_cross          default  3.0     (y at which the pedestrian walks)
  ~x_start          default -3.0
  ~x_end            default  3.0
  ~speed            default  1.0     (m/s, walking speed)
  ~loop             default  true    (loop back to x_start when reaching x_end)
  ~start_delay      default  3.0
  ~rate             default 30.0
  ~model_name       default "scene3_crossing"
  ~frame_id         default "world"
"""

import math
import os
import sys

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from mpc_nav.msg import Obstacle, ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray

# Reuse the SDF helper that sim_exp_1.py / dync_scene1.py use, located in
# the same scripts/ directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sdf import cylinder_sdf  # noqa: E402


def _quat_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _make_pose(x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> Pose:
    p = Pose()
    p.position = Point(x, y, z)
    p.orientation = _quat_from_yaw(yaw)
    return p


class CrossingPedestrianScene:
    """Single pedestrian crossing the robot's straight-line goal path."""

    def __init__(self):
        rospy.init_node("scene3_crossing_pedestrian")

        # ── Geometry / motion ───────────────────────────────────────────
        self.radius = float(rospy.get_param("~radius", 0.30))
        self.height = float(rospy.get_param("~height", 1.00))
        self.mass = float(rospy.get_param("~mass", 5.0))

        self.y_cross = float(rospy.get_param("~y_cross", 3.0))
        self.x_start = float(rospy.get_param("~x_start", -3.0))
        self.x_end = float(rospy.get_param("~x_end", 3.0))
        self.speed = float(rospy.get_param("~speed", 1.0))
        self.loop = bool(rospy.get_param("~loop", True))

        if self.x_end <= self.x_start:
            rospy.logerr(
                "[scene3] x_end (%.2f) must be greater than x_start (%.2f); "
                "aborting.", self.x_end, self.x_start,
            )
            raise ValueError("invalid x range")
        if self.speed <= 0.0:
            rospy.logerr("[scene3] speed must be positive; got %.2f", self.speed)
            raise ValueError("invalid speed")

        self.start_delay = float(rospy.get_param("~start_delay", 3.0))
        self.rate_hz = float(rospy.get_param("~rate", 30.0))

        # ── Naming / framing ────────────────────────────────────────────
        self.model_name = str(rospy.get_param("~model_name", "scene3_crossing"))
        self.frame_id = str(rospy.get_param("~frame_id", "world"))

        # ── State (mutable runtime values) ──────────────────────────────
        # Direction is fixed at +1 (left → right). When loop=False we just
        # stop publishing motion once x >= x_end. When loop=True we
        # teleport back to (x_start, y_cross) and resume.
        self._x = self.x_start
        self._y = self.y_cross
        self._vx = self.speed   # +x direction, constant
        self._vy = 0.0
        self._move_time = rospy.Time(0)
        self._cycles = 0

        # ── ROS publishers ──────────────────────────────────────────────
        self.obs_pub = rospy.Publisher("obstacles", ObstacleArray, queue_size=1)
        self.marker_pub = rospy.Publisher("obstacles_marker", MarkerArray, queue_size=1)

        # ── Gazebo services ─────────────────────────────────────────────
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        self._spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self._set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # The robot's straight-line path goes through (0, 0) → (0, +y).
        # The pedestrian walks at y = y_cross, so it crosses x = 0 at:
        t_cross = (0.0 - self.x_start) / self.speed
        rospy.loginfo(
            "[scene3] Crossing pedestrian initialised.\n"
            "  pedestrian: (%.2f → %.2f, %.2f) at %.2f m/s\n"
            "  crosses x=0 at t≈%.2fs after departure\n"
            "  loop=%s  start_delay=%.1fs",
            self.x_start, self.x_end, self.y_cross,
            self.speed, t_cross,
            self.loop, self.start_delay,
        )

    # ──────────────────────────────────────────────────────────────────
    # Gazebo helpers
    # ──────────────────────────────────────────────────────────────────

    def spawn(self):
        sdf = cylinder_sdf(
            self.model_name,
            radius=self.radius, height=self.height, mass=self.mass,
            material_name="Gazebo/Blue",
        )
        pose = _make_pose(self._x, self._y, z=self.height * 0.5, yaw=0.0)
        req = SpawnModelRequest()
        req.model_name = self.model_name
        req.model_xml = sdf
        req.robot_namespace = ""
        req.initial_pose = pose
        req.reference_frame = self.frame_id
        try:
            resp = self._spawn_srv(req)
            if not resp.success:
                rospy.logwarn("[scene3] Spawn failed: %s", resp.status_message)
            else:
                rospy.loginfo("[scene3] Spawned %s at (%.2f, %.2f)",
                              self.model_name, self._x, self._y)
        except Exception as exc:
            rospy.logwarn("[scene3] Spawn exception: %s", str(exc))

    def _teleport(self, moving: bool):
        st = ModelState()
        st.model_name = self.model_name
        st.reference_frame = self.frame_id
        st.pose = _make_pose(self._x, self._y, z=self.height * 0.5, yaw=0.0)
        st.twist = Twist()
        if moving:
            st.twist.linear.x = self._vx
            st.twist.linear.y = self._vy
        try:
            self._set_state_srv(st)
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, "[scene3] set_model_state failed: %s", str(exc))

    # ──────────────────────────────────────────────────────────────────
    # Simulation step
    # ──────────────────────────────────────────────────────────────────

    def _step(self, dt: float):
        now = rospy.Time.now()

        if now < self._move_time:
            # Still in pre-departure delay: hold position, no velocity.
            self._teleport(moving=False)
            self._publish(is_waiting=True)
            return

        # Advance position at constant velocity.
        self._x += self._vx * dt

        if self._x >= self.x_end:
            if self.loop:
                self._cycles += 1
                rospy.loginfo("[scene3] cycle %d complete — looping back",
                              self._cycles)
                self._x = self.x_start
            else:
                # Park at end, stop publishing nonzero velocity.
                self._x = self.x_end
                self._vx = 0.0

        self._teleport(moving=(self._vx != 0.0))
        self._publish(is_waiting=False)

    # ──────────────────────────────────────────────────────────────────
    # Publish ground-truth ObstacleArray and visualization markers.
    # No KF covariance fields populated → downstream MPC falls back to a
    # pure geometric capsule with no uncertainty inflation, which is
    # exactly what we want when validating the swept-capsule cost itself.
    # ──────────────────────────────────────────────────────────────────

    def _publish(self, is_waiting: bool):
        now = rospy.Time.now()

        obs_array = ObstacleArray()
        obs_array.header.frame_id = self.frame_id
        obs_array.header.stamp = now

        msg = Obstacle()
        msg.id = 0
        msg.position = Point(self._x, self._y, 0.0)
        msg.velocity = Vector3(0.0 if is_waiting else self._vx,
                               0.0 if is_waiting else self._vy,
                               0.0)
        msg.radius = self.radius
        # The downstream adapter / MPC understands semi_major / semi_minor;
        # we publish the geometric ellipse derived from radius. Setting
        # them equal to radius makes the capsule degenerate to a circle
        # in the perpendicular dimension while still letting the planner
        # apply its own velocity-based forward-swept lookahead.
        msg.semi_major = self.radius
        msg.semi_minor = self.radius
        msg.type = 1  # cylinder
        msg.confidence = 1.0
        # sigma2_* left at 0 → no uncertainty inflation in MPC.
        obs_array.obstacles.append(msg)
        self.obs_pub.publish(obs_array)

        marker_array = MarkerArray()
        self._build_markers(marker_array, is_waiting, now)
        self.marker_pub.publish(marker_array)

    def _build_markers(self, marker_array: MarkerArray,
                       is_waiting: bool, stamp):
        # Cylinder
        cyl = Marker()
        cyl.header.frame_id = self.frame_id
        cyl.header.stamp = stamp
        cyl.ns = "scene3_cylinder"
        cyl.id = 0
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.pose = _make_pose(self._x, self._y, z=self.height * 0.5, yaw=0.0)
        cyl.scale.x = self.radius * 2
        cyl.scale.y = self.radius * 2
        cyl.scale.z = self.height
        cyl.color.a = 0.4 if is_waiting else 0.9
        cyl.color.r, cyl.color.g, cyl.color.b = 0.2, 0.5, 1.0
        marker_array.markers.append(cyl)

        # Velocity arrow (only when actually walking)
        if not is_waiting and self._vx != 0.0:
            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = stamp
            arrow.ns = "scene3_velocity"
            arrow.id = 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = _make_pose(
                self._x, self._y, z=self.height * 0.5 + 0.05,
                yaw=math.atan2(self._vy, self._vx),
            )
            arrow_len = 2.0 * self.speed   # exaggerate for visibility
            arrow.scale.x = arrow_len
            arrow.scale.y = 0.07
            arrow.scale.z = 0.07
            arrow.color.a = 0.95
            arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 0.9, 0.0
            marker_array.markers.append(arrow)

        # Label
        text = Marker()
        text.header.frame_id = self.frame_id
        text.header.stamp = stamp
        text.ns = "scene3_label"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self._x
        text.pose.position.y = self._y
        text.pose.position.z = self.height + 0.3
        text.pose.orientation = _quat_from_yaw(0.0)
        text.scale.z = 0.28
        text.color.a = 1.0
        text.color.r = text.color.g = text.color.b = 1.0
        if is_waiting:
            text.text = f"crossing (waiting…)"
        else:
            text.text = f"crossing v={self.speed:.1f}m/s  x={self._x:+.2f}"
        marker_array.markers.append(text)

    # ──────────────────────────────────────────────────────────────────
    # Main loop
    # ──────────────────────────────────────────────────────────────────

    def run(self):
        self.spawn()

        if self.start_delay > 0.0:
            rospy.loginfo("[scene3] Waiting %.1f s before pedestrian departs...",
                          self.start_delay)
        self._move_time = rospy.Time.now() + rospy.Duration(self.start_delay)

        rate = rospy.Rate(self.rate_hz)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(1.0 / (5.0 * self.rate_hz),
                     min(0.2, (now - last).to_sec()))
            last = now
            self._step(dt)
            rate.sleep()


if __name__ == "__main__":
    CrossingPedestrianScene().run()
