#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
occlusion_scene.py — Scene 5: Controlled Occlusion / Re-ID

Purpose
=======
A *deterministic* controlled-occlusion scenario for the selective-cue
trade-off study. Two visually-distinct pedestrians of the SAME size cross
in opposite directions at a fixed y, and both pass behind a STATIC occluder
box placed between the (stationary) robot camera and the crossing line. They
disappear from the depth FOV together for a short window and reappear near
each other — the classic identity-confusion case where geometry alone can
swap the two tracks and appearance cues (FastSAM/CLIP) should disambiguate.

Like crossing_pedestrian.py this INTENTIONALLY publishes ground-truth
position / velocity directly (no ID jitter, no TrackEstimator noise): the
purpose is to isolate the *perception* Re-ID behaviour, evaluated offline by
calc_occlusion_reid.py. The robot is expected to stay put
(enable_navigation:=false) so the occlusion window is identical across every
method and run.

The occluder box is spawned in Gazebo (so it actually blocks the depth
camera) but is NEVER appended to the ObstacleArray, so it does not appear in
the ground-truth /obstacles stream — GT contains exactly the two pedestrians
(ids 0 and 1). This matches how GT is defined everywhere in this project:
whatever the scene script explicitly publishes, nothing more.

Interface
=========
Same as crossing_pedestrian.py / sim_exp_1.py:
  * publishes ObstacleArray on   obstacles      (→ /obstacles)
  * publishes MarkerArray on     obstacles_marker
  * uses /gazebo/spawn_sdf_model and /gazebo/set_model_state

Wired into dync_obs.launch under scene=5.

Tunable params (all rospy ~private):
  ~radius        default 0.30    (both peds, identical)
  ~height        default 1.00
  ~mass          default  5.0
  ~y_cross       default  3.0     (y at which both pedestrians walk)
  ~x_start       default -2.0     (ped 0 start / ped 1 end)
  ~x_end         default  2.0     (ped 0 end / ped 1 start)
  ~speed         default  0.5     (m/s, shared magnitude)
  ~start_delay   default  3.0     (s before both peds depart)
  ~rate          default 30.0
  ~occ_x         default  0.0     (occluder center x)
  ~occ_y         default  1.5     (occluder center y; between camera and y_cross)
  ~occ_length    default  0.6     (box size along x — sets the shadow width)
  ~occ_thickness default  0.2     (box size along y)
  ~occ_height    default  1.6     (taller than peds so it fully occludes)
  ~frame_id      default "world"
  ~random_seed   accepted for interface parity with the runner; ignored
                 (this scene is deterministic).
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

# Reuse the SDF helpers that the sibling scene scripts use, located in the
# same scripts/ directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sdf import cylinder_sdf, wall_sdf  # noqa: E402


def _quat_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _make_pose(x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> Pose:
    p = Pose()
    p.position = Point(x, y, z)
    p.orientation = _quat_from_yaw(yaw)
    return p


class _Ped:
    """Runtime state for one crossing pedestrian."""

    def __init__(self, obs_id, model_name, material, x0, vx, y):
        self.obs_id = obs_id
        self.model_name = model_name
        self.material = material
        self.x = x0
        self.y = y
        self.vx = vx          # constant, signed
        self.vy = 0.0


class OcclusionScene:
    """Two crossing pedestrians occluded by a static box (Re-ID scenario)."""

    def __init__(self):
        rospy.init_node("scene5_occlusion")

        # ── Pedestrian geometry / motion (shared) ────────────────────────
        self.radius = float(rospy.get_param("~radius", 0.30))
        self.height = float(rospy.get_param("~height", 1.00))
        self.mass = float(rospy.get_param("~mass", 5.0))

        self.y_cross = float(rospy.get_param("~y_cross", 3.0))
        self.x_start = float(rospy.get_param("~x_start", -2.0))
        self.x_end = float(rospy.get_param("~x_end", 2.0))
        self.speed = float(rospy.get_param("~speed", 0.5))

        if self.x_end <= self.x_start:
            rospy.logerr(
                "[scene5] x_end (%.2f) must be greater than x_start (%.2f); "
                "aborting.", self.x_end, self.x_start,
            )
            raise ValueError("invalid x range")
        if self.speed <= 0.0:
            rospy.logerr("[scene5] speed must be positive; got %.2f", self.speed)
            raise ValueError("invalid speed")

        self.start_delay = float(rospy.get_param("~start_delay", 3.0))
        self.rate_hz = float(rospy.get_param("~rate", 30.0))

        # ~random_seed is accepted (the runner always passes it) but ignored:
        # this scene is deterministic by design.
        rospy.get_param("~random_seed", -1)

        # ── Occluder box ─────────────────────────────────────────────────
        self.occ_x = float(rospy.get_param("~occ_x", 0.0))
        self.occ_y = float(rospy.get_param("~occ_y", 1.5))
        self.occ_length = float(rospy.get_param("~occ_length", 0.6))
        self.occ_thickness = float(rospy.get_param("~occ_thickness", 0.2))
        self.occ_height = float(rospy.get_param("~occ_height", 1.6))
        self.occ_name = "occluder_box"

        self.frame_id = str(rospy.get_param("~frame_id", "world"))

        # ── Pedestrians: ped 0 L→R (+vx), ped 1 R→L (−vx) ────────────────
        # Identical size, but visually RICH and DISTINCT surfaces. Flat
        # Gazebo/Blue vs Gazebo/Red was found non-discriminative for CLIP
        # (cosine similarity ~0.975 between the two), because CLIP embeds the
        # rendered surface pixels (via the SAM-mask ROI) and two solid-color
        # blobs look nearly identical to it. Textured built-in materials
        # (brick pattern vs wooden planks) give CLIP separable embeddings, so
        # the appearance cue can actually disambiguate identity here.
        self.peds = [
            _Ped(0, "occ_ped_0", "Gazebo/Bricks",
                 x0=self.x_start, vx=+self.speed, y=self.y_cross),
            _Ped(1, "occ_ped_1", "Gazebo/WoodPallet",
                 x0=self.x_end, vx=-self.speed, y=self.y_cross),
        ]

        self._move_time = rospy.Time(0)

        # ── ROS publishers ──────────────────────────────────────────────
        self.obs_pub = rospy.Publisher("obstacles", ObstacleArray, queue_size=1)
        self.marker_pub = rospy.Publisher(
            "obstacles_marker", MarkerArray, queue_size=1)

        # ── Gazebo services ─────────────────────────────────────────────
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        self._spawn_srv = rospy.ServiceProxy(
            "/gazebo/spawn_sdf_model", SpawnModel)
        self._set_state_srv = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState)

        self._log_geometry()

    # ──────────────────────────────────────────────────────────────────
    # Occlusion-window geometry (informational + for --occlusion-start).
    # A pedestrian is behind the box (from a camera at the origin) while its
    # line-of-sight through (x, y_cross) intersects the box's x-extent at the
    # box depth. Approximated by the box's own x-shadow scaled to y_cross.
    # ──────────────────────────────────────────────────────────────────

    def _log_geometry(self):
        half = 0.5 * self.occ_length
        # Shadow x half-width projected from origin onto the crossing line.
        shadow_half = half * (self.y_cross / self.occ_y) if self.occ_y > 0 else half
        # Ped 0 travels +x from x_start; it enters the shadow [−shadow_half,
        # +shadow_half] centred on occ_x.
        enter = (self.occ_x - shadow_half - self.x_start) / self.speed
        exit_ = (self.occ_x + shadow_half - self.x_start) / self.speed
        occ_start = self.start_delay + max(0.0, enter)
        occ_dur = max(0.0, exit_ - enter)
        rospy.loginfo(
            "[scene5] Controlled-occlusion scene initialised.\n"
            "  ped0: (%.2f → %.2f) +%.2f m/s | ped1: (%.2f → %.2f) −%.2f m/s "
            "at y=%.2f\n"
            "  occluder box: center (%.2f, %.2f) size %.2f×%.2f×%.2f\n"
            "  projected shadow half-width at y_cross ≈ %.2f m\n"
            "  NOMINAL occlusion window: start ≈ %.2fs  duration ≈ %.2fs "
            "(relative to run start; verify against tracker gap)\n"
            "  start_delay=%.1fs  rate=%.1fHz",
            self.peds[0].x, self.x_end, self.speed,
            self.peds[1].x, self.x_start, self.speed, self.y_cross,
            self.occ_x, self.occ_y,
            self.occ_length, self.occ_thickness, self.occ_height,
            shadow_half, occ_start, occ_dur,
            self.start_delay, self.rate_hz,
        )

    # ──────────────────────────────────────────────────────────────────
    # Gazebo helpers
    # ──────────────────────────────────────────────────────────────────

    def _spawn_model(self, name, xml, x, y, z):
        req = SpawnModelRequest()
        req.model_name = name
        req.model_xml = xml
        req.robot_namespace = ""
        req.initial_pose = _make_pose(x, y, z=z, yaw=0.0)
        req.reference_frame = self.frame_id
        try:
            resp = self._spawn_srv(req)
            if not resp.success:
                rospy.logwarn("[scene5] Spawn %s failed: %s",
                              name, resp.status_message)
            else:
                rospy.loginfo("[scene5] Spawned %s at (%.2f, %.2f)", name, x, y)
        except Exception as exc:
            rospy.logwarn("[scene5] Spawn %s exception: %s", name, str(exc))

    def spawn(self):
        # Static occluder box (NOT published to /obstacles → invisible to GT).
        self._spawn_model(
            self.occ_name,
            wall_sdf(self.occ_name, length=self.occ_length,
                     thickness=self.occ_thickness, height=self.occ_height),
            self.occ_x, self.occ_y, z=self.occ_height * 0.5,
        )
        # Two pedestrians.
        for ped in self.peds:
            self._spawn_model(
                ped.model_name,
                cylinder_sdf(ped.model_name, radius=self.radius,
                             height=self.height, mass=self.mass,
                             material_name=ped.material),
                ped.x, ped.y, z=self.height * 0.5,
            )

    def _teleport(self, ped: _Ped, moving: bool):
        st = ModelState()
        st.model_name = ped.model_name
        st.reference_frame = self.frame_id
        st.pose = _make_pose(ped.x, ped.y, z=self.height * 0.5, yaw=0.0)
        st.twist = Twist()
        if moving:
            st.twist.linear.x = ped.vx
            st.twist.linear.y = ped.vy
        try:
            self._set_state_srv(st)
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, "[scene5] set_model_state(%s) failed: %s",
                ped.model_name, str(exc))

    # ──────────────────────────────────────────────────────────────────
    # Simulation step
    # ──────────────────────────────────────────────────────────────────

    def _step(self, dt: float):
        now = rospy.Time.now()
        waiting = now < self._move_time

        if not waiting:
            for ped in self.peds:
                ped.x += ped.vx * dt
                # Clamp at the far edge and stop (no loop: one clean crossing
                # gives one clean occlusion window per run).
                if ped.vx > 0.0 and ped.x >= self.x_end:
                    ped.x, ped.vx = self.x_end, 0.0
                elif ped.vx < 0.0 and ped.x <= self.x_start:
                    ped.x, ped.vx = self.x_start, 0.0

        for ped in self.peds:
            self._teleport(ped, moving=(not waiting and ped.vx != 0.0))
        self._publish(waiting)

    # ──────────────────────────────────────────────────────────────────
    # Publish ground-truth ObstacleArray (both peds only) + markers.
    # The occluder box is deliberately absent. No KF covariance fields.
    # ──────────────────────────────────────────────────────────────────

    def _publish(self, waiting: bool):
        now = rospy.Time.now()

        obs_array = ObstacleArray()
        obs_array.header.frame_id = self.frame_id
        obs_array.header.stamp = now
        for ped in self.peds:
            msg = Obstacle()
            msg.id = ped.obs_id
            msg.position = Point(ped.x, ped.y, 0.0)
            moving = (not waiting) and ped.vx != 0.0
            msg.velocity = Vector3(ped.vx if moving else 0.0,
                                   ped.vy if moving else 0.0, 0.0)
            msg.radius = self.radius
            msg.semi_major = self.radius
            msg.semi_minor = self.radius
            msg.type = 1  # cylinder
            msg.confidence = 1.0
            # sigma2_* left at 0 → no uncertainty inflation downstream.
            obs_array.obstacles.append(msg)
        self.obs_pub.publish(obs_array)

        marker_array = MarkerArray()
        self._build_markers(marker_array, waiting, now)
        self.marker_pub.publish(marker_array)

    def _build_markers(self, marker_array: MarkerArray, waiting: bool, stamp):
        # Static occluder box (grey), so it is visible in RViz.
        box = Marker()
        box.header.frame_id = self.frame_id
        box.header.stamp = stamp
        box.ns = "scene5_occluder"
        box.id = 100
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose = _make_pose(self.occ_x, self.occ_y,
                              z=self.occ_height * 0.5, yaw=0.0)
        box.scale.x = self.occ_length
        box.scale.y = self.occ_thickness
        box.scale.z = self.occ_height
        box.color.a = 0.85
        box.color.r = box.color.g = box.color.b = 0.7
        marker_array.markers.append(box)

        colors = {0: (0.2, 0.5, 1.0), 1: (1.0, 0.2, 0.2)}
        for ped in self.peds:
            cyl = Marker()
            cyl.header.frame_id = self.frame_id
            cyl.header.stamp = stamp
            cyl.ns = "scene5_ped"
            cyl.id = ped.obs_id
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose = _make_pose(ped.x, ped.y, z=self.height * 0.5, yaw=0.0)
            cyl.scale.x = cyl.scale.y = self.radius * 2
            cyl.scale.z = self.height
            cyl.color.a = 0.4 if waiting else 0.9
            cyl.color.r, cyl.color.g, cyl.color.b = colors[ped.obs_id]
            marker_array.markers.append(cyl)

    # ──────────────────────────────────────────────────────────────────
    # Main loop
    # ──────────────────────────────────────────────────────────────────

    def run(self):
        self.spawn()

        if self.start_delay > 0.0:
            rospy.loginfo(
                "[scene5] Waiting %.1f s before pedestrians depart...",
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
    OcclusionScene().run()
