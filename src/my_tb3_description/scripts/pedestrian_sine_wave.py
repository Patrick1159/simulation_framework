#!/usr/bin/env python3
"""
pedestrian_sine_wave.py — Scene 4: Dual pedestrians with sine wave trajectories.

Two pedestrians emerge from separate doorways into an office corridor and move
forward with sinusoidal lateral motion. Designed for S1_static_corridor.world.

Pedestrians:
  ped_001 (gt_id=1): Door 1 (x=5.5), starts at t=3.0s, y_center=0.10, A=0.15
  ped_002 (gt_id=2): Door 2 (x=14.0), starts at t=5.0s, y_center=-0.05, A=0.15

Collision-free design (all clearances >= 0.07m vs static obstacles):
  ped_001 y in [-0.05, 0.25] — upper lane, avoids barrel_01 & crate_02
  ped_002 y in [-0.20, 0.10] — lower lane, avoids barrel_01 & crate_02
  Staggered starts (Dt=2s) + same speed (1m/s) → x-separation always >5m

Publishes:
  ~obstacles          — mpc_nav/ObstacleArray (for MPC consumption)
  ~obstacles_marker   — visualization_msgs/MarkerArray (for RViz)
"""

import math
import os
import sys

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SetModelState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from mpc_nav.msg import Obstacle, ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sdf import cylinder_sdf


# ---- Helpers ----

def _quat_from_yaw(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _make_pose(x, y, z=0.0, yaw=0.0):
    p = Pose()
    p.position = Point(x, y, z)
    p.orientation = _quat_from_yaw(yaw)
    return p


# ---- Pedestrian config class ----

class PedConfig:
    """Immutable configuration for one pedestrian."""
    def __init__(self, model_name, gt_id, x_spawn, y_spawn, z_spawn,
                 t_start, v_x, exit_duration, y_center, A, omega,
                 radius, height, mass, material_name):
        self.model_name = model_name
        self.gt_id = gt_id
        self.x_spawn = x_spawn
        self.y_spawn = y_spawn
        self.z_spawn = z_spawn
        self.t_start = t_start
        self.v_x = v_x
        self.exit_duration = exit_duration
        self.y_center = y_center
        self.A = A
        self.omega = omega
        self.radius = radius
        self.height = height
        self.mass = mass
        self.material_name = material_name


# ---- Main node class ----

class DualPedestrianSineWave:
    def __init__(self):
        rospy.init_node("pedestrian_sine_wave")

        # Build configs (all params overridable via ~param)
        # ped_001
        self.ped_configs = [
            PedConfig(
                model_name=rospy.get_param("~ped1_name", "ped_001"),
                gt_id=rospy.get_param("~ped1_gt_id", 1),
                x_spawn=rospy.get_param("~ped1_x_spawn", 5.5),
                y_spawn=rospy.get_param("~ped1_y_spawn", -1.15),
                z_spawn=rospy.get_param("~ped1_z_spawn", 0.85),
                t_start=rospy.get_param("~ped1_t_start", 3.0),
                v_x=rospy.get_param("~ped1_v_x", 1.0),
                exit_duration=rospy.get_param("~ped1_exit_duration", 1.5),
                y_center=rospy.get_param("~ped1_y_center", 0.10),
                A=rospy.get_param("~ped1_amplitude", 0.15),
                omega=rospy.get_param("~ped1_omega", 0.6),
                radius=0.3,
                height=1.7,
                mass=5.0,
                material_name="Gazebo/Blue",
            ),
            PedConfig(
                model_name=rospy.get_param("~ped2_name", "ped_002"),
                gt_id=rospy.get_param("~ped2_gt_id", 2),
                x_spawn=rospy.get_param("~ped2_x_spawn", 14.0),
                y_spawn=rospy.get_param("~ped2_y_spawn", -1.15),
                z_spawn=rospy.get_param("~ped2_z_spawn", 0.85),
                t_start=rospy.get_param("~ped2_t_start", 5.0),
                v_x=rospy.get_param("~ped2_v_x", 1.0),
                exit_duration=rospy.get_param("~ped2_exit_duration", 1.5),
                y_center=rospy.get_param("~ped2_y_center", -0.05),
                A=rospy.get_param("~ped2_amplitude", 0.15),
                omega=rospy.get_param("~ped2_omega", 0.6),
                radius=0.3,
                height=1.7,
                mass=5.0,
                material_name="Gazebo/Red",
            ),
        ]

        self.rate_hz = rospy.get_param("~rate", 30.0)
        self.frame_id = rospy.get_param("~frame_id", "world")

        # Publishers
        self.obs_pub = rospy.Publisher("obstacles", ObstacleArray, queue_size=1)
        self.marker_pub = rospy.Publisher("obstacles_marker", MarkerArray, queue_size=1)

        # Gazebo services
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        self._spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self._set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # Runtime state per pedestrian
        self._states = []  # list of dicts with x, y, vx, vy, active, spawned

        for cfg in self.ped_configs:
            self._states.append({
                "cfg": cfg,
                "x": cfg.x_spawn,
                "y": cfg.y_spawn,
                "vx": 0.0,
                "vy": 0.0,
                "spawned": False,
                "active": False,
                "exit_done": False,
            })

        rospy.loginfo("[sine_wave] Dual pedestrian node initialized")
        for cfg in self.ped_configs:
            rospy.loginfo(f"  {cfg.model_name}: spawn=({cfg.x_spawn:.1f}, {cfg.y_spawn:.1f}) "
                          f"t_start={cfg.t_start:.1f}s v={cfg.v_x:.1f}m/s "
                          f"sine: y_center={cfg.y_center:.1f} A={cfg.A:.1f} w={cfg.omega:.1f}")

    # ---- Spawn ----

    def spawn(self, cfg):
        sdf = cylinder_sdf(cfg.model_name, radius=cfg.radius, height=cfg.height,
                           mass=cfg.mass, material_name=cfg.material_name)
        pose = _make_pose(cfg.x_spawn, cfg.y_spawn, cfg.z_spawn, yaw=0.0)
        req = SpawnModelRequest()
        req.model_name = cfg.model_name
        req.model_xml = sdf
        req.robot_namespace = ""
        req.initial_pose = pose
        req.reference_frame = self.frame_id
        try:
            resp = self._spawn_srv(req)
            if resp.success:
                rospy.loginfo(f"[sine_wave] Spawned {cfg.model_name} at "
                              f"({cfg.x_spawn:.2f}, {cfg.y_spawn:.2f})")
                return True
            else:
                rospy.logerr(f"[sine_wave] Spawn failed for {cfg.model_name}: {resp.status_message}")
                return False
        except Exception as e:
            rospy.logerr(f"[sine_wave] Spawn exception for {cfg.model_name}: {e}")
            return False

    # ---- Motion computation ----

    def compute_y(self, cfg, x):
        """Compute lateral position for the sine wave cruise phase."""
        return cfg.y_center + cfg.A * math.sin(cfg.omega * (x - cfg.x_spawn))

    # ---- Teleport ----

    def _teleport(self, state):
        cfg = state["cfg"]
        st = ModelState()
        st.model_name = cfg.model_name
        st.reference_frame = self.frame_id
        st.pose = _make_pose(state["x"], state["y"], cfg.z_spawn, yaw=0.0)
        st.twist = Twist()
        if state["active"]:
            st.twist.linear.x = state["vx"]
            st.twist.linear.y = state["vy"]
        try:
            self._set_state_srv(st)
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[sine_wave] set_model_state failed for {cfg.model_name}: {e}")

    # ---- Main step ----

    def _step(self, dt, now):
        for state in self._states:
            cfg = state["cfg"]
            t_elapsed = (now - rospy.Time.from_sec(cfg.t_start)).to_sec()

            # Not yet time to activate
            if t_elapsed < 0:
                if state["spawned"]:
                    self._teleport(state)  # hold in place
                continue

            # Spawn on activation
            if not state["spawned"]:
                if self.spawn(cfg):
                    state["spawned"] = True
                    state["active"] = True
                    state["direction"] = 1
                    state["x"] = cfg.x_spawn
                    state["y"] = cfg.y_spawn
                    state["vx"] = cfg.v_x
                    state["vy"] = 0.0
                    state["exit_accum"] = 0.0
                    rospy.loginfo(f"[sine_wave] {cfg.model_name} activated at t={now.to_sec():.2f}s")
                else:
                    rospy.logerr(f"[sine_wave] {cfg.model_name} spawn failed, will retry")
                    continue

            # ---- Update velocity with direction sign ----
            state["vx"] = state["direction"] * cfg.v_x

            # ---- Update position incrementally (dt-based) ----
            state["x"] += state["vx"] * dt

            # ---- Bounce at corridor boundaries ----
            if state["x"] >= 20.0 and state["direction"] > 0:
                state["direction"] = -1
                state["x"] = 20.0  # clamp
                rospy.loginfo(f"[sine_wave] {cfg.model_name} reached end, turning back")
            elif state["x"] <= 0.0 and state["direction"] < 0:
                state["direction"] = 1
                state["x"] = 0.0  # clamp
                rospy.loginfo(f"[sine_wave] {cfg.model_name} reached start, turning back")

            # ---- Exit phase: linear interpolation from spawn y to y_center ----
            state["exit_accum"] += dt
            if state["exit_accum"] <= cfg.exit_duration:
                frac = state["exit_accum"] / cfg.exit_duration
                state["y"] = cfg.y_spawn + (cfg.y_center - cfg.y_spawn) * frac
                state["vy"] = (cfg.y_center - cfg.y_spawn) / cfg.exit_duration
                state["exit_done"] = False
            else:
                # Cruise phase: sine wave
                if not state["exit_done"]:
                    state["exit_done"] = True
                    rospy.loginfo(f"[sine_wave] {cfg.model_name} entered cruise phase")
                state["y"] = self.compute_y(cfg, state["x"])
                # vy = dy/dt = A * ω * vx * cos(ω*(x - x_spawn))
                dx = state["x"] - cfg.x_spawn
                state["vy"] = cfg.A * cfg.omega * state["vx"] * math.cos(cfg.omega * dx)

            self._teleport(state)

    # ---- Publish ObstacleArray + MarkerArray ----

    def _publish(self, now):
        obs_array = ObstacleArray()
        obs_array.header.frame_id = self.frame_id
        obs_array.header.stamp = now

        marker_array = MarkerArray()
        marker_id = 0

        for state in self._states:
            cfg = state["cfg"]
            if not state["spawned"]:
                continue

            # Obstacle message
            obs = Obstacle()
            obs.id = cfg.gt_id
            obs.position = Point(state["x"], state["y"], 0.0)
            obs.velocity = Vector3(state["vx"], state["vy"], 0.0)
            obs.radius = cfg.radius  # matches collision cylinder radius
            obs.semi_major = 0.3
            obs.semi_minor = 0.3
            obs.type = 0  # pedestrian
            obs.confidence = 1.0
            obs_array.obstacles.append(obs)

            # Cylinder marker
            cyl = Marker()
            cyl.header.frame_id = self.frame_id
            cyl.header.stamp = now
            cyl.ns = "pedestrian"
            cyl.id = marker_id; marker_id += 1
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose = _make_pose(state["x"], state["y"], cfg.z_spawn, yaw=0.0)
            cyl.scale.x = cfg.radius * 2.0   # diameter
            cyl.scale.y = cfg.radius * 2.0
            cyl.scale.z = cfg.height         # height
            if cfg.material_name == "Gazebo/Blue":
                cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = 0.2, 0.5, 1.0, 0.9
            elif cfg.material_name == "Gazebo/Red":
                cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = 1.0, 0.3, 0.2, 0.9
            else:
                cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = 1.0, 1.0, 1.0, 0.9
            marker_array.markers.append(cyl)

            # Velocity arrow (only when moving)
            if state["active"] and (abs(state["vx"]) > 1e-6 or abs(state["vy"]) > 1e-6):
                arrow = Marker()
                arrow.header.frame_id = self.frame_id
                arrow.header.stamp = now
                arrow.ns = "pedestrian_vel"
                arrow.id = marker_id; marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose = _make_pose(state["x"], state["y"], cfg.z_spawn + 0.1,
                                        yaw=math.atan2(state["vy"], state["vx"]))
                speed = math.sqrt(state["vx"]**2 + state["vy"]**2)
                arrow.scale.x = max(0.5, speed * 1.5)
                arrow.scale.y = 0.06
                arrow.scale.z = 0.06
                arrow.color.a = 0.95
                arrow.color.r = 1.0
                arrow.color.g = 0.9
                arrow.color.b = 0.0
                marker_array.markers.append(arrow)

            # Text label
            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = now
            text.ns = "pedestrian_label"
            text.id = marker_id; marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = _make_pose(state["x"], state["y"], cfg.z_spawn + 0.4, yaw=0.0)
            text.scale.z = 0.25
            text.color.a = 1.0
            text.color.r = text.color.g = text.color.b = 1.0
            phase = "exit" if not state["exit_done"] else "cruise"
            text.text = f"{cfg.model_name} (gt={cfg.gt_id}) [{phase}]"
            marker_array.markers.append(text)

        self.obs_pub.publish(obs_array)
        self.marker_pub.publish(marker_array)

    # ---- Main loop ----

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        last = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(1.0 / (5.0 * self.rate_hz), min(0.2, (now - last).to_sec()))
            last = now
            self._step(dt, now)
            self._publish(now)
            rate.sleep()


if __name__ == "__main__":
    DualPedestrianSineWave().run()
