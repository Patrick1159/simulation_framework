#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import random
import rospy
from gazebo_msgs.srv import SetModelState, SpawnModel, SpawnModelRequest
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Pose, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from mpc_nav.msg import Obstacle, ObstacleArray
from sdf import cylinder_sdf


def yaw_to_quat(yaw):
    # planar yaw -> quaternion
    # q = [0,0,sin(yaw/2),cos(yaw/2)]
    import geometry_msgs.msg
    q = geometry_msgs.msg.Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class RandomDynamicObstacle:
    def __init__(self):
        rospy.init_node("random_dynamic_obstacle")

        # --- params ---
        self.num_obstacles = int(rospy.get_param("~num_obstacles", 3))
        self.model_prefix = rospy.get_param("~model_prefix", "rand_obs")
        self.frame_id = rospy.get_param("~frame_id", "world")

        # obstacle geometry
        self.radius = float(rospy.get_param("~radius", 0.3))
        self.height = float(rospy.get_param("~height", 1.0))
        self.mass = float(rospy.get_param("~mass", 5.0))

        # motion bounds
        self.x_min = float(rospy.get_param("~x_min", -2.0))
        self.x_max = float(rospy.get_param("~x_max",  2.0))
        self.y_min = float(rospy.get_param("~y_min", -2.0))
        self.y_max = float(rospy.get_param("~y_max",  2.0))

        # speed profile
        self.v_min = float(rospy.get_param("~v_min", 0.1))
        self.v_max = float(rospy.get_param("~v_max", 0.6))
        self.w_max = float(rospy.get_param("~w_max", 1.2))

        # random command change
        self.cmd_hold_sec = float(rospy.get_param("~cmd_hold_sec", 1.0))
        self.rate_hz = float(rospy.get_param("~rate", 30.0))

        # --- state list ---
        self.obstacles = []  # list of dict: {name, x, y, yaw, v, w, next_cmd_time}

        # publishers
        self.obs_pub = rospy.Publisher("obstacles", ObstacleArray, queue_size=1)
        self.marker_pub = rospy.Publisher("obstacles_marker", MarkerArray, queue_size=1)

        # gazebo services
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        rospy.loginfo("Random obstacle controller: num=%d bounds x[%.2f,%.2f] y[%.2f,%.2f]",
                      self.num_obstacles, self.x_min, self.x_max, self.y_min, self.y_max)

    def spawn_obstacles(self):
        """Spawn all obstacles in Gazebo and initialize their states."""
        for i in range(self.num_obstacles):
            name = f"{self.model_prefix}_{i:02d}"
            sdf = cylinder_sdf(name, radius=self.radius, height=self.height, mass=self.mass)

            # Random initial position
            x0 = random.uniform(self.x_min, self.x_max)
            y0 = random.uniform(self.y_min, self.y_max)
            yaw0 = random.uniform(-math.pi, math.pi)

            pose = Pose()
            pose.position = Point(x0, y0, self.height * 0.5)
            pose.orientation = yaw_to_quat(yaw0)

            req = SpawnModelRequest()
            req.model_name = name
            req.model_xml = sdf
            req.robot_namespace = ""
            req.initial_pose = pose
            req.reference_frame = self.frame_id

            try:
                resp = self.spawn_srv(req)
                if not resp.success:
                    rospy.logwarn("Spawn failed for %s: %s", name, resp.status_message)
            except Exception as e:
                rospy.logwarn("Spawn exception for %s: %s", name, str(e))

            # Initialize state
            self.obstacles.append({
                "name": name,
                "id": i,
                "x": x0,
                "y": y0,
                "yaw": yaw0,
                "v": random.uniform(self.v_min, self.v_max),
                "w": 0.0,
                "next_cmd_time": rospy.Time.now()
            })

        rospy.loginfo("Spawned %d obstacles", len(self.obstacles))

    def sample_new_cmd(self, obs):
        obs["v"] = random.uniform(self.v_min, self.v_max)
        obs["w"] = random.uniform(-self.w_max, self.w_max)
        obs["next_cmd_time"] = rospy.Time.now() + rospy.Duration(self.cmd_hold_sec)

    def bounce_if_needed(self, obs):
        # If near boundary, steer back inward
        margin = 0.25
        desired_yaw = None

        if obs["x"] < self.x_min + margin:
            desired_yaw = 0.0
        elif obs["x"] > self.x_max - margin:
            desired_yaw = math.pi
        if obs["y"] < self.y_min + margin:
            desired_yaw = math.pi / 2.0
        elif obs["y"] > self.y_max - margin:
            desired_yaw = -math.pi / 2.0

        if desired_yaw is not None:
            yaw_err = (desired_yaw - obs["yaw"] + math.pi) % (2 * math.pi) - math.pi
            obs["w"] = max(-self.w_max, min(self.w_max, 2.0 * yaw_err))

    def publish_markers(self):
        """Publish obstacle array and visualization markers for all obstacles."""
        # Obstacle array for MPC
        obs_array = ObstacleArray()
        obs_array.header.frame_id = self.frame_id
        obs_array.header.stamp = rospy.Time.now()
        
        # Marker array for visualization
        marker_array = MarkerArray()

        for idx, o in enumerate(self.obstacles):
            # Obstacle message
            obs_msg = Obstacle()
            obs_msg.id = o["id"]
            obs_msg.position = Point(o["x"], o["y"], 0.0)
            obs_msg.velocity = Vector3(o["v"] * math.cos(o["yaw"]), o["v"] * math.sin(o["yaw"]), 0.0)
            obs_msg.radius = self.radius
            obs_msg.type = 1  # cylinder
            obs_array.obstacles.append(obs_msg)

            # Visual marker
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacle_visual"
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = o["x"]
            marker.pose.position.y = o["y"]
            marker.pose.position.z = self.height * 0.5
            marker.pose.orientation = yaw_to_quat(o["yaw"])
            marker.scale.x = self.radius * 2
            marker.scale.y = self.radius * 2
            marker.scale.z = self.height
            marker.color.a = 0.9
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker_array.markers.append(marker)

        self.obs_pub.publish(obs_array)
        self.marker_pub.publish(marker_array)

    def step(self, dt):
        now = rospy.Time.now()

        for o in self.obstacles:
            # Change command occasionally
            if now >= o["next_cmd_time"]:
                self.sample_new_cmd(o)

            self.bounce_if_needed(o)

            # Integrate kinematics
            o["yaw"] += o["w"] * dt
            o["yaw"] = (o["yaw"] + math.pi) % (2 * math.pi) - math.pi

            o["x"] += o["v"] * math.cos(o["yaw"]) * dt
            o["y"] += o["v"] * math.sin(o["yaw"]) * dt

            # Clamp inside bounds
            o["x"] = max(self.x_min, min(self.x_max, o["x"]))
            o["y"] = max(self.y_min, min(self.y_max, o["y"]))

            # Send to Gazebo
            state = ModelState()
            state.model_name = o["name"]
            state.reference_frame = self.frame_id
            state.pose.position.x = o["x"]
            state.pose.position.y = o["y"]
            state.pose.position.z = 0.0
            state.pose.orientation = yaw_to_quat(o["yaw"])
            state.twist = Twist()
            state.twist.linear.x = o["v"]
            state.twist.angular.z = o["w"]

            try:
                self.set_state(state)
            except Exception as e:
                rospy.logwarn_throttle(1.0, "set_model_state failed for %s: %s", o["name"], str(e))

        self.publish_markers()

    def run(self):
        self.spawn_obstacles()
        
        rate = rospy.Rate(self.rate_hz)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last).to_sec()
            last = now
            dt = max(1.0 / (5.0 * self.rate_hz), min(0.2, dt))  # keep dt reasonable
            self.step(dt)
            rate.sleep()


if __name__ == "__main__":
    RandomDynamicObstacle().run()
