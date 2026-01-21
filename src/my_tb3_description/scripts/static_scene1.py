#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import random
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel
from visualization_msgs.msg import Marker, MarkerArray
from mpc_nav.msg import Obstacle, ObstacleArray
from sdf import cylinder_sdf


def quat_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class StaticObstacleField:
    def __init__(self):
        rospy.init_node("static_obstacle_field")

        # Params
        self.num_obstacles = int(rospy.get_param("~num_obstacles", 5))
        self.xmin = rospy.get_param("~xmin", -2.0)
        self.xmax = rospy.get_param("~xmax", 2.0)
        self.ymin = rospy.get_param("~ymin", 0.0)
        self.ymax = rospy.get_param("~ymax", 10.0)
        self.radius = rospy.get_param("~radius", 0.3)
        self.height = rospy.get_param("~height", 1.8)
        self.min_distance = rospy.get_param("~min_distance", 0.8)  # min spacing between obstacles
        
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.obs_prefix = rospy.get_param("~obs_prefix", "static_obs")

        # Internal
        self.obstacles = []  # list of dict: {name, x, y, id}
        self.spawned_models = set()

        # Publishers
        self.obs_pub = rospy.Publisher("obstacles", ObstacleArray, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("obstacles_marker", MarkerArray, queue_size=1, latch=True)

        # Gazebo service
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        # Optional cleanup
        self.delete_srv = None
        try:
            rospy.wait_for_service("/gazebo/delete_model", timeout=1.0)
            self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        except Exception:
            pass

        rospy.loginfo("Static obstacle field: num=%d bounds x[%.2f,%.2f] y[%.2f,%.2f]",
                      self.num_obstacles, self.xmin, self.xmax, self.ymin, self.ymax)

    def check_collision(self, x, y, existing):
        """Check if new obstacle at (x,y) collides with existing obstacles."""
        for obs in existing:
            dx = x - obs["x"]
            dy = y - obs["y"]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < self.min_distance:
                return True
        return False

    def spawn_obstacles(self):
        """Spawn static obstacles with random non-overlapping positions."""
        max_attempts = 1000
        for i in range(self.num_obstacles):
            placed = False
            for _ in range(max_attempts):
                x0 = random.uniform(self.xmin, self.xmax)
                y0 = random.uniform(self.ymin, self.ymax)
                
                if not self.check_collision(x0, y0, self.obstacles):
                    name = f"{self.obs_prefix}_{i:02d}"
                    sdf = cylinder_sdf(name, radius=self.radius, height=self.height, mass=5.0)

                    pose = Pose()
                    pose.position = Point(x0, y0, self.height * 0.5)
                    pose.orientation = quat_from_yaw(0.0)

                    req = SpawnModelRequest()
                    req.model_name = name
                    req.model_xml = sdf
                    req.robot_namespace = ""
                    req.initial_pose = pose
                    req.reference_frame = self.world_frame

                    try:
                        resp = self.spawn_srv(req)
                        if resp.success:
                            self.obstacles.append({"name": name, "x": x0, "y": y0, "id": i})
                            self.spawned_models.add(name)
                            placed = True
                            break
                        else:
                            rospy.logwarn("Spawn failed for %s: %s", name, resp.status_message)
                    except Exception as e:
                        rospy.logwarn("Spawn exception for %s: %s", name, str(e))

            if not placed:
                rospy.logwarn("Could not place obstacle %d after %d attempts", i, max_attempts)

        rospy.loginfo("Spawned %d static obstacles", len(self.obstacles))

    def publish_obstacles(self):
        """Publish obstacle array and markers (once, latched)."""
        # Obstacle array for MPC
        obs_array = ObstacleArray()
        obs_array.header.frame_id = self.world_frame
        obs_array.header.stamp = rospy.Time.now()
        
        # Marker array for visualization
        marker_array = MarkerArray()
        
        for idx, o in enumerate(self.obstacles):
            # Obstacle message (static: zero velocity)
            obs_msg = Obstacle()
            obs_msg.id = o["id"]
            obs_msg.position = Point(o["x"], o["y"], 0.0)
            obs_msg.velocity = Vector3(0.0, 0.0, 0.0)
            obs_msg.radius = self.radius
            obs_msg.type = 1  # cylinder
            obs_array.obstacles.append(obs_msg)

            # Visualization marker
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacle_visual"
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = o["x"]
            marker.pose.position.y = o["y"]
            marker.pose.position.z = self.height * 0.5
            marker.pose.orientation = quat_from_yaw(0.0)
            marker.scale.x = self.radius * 2
            marker.scale.y = self.radius * 2
            marker.scale.z = self.height
            marker.color.a = 0.9
            marker.color.r = 0.3
            marker.color.g = 0.3
            marker.color.b = 0.9
            marker_array.markers.append(marker)

        self.obs_pub.publish(obs_array)
        self.marker_pub.publish(marker_array)

    def cleanup(self):
        """Delete all spawned models from previous runs."""
        if not self.delete_srv:
            return
        for name in list(self.spawned_models):
            try:
                self.delete_srv(name)
            except Exception:
                pass

    def run(self):
        cleanup = rospy.get_param("~cleanup_on_start", False)
        if cleanup:
            self.cleanup()

        self.spawn_obstacles()
        self.publish_obstacles()
        
        rospy.loginfo("Static obstacle field ready. Obstacles published (latched).")
        rospy.spin()


if __name__ == "__main__":
    node = StaticObstacleField()
    node.run()
