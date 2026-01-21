#!/usr/bin/env python3
# filepath: /home/patrick/my_sim/src/my_tb3_description/scripts/dynamic_obstacle_field.py
import math
import random
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SetModelState, DeleteModel
from visualization_msgs.msg import Marker, MarkerArray
from mpc_nav.msg import Obstacle, ObstacleArray
from sdf import wall_sdf, cylinder_sdf


def quat_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def make_pose(x, y, z=0.0, yaw=0.0) -> Pose:
    p = Pose()
    p.position = Point(x, y, z)
    p.orientation = quat_from_yaw(yaw)
    return p


class DynamicObstacleField:
    def __init__(self):
        # Field range (params)
        self.xmin = rospy.get_param("~xmin", -1.5)
        self.xmax = rospy.get_param("~xmax", 1.5)
        self.ymin = rospy.get_param("~ymin", -1.0)
        self.ymax = rospy.get_param("~ymax", 11.0)

        # Motion / obstacles
        self.spacing = rospy.get_param("~spacing", 2.0)          # meters along y
        self.speed = rospy.get_param("~speed", 0.5)              # m/s (back-and-forth)
        self.speed_variance = rospy.get_param("~speed_variance", 0.2)  # random variation
        self.radius = rospy.get_param("~radius", 0.3)
        self.height = rospy.get_param("~height", 1.8)

        # Walls
        self.wall_thickness = rospy.get_param("~wall_thickness", 0.08)
        self.wall_height = rospy.get_param("~wall_height", 2.0)

        # Namespaces
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.wall_prefix = rospy.get_param("~wall_prefix", "test_wall")
        self.obs_prefix = rospy.get_param("~obs_prefix", "dyn_cyl")

        # Internal
        self.obstacles = []   # list of dict: {name, y, x, dir}
        self.spawned_models = set()

        # Publishers for obstacles and markers
        self.obs_pub = rospy.Publisher("obstacles", ObstacleArray, queue_size=1)
        self.marker_pub = rospy.Publisher("obstacles_marker", MarkerArray, queue_size=1)

        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")

        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # Optional cleanup
        self.delete_srv = None
        try:
            rospy.wait_for_service("/gazebo/delete_model", timeout=1.0)
            self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        except Exception:
            pass

    def spawn_model(self, name: str, sdf_xml: str, pose: Pose):
        req = SpawnModelRequest()
        req.model_name = name
        req.model_xml = sdf_xml
        req.robot_namespace = ""
        req.initial_pose = pose
        req.reference_frame = self.world_frame
        resp = self.spawn_srv(req)
        if not resp.success:
            rospy.logwarn("Spawn failed for %s: %s", name, resp.status_message)
        else:
            self.spawned_models.add(name)

    def set_model_pose(self, name: str, pose: Pose):
        st = ModelState()
        st.model_name = name
        st.pose = pose
        st.twist = Twist()  # we command pose directly
        st.reference_frame = self.world_frame
        self.set_state_srv(st)

    def publish_truth_and_markers(self):
        """Publish obstacle array and visualization markers for all obstacles."""
        # Obstacle array for MPC
        obs_array = ObstacleArray()
        obs_array.header.frame_id = self.world_frame
        obs_array.header.stamp = rospy.Time.now()
        
        # Marker array for visualization
        marker_array = MarkerArray()
        
        for idx, o in enumerate(self.obstacles):
            # Obstacle message
            obs_msg = Obstacle()
            obs_msg.id = o["id"]
            obs_msg.position = Point(o["x"], o["y"], 0.0)
            obs_msg.velocity = Vector3(o["dir"] * o["speed"], 0.0, 0.0)
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
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.1
            marker_array.markers.append(marker)

        self.obs_pub.publish(obs_array)
        self.marker_pub.publish(marker_array)

    def cleanup(self):
        if not self.delete_srv:
            return
        for name in list(self.spawned_models):
            try:
                self.delete_srv(name)
            except Exception:
                pass

    def spawn_walls(self):
        # Rectangle boundaries:
        # left/right walls: along y (length = ymax-ymin), thickness in x, centered at (xmin or xmax)
        # bottom/top walls: along x (length = xmax-xmin), thickness in y, centered at (ymin or ymax)
        Lx = max(0.1, self.xmax - self.xmin)
        Ly = max(0.1, self.ymax - self.ymin)
        cx = 0.5 * (self.xmin + self.xmax)
        cy = 0.5 * (self.ymin + self.ymax)

        # Left wall (at xmin)
        left_name = f"{self.wall_prefix}_left"
        left_sdf = wall_sdf(left_name, length=Ly, thickness=self.wall_thickness, height=self.wall_height)
        left_pose = make_pose(self.xmin, cy, self.wall_height * 0.5, yaw=math.pi * 0.5)
        self.spawn_model(left_name, left_sdf, left_pose)

        # Right wall (at xmax)
        right_name = f"{self.wall_prefix}_right"
        right_sdf = wall_sdf(right_name, length=Ly, thickness=self.wall_thickness, height=self.wall_height)
        right_pose = make_pose(self.xmax, cy, self.wall_height * 0.5, yaw=math.pi * 0.5)
        self.spawn_model(right_name, right_sdf, right_pose)

        # Bottom wall (at ymin)
        bottom_name = f"{self.wall_prefix}_bottom"
        bottom_sdf = wall_sdf(bottom_name, length=Lx, thickness=self.wall_thickness, height=self.wall_height)
        bottom_pose = make_pose(cx, self.ymin, self.wall_height * 0.5, yaw=0.0)
        self.spawn_model(bottom_name, bottom_sdf, bottom_pose)

        # Top wall (at ymax)
        top_name = f"{self.wall_prefix}_top"
        top_sdf = wall_sdf(top_name, length=Lx, thickness=self.wall_thickness, height=self.wall_height)
        top_pose = make_pose(cx, self.ymax, self.wall_height * 0.5, yaw=0.0)
        self.spawn_model(top_name, top_sdf, top_pose)

    def spawn_obstacles(self):
        # Place cylinders every spacing meters along y, centered in x initially.
        y = self.ymin + self.spacing
        idx = 0
        while y < self.ymax - 1e-6:
            name = f"{self.obs_prefix}_{idx:02d}"
            sdf = cylinder_sdf(name, radius=self.radius, height=self.height, mass=5.0)

            x0 = 0.5 * (self.xmin + self.xmax)
            pose = make_pose(x0, y, self.height * 0.5, yaw=0.0)
            self.spawn_model(name, sdf, pose)

            # Assign random speed for each obstacle
            speed = self.speed + random.uniform(-self.speed_variance, self.speed_variance)
            speed = max(0.1, speed)  # ensure positive speed
            
            self.obstacles.append({"name": name, "x": x0, "y": y, "dir": 1.0, "speed": speed, "id": idx})
            idx += 1
            y += self.spacing

    def update_loop(self):
        rate_hz = rospy.get_param("~rate", 50.0)
        rate = rospy.Rate(rate_hz)

        # Keep a margin so cylinder doesn't intersect the wall
        margin = rospy.get_param("~wall_margin", 0.02)
        xmin = self.xmin + self.radius + 0.5 * self.wall_thickness + margin
        xmax = self.xmax - self.radius - 0.5 * self.wall_thickness - margin

        if xmax <= xmin:
            rospy.logerr("Invalid bounds after margin: xmin=%.3f xmax=%.3f", xmin, xmax)
            return

        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last).to_sec()
            last = now
            if dt <= 0.0:
                rate.sleep()
                continue

            for o in self.obstacles:
                o["x"] += o["dir"] * o["speed"] * dt
                if o["x"] >= xmax:
                    o["x"] = xmax
                    o["dir"] = -1.0
                elif o["x"] <= xmin:
                    o["x"] = xmin
                    o["dir"] = 1.0

                self.set_model_pose(o["name"], make_pose(o["x"], o["y"], self.height * 0.5, yaw=0.0))

            self.publish_truth_and_markers()
            rate.sleep()


def main():
    rospy.init_node("dynamic_obstacle_field")

    node = DynamicObstacleField()

    # Optional: delete old models from previous runs (comment out if undesired)
    cleanup = rospy.get_param("~cleanup_on_start", False)
    if cleanup:
        node.cleanup()

    node.spawn_walls()
    node.spawn_obstacles()

    rospy.loginfo("Spawned walls + %d dynamic cylinders. Moving at %.2f m/s.",
                  len(node.obstacles), node.speed)

    node.update_loop()


if __name__ == "__main__":
    main()