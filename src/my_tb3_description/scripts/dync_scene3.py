#!/usr/bin/env python3
# Scene 3: open-field constant-velocity obstacles (no walls).
# Fixed set of cylinders, each moving in a straight line with its own 2D
# velocity, reflecting billiard-style off a rectangular boundary. Motion is
# pure constant velocity between reflections.
import math
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SetModelState, DeleteModel
from visualization_msgs.msg import Marker, MarkerArray
from mpc_nav.msg import Obstacle, ObstacleArray
from sdf import cylinder_sdf


def quat_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def make_pose(x, y, z=0.0, yaw=0.0) -> Pose:
    p = Pose()
    p.position = Point(x, y, z)
    p.orientation = quat_from_yaw(yaw)
    return p


def reflect_step(x, y, vx, vy, dt, xmin, xmax, ymin, ymax):
    """Advance one CV step and reflect off the rectangle [xmin,xmax]x[ymin,ymax].
    Pure function (no ROS). Returns (x, y, vx, vy) with velocity flipped on the
    axis whose boundary was crossed. Speed magnitude is preserved."""
    x += vx * dt
    y += vy * dt
    if x < xmin:
        x = xmin + (xmin - x)
        vx = -vx
    elif x > xmax:
        x = xmax - (x - xmax)
        vx = -vx
    if y < ymin:
        y = ymin + (ymin - y)
        vy = -vy
    elif y > ymax:
        y = ymax - (y - ymax)
        vy = -vy
    return x, y, vx, vy


DEFAULT_OBSTACLES = [
    # id, x0, y0, vx, vy
    (0, -6.0, 4.0, 0.60, 0.0),    # crossing L->R
    (1, 6.0, 7.0, -0.60, 0.0),    # crossing R->L
    (2, -5.0, 11.0, 0.50, -0.40),  # diagonal
    (3, 0.5, 15.0, 0.0, -0.50),   # head-on toward robot
    (4, -1.0, 3.0, 0.0, 0.35),    # same direction, slow
    (5, 5.0, 6.0, -0.45, 0.45),   # diagonal
]


class OpenFieldObstacles:
    def __init__(self):
        self.xmin = rospy.get_param("~xmin", -6.0)
        self.xmax = rospy.get_param("~xmax", 6.0)
        self.ymin = rospy.get_param("~ymin", 0.0)
        self.ymax = rospy.get_param("~ymax", 16.0)
        self.radius = rospy.get_param("~radius", 0.3)
        self.height = rospy.get_param("~height", 1.0)

        self.world_frame = rospy.get_param("~world_frame", "world")
        self.obs_prefix = rospy.get_param("~obs_prefix", "scene3_cyl")

        self.obstacles = []  # dict: id, x, y, vx, vy, name
        self.spawned_models = set()

        self.obs_pub = rospy.Publisher("obstacles", ObstacleArray, queue_size=1)
        self.marker_pub = rospy.Publisher("obstacles_marker", MarkerArray, queue_size=1)

        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.delete_srv = None
        try:
            rospy.wait_for_service("/gazebo/delete_model", timeout=1.0)
            self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        except Exception:
            pass

    def spawn_model(self, name, sdf_xml, pose):
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

    def set_model_pose(self, name, pose):
        st = ModelState()
        st.model_name = name
        st.pose = pose
        st.twist = Twist()
        st.reference_frame = self.world_frame
        self.set_state_srv(st)

    def cleanup(self):
        if not self.delete_srv:
            return
        for name in list(self.spawned_models):
            try:
                self.delete_srv(name)
            except Exception:
                pass
    def publish_truth_and_markers(self):
        obs_array = ObstacleArray()
        obs_array.header.frame_id = self.world_frame
        obs_array.header.stamp = rospy.Time.now()

        marker_array = MarkerArray()

        for idx, o in enumerate(self.obstacles):
            obs_msg = Obstacle()
            obs_msg.id = o["id"]
            obs_msg.position = Point(o["x"], o["y"], 0.0)
            obs_msg.velocity = Vector3(o["vx"], o["vy"], 0.0)
            obs_msg.radius = self.radius
            obs_msg.type = 1  # cylinder
            obs_array.obstacles.append(obs_msg)

            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = obs_array.header.stamp
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

    def spawn_obstacles(self):
        for oid, x0, y0, vx, vy in DEFAULT_OBSTACLES:
            name = f"{self.obs_prefix}_{oid:02d}"
            sdf = cylinder_sdf(name, radius=self.radius, height=self.height, mass=5.0)
            pose = make_pose(x0, y0, self.height * 0.5, yaw=0.0)
            self.spawn_model(name, sdf, pose)
            self.obstacles.append(
                {"id": oid, "name": name, "x": x0, "y": y0, "vx": vx, "vy": vy})

    def update_loop(self):
        rate_hz = rospy.get_param("~rate", 50.0)
        rate = rospy.Rate(rate_hz)

        # Inset bounds by radius so the cylinder body stays inside the box.
        xmin = self.xmin + self.radius
        xmax = self.xmax - self.radius
        ymin = self.ymin + self.radius
        ymax = self.ymax - self.radius
        if xmax <= xmin or ymax <= ymin:
            rospy.logerr("Invalid bounds after radius inset: "
                         "x[%.3f,%.3f] y[%.3f,%.3f]", xmin, xmax, ymin, ymax)
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
                o["x"], o["y"], o["vx"], o["vy"] = reflect_step(
                    o["x"], o["y"], o["vx"], o["vy"], dt,
                    xmin, xmax, ymin, ymax)
                self.set_model_pose(
                    o["name"], make_pose(o["x"], o["y"], self.height * 0.5, yaw=0.0))

            self.publish_truth_and_markers()
            rate.sleep()


def main():
    rospy.init_node("open_field_obstacles")
    node = OpenFieldObstacles()

    if rospy.get_param("~cleanup_on_start", False):
        node.cleanup()

    node.spawn_obstacles()
    rospy.loginfo("Spawned %d open-field constant-velocity cylinders.",
                  len(node.obstacles))
    node.update_loop()


if __name__ == "__main__":
    main()
