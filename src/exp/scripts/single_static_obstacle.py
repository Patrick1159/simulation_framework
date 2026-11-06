#!/usr/bin/env python3
"""
single_static_obstacle.py — Spawn one static cylinder in Gazebo and publish GT.

Parameters:
  ~x, ~y, ~z       Cylinder center position in world/odom frame
                   (default: 2.0, 0.0, 0.5)
  ~radius           Cylinder radius (default: 0.2)
  ~height           Cylinder height (default: 1.0)
  ~frame_id         Frame for GT messages (default: world)
  ~rate             GT publish rate Hz (default: 15)
"""

import math
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, SetModelState
from mpc_nav.msg import Obstacle, ObstacleArray


CYLINDER_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        <surface><friction><ode><mu>0.5</mu><mu2>0.5</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


class SingleStaticObstacle:
    def __init__(self):
        self.x = rospy.get_param('~x', 2.0)
        self.y = rospy.get_param('~y', 0.0)
        self.z = rospy.get_param('~z', 0.5)
        self.radius = rospy.get_param('~radius', 0.2)
        self.height = rospy.get_param('~height', 1.0)
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.rate_hz = rospy.get_param('~rate', 15.0)
        self.model_name = rospy.get_param('~model_name', 'stage2_obs')

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        rospy.on_shutdown(self._cleanup)

        # Spawn
        sdf = CYLINDER_SDF.format(name=self.model_name, r=self.radius,
                                   h=self.height)
        req = SpawnModelRequest()
        req.model_name = self.model_name
        req.model_xml = sdf
        req.robot_namespace = ''
        # `z` is documented and published as the cylinder center. Spawning at
        # z + h/2 incorrectly makes the obstacle float by half its height.
        req.initial_pose = Pose(Point(self.x, self.y, self.z),
                                Quaternion(0, 0, 0, 1))
        req.reference_frame = self.frame_id
        resp = self.spawn_srv(req)
        if resp.success:
            rospy.loginfo("Spawned '%s' at (%.2f, %.2f, %.2f)",
                          self.model_name, self.x, self.y, self.z)
        else:
            rospy.logerr("Spawn failed: %s", resp.status_message)

        # GT publisher
        self.gt_pub = rospy.Publisher('/obstacles', ObstacleArray, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz),
                                  self._publish_gt)

    def _publish_gt(self, _event):
        gt = ObstacleArray()
        gt.header.frame_id = self.frame_id
        gt.header.stamp = rospy.Time.now()
        obs = Obstacle()
        obs.id = 0
        obs.position = Point(self.x, self.y, self.z)
        obs.velocity = Vector3(0, 0, 0)
        obs.radius = self.radius
        obs.type = 1
        obs.confidence = 1.0
        gt.obstacles.append(obs)
        self.gt_pub.publish(gt)

    def _cleanup(self):
        try:
            self.delete_srv(self.model_name)
            rospy.loginfo("Deleted '%s'", self.model_name)
        except Exception:
            pass


if __name__ == '__main__':
    rospy.init_node('single_static_obstacle')
    SingleStaticObstacle()
    rospy.spin()
