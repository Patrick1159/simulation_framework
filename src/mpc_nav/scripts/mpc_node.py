#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#TODO:需要把 solve_mpc() 替换成你的 CasADi/Ipopt 或其他 solver 即可。
import time
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray
import tf.transformations as tft

from mpc_nav.msg import ObstacleArray, MPCStatus
from mpc_solver import CasadiMPCSolver

@dataclass
class RobotState:
    x: float
    y: float
    yaw: float


@dataclass
class Obstacle:
    oid: int
    x: float
    y: float
    vx: float
    vy: float
    r: float


class MPCNode:
    def __init__(self):
        rospy.init_node("mpc_node")

        # Params
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.dt = float(rospy.get_param("~dt", 0.1))
        self.N = int(rospy.get_param("~N", 20))

        self.v_min = float(rospy.get_param("~v_min", -0.1))
        self.v_max = float(rospy.get_param("~v_max", 0.4))
        self.w_max = float(rospy.get_param("~w_max", 1.2))
        self.a_v_max = float(rospy.get_param("~a_v_max", 0.0))
        self.a_w_max = float(rospy.get_param("~a_w_max", 0.0))

        self.robot_radius = float(rospy.get_param("~robot_radius", 0.12))
        self.safety_margin = float(rospy.get_param("~safety_margin", 0.15))
        self.obstacle_num = int(rospy.get_param("~obstacle_num", 5))

        self.w_goal_pos = float(rospy.get_param("~w_goal_pos", 5.0))
        self.w_goal_yaw = float(rospy.get_param("~w_goal_yaw", 1.0))
        self.w_u = float(rospy.get_param("~w_u", 0.1))
        self.w_du = float(rospy.get_param("~w_du", 0.5))
        self.w_obs = float(rospy.get_param("~w_obs", 50.0))

        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal") # mpc/goal
        self.obstacles_topic = rospy.get_param("~obstacles_topic", "/obstacles")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")

        # State buffers
        self.robot: Optional[RobotState] = None
        self.goal: Optional[RobotState] = None
        self.obstacles: List[Obstacle] = []

        # Warm-start (previous control sequence)
        self.prev_u: List[Tuple[float, float]] = [(0.0, 0.0)] * self.N
        self.casadi_solver = CasadiMPCSolver(
            N=self.N,
            dt=self.dt,
            v_min=self.v_min,
            v_max=self.v_max,
            w_max=self.w_max,
            obstacle_num=self.obstacle_num,
            robot_radius=self.robot_radius,
            safety_margin=self.safety_margin,
            w_goal_pos=self.w_goal_pos,
            w_goal_yaw=self.w_goal_yaw,
            w_u=self.w_u,
            w_du=self.w_du,
            w_obs=self.w_obs,
        )
        self.u_prev = (0.0, 0.0)

        # Publishers
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.path_pub = rospy.Publisher("/mpc/pred_path", Path, queue_size=1)
        self.status_pub = rospy.Publisher("/mpc/status", MPCStatus, queue_size=1)

        # Subscribers
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.on_goal, queue_size=1)
        rospy.Subscriber(self.obstacles_topic, ObstacleArray, self.on_obstacles, queue_size=1)

        rospy.loginfo("MPC node started: N=%d dt=%.3f rate=%.1fHz", self.N, self.dt, self.rate_hz)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.robot = RobotState(p.x, p.y, yaw)

    def on_goal(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.goal = RobotState(p.x, p.y, yaw)
        rospy.loginfo("New goal received: x=%.2f y=%.2f yaw=%.2f", p.x, p.y, yaw)

    def on_obstacles(self, msg: ObstacleArray):
        obs = []
        for o in msg.obstacles:
            obs.append(Obstacle(
                oid=int(o.id),
                x=float(o.position.x),
                y=float(o.position.y),
                vx=float(o.velocity.x),
                vy=float(o.velocity.y),
                r=float(o.radius),
            ))
        self.obstacles = obs

    # -------------------------
    # Core MPC Solver
    # -------------------------
    def solve_mpc(self, x0: RobotState, goal: RobotState, obstacles: List[Obstacle]):
        # rospy.loginfo("Solving MPC...")
        t0 = time.time()

        x0_vec = [x0.x, x0.y, x0.yaw]
        g_vec = [goal.x, goal.y, goal.yaw]
        u_prev = [self.u_prev[0], self.u_prev[1]]

        # warm-start init from prev_u (controls)
        u_init = None
        if self.prev_u is not None and len(self.prev_u) == self.N:
            import casadi as ca
            u_init = ca.DM.zeros(2, self.N)
            for k in range(self.N):
                u_init[0, k] = self.prev_u[k][0]
                u_init[1, k] = self.prev_u[k][1]

        # ---- pack obstacles into fixed-length params (M*5) ----
        M = self.obstacle_num
        # 取最近 M 个障碍物（按到机器人当前位置距离）
        obs_sorted = sorted(
            obstacles,
            key=lambda o: (o.x - x0.x) ** 2 + (o.y - x0.y) ** 2
        )
        obs_sel = obs_sorted[:M]

        obs_params = []
        for i in range(M):
            if i < len(obs_sel):
                o = obs_sel[i]
                obs_params += [o.x, o.y, o.vx, o.vy, o.r]
            else:
                # padding：放很远、半径0，相当于“无障碍”
                obs_params += [1e6, 1e6, 0.0, 0.0, 0.0]

        # -------------------Call solver-------------------
        try:
            ok, solve_ms, cost, X_opt, U_opt = self.casadi_solver.solve(
                x0=x0_vec,
                goal=g_vec,
                obs_params=obs_params,
                u_prev=u_prev,
                u_init=u_init
            )
        except Exception as e:
            err = f"{type(e).__name__}: {str(e)}"
            rospy.logerr_throttle(1.0, "MPC solve exception: %s", err)
            return False, (time.time() - t0) * 1000.0, 0.0, [x0], [(0.0, 0.0)] * self.N

        # extract predicted states + control sequence
        x_pred = []
        u_seq = []
        for k in range(self.N + 1):
            x_pred.append(RobotState(float(X_opt[0, k]), float(X_opt[1, k]), float(X_opt[2, k])))
        for k in range(self.N):
            u_seq.append((float(U_opt[0, k]), float(U_opt[1, k])))

        return ok, solve_ms, cost, x_pred, u_seq

    def rollout(self, x0: RobotState, u_seq: List[Tuple[float, float]]) -> List[RobotState]:
        xs = [x0]
        x, y, yaw = x0.x, x0.y, x0.yaw
        for (v, w) in u_seq:
            x += self.dt * v * math.cos(yaw)
            y += self.dt * v * math.sin(yaw)
            yaw = self.wrap_angle(yaw + self.dt * w)
            xs.append(RobotState(x, y, yaw))
        return xs

    @staticmethod
    def wrap_angle(a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi

    # -------------------------
    # Publishing helpers
    # -------------------------
    def publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

    def publish_pred_path(self, states: List[RobotState]):
        path = Path()
        path.header.stamp = rospy.Time(0) # rospy.Time.now()
        path.header.frame_id = self.odom_frame

        for st in states:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = st.x
            ps.pose.position.y = st.y
            q = tft.quaternion_from_euler(0, 0, st.yaw)
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            path.poses.append(ps)

        self.path_pub.publish(path)

    def publish_status(self, success: bool, solve_ms: float, cost: float, text: str):
        st = MPCStatus()
        st.header.stamp = rospy.Time.now()
        st.success = success
        st.solve_time_ms = solve_ms
        st.cost = cost
        st.status_text = text
        self.status_pub.publish(st)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.robot is None or self.goal is None:
                rate.sleep()
                continue

            success, solve_ms, cost, x_pred, u_seq = self.solve_mpc(self.robot, self.goal, self.obstacles)

            # Choose first control
            self.u_prev = u_seq[0]
            v0, w0 = u_seq[0]
            self.publish_cmd(v0, w0)
            self.publish_pred_path(x_pred)
            self.publish_status(success, solve_ms, cost, "OK" if success else "FAIL")

            # warm-start update (shift)
            self.prev_u = u_seq[1:] + [u_seq[-1]]
            rate.sleep()


if __name__ == "__main__":
    node = MPCNode()
    node.spin()