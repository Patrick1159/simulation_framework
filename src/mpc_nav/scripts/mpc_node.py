#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#TODO:需要把 solve_mpc() 替换成你的 CasADi/Ipopt 或其他 solver 即可。
import time
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
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
    r: float           # legacy circular fallback / used for near hysteresis
    a: float           # ellipse semi-major (along velocity)
    b: float           # ellipse semi-minor (perpendicular to velocity)
    # KF covariance diagonal at current step (post-update). 0 = no info.
    s2_xx: float = 0.0
    s2_yy: float = 0.0
    s2_vxvx: float = 0.0
    s2_vyvy: float = 0.0


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
        self.hard_safety_margin = float(rospy.get_param("~hard_safety_margin", 0.05))
        self.obstacle_num = int(rospy.get_param("~obstacle_num", 5))
        # Velocity-scaled lookahead applied along the obstacle's motion
        # direction inside the MPC objective. Models the swept "danger
        # corridor" the obstacle will trace over the next ~tau_lookahead
        # seconds as a forward swept capsule (line segment from p_obs to
        # p_obs + tau*v_obs, inflated by max(a,b) plus margins). This is
        # what biases the planner toward going behind a moving pedestrian
        # rather than cutting in front. Pure planner-side; does not touch
        # Obstacle.msg geometry or visualization.
        self.tau_lookahead = float(rospy.get_param("~tau_lookahead", 1.0))
        # Below this speed the obstacle is treated as static (no swept
        # corridor — capsule degenerates to a point with the geometric
        # ellipse footprint).
        self.dynamic_v_min = float(rospy.get_param("~dynamic_v_min", 0.1))
        # Capsule radius / length uncertainty inflation, expressed as
        # multiples of the KF position+velocity sigma at each step:
        #   capsule_radius += alpha * sigma_perp(k)
        #   capsule_endpoint += alpha * sigma_par(k) * v_hat
        # Set to 0.0 to disable uncertainty inflation (recovers a pure
        # geometric capsule).
        self.uncertainty_alpha = float(rospy.get_param("~uncertainty_alpha", 1.0))

        # Swept-capsule visualization. Draws the same exclusion zones the
        # solver actually uses: at each sampled prediction step k we emit
        # a stadium-shaped outline (2 side lines + 2 semicircle arcs) in
        # the obstacle's swept position at t_k. Sampling every `stride`
        # steps keeps the picture readable (N=45 -> ~6 outlines per obstacle).
        self.swept_capsule_vis_enable = bool(rospy.get_param("~swept_capsule_vis_enable", True))
        self.swept_capsule_vis_stride = max(1, int(rospy.get_param("~swept_capsule_vis_stride", 8)))
        self.swept_capsule_arc_points = max(4, int(rospy.get_param("~swept_capsule_arc_points", 10)))

        self.w_goal_pos = float(rospy.get_param("~w_goal_pos", 5.0))
        self.w_goal_yaw = float(rospy.get_param("~w_goal_yaw", 1.0))
        self.w_u = float(rospy.get_param("~w_u", 0.1))
        self.w_du = float(rospy.get_param("~w_du", 0.5))
        self.w_obs = float(rospy.get_param("~w_obs", 50.0))
        self.rho_slack = float(rospy.get_param("~rho_slack", 5000.0))

        self.near_dist_activate = float(rospy.get_param("~near_dist_activate", 1.2))
        self.near_dist_release = float(rospy.get_param("~near_dist_release", 1.6))
        self.near_ttc_activate = float(rospy.get_param("~near_ttc_activate", 2.5))
        self.near_ttc_release = float(rospy.get_param("~near_ttc_release", 3.5))

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
        self.near_obstacle_state: Dict[int, bool] = {}

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
            soft_margin=self.safety_margin,
            hard_margin=self.hard_safety_margin,
            tau_lookahead=self.tau_lookahead,
            dynamic_v_min=self.dynamic_v_min,
            uncertainty_alpha=self.uncertainty_alpha,
            w_goal_pos=self.w_goal_pos,
            w_goal_yaw=self.w_goal_yaw,
            w_u=self.w_u,
            w_du=self.w_du,
            w_obs=self.w_obs,
            rho_slack=self.rho_slack,
        )
        self.u_prev = (0.0, 0.0)

        # Publishers
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.path_pub = rospy.Publisher("/mpc/pred_path", Path, queue_size=1)
        self.status_pub = rospy.Publisher("/mpc/status", MPCStatus, queue_size=1)
        self.swept_capsule_pub = rospy.Publisher("/mpc/swept_capsules", MarkerArray, queue_size=1)

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
        active_ids = set()
        for o in msg.obstacles:
            oid = int(o.id)
            r = float(o.radius)
            # Ellipse fields (geometric only — no lookahead); fall back to
            # circular `radius` when the publisher (e.g. legacy GT scene
            # scripts) does not populate them.
            sm = float(getattr(o, "semi_major", 0.0) or 0.0)
            sn = float(getattr(o, "semi_minor", 0.0) or 0.0)
            if sm <= 0.0 or sn <= 0.0:
                sm = r
                sn = r
            obs.append(Obstacle(
                oid=oid,
                x=float(o.position.x),
                y=float(o.position.y),
                vx=float(o.velocity.x),
                vy=float(o.velocity.y),
                r=r,
                a=sm,
                b=sn,
                s2_xx=float(getattr(o, "sigma2_xx", 0.0) or 0.0),
                s2_yy=float(getattr(o, "sigma2_yy", 0.0) or 0.0),
                s2_vxvx=float(getattr(o, "sigma2_vxvx", 0.0) or 0.0),
                s2_vyvy=float(getattr(o, "sigma2_vyvy", 0.0) or 0.0),
            ))
            active_ids.add(oid)
        self.obstacles = obs
        stale_ids = [oid for oid in self.near_obstacle_state if oid not in active_ids]
        for oid in stale_ids:
            self.near_obstacle_state.pop(oid, None)

    def _robot_planar_velocity(self, state: RobotState) -> Tuple[float, float]:
        v = float(self.u_prev[0])
        return v * math.cos(state.yaw), v * math.sin(state.yaw)

    def _obstacle_metrics(self, robot: RobotState, obs: Obstacle) -> Tuple[float, float]:
        rvx, rvy = self._robot_planar_velocity(robot)
        rel_x = obs.x - robot.x
        rel_y = obs.y - robot.y
        rel_vx = obs.vx - rvx
        rel_vy = obs.vy - rvy

        # near-hysteresis uses a circular outer bound (max half-axis) — keeps
        # the activate/release logic simple and conservative.
        outer_r = max(obs.a, obs.b)
        base_clearance = max(0.0, math.hypot(rel_x, rel_y) - (self.robot_radius + outer_r))
        min_clearance = base_clearance
        for k in range(self.N + 1):
            t = k * self.dt
            dx = rel_x + t * rel_vx
            dy = rel_y + t * rel_vy
            clearance = math.hypot(dx, dy) - (self.robot_radius + outer_r)
            if clearance < min_clearance:
                min_clearance = clearance

        rel_v2 = rel_vx * rel_vx + rel_vy * rel_vy
        if rel_v2 < 1e-6:
            ttc = float("inf")
        else:
            dot = rel_x * rel_vx + rel_y * rel_vy
            ttc = -dot / rel_v2 if dot < 0.0 else float("inf")

        return min_clearance, ttc

    def _is_near_obstacle(self, robot: RobotState, obs: Obstacle) -> bool:
        min_clearance, ttc = self._obstacle_metrics(robot, obs)
        was_near = self.near_obstacle_state.get(obs.oid, False)

        activate = (min_clearance <= self.near_dist_activate) or (ttc <= self.near_ttc_activate)
        release = (min_clearance >= self.near_dist_release) and (ttc >= self.near_ttc_release)

        if was_near:
            is_near = not release
        else:
            is_near = activate

        self.near_obstacle_state[obs.oid] = is_near
        return is_near

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

        # ---- pack obstacles into fixed-length params (M*11) ----
        # 11 floats per slot:
        #   [ox, oy, ovx, ovy, semi_major, semi_minor, near_flag,
        #    sigma2_xx, sigma2_yy, sigma2_vxvx, sigma2_vyvy]
        # The four sigma2_* values may be 0 when upstream has no probabilistic
        # tracker; the solver treats that as "no uncertainty inflation".
        M = self.obstacle_num
        obs_with_flags = []
        for o in obstacles:
            near_flag = 1.0 if self._is_near_obstacle(x0, o) else 0.0
            dist2 = (o.x - x0.x) ** 2 + (o.y - x0.y) ** 2
            obs_with_flags.append((near_flag, dist2, o))

        # 近场障碍物优先，其余按距离排序
        obs_sorted = sorted(
            obs_with_flags,
            key=lambda item: (-item[0], item[1])
        )
        obs_sel = obs_sorted[:M]

        obs_params = []
        for i in range(M):
            if i < len(obs_sel):
                near_flag, _, o = obs_sel[i]
                obs_params += [
                    o.x, o.y, o.vx, o.vy,
                    o.a, o.b, near_flag,
                    o.s2_xx, o.s2_yy, o.s2_vxvx, o.s2_vyvy,
                ]
            else:
                # Empty slot: park far away with zero ellipse + zero velocity
                # so its constraint is always inactive.
                obs_params += [1e6, 1e6, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0]

        try:
            self.publish_swept_capsules(obs_params)
        except Exception as e:
            rospy.logwarn_throttle(1.0, "Swept capsule visualization failed: %s", str(e))

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

    def _swept_capsule_steps(self) -> List[int]:
        last_solver_step = max(0, self.N - 1)
        steps = list(range(0, self.N, self.swept_capsule_vis_stride))
        if not steps or steps[-1] != last_solver_step:
            steps.append(last_solver_step)
        return steps

    @staticmethod
    def _mk_point(x: float, y: float, z: float = 0.04) -> Point:
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        return p

    def _capsule_outline_points(
        self,
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
        radius: float,
    ) -> List[Point]:
        dx = end_x - start_x
        dy = end_y - start_y
        seg_len = math.hypot(dx, dy)

        if seg_len < 1e-6:
            count = max(12, 2 * self.swept_capsule_arc_points)
            return [
                self._mk_point(
                    start_x + radius * math.cos(2.0 * math.pi * j / count),
                    start_y + radius * math.sin(2.0 * math.pi * j / count),
                )
                for j in range(count + 1)
            ]

        ux = dx / seg_len
        uy = dy / seg_len
        px = -uy
        py = ux
        theta = math.atan2(uy, ux)
        points = [
            self._mk_point(start_x + px * radius, start_y + py * radius),
            self._mk_point(end_x + px * radius, end_y + py * radius),
        ]

        for j in range(1, self.swept_capsule_arc_points + 1):
            ang = theta + math.pi / 2.0 - j * math.pi / self.swept_capsule_arc_points
            points.append(self._mk_point(end_x + radius * math.cos(ang),
                                         end_y + radius * math.sin(ang)))

        points.append(self._mk_point(start_x - px * radius, start_y - py * radius))

        for j in range(1, self.swept_capsule_arc_points + 1):
            ang = theta - math.pi / 2.0 - j * math.pi / self.swept_capsule_arc_points
            points.append(self._mk_point(start_x + radius * math.cos(ang),
                                         start_y + radius * math.sin(ang)))

        points.append(points[0])
        return points

    def _swept_capsule_color(self, k: int) -> ColorRGBA:
        phase = float(k) / float(max(1, self.N - 1))
        color = ColorRGBA()
        color.r = 0.1 + 0.9 * phase
        color.g = 0.95 - 0.55 * phase
        color.b = 1.0 - 0.75 * phase
        color.a = max(0.16, 0.78 - 0.58 * phase)
        return color

    def _swept_capsule_geometry(
        self,
        params: List[float],
        k: int,
    ) -> Optional[Tuple[float, float, float, float, float]]:
        ox, oy, ovx, ovy, semi_major, semi_minor, _, s2_xx, s2_yy, s2_vxvx, s2_vyvy = params
        if abs(ox) > 1e5 or abs(oy) > 1e5:
            return None

        oxk = ox + (k * self.dt) * ovx
        oyk = oy + (k * self.dt) * ovy
        v_norm = math.sqrt(ovx * ovx + ovy * ovy + 1e-9)
        e_par_x = ovx / v_norm
        e_par_y = ovy / v_norm
        e_perp_x = -e_par_y
        e_perp_y = e_par_x
        gate = 0.5 * (1.0 + math.tanh(20.0 * (v_norm - self.dynamic_v_min)))

        t_k = k * self.dt
        sigma_pos_par2 = e_par_x * e_par_x * s2_xx + e_par_y * e_par_y * s2_yy
        sigma_pos_perp2 = e_perp_x * e_perp_x * s2_xx + e_perp_y * e_perp_y * s2_yy
        sigma_vel_par2 = e_par_x * e_par_x * s2_vxvx + e_par_y * e_par_y * s2_vyvy
        sigma_vel_perp2 = e_perp_x * e_perp_x * s2_vxvx + e_perp_y * e_perp_y * s2_vyvy
        sigma_par_k = math.sqrt(max(0.0, sigma_pos_par2 + t_k * t_k * sigma_vel_par2) + 1e-12)
        sigma_perp_k = math.sqrt(max(0.0, sigma_pos_perp2 + t_k * t_k * sigma_vel_perp2) + 1e-12)

        seg_len = (self.tau_lookahead * v_norm + self.uncertainty_alpha * sigma_par_k) * gate
        end_x = oxk + seg_len * e_par_x
        end_y = oyk + seg_len * e_par_y
        radius = (
            self.robot_radius
            + self.safety_margin
            + max(semi_major, semi_minor)
            + self.uncertainty_alpha * sigma_perp_k
        )
        if radius <= 0.0:
            return None
        return oxk, oyk, end_x, end_y, radius

    def publish_swept_capsules(self, obs_params: List[float]):
        marker_array = MarkerArray()
        stamp = rospy.Time.now()

        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.odom_frame
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        if not self.swept_capsule_vis_enable:
            self.swept_capsule_pub.publish(marker_array)
            return

        n_per_obs = 11
        marker_id = 0
        lifetime = rospy.Duration(max(0.2, 2.0 / max(1.0, self.rate_hz)))
        for obs_idx in range(min(self.obstacle_num, len(obs_params) // n_per_obs)):
            base = obs_idx * n_per_obs
            params = obs_params[base:base + n_per_obs]
            for k in self._swept_capsule_steps():
                geometry = self._swept_capsule_geometry(params, k)
                if geometry is None:
                    continue
                start_x, start_y, end_x, end_y, radius = geometry

                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = self.odom_frame
                marker.ns = "mpc_swept_capsules"
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.025
                marker.color = self._swept_capsule_color(k)
                marker.lifetime = lifetime
                marker.points = self._capsule_outline_points(start_x, start_y, end_x, end_y, radius)
                marker_array.markers.append(marker)
                marker_id += 1

        self.swept_capsule_pub.publish(marker_array)

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
