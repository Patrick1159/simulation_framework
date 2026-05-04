#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import casadi as ca


class CasadiMPCSolver:
    def __init__(self, N, dt,
                 v_min, v_max, w_max,
                 obstacle_num, robot_radius, soft_margin, hard_margin,
                 tau_lookahead,
                 w_goal_pos, w_goal_yaw, w_u, w_du, w_obs, rho_slack):
        self.N = N
        self.dt = dt
        self.M = obstacle_num
        # number of float slots per obstacle in the parameter vector;
        # see mpc_node._solve_mpc obs_params packing.
        self._n_per_obs = 8

        # decision variables: X(3, N+1), U(2, N), S(M, N)
        X = ca.SX.sym("X", 3, N + 1)   # [x,y,yaw]
        U = ca.SX.sym("U", 2, N)       # [v,w]
        S = ca.SX.sym("S", self.M, N)  # slack for near obstacles

        # parameters: x0(3), goal(3), u_prev(2), per-obstacle 8 floats:
        #   [ox, oy, ovx, ovy, semi_major, semi_minor, near_flag, _reserved]
        P = ca.SX.sym("P", 3 + 3 + 2 + self.M * self._n_per_obs)

        x0 = P[0:3]
        g = P[3:6]
        u_prev = P[6:8]
        obs_base = 8
        
        # dynamics (unicycle)
        def f(x, u):
            px, py, yaw = x[0], x[1], x[2]
            v, w = u[0], u[1]
            return ca.vertcat(
                px + dt * v * ca.cos(yaw),
                py + dt * v * ca.sin(yaw),
                yaw + dt * w
            )

        # constraints list
        g_constr = []
        lbg = []
        ubg = []

        # initial condition constraint
        g_constr.append(X[:, 0] - x0)
        lbg += [0, 0, 0]
        ubg += [0, 0, 0]

        # objective
        J = 0

        # weights
        Wp = w_goal_pos
        Wy = w_goal_yaw
        Wu = w_u
        Wdu = w_du

        for k in range(N):
            xk = X[:, k]
            uk = U[:, k]

            # dynamics constraint
            x_next = f(xk, uk)
            g_constr.append(X[:, k + 1] - x_next)
            lbg += [0, 0, 0]
            ubg += [0, 0, 0]

            # running cost: position to goal
            pos_err = xk[0:2] - g[0:2]
            yaw_err = xk[2] - g[2]
            # wrap yaw error softly (approx): using sin/cos trick is better, but keep simple now
            J += Wp * ca.dot(pos_err, pos_err) + Wy * (ca.sin(yaw_err) ** 2)

            # control effort
            J += Wu * (uk[0] ** 2 + uk[1] ** 2)

            # control smoothness (Δu)
            if k == 0:
                duk = uk - u_prev
            else:
                duk = uk - U[:, k - 1]
            J += Wdu * (duk[0] ** 2 + duk[1] ** 2)

            # obstacle avoidance: elliptical footprint oriented along the
            # obstacle's velocity vector, with a velocity-scaled lookahead
            # added to the major axis. The lookahead bakes the obstacle's
            # "swept corridor" over the next ~tau_lookahead seconds into the
            # *single-time-step* exclusion zone, so the planner sees a true
            # walking-corridor as forbidden territory rather than a series
            # of disjoint instantaneous circles. This is what biases the
            # planner toward going behind a moving pedestrian instead of
            # cutting in front.
            for i in range(self.M):
                base = obs_base + i * self._n_per_obs
                ox = P[base + 0]
                oy = P[base + 1]
                ovx = P[base + 2]
                ovy = P[base + 3]
                semi_major = P[base + 4]
                semi_minor = P[base + 5]
                near_flag = P[base + 6]
                # P[base + 7] reserved

                # 常速度预测到k步
                oxk = ox + (k * dt) * ovx
                oyk = oy + (k * dt) * ovy

                # Relative position robot -> obstacle (k-th step).
                rx = X[0, k] - oxk
                ry = X[1, k] - oyk

                # Orient the ellipse along the velocity vector. eps_v guards
                # the divisions when |v| ≈ 0; in that case (cos_t, sin_t) is
                # noise but a ≈ b (the publisher sends a degenerate circle
                # when stationary), so the rotation has no observable effect.
                v_norm = ca.sqrt(ovx * ovx + ovy * ovy + 1e-9)
                cos_t = ovx / v_norm
                sin_t = ovy / v_norm

                # Rotate (rx, ry) into the obstacle's local frame (x' along
                # velocity, y' perpendicular).
                rx_local =  rx * cos_t + ry * sin_t
                ry_local = -rx * sin_t + ry * cos_t

                # Effective semi-axes used by the planner:
                #   - A_eff: geometric semi-major + collision margins +
                #            velocity-scaled lookahead (motion-aware
                #            inflation along the direction of travel).
                #   - B_eff: geometric semi-minor + collision margins
                #            (no lookahead — perpendicular dimension stays
                #            geometric).
                A_eff_soft = robot_radius + soft_margin + semi_major + tau_lookahead * v_norm
                B_eff_soft = robot_radius + soft_margin + semi_minor
                A_eff_hard = robot_radius + hard_margin + semi_major + tau_lookahead * v_norm
                B_eff_hard = robot_radius + hard_margin + semi_minor

                # Implicit ellipse value: <1 inside, =1 on boundary, >1 outside.
                e_soft = (rx_local / A_eff_soft) ** 2 + (ry_local / B_eff_soft) ** 2
                e_hard = (rx_local / A_eff_hard) ** 2 + (ry_local / B_eff_hard) ** 2

                # 软代价：所有选中的障碍物始终保留
                intr = ca.fmax(0, 1 - e_soft)
                J += w_obs * (intr * intr)

                # 近场障碍物：加可松弛硬约束 e_hard + s >= 1, s >= 0
                g_constr.append(near_flag * (1 - e_hard - S[i, k]))
                lbg += [-float("inf")]
                ubg += [0.0]
                J += rho_slack * near_flag * (S[i, k] ** 2)

        # terminal cost (stronger goal pull)
        xN = X[:, N]
        pos_err_N = xN[0:2] - g[0:2]
        yaw_err_N = xN[2] - g[2]
        J += 5.0 * Wp * ca.dot(pos_err_N, pos_err_N) + 2.0 * Wy * (ca.sin(yaw_err_N) ** 2)

        # bounds on controls
        lbx = []
        ubx = []

        # X bounds (unbounded)
        for _ in range(3 * (N + 1)):
            lbx.append(-ca.inf)
            ubx.append(ca.inf)

        # U bounds
        for k in range(N):
            # v
            lbx.append(v_min)
            ubx.append(v_max)
            # w
            lbx.append(-w_max)
            ubx.append(w_max)

        # S bounds
        for _ in range(self.M * N):
            lbx.append(0.0)
            ubx.append(ca.inf)

        # pack decision variables as a vector: [vec(X); vec(U); vec(S)]
        opt_vars = ca.vertcat(
            ca.reshape(X, -1, 1),
            ca.reshape(U, -1, 1),
            ca.reshape(S, -1, 1),
        )
        g_all = ca.vertcat(*g_constr)

        nlp = {"x": opt_vars, "f": J, "g": g_all, "p": P}

        opts = {
            "ipopt.print_level": 0,
            "ipopt.max_iter": 100,
            "ipopt.tol": 1e-3,
            "print_time": 0, # detailed solver time
        }

        self.solver = ca.nlpsol("solver", "ipopt", nlp, opts)

        self.lbg = lbg
        self.ubg = ubg
        self.lbx = lbx
        self.ubx = ubx

        self.nx = 3
        self.nu = 2

        self.nX = 3 * (N + 1)
        self.nU = 2 * N
        self.nS = self.M * N

    def solve(self, x0, goal, u_prev, obs_params, x_init=None, u_init=None, s_init=None):
        """
        x0: (3,)
        goal: (3,)  -> [gx,gy,gyaw]
        u_prev: (2,)
        x_init: optional initial guess for X (3,N+1)
        u_init: optional initial guess for U (2,N)
        """
        if len(obs_params) != self.M * self._n_per_obs:
            raise ValueError(
                f"obs_params length must be {self.M * self._n_per_obs}, got {len(obs_params)}")
        
        P = ca.DM(list(x0) + list(goal) + list(u_prev) + list(obs_params))

        # initial guess
        if x_init is None:
            # straight-line guess
            x_init = ca.DM.zeros(3, self.N + 1)
            x_init[:, 0] = ca.DM(x0)
            for k in range(self.N):
                x_init[:, k + 1] = x_init[:, k]
        if u_init is None:
            u_init = ca.DM.zeros(2, self.N)
        if s_init is None:
            s_init = ca.DM.zeros(self.M, self.N)

        x0_guess = ca.vertcat(
            ca.reshape(x_init, -1, 1),
            ca.reshape(u_init, -1, 1),
            ca.reshape(s_init, -1, 1),
        )

        t0 = time.time()
        sol = self.solver(
            x0=x0_guess,
            p=P,
            lbg=self.lbg,
            ubg=self.ubg,
            lbx=self.lbx,
            ubx=self.ubx,
        )
        solve_ms = (time.time() - t0) * 1000.0

        opt = sol["x"]
        X_opt = ca.reshape(opt[0:self.nX], 3, self.N + 1)
        U_opt = ca.reshape(opt[self.nX:self.nX + self.nU], 2, self.N)

        cost = float(sol["f"])
        return True, solve_ms, cost, X_opt, U_opt