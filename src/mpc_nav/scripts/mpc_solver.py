#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import casadi as ca


class CasadiMPCSolver:
    def __init__(self, N, dt,
                 v_min, v_max, w_max,
                 obstacle_num, robot_radius, soft_margin, hard_margin,
                 w_goal_pos, w_goal_yaw, w_u, w_du, w_obs, rho_slack):
        self.N = N
        self.dt = dt
        self.M = obstacle_num

        # decision variables: X(3, N+1), U(2, N), S(M, N)
        X = ca.SX.sym("X", 3, N + 1)   # [x,y,yaw]
        U = ca.SX.sym("U", 2, N)       # [v,w]
        S = ca.SX.sym("S", self.M, N)  # slack for near obstacles

        # parameters: x0(3), goal(3), u_prev(2), obs(x,y,vx,vy,r,near_flag)
        P = ca.SX.sym("P", 3 + 3 + 2 + self.M*6)

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

            # obstacle avoidance: all selected obstacles keep soft penalty,
            # near obstacles add relaxed hard constraints with slack.
            for i in range(self.M):
                ox = P[obs_base + i*6 + 0]
                oy = P[obs_base + i*6 + 1]
                ovx = P[obs_base + i*6 + 2]
                ovy = P[obs_base + i*6 + 3]
                orad = P[obs_base + i*6 + 4]
                near_flag = P[obs_base + i*6 + 5]

                # 常速度预测到k步
                oxk = ox + (k * dt) * ovx
                oyk = oy + (k * dt) * ovy

                dx = X[0, k] - oxk
                dy = X[1, k] - oyk

                ds2 = dx*dx + dy*dy
                soft2 = (robot_radius + soft_margin + orad)**2
                hard2 = (robot_radius + hard_margin + orad)**2

                # 软代价：所有选中的障碍物始终保留
                intr = ca.fmax(0, soft2 - ds2)
                J += w_obs * (intr * intr)

                # 近场障碍物：加可松弛硬约束 d^2 + s >= r_hard^2, s >= 0
                g_constr.append(near_flag * (hard2 - ds2 - S[i, k]))
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
        if len(obs_params) != self.M * 6:
            raise ValueError(f"obs_params length must be {self.M*6}, got {len(obs_params)}")
        
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