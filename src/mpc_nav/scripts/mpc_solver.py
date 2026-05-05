#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import casadi as ca


class CasadiMPCSolver:
    def __init__(self, N, dt,
                 v_min, v_max, w_max,
                 obstacle_num, robot_radius, soft_margin, hard_margin,
                 tau_lookahead, dynamic_v_min, uncertainty_alpha,
                 w_goal_pos, w_goal_yaw, w_u, w_du, w_obs, rho_slack):
        self.N = N
        self.dt = dt
        self.M = obstacle_num
        # number of float slots per obstacle in the parameter vector;
        # see mpc_node._solve_mpc obs_params packing.
        self._n_per_obs = 11

        # decision variables: X(3, N+1), U(2, N), S(M, N)
        X = ca.SX.sym("X", 3, N + 1)   # [x,y,yaw]
        U = ca.SX.sym("U", 2, N)       # [v,w]
        S = ca.SX.sym("S", self.M, N)  # slack for near obstacles

        # parameters: x0(3), goal(3), u_prev(2), per-obstacle 11 floats:
        #   [ox, oy, ovx, ovy, semi_major, semi_minor, near_flag,
        #    sigma2_xx, sigma2_yy, sigma2_vxvx, sigma2_vyvy]
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

            # obstacle avoidance: forward swept capsule.
            #
            # Each obstacle is modelled as a line segment from its current
            # predicted position p_o(t_k) to p_o(t_k) + tau_lookahead * v_o,
            # inflated by the geometric footprint and safety margins. This
            # approximates the swept volume the obstacle traces over the
            # next ~tau seconds as a true capsule (stadium-shaped exclusion
            # zone), not a symmetrically inflated ellipse around the current
            # centroid. The asymmetry — back side gets *no* inflation,
            # front side gets the full lookahead — is what makes "go behind"
            # a strictly cheaper option than "cut in front" for a moving
            # obstacle.
            #
            # We additionally inflate by alpha * KF sigma to fold tracking
            # uncertainty into the exclusion zone:
            #   - sigma_par(k):  along the line of motion (extends segment)
            #   - sigma_perp(k): perpendicular        (grows capsule radius)
            for i in range(self.M):
                base = obs_base + i * self._n_per_obs
                ox = P[base + 0]
                oy = P[base + 1]
                ovx = P[base + 2]
                ovy = P[base + 3]
                semi_major = P[base + 4]
                semi_minor = P[base + 5]
                near_flag = P[base + 6]
                s2_xx = P[base + 7]
                s2_yy = P[base + 8]
                s2_vxvx = P[base + 9]
                s2_vyvy = P[base + 10]

                # 常速度预测到k步 (matches what mpc_node currently provides;
                # could be replaced by tracker-side predicted_positions[k]
                # in a future iteration).
                oxk = ox + (k * dt) * ovx
                oyk = oy + (k * dt) * ovy

                # Speed and unit motion direction. The +1e-9 guards the
                # division when |v| ≈ 0; the smooth gate below kills the
                # corridor in that regime so direction noise is harmless.
                v_norm = ca.sqrt(ovx * ovx + ovy * ovy + 1e-9)
                e_par_x = ovx / v_norm
                e_par_y = ovy / v_norm
                e_perp_x = -e_par_y
                e_perp_y =  e_par_x

                # Smoothly fade the swept-capsule extension between
                # static (gate ~ 0) and dynamic (gate ~ 1). The factor 20
                # makes the transition band ~0.05 m/s wide.
                gate = 0.5 * (1.0 + ca.tanh(20.0 * (v_norm - dynamic_v_min)))

                # Project current-step KF covariance onto motion-aligned
                # axes (sigma_xy ≈ 0 in this CV-KF — see TrackerDetail.msg).
                # Then propagate one step ahead with closed-form CV-KF:
                #   sigma_par(k)^2  = sigma_par_pos^2 + (k*dt)^2 * sigma_par_vel^2
                t_k = k * dt
                sigma_pos_par2  = (e_par_x  ** 2) * s2_xx     + (e_par_y  ** 2) * s2_yy
                sigma_pos_perp2 = (e_perp_x ** 2) * s2_xx     + (e_perp_y ** 2) * s2_yy
                sigma_vel_par2  = (e_par_x  ** 2) * s2_vxvx   + (e_par_y  ** 2) * s2_vyvy
                sigma_vel_perp2 = (e_perp_x ** 2) * s2_vxvx   + (e_perp_y ** 2) * s2_vyvy
                sigma_par_k  = ca.sqrt(sigma_pos_par2  + (t_k ** 2) * sigma_vel_par2  + 1e-12)
                sigma_perp_k = ca.sqrt(sigma_pos_perp2 + (t_k ** 2) * sigma_vel_perp2 + 1e-12)

                # Capsule endpoints in world frame:
                #   start = p_o(t_k)              (current predicted position)
                #   end   = start + (tau*|v| + alpha*sigma_par) * e_par
                # Length is gated by `gate` so static obstacles collapse to
                # a single point (start == end) and reduce to circle below.
                seg_len = (tau_lookahead * v_norm + uncertainty_alpha * sigma_par_k) * gate
                seg_dx = seg_len * e_par_x
                seg_dy = seg_len * e_par_y
                end_x = oxk + seg_dx
                end_y = oyk + seg_dy

                # Capsule radius. C1 isotropy: take the larger of the
                # geometric semi-axes (conservative). Add robot radius and
                # safety margins (soft / hard versions used below).
                geom_r = ca.fmax(semi_major, semi_minor)
                r_soft = robot_radius + soft_margin + geom_r + uncertainty_alpha * sigma_perp_k
                r_hard = robot_radius + hard_margin + geom_r + uncertainty_alpha * sigma_perp_k

                # Point (robot at step k) to segment (capsule axis) distance.
                # Standard formula: project rel onto seg, clamp to [0,1],
                # then take || rel - clamp_t * seg ||. fmin/fmax for the
                # clamp — IPOPT handles these subgradient functions fine.
                rel_x = X[0, k] - oxk
                rel_y = X[1, k] - oyk
                seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy + 1e-12  # >0 always
                t_proj = (rel_x * seg_dx + rel_y * seg_dy) / seg_len2
                t_clamped = ca.fmin(1.0, ca.fmax(0.0, t_proj))
                near_x = oxk + t_clamped * seg_dx
                near_y = oyk + t_clamped * seg_dy
                d2 = (X[0, k] - near_x) ** 2 + (X[1, k] - near_y) ** 2

                # Soft cost: penalize intrusion into the soft capsule.
                # We use squared margin like the previous ellipse formulation,
                # so the cost stays smooth at the boundary.
                soft2 = r_soft * r_soft
                hard2 = r_hard * r_hard
                intr_soft = ca.fmax(0.0, soft2 - d2)
                J += w_obs * (intr_soft * intr_soft)

                # Hard (relaxed) constraint near obstacles: d^2 + s >= r_hard^2.
                g_constr.append(near_flag * (hard2 - d2 - S[i, k]))
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