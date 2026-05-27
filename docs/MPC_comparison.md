# MPC Comparison: mpc_nav vs mpc_planner (T-MPC)

## At a Glance

| Dimension | mpc_nav (Python) | mpc_planner (C++) |
|-----------|------------------|-------------------|
| Implementation | Python 3 + CasADi | C++ + Acados/ForcesPro |
| Solver | IPOPT (interior-point NLP) | SQP-RTI (real-time iteration) |
| Trajectories | Single | Multiple parallel (T-MPC++) |
| Max obstacles | 5 fixed slots | 12 configurable |
| Obstacle model | Swept capsule + KF uncertainty | Ellipsoid, GMM mixture, Scenario-based |
| Global planner | None (point goal) | guidance_planner (Visibility-PRM) |
| Horizon | N=45, dt=0.1s (4.5s) | N=30, dt=0.2s (6.0s) |
| Max linear velocity | 0.6 m/s | 0.6 m/s (configurable) |
| Safety margin | 0.05m | configurable per scenario |
| DOA bridge topic | `/doa_obstacles` | `/doa_obstacles_tmpc` |
| Message type | `mpc_nav/ObstacleArray` | `mpc_planner_msgs/ObstacleArray` w/ GMM |
| Launch file | `mpc.launch` | `tmpc_doa.launch` |

## Key Architectural Differences

### 1. Solver Approach

**mpc_nav:** Formulates a single NLP with CasADi symbolic differentiation, solved by IPOPT at each timestep. Warm-starts from previous solution. Straightforward but relatively slow for large N or many obstacles.

**mpc_planner:** Uses Acados SQP-RTI which linearizes the problem and performs one SQP iteration per timestep — much faster, suitable for real-time control with longer horizons.

### 2. Obstacle Modeling

**mpc_nav:** Swept-capsule collision model (robot footprint swept along trajectory). Obstacle uncertainty handled via Kalman filter covariance, expanded into safety radius. Fixed 5-slot obstacle array.

**mpc_planner:** Three modes:
- **Ellipsoid:** Simple quadratic constraint
- **GMM:** Gaussian mixture model predictions from obstacle velocity uncertainty
- **Scenario-based:** Multiple discrete future trajectories per obstacle

### 3. Trajectory Optimization

**mpc_nav:** Single trajectory optimized against goal cost + obstacle penalties.

**mpc_planner:** T-MPC++ maintains multiple trajectory hypotheses in parallel, each converging toward a distinct homotopy class (left/right/brake). Chooses the best feasible one after optimization.

### 4. Global Guidance

**mpc_nav:** No global planner. Goal is passed as a point; solver finds direct path. Can get stuck in U-shaped obstacle configurations.

**mpc_planner:** `guidance_planner` provides topology-distinct reference paths via Visibility-PRM. Each trajectory hypothesis follows a different topological route, reducing local minima problems.

## When to Use Which

| Scenario | Recommended Pipeline |
|----------|---------------------|
| Simple crossing pedestrians (1-3 obstacles) | Both work well |
| Dense obstacles (5+) | mpc_planner (more slots) |
| Real-time / low-latency requirement | mpc_planner (SQP-RTI) |
| Debugging / visualization | mpc_nav (Python, easier to hack) |
| Publication-quality results | mpc_planner (parallel T-MPC++) |
