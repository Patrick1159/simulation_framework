# DOA → T-MPC Bridge Redesign

**Date:** 2026-06-23
**Author:** Patrick (with Claude)
**Status:** Design — pending review
**File touched:** `src/mpc_nav/scripts/doa_to_tmpc.py` (+ new unit test)

---

## 1. Problem

The current bridge (`doa_to_tmpc.py`) converts DOA tracker output into the
T-MPC `ObstacleArray` input, but its prediction model is wrong in two ways that
crush the T-MPC solve rate:

### 1.1 Long deterministic constant-velocity extrapolation
For each tracker the bridge extrapolates the (noisy, frame-differenced) DOA
velocity over the **full horizon** `N=30, dt=0.2 → 6.0 s`:

```
px_k = x + cvx*(dt*k),  k = 1 .. N-1     # doa_to_tmpc.py:114
```

The DOA velocity is a bbox-centroid frame difference: noisy in magnitude,
jittery in direction. Multiplying it by up to 6 s throws the horizon-end
prediction `6·v` metres away (≈6 m at 1 m/s) in a direction that changes every
20 Hz replan. These long arcs are fed to `guidance_planner` (Visibility-PRM),
which carves long space-time tunnels; with several obstacles the PRM cannot
find stable topology-distinct homotopy classes, so the parallel solvers churn
and frequently report infeasible. In RViz this reads as the "swept corridor".

### 1.2 Covariance misused as risk, AND double-integrated
- The KF diagonal exported by DOA (`detailed_matcher.cpp:401-404`) is
  **tracking precision** ("how well do I know the current state"), not motion
  risk ("how far might it deviate from constant velocity"). `TrackerDetail.msg`
  documents it as precision.
- The bridge already integrates it over time into a **cumulative** sigma:
  `σ(k) = sqrt(σ_xx + (k·dt)²·σ_vxvx)` (`doa_to_tmpc.py:126-129`) and writes
  that into `major/minor_semiaxis[k]`.
- With `probabilistic.enable:false` (current `settings.yaml:71`) the consumer
  **discards** this entirely: `obstacleCallback` forces the prediction to
  `DETERMINISTIC` (`ros1_jackalsimulator.cpp:291`), and
  `propagatePredictionUncertainty` early-returns for non-GAUSSIAN
  (`data_preparation.cpp:172`). So today the whole sigma block is dead code.
- If `probabilistic.enable:true` is set, it gets **worse**:
  `propagatePredictionUncertainty` (`data_preparation.cpp:179-185`) treats the
  incoming `major_semiaxis[k]` as a **per-step rate** and integrates it a
  *second* time cumulatively. The keep-out ellipses explode → guaranteed
  infeasible. This is why "turning on probabilistic has the same problem".

### 1.3 The planner's actual contract
`getConstantVelocityPrediction` (`data_preparation.cpp:59-79`) is the planner's
canonical obstacle builder. When `probabilistic.enable` is true it writes a
**constant** per-step `noise = 0.3` into every `PredictionStep`, then calls
`propagatePredictionUncertainty` once. The planner therefore expects an
uncertainty **growth rate per step**, not a pre-integrated cumulative sigma.
Any source feeding the planner must match this contract.

---

## 2. Goals / Non-goals

**Goals**
- Stop producing long 6 s deterministic tunnels.
- Hand uncertainty to the planner in the exact form it expects, with a single
  clear owner of time-integration (no double integration).
- Make the conversion logic a pure, unit-testable function.
- Keep the ROS node thin: subscribe, convert, publish, watchdog, markers.

**Non-goals**
- No change to DOA perception, the KF, or the message definitions.
- No change to the T-MPC C++ side or `settings.yaml` (the design works with
  `probabilistic.enable` either off or on, correctly, without edits there).
- No new motion models (no CTRV / IMM). Pure CV + static hold only.

---

## 3. Design

### 3.1 Prediction geometry — short CV + static hold
Replace full-horizon extrapolation with a **bounded** constant-velocity arc that
freezes:

- Parameter `cv_horizon` (seconds, default `1.0`). Let
  `k_cv = round(cv_horizon / dt)` (=5 at dt=0.2).
- For `k = 1 .. N-1`:
  - if `k <= k_cv`: `p_k = p0 + v_clamped · (dt · k)`
  - else (hold): `p_k = p0 + v_clamped · (dt · k_cv)`  (frozen at the
    end-of-CV point)
- Still emit all `N-1` poses (consumer indexes `mode[k-1]` for every stage
  k=1..N-1; short arrays would read out of bounds). The hold region is what
  removes the tunnel, not a shorter array.

**Velocity deadband:** parameter `v_min` (default `0.1` m/s). If
`|v| < v_min`, treat the obstacle as fully static: all `p_k = p0`, orientation
identity. (Current code only freezes orientation, still extrapolates position —
that is removed.)

**Velocity clamp:** retain existing `velocity_limit` clamp on magnitude.

**Orientation:** from `atan2(vy, vx)` when moving, identity when static
(unchanged), applied to every emitted pose.

### 3.2 Uncertainty — single owner, two exclusive modes
One bridge parameter `emit_uncertainty` (bool, default `false`) selects the
mode. The two modes are mutually exclusive and each is internally consistent
with the planner.

**Mode A — deterministic (default, pairs with `probabilistic.enable:false`):**
- `major_semiaxis[k] = minor_semiaxis[k] = 0.0` for all k.
- Consumer marks the prediction DETERMINISTIC; no sigma math anywhere.
- The bridge performs **zero** covariance arithmetic in this mode.

**Mode B — probabilistic rate (pairs with `probabilistic.enable:true`):**
- Emit a **constant per-step rate** for every step, matching the planner's
  `noise` convention:
  - `rate = clamp( sqrt(max(sigma2_vxvx, sigma2_vyvy)), rate_floor, rate_ceil )`
  - `major_semiaxis[k] = minor_semiaxis[k] = rate`  (same constant for all k)
- The bridge does **not** integrate over time. `propagatePredictionUncertainty`
  integrates once on the consumer side → correct cumulative growth, no double
  count.
- Params: `rate_floor` (default `0.05`), `rate_ceil` (default `0.5`).
- Note: this requires `probabilistic.enable:true` in settings.yaml to take
  effect; with it false, Mode B output is harmless (consumer ignores it). We do
  not edit settings.yaml in this change; the operator flips both together.

The closed-form `sqrt(σ_xx + (k·dt)²·σ_vxvx)` block is **deleted**.

### 3.3 Structure — pure function + thin shell
- New pure function (ROS-free, no rospy calls):
  ```python
  def tracker_to_obstacle(tracker, cfg) -> ObstacleGMM
  ```
  where `cfg` is a small dataclass/namedtuple of resolved params
  (`N, dt, k_cv, v_min, velocity_limit, emit_uncertainty, rate_floor,
  rate_ceil, target_frame, stamp`). Builds pose, orientation, the GMM mean
  Path, and the semiaxis arrays. Returns `None` if the tracker should be
  dropped (age filter is applied by the caller, kept simple).
- The ROS `callback` becomes: stamp bookkeeping → for each tracker apply age
  filter, call `tracker_to_obstacle`, append → publish → build markers.
- Watchdog (`_watchdog_callback`) unchanged — still needed so a silent DOA
  yields an empty array that `ensureObstacleSize` pads to `max_obstacles`,
  keeping `isDataReady` true.
- Markers: keep behind the existing `publish_markers` flag; the arrow should
  reflect the (clamped) velocity, the sphere the current position. Minor: marker
  building may stay in the node (it is presentation, not conversion).

### 3.4 Parameters (all `~private`, with defaults)
| param | default | meaning |
|-------|---------|---------|
| `N` | 30 | horizon steps (match settings.yaml) |
| `dt` | 0.2 | step (match settings.yaml integrator_step) |
| `cv_horizon` | 1.0 | seconds of real CV before freezing |
| `v_min` | 0.1 | deadband; below this obstacle is static |
| `velocity_limit` | 5.0 | magnitude clamp (retained) |
| `max_age` | 0.2 | drop stale trackers (retained) |
| `emit_uncertainty` | false | Mode A vs Mode B |
| `rate_floor` | 0.05 | min per-step uncertainty rate (Mode B) |
| `rate_ceil` | 0.5 | max per-step uncertainty rate (Mode B) |
| `target_frame`, `*_topic`, watchdog params | as today | unchanged |

---

## 4. Data flow (after)

```
/rematch/tracker_details (TrackerDetailArray)
  → callback: age filter
  → tracker_to_obstacle() [pure]
       pose+orientation (static if |v|<v_min)
       mean Path: CV for k<=k_cv, frozen after
       semiaxis: 0 (Mode A) | constant rate (Mode B)
  → ObstacleArray (single-mode GMM, prob=[1.0])
  → /doa_obstacles_tmpc
  → ros1_jackalsimulator obstacleCallback → Ellipsoid/Guidance constraints
```

---

## 5. Testing

Pure function enables a roscore-free unit test (`pytest`), placed alongside the
script or under `mpc_nav/test/`. Cases:

1. **Moving tracker, CV region:** `|v|>v_min`, check `p_1 .. p_{k_cv}` lie on
   the CV line and spacing = `v·dt`.
2. **Hold region:** `p_k` for `k>k_cv` all equal `p0 + v·dt·k_cv` (frozen).
3. **Array length:** exactly `N-1` poses and `N-1` semiaxis entries.
4. **Deadband:** `|v|<v_min` → every `p_k == p0`, orientation identity.
5. **Velocity clamp:** `|v|>velocity_limit` → arc uses clamped magnitude.
6. **Mode A:** `emit_uncertainty=false` → all semiaxis == 0.
7. **Mode B:** `emit_uncertainty=true` → all semiaxis == constant `rate` within
   `[rate_floor, rate_ceil]`, and **not** time-growing (asserts the
   no-double-integration contract).

Manual/integration check (cannot be unit-tested here, stated explicitly): run
`roslaunch mpc_nav tmpc_doa.launch`, confirm solve rate via
`exp/scripts/parse_tmpc_log.py`, and visually confirm obstacles are compact
blobs not long corridors in RViz.

---

## 6. Risks / open points
- `cv_horizon=1.0 s` is a starting value; may need tuning per scene. It is a
  param, so no code change to retune.
- Freezing velocity may under-predict fast crossing pedestrians; mitigated by
  T-MPC's 20 Hz replan and (optionally) Mode B uncertainty. If insufficient,
  a follow-up could raise `cv_horizon` or add a reachable-set inflation — out
  of scope here.
- Mode B only matters if `probabilistic.enable:true`; the operator must flip
  both. Documented, not auto-coupled, to keep this change to one file.
```
