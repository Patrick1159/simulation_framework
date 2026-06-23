# DOA → T-MPC Bridge Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite `doa_to_tmpc.py` so obstacle predictions are short constant-velocity arcs that freeze in place, with uncertainty handed to the planner as a per-step rate (never pre-integrated), fixing the T-MPC solve-rate collapse.

**Architecture:** Extract a pure, ROS-free `tracker_to_obstacle(tracker, cfg)` function that builds one `ObstacleGMM` from one `TrackerDetail`. A thin ROS node calls it per tracker. Two mutually exclusive uncertainty modes: deterministic (semiaxis=0, default) and probabilistic-rate (constant per-step rate, integrated once by the planner's `propagatePredictionUncertainty`).

**Tech Stack:** Python 3 (ROS Noetic), rospy, `umap.msg`, `mpc_planner_msgs.msg`, pytest.

## Global Constraints

- Only one production file changes: `src/mpc_nav/scripts/doa_to_tmpc.py`. Plus a new test file.
- Emit exactly `N-1` mean poses and `N-1` semiaxis entries per obstacle (consumer indexes `mode[k-1]` for k=1..N-1; short arrays read out of bounds).
- The bridge NEVER integrates uncertainty over time. Time-integration is the planner's job (`data_preparation.cpp:179-185`).
- Modes are mutually exclusive: deterministic ⇒ all semiaxis = 0.0; probabilistic ⇒ constant per-step `rate` for all steps.
- No changes to DOA, KF, message definitions, T-MPC C++, or `settings.yaml`.
- Defaults: `cv_horizon=1.0`, `v_min=0.1`, `velocity_limit=5.0`, `max_age=0.2`, `emit_uncertainty=false`, `rate_floor=0.05`, `rate_ceil=0.5`, `N=30`, `dt=0.2`.
- Pure function makes NO rospy calls (no `rospy.Time.now()`, no logging). It receives a stamp via `cfg`.
- Tests run in a sourced workspace WITHOUT roscore: `source devel/setup.zsh` then `pytest`. Use `rospy.Time(0)` for stamps (no master needed). Do NOT run tests inside the `fastsam` conda env unless it has the workspace messages on PYTHONPATH; use the plain sourced shell.

---

### Task 1: Pure conversion — config + deterministic short-CV/static-hold geometry

**Files:**
- Modify: `src/mpc_nav/scripts/doa_to_tmpc.py` (add `BridgeConfig` dataclass + `tracker_to_obstacle` function at module level, above the class)
- Test: `src/mpc_nav/test/test_doa_to_tmpc.py` (create)

**Interfaces:**
- Produces:
  - `BridgeConfig` dataclass with fields: `N:int, dt:float, k_cv:int, v_min:float, velocity_limit:float, emit_uncertainty:bool, rate_floor:float, rate_ceil:float, target_frame:str, stamp` (a `rospy.Time`).
  - `tracker_to_obstacle(tracker, cfg: BridgeConfig) -> ObstacleGMM` — builds pose, orientation, GMM mean Path (`N-1` poses), and semiaxis arrays (`N-1` each). Pure, no rospy calls beyond constructing message objects.
- Consumes (from `umap.msg.TrackerDetail`): `tracker_id, position(.x/.y/.z), velocity(.x/.y), sigma2_vxvx, sigma2_vyvy`.

- [ ] **Step 1: Write the failing tests for geometry**

Create `src/mpc_nav/test/test_doa_to_tmpc.py`:

```python
import os
import sys
import math
import rospy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import doa_to_tmpc as bridge  # noqa: E402
from umap.msg import TrackerDetail  # noqa: E402


def make_cfg(**kw):
    defaults = dict(
        N=30, dt=0.2, k_cv=5, v_min=0.1, velocity_limit=5.0,
        emit_uncertainty=False, rate_floor=0.05, rate_ceil=0.5,
        target_frame="odom", stamp=rospy.Time(0),
    )
    defaults.update(kw)
    return bridge.BridgeConfig(**defaults)


def make_tracker(px=0.0, py=0.0, vx=1.0, vy=0.0,
                 s_vxvx=0.0, s_vyvy=0.0, tid=7):
    t = TrackerDetail()
    t.tracker_id = tid
    t.position.x, t.position.y, t.position.z = px, py, 0.0
    t.velocity.x, t.velocity.y = vx, vy
    t.sigma2_vxvx, t.sigma2_vyvy = s_vxvx, s_vyvy
    return t


def poses(obs):
    return obs.gaussians[0].mean.poses


def test_array_lengths():
    cfg = make_cfg()
    obs = bridge.tracker_to_obstacle(make_tracker(), cfg)
    assert len(poses(obs)) == cfg.N - 1
    assert len(obs.gaussians[0].major_semiaxis) == cfg.N - 1
    assert len(obs.gaussians[0].minor_semiaxis) == cfg.N - 1
    assert obs.id == 7
    assert obs.probabilities == [1.0]


def test_cv_region_on_line():
    cfg = make_cfg()
    obs = bridge.tracker_to_obstacle(make_tracker(vx=1.0, vy=0.0), cfg)
    ps = poses(obs)
    # k = 1 .. k_cv follow p0 + v*dt*k  (index k-1)
    for k in range(1, cfg.k_cv + 1):
        assert math.isclose(ps[k - 1].pose.position.x, 1.0 * cfg.dt * k, abs_tol=1e-6)
        assert math.isclose(ps[k - 1].pose.position.y, 0.0, abs_tol=1e-6)


def test_hold_region_frozen():
    cfg = make_cfg()
    obs = bridge.tracker_to_obstacle(make_tracker(vx=1.0, vy=0.0), cfg)
    ps = poses(obs)
    frozen_x = 1.0 * cfg.dt * cfg.k_cv
    for k in range(cfg.k_cv + 1, cfg.N):  # k_cv+1 .. N-1
        assert math.isclose(ps[k - 1].pose.position.x, frozen_x, abs_tol=1e-6)


def test_deadband_static():
    cfg = make_cfg()
    obs = bridge.tracker_to_obstacle(make_tracker(vx=0.05, vy=0.0), cfg)
    ps = poses(obs)
    for p in ps:
        assert math.isclose(p.pose.position.x, 0.0, abs_tol=1e-6)
        assert math.isclose(p.pose.position.y, 0.0, abs_tol=1e-6)
    o = obs.pose.orientation
    assert (o.x, o.y, o.z, o.w) == (0.0, 0.0, 0.0, 1.0)


def test_velocity_clamp():
    cfg = make_cfg(velocity_limit=2.0)
    obs = bridge.tracker_to_obstacle(make_tracker(vx=10.0, vy=0.0), cfg)
    ps = poses(obs)
    # first CV step uses clamped speed 2.0
    assert math.isclose(ps[0].pose.position.x, 2.0 * cfg.dt * 1, abs_tol=1e-6)


def test_deterministic_zero_semiaxis():
    cfg = make_cfg(emit_uncertainty=False)
    obs = bridge.tracker_to_obstacle(make_tracker(s_vxvx=0.9, s_vyvy=0.9), cfg)
    g = obs.gaussians[0]
    assert all(v == 0.0 for v in g.major_semiaxis)
    assert all(v == 0.0 for v in g.minor_semiaxis)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/patrick/sim_dev/simulation_framework && source devel/setup.zsh && pytest src/mpc_nav/test/test_doa_to_tmpc.py -v`
Expected: FAIL — `AttributeError: module 'doa_to_tmpc' has no attribute 'BridgeConfig'` (or import error on `tracker_to_obstacle`).

- [ ] **Step 3: Add the dataclass and pure function**

In `src/mpc_nav/scripts/doa_to_tmpc.py`, add `from dataclasses import dataclass` to imports, and insert the following at module level after the imports and before `class DOAToMPCAdapter`:

```python
@dataclass
class BridgeConfig:
    N: int
    dt: float
    k_cv: int
    v_min: float
    velocity_limit: float
    emit_uncertainty: bool
    rate_floor: float
    rate_ceil: float
    target_frame: str
    stamp: object  # rospy.Time, passed in (keeps this function ROS-call-free)


def _uncertainty_rate(tracker, cfg):
    """Constant per-step uncertainty rate (m per step-rate units).
    Returns 0.0 in deterministic mode. Never integrated here — the planner
    integrates once via propagatePredictionUncertainty."""
    if not cfg.emit_uncertainty:
        return 0.0
    var = max(tracker.sigma2_vxvx, tracker.sigma2_vyvy)
    rate = math.sqrt(var) if var > 0.0 else 0.0
    return min(max(rate, cfg.rate_floor), cfg.rate_ceil)


def tracker_to_obstacle(tracker, cfg):
    """Convert one umap TrackerDetail into one mpc_planner_msgs ObstacleGMM.
    Geometry: constant velocity for k<=k_cv, frozen afterward. Pure (no rospy
    calls other than message construction; stamp comes from cfg)."""
    obs = ObstacleGMM()
    obs.id = tracker.tracker_id
    obs.pose.position = tracker.position

    vx = tracker.velocity.x
    vy = tracker.velocity.y
    v_mag = math.sqrt(vx * vx + vy * vy)

    if v_mag < cfg.v_min:
        # Static: identity orientation, zero motion.
        obs.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        cvx = cvy = 0.0
    else:
        yaw = math.atan2(vy, vx)
        q = tft.quaternion_from_euler(0, 0, yaw)
        obs.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        if v_mag > cfg.velocity_limit:
            scale = cfg.velocity_limit / v_mag
            cvx, cvy = vx * scale, vy * scale
        else:
            cvx, cvy = vx, vy

    rate = _uncertainty_rate(tracker, cfg)

    gaussian = Gaussian()
    gaussian.mean.header.stamp = cfg.stamp
    gaussian.mean.header.frame_id = cfg.target_frame

    for k in range(1, cfg.N):
        k_eff = min(k, cfg.k_cv)  # freeze after k_cv
        dt_eff = cfg.dt * k_eff
        px_k = tracker.position.x + cvx * dt_eff
        py_k = tracker.position.y + cvy * dt_eff

        ps = PoseStamped()
        ps.header.stamp = cfg.stamp
        ps.header.frame_id = cfg.target_frame
        ps.pose.position = Point(x=px_k, y=py_k, z=tracker.position.z)
        ps.pose.orientation = obs.pose.orientation
        gaussian.mean.poses.append(ps)

        gaussian.major_semiaxis.append(rate)
        gaussian.minor_semiaxis.append(rate)

    obs.gaussians.append(gaussian)
    obs.probabilities = [1.0]
    return obs
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/patrick/sim_dev/simulation_framework && source devel/setup.zsh && pytest src/mpc_nav/test/test_doa_to_tmpc.py -v`
Expected: PASS (6 tests).

- [ ] **Step 5: Commit**

```bash
git add src/mpc_nav/scripts/doa_to_tmpc.py src/mpc_nav/test/test_doa_to_tmpc.py
git commit -m "feat(bridge): pure tracker_to_obstacle with short-CV + static hold"
```

---

### Task 2: Probabilistic-rate mode (Mode B) — constant per-step rate, not time-growing

**Files:**
- Test: `src/mpc_nav/test/test_doa_to_tmpc.py` (append cases)
- Modify: none expected — `_uncertainty_rate` from Task 1 already implements this. This task verifies the contract and fixes only if a test fails.

**Interfaces:**
- Consumes: `tracker_to_obstacle`, `_uncertainty_rate`, `BridgeConfig` from Task 1.

- [ ] **Step 1: Write the failing/contract tests**

Append to `src/mpc_nav/test/test_doa_to_tmpc.py`:

```python
def test_mode_b_constant_rate_not_growing():
    cfg = make_cfg(emit_uncertainty=True, rate_floor=0.05, rate_ceil=0.5)
    obs = bridge.tracker_to_obstacle(make_tracker(s_vxvx=0.04, s_vyvy=0.01), cfg)
    g = obs.gaussians[0]
    expected = math.sqrt(0.04)  # = 0.2, within [floor, ceil]
    assert all(math.isclose(v, expected, abs_tol=1e-6) for v in g.major_semiaxis)
    assert all(math.isclose(v, expected, abs_tol=1e-6) for v in g.minor_semiaxis)
    # Critical: constant across k (NOT integrated/growing in the bridge)
    assert math.isclose(g.major_semiaxis[0], g.major_semiaxis[-1], abs_tol=1e-9)


def test_mode_b_floor_and_ceil():
    cfg_floor = make_cfg(emit_uncertainty=True, rate_floor=0.1, rate_ceil=0.5)
    obs = bridge.tracker_to_obstacle(make_tracker(s_vxvx=0.0, s_vyvy=0.0), cfg_floor)
    assert all(math.isclose(v, 0.1, abs_tol=1e-6)
               for v in obs.gaussians[0].major_semiaxis)

    cfg_ceil = make_cfg(emit_uncertainty=True, rate_floor=0.05, rate_ceil=0.3)
    obs2 = bridge.tracker_to_obstacle(make_tracker(s_vxvx=100.0, s_vyvy=100.0), cfg_ceil)
    assert all(math.isclose(v, 0.3, abs_tol=1e-6)
               for v in obs2.gaussians[0].major_semiaxis)
```

- [ ] **Step 2: Run tests**

Run: `cd /home/patrick/sim_dev/simulation_framework && source devel/setup.zsh && pytest src/mpc_nav/test/test_doa_to_tmpc.py -v`
Expected: PASS. If FAIL, fix `_uncertainty_rate` to match the clamp/sqrt contract above, then re-run.

- [ ] **Step 3: Commit**

```bash
git add src/mpc_nav/test/test_doa_to_tmpc.py src/mpc_nav/scripts/doa_to_tmpc.py
git commit -m "test(bridge): lock per-step (non-integrated) uncertainty contract"
```

---

### Task 3: Rewire ROS node to use the pure function; delete old extrapolation + sigma block

**Files:**
- Modify: `src/mpc_nav/scripts/doa_to_tmpc.py` — `__init__` (params), `callback` (delegate to pure fn), keep `_watchdog_callback` unchanged.

**Interfaces:**
- Consumes: `tracker_to_obstacle`, `BridgeConfig` from Task 1.

- [ ] **Step 1: Add new params in `__init__`**

In `DOAToMPCAdapter.__init__`, after the existing `self.dt` line, add:

```python
        self.cv_horizon = rospy.get_param('~cv_horizon', 1.0)
        self.v_min = rospy.get_param('~v_min', 0.1)
        self.emit_uncertainty = rospy.get_param('~emit_uncertainty', False)
        self.rate_floor = rospy.get_param('~rate_floor', 0.05)
        self.rate_ceil = rospy.get_param('~rate_ceil', 0.5)
        self.k_cv = max(1, int(round(self.cv_horizon / self.dt)))
        assert self.cv_horizon > 0.0, "cv_horizon must be positive"
        assert self.v_min >= 0.0, "v_min must be non-negative"
```

- [ ] **Step 2: Replace the per-tracker body in `callback`**

In `callback`, replace the entire `for idx, tracker in enumerate(msg.trackers):` loop body (current lines ~70–190, everything from `# 1. Skip trackers older than max_age` through the marker `ARROW` append, up to but not including `self.pub.publish(out_msg)`) with:

```python
        cfg = BridgeConfig(
            N=self.N, dt=self.dt, k_cv=self.k_cv, v_min=self.v_min,
            velocity_limit=self.velocity_limit,
            emit_uncertainty=self.emit_uncertainty,
            rate_floor=self.rate_floor, rate_ceil=self.rate_ceil,
            target_frame=self.target_frame, stamp=msg.header.stamp,
        )

        for idx, tracker in enumerate(msg.trackers):
            if tracker.age > self.max_age:
                continue

            obs = tracker_to_obstacle(tracker, cfg)
            out_msg.obstacles.append(obs)

            if self.publish_markers:
                self._append_markers(marker_array, idx, tracker, obs)
```

- [ ] **Step 3: Add the `_append_markers` helper**

Add this method to `DOAToMPCAdapter` (moves the existing marker code verbatim into a helper; recompute clamped velocity locally for the arrow):

```python
    def _append_markers(self, marker_array, idx, tracker, obs):
        sphere = Marker()
        sphere.header.frame_id = self.target_frame
        sphere.header.stamp = obs.gaussians[0].mean.header.stamp
        sphere.ns = "doa_tmpc"
        sphere.id = idx * 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = obs.pose
        sphere.scale.x = max(float(tracker.box_dimensions.x), 0.1)
        sphere.scale.y = max(float(tracker.box_dimensions.y), 0.1)
        sphere.scale.z = max(float(tracker.box_dimensions.z), 0.1)
        conf = tracker.confidence
        sphere.color.r = 1.0 - conf
        sphere.color.g = conf
        sphere.color.b = 0.0
        sphere.color.a = 0.6
        sphere.lifetime = rospy.Duration(0.2)
        marker_array.markers.append(sphere)

        arrow = Marker()
        arrow.header = sphere.header
        arrow.ns = "doa_tmpc"
        arrow.id = idx * 2 + 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05, 0.1, 0.1
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.4, 0.1, 0.9
        arrow.lifetime = rospy.Duration(0.2)
        vx, vy = tracker.velocity.x, tracker.velocity.y
        v_mag = math.sqrt(vx * vx + vy * vy)
        if v_mag > self.velocity_limit and v_mag > 0.0:
            s = self.velocity_limit / v_mag
            vx, vy = vx * s, vy * s
        arrow_len = max(0.2, v_mag)
        start = Point(x=tracker.position.x, y=tracker.position.y, z=tracker.position.z)
        end = Point(x=tracker.position.x + vx * arrow_len,
                    y=tracker.position.y + vy * arrow_len,
                    z=tracker.position.z)
        arrow.points.append(start)
        arrow.points.append(end)
        marker_array.markers.append(arrow)
```

- [ ] **Step 4: Verify the module still imports and tests pass**

Run: `cd /home/patrick/sim_dev/simulation_framework && source devel/setup.zsh && python3 -c "import sys; sys.path.insert(0,'src/mpc_nav/scripts'); import doa_to_tmpc; print('import OK')" && pytest src/mpc_nav/test/test_doa_to_tmpc.py -v`
Expected: `import OK` then PASS (8 tests). Confirms no orphaned references to deleted code (the old inline sigma/extrapolation block) remain.

- [ ] **Step 5: Grep to confirm dead code is gone**

Run: `grep -n "sigma2_xx\|has_uncertainty\|sqrt(tracker.sigma2" src/mpc_nav/scripts/doa_to_tmpc.py || echo "clean"`
Expected: `clean` (the closed-form pre-integrated sigma block is fully removed). Note: `sigma2_vxvx`/`sigma2_vyvy` still appear inside `_uncertainty_rate` — that is expected and correct.

- [ ] **Step 6: Commit**

```bash
git add src/mpc_nav/scripts/doa_to_tmpc.py
git commit -m "refactor(bridge): node delegates to pure fn; remove pre-integrated sigma"
```

---

### Task 4: Update launch params + manual integration verification

**Files:**
- Modify: `src/mpc_nav/launch/tmpc_doa.launch` (add new params to the `doa_to_tmpc` node block)

**Interfaces:** none (config only).

- [ ] **Step 1: Add params to the launch node**

In the `<node ... name="doa_to_tmpc" ...>` block of `src/mpc_nav/launch/tmpc_doa.launch`, add inside the node element:

```xml
    <param name="cv_horizon" value="1.0"/>
    <param name="v_min" value="0.1"/>
    <param name="emit_uncertainty" value="false"/>
    <param name="rate_floor" value="0.05"/>
    <param name="rate_ceil" value="0.5"/>
```

- [ ] **Step 2: Launch-file XML sanity check**

Run: `python3 -c "import xml.dom.minidom as m; m.parse('src/mpc_nav/launch/tmpc_doa.launch'); print('xml OK')"`
Expected: `xml OK`.

- [ ] **Step 3: Manual integration run (cannot be unit-tested — state explicitly)**

This step is human-verified, not automated. Run the system and confirm the fix:

```
# Terminal A (perception, needs conda):
conda activate fastsam && source devel/setup.zsh && roslaunch umap simulation.launch
# Terminal B (planner + bridge + gazebo):
source devel/setup.zsh && roslaunch mpc_nav tmpc_doa.launch
# After a run, parse solve rate:
python3 exp/scripts/parse_tmpc_log.py ~/.ros/log/latest/mpc_planner-*.log
```

Acceptance: in RViz, `/doa_obstacles_tmpc_markers` obstacles are compact blobs (not 6 m corridors); `parse_tmpc_log.py` reports a materially higher solve success rate than before. Record the before/after numbers in the commit message.

- [ ] **Step 4: Commit**

```bash
git add src/mpc_nav/launch/tmpc_doa.launch
git commit -m "feat(bridge): expose cv_horizon/v_min/uncertainty params in launch"
```

---

## Self-Review

**Spec coverage:**
- §3.1 short CV + static hold → Task 1 (`k_eff = min(k, k_cv)`), tests `test_cv_region_on_line`, `test_hold_region_frozen`. ✓
- §3.1 deadband + clamp → Task 1, tests `test_deadband_static`, `test_velocity_clamp`. ✓
- §3.1 emit N-1 poses → Task 1, `test_array_lengths`. ✓
- §3.2 Mode A deterministic → Task 1, `test_deterministic_zero_semiaxis`. ✓
- §3.2 Mode B constant rate, no double integration → Task 2, `test_mode_b_constant_rate_not_growing`. ✓
- §3.2 delete closed-form sigma block → Task 3 Step 5 grep gate. ✓
- §3.3 pure function + thin shell + watchdog unchanged + markers behind flag → Tasks 1 & 3. ✓
- §3.4 params → Task 3 Step 1 + Task 4. ✓
- §5 testing (unit + manual integration) → Tasks 1–2 (unit), Task 4 Step 3 (manual). ✓

**Placeholder scan:** No TBD/TODO; all code steps show full code. ✓

**Type consistency:** `BridgeConfig`, `tracker_to_obstacle`, `_uncertainty_rate`, `_append_markers` names consistent across Tasks 1–3. `k_cv` field used identically everywhere. ✓
