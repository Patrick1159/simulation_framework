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
    for k in range(1, cfg.k_cv + 1):
        assert math.isclose(ps[k - 1].pose.position.x, 1.0 * cfg.dt * k, abs_tol=1e-6)
        assert math.isclose(ps[k - 1].pose.position.y, 0.0, abs_tol=1e-6)


def test_hold_region_frozen():
    cfg = make_cfg()
    obs = bridge.tracker_to_obstacle(make_tracker(vx=1.0, vy=0.0), cfg)
    ps = poses(obs)
    frozen_x = 1.0 * cfg.dt * cfg.k_cv
    for k in range(cfg.k_cv + 1, cfg.N):
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
    assert math.isclose(ps[0].pose.position.x, 2.0 * cfg.dt * 1, abs_tol=1e-6)


def test_deterministic_zero_semiaxis():
    cfg = make_cfg(emit_uncertainty=False)
    obs = bridge.tracker_to_obstacle(make_tracker(s_vxvx=0.9, s_vyvy=0.9), cfg)
    g = obs.gaussians[0]
    assert all(v == 0.0 for v in g.major_semiaxis)
    assert all(v == 0.0 for v in g.minor_semiaxis)


def test_mode_b_constant_rate_not_growing():
    cfg = make_cfg(emit_uncertainty=True, rate_floor=0.05, rate_ceil=0.5)
    obs = bridge.tracker_to_obstacle(make_tracker(s_vxvx=0.04, s_vyvy=0.01), cfg)
    g = obs.gaussians[0]
    expected = math.sqrt(0.04)  # = 0.2, within [floor, ceil]
    assert all(math.isclose(v, expected, abs_tol=1e-6) for v in g.major_semiaxis)
    assert all(math.isclose(v, expected, abs_tol=1e-6) for v in g.minor_semiaxis)
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


if __name__ == "__main__":
    # Plain-python3 runner (pytest unavailable in this workspace). Runs every
    # top-level test_* function and reports pass/fail.
    mod = sys.modules[__name__]
    tests = [getattr(mod, n) for n in sorted(dir(mod))
             if n.startswith("test_") and callable(getattr(mod, n))]
    failures = 0
    for fn in tests:
        try:
            fn()
            print("PASS", fn.__name__)
        except Exception as e:
            failures += 1
            print("FAIL", fn.__name__, "->", repr(e))
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    sys.exit(1 if failures else 0)
