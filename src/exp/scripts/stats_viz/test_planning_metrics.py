"""Pure-function tests for calc_planning_metrics (no ROS needed).

Run: python3 -m pytest test_planning_metrics.py   or   python3 test_planning_metrics.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calc_planning_metrics as cpm  # noqa: E402

GOAL = (0.0, 8.0)
TOL = 0.35


def odom(points, dt=0.1):
    return [{"t": i * dt, "x": x, "y": y} for i, (x, y) in enumerate(points)]


def base_metrics(**kw):
    m = {"path_length_m": 5.0, "planner_obstacle_msgs": 10, "rematch_msgs": 10,
         "goal_success": 1}
    m.update(kw)
    return m


def test_static_near_goal_not_success():
    rows = odom([(0.0, 7.9)] * 50)  # sits inside goal tolerance, never moves
    g = cpm.compute_goal_metrics(rows, *GOAL, TOL)
    assert g["goal_success"] == 0
    assert g["path_length_m"] == 0.0
    ok, reasons = cpm.compute_validity({**base_metrics(), **g}, "dodt")
    assert ok is False
    assert "static" in reasons


def test_old_behaviour_would_flag_goal_without_motion():
    ok, reasons = cpm.compute_validity(base_metrics(path_length_m=0.0, goal_success=1), "dodt")
    assert not ok and "goal-without-motion" in reasons and "static" in reasons


def test_driving_robot_reaches_goal():
    rows = odom([(0.0, 0.1 * i) for i in range(0, 81)])
    g = cpm.compute_goal_metrics(rows, *GOAL, TOL)
    assert g["goal_success"] == 1
    assert 7.0 < g["path_length_m"] < 8.0
    ok, reasons = cpm.compute_validity({**base_metrics(), **g}, "ours_x")
    assert ok and reasons == []


def test_validity_rules():
    assert "no-planner-obs" in cpm.compute_validity(base_metrics(planner_obstacle_msgs=0), "dodt")[1]
    assert "no-rematch" in cpm.compute_validity(base_metrics(rematch_msgs=0), "ours_full")[1]
    assert "no-rematch" not in cpm.compute_validity(base_metrics(rematch_msgs=0), "dodt")[1]
    assert cpm.compute_validity(base_metrics(), None)[0] is True


def test_planner_obs_always_empty():
    ok, r = cpm.compute_validity(
        base_metrics(planner_obstacle_msgs=100, planner_obstacle_nonempty_msgs=0), "dodt")
    assert not ok and r == ["planner-obs-always-empty"]
    ok, r = cpm.compute_validity(
        base_metrics(planner_obstacle_msgs=100, planner_obstacle_nonempty_msgs=3), "dodt")
    assert ok and r == []
    # zero messages is reported as no-planner-obs, not as always-empty
    r = cpm.compute_validity(
        base_metrics(planner_obstacle_msgs=0, planner_obstacle_nonempty_msgs=0), "dodt")[1]
    assert r == ["no-planner-obs"]


def test_start_at_goal_needs_motion_then_hit():
    # starts inside goal tolerance, drives away and comes back: first hit
    # only counts once >= MIN_MOTION_M has been driven.
    pts = [(0.0, 8.0)] * 5 + [(0.0, 8.0 - 0.2 * i) for i in range(1, 11)] \
        + [(0.0, 6.0 + 0.2 * i) for i in range(1, 11)]
    g = cpm.compute_goal_metrics(odom(pts), *GOAL, TOL)
    assert g["goal_success"] == 1
    assert g["path_length_m"] >= cpm.MIN_MOTION_M


def test_goal_without_motion_legacy_flag():
    ok, r = cpm.compute_validity(base_metrics(path_length_m=0.0, goal_success=1), "dodt")
    assert "goal-without-motion" in r


def test_empty_odom():
    g = cpm.compute_goal_metrics([], *GOAL, TOL)
    assert g["goal_success"] == 0
    ok, r = cpm.compute_validity({**base_metrics(), **g}, "dodt")
    assert not ok and "static" in r


def test_infer_method():
    assert cpm.infer_method("/d/pair_crossing/ours_full/run_3") == "ours_full"
    assert cpm.infer_method("/d/pair_crossing/dodt/run_1/sub") == "dodt"
    assert cpm.infer_method("/d/foo") == "foo"


def test_goal_from_messages():
    class P:  # minimal PoseStamped stand-in
        def __init__(self, x, y):
            self.pose = type("o", (), {"position": type("p", (), {"x": x, "y": y})()})()
    assert cpm.goal_from_messages([]) is None
    assert cpm.goal_from_messages([P(1, 2), P(3, 4)]) == (3.0, 4.0)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
