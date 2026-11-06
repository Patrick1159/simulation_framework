import os
import sys
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import dync_scene3 as scene  # noqa: E402


def test_straight_cv_no_boundary():
    x, y, vx, vy = scene.reflect_step(0.0, 0.0, 1.0, 0.5, 0.1,
                                      xmin=-5, xmax=5, ymin=-5, ymax=5)
    assert math.isclose(x, 0.1, abs_tol=1e-9)
    assert math.isclose(y, 0.05, abs_tol=1e-9)
    assert vx == 1.0 and vy == 0.5


def test_x_bounce_flips_vx_only():
    # Start near xmax, moving +x, dt large enough to cross
    x, y, vx, vy = scene.reflect_step(4.95, 0.0, 1.0, 0.0, 0.2,
                                      xmin=-5, xmax=5, ymin=-5, ymax=5)
    assert vx == -1.0      # x velocity reflected
    assert vy == 0.0
    assert x <= 5.0        # reflected back inside
    assert x >= -5.0


def test_y_bounce_flips_vy_only():
    x, y, vx, vy = scene.reflect_step(0.0, -4.95, 0.0, -1.0, 0.2,
                                      xmin=-5, xmax=5, ymin=-5, ymax=5)
    assert vy == 1.0
    assert vx == 0.0
    assert y >= -5.0


def test_corner_double_bounce():
    x, y, vx, vy = scene.reflect_step(4.95, 4.95, 1.0, 1.0, 0.2,
                                      xmin=-5, xmax=5, ymin=-5, ymax=5)
    assert vx == -1.0 and vy == -1.0
    assert -5.0 <= x <= 5.0
    assert -5.0 <= y <= 5.0


def test_stays_in_bounds_after_reflection():
    # Many steps must never leave the box
    x, y, vx, vy = 0.0, 0.0, 0.73, -0.41
    for _ in range(2000):
        x, y, vx, vy = scene.reflect_step(x, y, vx, vy, 0.1,
                                          xmin=-3, xmax=3, ymin=-3, ymax=3)
        assert -3.0 <= x <= 3.0, f"x out of bounds: {x}"
        assert -3.0 <= y <= 3.0, f"y out of bounds: {y}"


def test_speed_preserved():
    x, y, vx, vy = scene.reflect_step(2.99, 0.0, 0.6, 0.8, 0.2,
                                      xmin=-3, xmax=3, ymin=-3, ymax=3)
    assert math.isclose(math.hypot(vx, vy), 1.0, abs_tol=1e-9)


if __name__ == "__main__":
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
