uint32 id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
# Circular footprint (legacy / fallback). Consumers that don't understand
# the elliptical fields below MUST fall back to this radius.
float32 radius
# Elliptical footprint (preferred, when the upstream knows the obstacle's
# bounding box and direction of motion).
#   - semi_major: half-axis aligned with the velocity vector
#   - semi_minor: half-axis perpendicular to the velocity vector
# Both are GEOMETRIC sizes only — no lookahead inflation. The MPC planner
# applies any motion-aware inflation (e.g. velocity-scaled lookahead)
# itself, so visualization stays a true picture of the obstacle.
# When the publisher does not provide ellipse fields (e.g. legacy GT
# scene scripts), both default to 0; consumers must then fall back to
# `radius` and treat the obstacle as a circle.
float32 semi_major
float32 semi_minor
uint8 type   # 0: unknown, 1: cylinder, 2: box, 3: human (optional)
# 1.0 = confirmed dynamic (passed hysteresis); 0.5 = candidate (early-published, motion suspected)
float32 confidence
