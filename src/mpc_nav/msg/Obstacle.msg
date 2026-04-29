uint32 id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 radius
uint8 type   # 0: unknown, 1: cylinder, 2: box, 3: human (optional)
# 1.0 = confirmed dynamic (passed hysteresis); 0.5 = candidate (early-published, motion suspected)
float32 confidence
