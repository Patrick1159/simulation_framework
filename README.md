# Simulation Framework

A ROS-based mobile robot simulation framework for testing navigation and planning algorithms in dynamic environments with Gazebo.

## Overview

This framework provides:
- **Custom robot descriptions** (TurtleBot3 variants with sensors)
- **Dynamic obstacle generation** (random movement, corridor scenarios)
- **Static obstacle fields**
- **MPC-based navigation** (Model Predictive Control)
- **Gazebo integration** for realistic physics simulation

## Repository Structure

```
my_sim/
├── src/
│   ├── dashgo_description/      # Dashgo robot URDF models
│   ├── DOA/                     # Dynamic Obstacle Avoidance (vision-based perception)
│   │   ├── src/umap/            # Detection & tracking pipeline
│   │   ├── src/pointcloud_generator/  # Depth to pointcloud processing
│   │   └── src/path_planning/   # (Optional) move_base integration
│   ├── my_tb3_description/      # TurtleBot3 variants with sensors
│   │   ├── urdf/                # Base TurtleBot3 models
│   │   ├── urdf_w_sensor/       # TurtleBot3 with camera/sensors
│   │   ├── scripts/             # Obstacle generation scripts
│   │   └── launch/              # Launch files
│   ├── mpc_nav/                 # MPC navigation package
│   │   ├── msg/                 # Custom messages (Obstacle, ObstacleArray, etc.)
│   │   ├── scripts/             # MPC node and solver
│   │   └── launch/              # MPC launch files
│   └── my_gazebo_test/          # Gazebo worlds and tests
├── build/                       # Build artifacts
└── devel/                       # Development space
```

## Prerequisites

- **ROS Noetic** (Ubuntu 20.04)
- **Gazebo 11**
- **Python 3.8+**
- **Dependencies:**
  ```bash
  sudo apt install ros-noetic-gazebo-ros-pkgs
  sudo apt install ros-noetic-robot-state-publisher
  sudo apt install ros-noetic-joint-state-publisher
  sudo apt install ros-noetic-xacro
  sudo apt install ros-noetic-tf
  sudo apt install python3-casadi  # For MPC solver
  ```

## Installation

1. **Clone the repository:**
   ```bash
   cd ~/
   git clone <repository-url> my_sim
   cd my_sim
   ```

2. **Build the workspace:**
   ```bash
   catkin_make
   source devel/setup.bash
   ```

3. **Add to your `.bashrc` (optional):**
   ```bash
   echo "source ~/my_sim/devel/setup.bash" >> ~/.bashrc
   ```

## Usage

### 1. Launch TurtleBot3 in Gazebo

**Basic spawn with camera:**
```bash
roslaunch my_tb3_description my_tb3_gazebo.launch
```

The robot is spawned at origin facing +Y direction with:
- Intel RealSense D435 camera
- 2D LiDAR
- IMU
- Differential drive controller

### 2. Obstacle Scenarios

#### Static Obstacles
Generate random non-overlapping static obstacles:
```bash
roslaunch my_tb3_description static_obs.launch
```

**Parameters:**
- `num_obstacles`: Number of obstacles (default: 8)
- `xmin/xmax/ymin/ymax`: Spawn area bounds
- `radius`: Cylinder radius (default: 0.3m)
- `min_distance`: Minimum spacing between obstacles (default: 1.0m)

**Example:**
```bash
roslaunch my_tb3_description static_obs.launch \
  num_obstacles:=12 xmin:=-2.0 xmax:=2.0 ymax:=15.0
```

#### Dynamic Obstacles

**Scene 0 - Random Movement:**
```bash
roslaunch my_tb3_description dync_obs.launch scene:=0
```
Multiple obstacles with random velocities and directions, bouncing off boundaries.

**Scene 1 - Corridor (default):**
```bash
roslaunch my_tb3_description dync_obs.launch scene:=1
```
Corridor with walls and obstacles moving back-and-forth along X-axis.

**Parameters (Scene 1):**
- `spacing`: Distance between obstacles along Y (default: 2.0m)
- `speed`: Base speed (default: 0.8 m/s)
- `speed_variance`: Random speed variation (default: 0.2 m/s)
- `xmin/xmax`: Corridor width (default: -1.5 to 1.5)
- `ymin/ymax`: Corridor length (default: 0 to 10)

**Example:**
```bash
roslaunch my_tb3_description dync_obs.launch scene:=1 \
  speed:=1.0 spacing:=3.0 ymax:=15.0
```

### 3. MPC Navigation

**Start MPC node:**
```bash
roslaunch mpc_nav mpc.launch
```

**Key parameters:**
- `N`: Prediction horizon steps (default: 20)
- `dt`: Time step (default: 0.1s)
- `v_max`: Max linear velocity (default: 0.4 m/s)
- `w_max`: Max angular velocity (default: 1.2 rad/s)
- `robot_radius`: Robot radius for collision (default: 0.12m)
- `safety_margin`: Additional safety buffer (default: 0.15m)

**Set goal via RViz:**
1. Open RViz: `rviz`
2. Add → Path (topic: `/mpc/pred_path`)
3. Add → MarkerArray (topic: `/obstacles_marker`)
4. Use "2D Nav Goal" tool to set target pose

**Or programmatically:**
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "header: {frame_id: 'odom'}
   pose: {position: {x: 5.0, y: 8.0, z: 0.0}, 
          orientation: {w: 1.0}}"
```

### 4. Complete Simulation Example

#### Option A: DOA Vision-Based Navigation (Recommended)

**Complete system with camera-based perception:**

```bash
# One-line launch: Gazebo + DOA perception + MPC + Dynamic obstacles
roslaunch umap simulation_dync.launch
```

This includes:
- ✅ Gazebo simulation + TurtleBot3 with camera
- ✅ Dynamic obstacles (corridor or random movement)
- ✅ DOA perception pipeline (depth/RGB → detection → tracking)
- ✅ Automatic format conversion (DOA → MPC)
- ✅ MPC navigation
- ✅ RViz visualization

**Set goal:**
- In RViz, use "2D Nav Goal" tool
- Or via command line:
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "header: {frame_id: 'odom'}
   pose: {position: {x: 5.0, y: 8.0, z: 0.0}, 
          orientation: {w: 1.0}}"
```

**Select scene:**
```bash
# Corridor scene (default)
roslaunch umap simulation_dync.launch dynamic_scene:=1

# Random movement scene
roslaunch umap simulation_dync.launch dynamic_scene:=0
```

---

#### Option B: Simple Obstacle Generation (Lightweight)

**For quick testing without vision processing:**

**Terminal 1 - Gazebo + Robot:**
```bash
roslaunch my_tb3_description my_tb3_gazebo.launch
```

**Terminal 2 - Dynamic Obstacles:**
```bash
roslaunch my_tb3_description dync_obs.launch scene:=1
```

**Terminal 3 - MPC Navigation:**
```bash
roslaunch mpc_nav mpc_node.launch
```

**Terminal 4 - Visualization:**
```bash
rviz
```

---

## DOA Vision-Based Perception

The framework includes DOA (Dynamic Obstacle Avoidance), a vision-based perception system that uses depth cameras and RGB images to detect and track dynamic obstacles.

### Key Features
- **Depth + RGB fusion**: Extracts 2D/3D bounding boxes from sensor data
- **Feature-based tracking**: Uses FastSAM and CLIP for robust object tracking
- **Kalman filtering**: Smooth trajectory prediction and velocity estimation
- **ROS integration**: Outputs `TrackerDetailArray` for planning systems

### Quick Start with DOA

See complete documentation in [`src/DOA/README.md`](src/DOA/README.md)

**Basic usage:**
```bash
# Complete system (perception + navigation)
roslaunch umap simulation_dync.launch

# Perception only (for debugging)
roslaunch umap simulation.launch
```

### DOA → MPC Integration

The `doa_to_obstacles.py` adapter converts DOA's vision-based tracking to MPC's obstacle format:

- **Input**: `/rematch/tracker_details` (TrackerDetailArray)
- **Output**: `/doa_obstacles` (ObstacleArray) ← MPC uses this for planning
- **Features**: Age filtering, velocity limiting, radius conversion

**Topic naming convention:**
- `/obstacles`: Ground truth from simulation scripts (for evaluation)
- `/doa_obstacles`: Vision-based observations from DOA (for planning)

---

## ROS Topics

### Published by Obstacle Scripts
- `/obstacles` ([`mpc_nav/ObstacleArray`](src/mpc_nav/msg/ObstacleArray.msg)) - **Ground truth** obstacle states from simulation
- `/obstacles_marker` ([`visualization_msgs/MarkerArray`](/opt/ros/noetic/share/visualization_msgs/msg/MarkerArray.msg)) - Visualization markers

### Published by DOA Adapter (vision-based observations)
- `/doa_obstacles` ([`mpc_nav/ObstacleArray`](src/mpc_nav/msg/ObstacleArray.msg)) - **Observed** obstacle states from DOA perception

### Published by MPC Node
- `/cmd_vel` ([`geometry_msgs/Twist`](/opt/ros/noetic/share/geometry_msgs/msg/Twist.msg)) - Velocity commands
- `/mpc/pred_path` ([`nav_msgs/Path`](/opt/ros/noetic/share/nav_msgs/msg/Path.msg)) - Predicted trajectory
- `/mpc/status` ([`mpc_nav/MPCStatus`](src/mpc_nav/msg/MPCStatus.msg)) - Solver status and metrics

### Subscribed by MPC Node
- `/odom` ([`nav_msgs/Odometry`](/opt/ros/noetic/share/nav_msgs/msg/Odometry.msg)) - Robot odometry
- `/move_base_simple/goal` ([`geometry_msgs/PoseStamped`](/opt/ros/noetic/share/geometry_msgs/msg/PoseStamped.msg)) - Goal pose
- `/obstacles` ([`mpc_nav/ObstacleArray`](src/mpc_nav/msg/ObstacleArray.msg)) - When using simple obstacle scripts
- `/doa_obstacles` ([`mpc_nav/ObstacleArray`](src/mpc_nav/msg/ObstacleArray.msg)) - When using DOA vision system

**Note:** MPC subscribes to either `/obstacles` (ground truth) or `/doa_obstacles` (vision-based), depending on configuration.

### Published by DOA System (when using vision-based perception)
- `/rematch/tracker_details` ([`umap/TrackerDetailArray`](src/DOA/src/umap/msg/TrackerDetailArray.msg)) - Tracked obstacles with ID, pose, velocity, dimensions
- `/umap/3D_bounding_boxes` ([`umap/BoundingBoxes3D`](src/DOA/src/umap/msg/BoundingBoxes3D.msg)) - Detected 3D bounding boxes
- See full list in [`src/DOA/README.md`](src/DOA/README.md)

## Robot Models

### TurtleBot3 Burger with Sensors
Location: `my_tb3_description/urdf_w_sensor/`

**Sensors:**
- **Intel RealSense D435**: RGB-D camera with depth
  - Topics: `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/depth/points`
- **2D LiDAR**: 360° laser scanner
- **IMU**: Inertial measurement unit

**Key frames:**
- `base_footprint`: Ground reference (for 2D navigation)
- `base_link`: Robot body center
- `camera_link`: Camera mount point

### Dashgo Robot
Location: `dashgo_description/urdf/dashgobase/`

Modular URDF with xacro macros for:
- Base platform with differential drive wheels
- Optional torso extension
- Material definitions

## Custom Messages

### Obstacle.msg
```
uint32 id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 radius
uint8 type   # 0: unknown, 1: cylinder, 2: box, 3: human
```

### ObstacleArray.msg
```
std_msgs/Header header
Obstacle[] obstacles
```

## Development

### Adding New Obstacle Scenarios

1. Create new script in `my_tb3_description/scripts/`
2. Use `sdf.py` utilities for model generation
3. Publish to `/obstacles` topic using `ObstacleArray`
4. Create corresponding launch file

**Example:**
```python
from mpc_nav.msg import Obstacle, ObstacleArray
from sdf import cylinder_sdf

obs_array = ObstacleArray()
obs_array.header.frame_id = "world"
# ... populate obstacles
self.obs_pub.publish(obs_array)
```

### Modifying Robot URDF

URDF files use xacro for modularity:
- Main entry: `turtlebot3_burger.urdf.xacro`
- Sensors: Added via `<xacro:sensor_d435 ...>`
- Generate URDF: `rosrun xacro xacro file.urdf.xacro > output.urdf`

### MPC Solver Customization

Edit `mpc_nav/scripts/mpc_solver.py` to:
- Adjust cost function weights
- Add new constraints
- Change kinematic model

## Troubleshooting

**Camera depth not publishing:**
- Check `turtlebot3_burger.gazebo.xacro` has depth sensor plugin
- Verify `reference="camera_link"` matches your URDF link name

**TF errors (base_link not found):**
- Ensure `robot_state_publisher` is running in your launch file
- Check `robot_description` parameter is loaded: `rosparam get /robot_description`

**Obstacles not visible in RViz:**
- Set Fixed Frame to `world` or `odom`
- Add MarkerArray display for topic `/obstacles_marker`

**MPC path too short:**
- Increase prediction horizon: `N:=30`
- Increase time step: `dt:=0.15`
- Check solver is returning full control sequence

## Contributing

When adding new features:
1. Follow ROS naming conventions
2. Document parameters in launch files
3. Use latched publishers for static data
4. Include example usage in commit messages

## License

[Specify your license here]

## References

- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
