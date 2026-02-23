# 项目概述 (Project Overview)

当前项目是一个基于 ROS Noetic 的移动机器人动态障碍物避障系统仿真与测试框架。主要研究方向为：结合深度相机与视觉特征融合的动态障碍物检测与跟踪（DOA），以及基于模型预测控制（MPC）的局部路径规划与动态避障。

## 项目结构 (Project Structure)

工作空间主要包含以下核心 ROS 包：

- **`src/DOA/`**: 动态障碍物检测与跟踪系统 (Dynamic Obstacle Avoidance)
  - `umap/`: 核心感知模块，从深度/RGB图像提取2D/3D边界框，使用特征匹配与卡尔曼滤波进行跟踪。
  - `pointcloud_generator/`: 深度图转点云模块。
  - `path_planning/`: 包含 costmap 层与 DWA 插件（用于对接 move_base）。
- **`src/mpc_nav/`**: 基于 MPC 的导航与避障模块
  - `scripts/mpc_node.py`: MPC 导航 ROS 节点，接收里程计、目标点和障碍物信息，输出速度指令。
  - `scripts/mpc_solver.py`: 基于 `CasADi` 的 MPC 求解器，实现差速驱动机器人的运动学约束与动态障碍物避障约束。
  - `scripts/doa_to_obstacles.py`: 适配器节点，将 DOA 模块输出的跟踪器信息转换为 MPC 所需的障碍物数组格式。
- **`src/my_tb3_description/`**: TurtleBot3 机器人的 URDF 描述（搭载 Realsense D435）与 Gazebo 仿真启动文件（包含动态障碍物场景 `dync_obs.launch`）。
- **`src/dashgo_description/`**: Dashgo 机器人的描述文件。
- **`src/my_gazebo_test/`**: Gazebo 仿真环境与测试场景。

## 关键话题与数据流 (Key Topics & Data Flow)

系统的核心数据流向为：**传感器 -> 感知与跟踪 -> 格式适配 -> 规划控制 -> 底层执行**。

1. **感知输入**:
   - `/d435/image_raw` (RGB 图像)
   - `/d435/depth/image_raw` (深度图像)
2. **障碍物检测与跟踪 (DOA -> MPC)**:
   - `/rematch/tracker_details` (`umap/TrackerDetailArray`): DOA 模块输出的障碍物跟踪信息（包含 ID、位置、速度、尺寸）。
   - `/doa_obstacles` (`mpc_nav/ObstacleArray`): 经过 `doa_to_obstacles.py` 转换后的障碍物信息，供 MPC 使用。
3. **导航与控制 (MPC)**:
   - `/odom` (`nav_msgs/Odometry`): 机器人当前状态。
   - `/move_base_simple/goal` (`geometry_msgs/PoseStamped`): 目标点（通常由 RViz 下发）。
   - `/cmd_vel` (`geometry_msgs/Twist`): MPC 输出的控制指令。
   - `/mpc/pred_path` (`nav_msgs/Path`): MPC 预测的未来轨迹（用于可视化）。

## 实现细节 (Implementation Details)

- **DOA 模块**: 采用"感知→检测→跟踪"管线，支持仿真模式（粗匹配，轻量高效）和真机模式（引入 FastSAM/CLIP 特征提取进行细匹配）。
- **MPC 求解器**: 使用 `CasADi` 构建非线性优化问题。
  - **运动学模型**: 差速驱动（Unicycle）模型。
  - **代价函数**: 包含目标点位置误差、航向误差、控制量（速度、角速度）惩罚、控制平滑性（加速度）惩罚。
  - **避障约束**: 基于障碍物的当前位置和速度，采用**恒速预测模型**预测未来 $N$ 步的障碍物轨迹，并在优化过程中加入机器人与障碍物之间的安全距离约束。
- **仿真环境**: 使用 Gazebo 进行物理仿真，包含 TurtleBot3 机器人模型以及动态障碍物场景。

## 当前测试状态与目的 (Current Testing Status & Purpose)

- **当前状态**: 正在进行动态障碍物避障系统的集成测试。
- **测试链路**: Gazebo 仿真环境 -> 深度/RGB图像 -> DOA 模块检测与跟踪 -> `doa_to_obstacles.py` 格式转换 -> MPC 节点求解 -> 输出速度指令控制机器人避障。
- **启动方式**:
  1. 启动 MPC、适配器、Gazebo 环境及动态障碍物场景：
     ```bash
     roslaunch mpc_nav mpc.launch
     ```
  2. 启动 DOA 感知系统（仿真模式）：
     ```bash
     roslaunch umap simulation.launch
     ```
- **研究目的**: 验证基于视觉特征融合的动态障碍物跟踪系统（DOA）与基于模型预测控制（MPC）的局部避障算法的联合工作性能，确保机器人在动态复杂环境中能够安全、平滑地导航。