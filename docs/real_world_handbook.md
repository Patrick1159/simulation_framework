# Real-World DOA + T-MPC Deployment Handbook

## 1. 硬件要求 (Hardware)

| 组件 | 规格 |
|------|------|
| 机器人 | 差分驱动 (TurtleBot3 Burger 或同类)，轮距 < 0.3m |
| RGB-D | Intel RealSense D435，前向安装，视场角 87deg(H) x 58deg(V)，最大有效深度 6m |
| 里程计 | Intel RealSense T265，6-DoF，安装在机器人中心上方 |
| 计算 | NVIDIA GPU 笔记本 (RTX 2060+)，SAM + CLIP 需要 CUDA |
| 网络 | 机器人 <-> 笔记本 WiFi/以太网，推荐 5GHz 或千兆有线 |

## 2. 环境配置 (Environment Setup)

### 2.1 系统依赖
```bash
# ROS Noetic (Ubuntu 20.04)
sudo apt install ros-noetic-desktop-full python3-rosdep python3-catkin-tools
sudo rosdep init && rosdep update

# CUDA 11.x + cuDNN 8.x，验证:
nvidia-smi && nvcc --version
```

### 2.2 acados 安装
acados 目录 (`src/mpc_planner/acados/`，~349MB) 不在 git 中，需单独下载：
```bash
cd ~/sim_dev/simulation_framework/src/mpc_planner
git clone https://github.com/acados/acados.git
cd acados && git submodule update --init --recursive
mkdir build && cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=OFF ..
make -j$(nproc) && make install
```
加入 `~/.bashrc`:
```bash
export ACADOS_SOURCE_DIR=$HOME/sim_dev/simulation_framework/src/mpc_planner/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
```

### 2.3 Conda 环境
```bash
conda create -n fastsam python=3.8
conda activate fastsam
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
pip install ultralytics clip opencv-python numpy scipy matplotlib
pip install git+https://github.com/facebookresearch/segment-anything.git
```

### 2.4 编译
**重要：编译前必须 `conda deactivate`，catkin 与 conda Python 冲突。**
```bash
conda deactivate
cd ~/sim_dev/simulation_framework
catkin build mpc_planner_jackalsimulator guidance_planner roadmap umap pointcloud_generator
source devel/setup.bash
```
验证：`rospack find mpc_planner_jackalsimulator` 应返回有效路径。

## 3. 传感器话题与 TF 校验

### 3.1 标准话题

| 话题 | 类型 | 来源 |
|------|------|------|
| `/d400/color/image_raw` | `sensor_msgs/Image` | D435 RGB |
| `/d400/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | D435 Depth |
| `/t265/odom/sample` | `nav_msgs/Odometry` | T265 里程计 |
| `/cmd_vel` | `geometry_msgs/Twist` | 控制输出 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | rviz 目标点 |

话题名不同时通过 launch arg 覆盖：
```bash
roslaunch mpc_nav real_world_tmpc.launch \
  rgb_topic:=/camera/color/image_raw \
  odom_topic:=/odometry/filtered
```

### 3.2 TF 校验
```bash
rosrun tf view_frames && evince frames.pdf
# 预期链路: map -> odom -> base_link -> d400_color_optical_frame
```
map->odom 需手动发布静态 TF：
```bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
如果 T265 默认输出 `t265_odom_frame` 而非 `odom`，可通过 `odom_frame:=t265_odom_frame` 覆盖。

## 4. 快速启动 (Quick Start)

```bash
# 终端 1: roscore
roscore

# 终端 2: 相机驱动
roslaunch realsense2_camera rs_d400_and_t265.launch

# 终端 3: DOA + T-MPC 完整管线
conda activate fastsam
source ~/sim_dev/simulation_framework/devel/setup.bash
roslaunch mpc_nav real_world_tmpc.launch
```

调试模式（不输出 cmd_vel，安全测试感知管线）：
```bash
roslaunch mpc_nav real_world_tmpc.launch plan_only:=true
```

启用全局规划器 (Visibility-PRM)：
```bash
roslaunch mpc_nav real_world_tmpc.launch use_global_planner:=true map_file:=/path/to/map.xml
```

### Launch 参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rgb_topic` | `/d400/color/image_raw` | RGB 图像话题 |
| `aligned_depth_topic` | `/d400/aligned_depth_to_color/image_raw` | 对齐深度图 |
| `odom_topic` | `/t265/odom/sample` | 里程计 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `base_frame` | `base_link` | 机器人基座坐标系 |
| `camera_frame` | `d400_color_optical_frame` | 相机光心坐标系 |
| `cmd_vel_topic` | `/cmd_vel` | 控制指令输出 |
| `goal_topic` | `/move_base_simple/goal` | 导航目标 |
| `use_global_planner` | `false` | 启用全局规划器 |
| `plan_only` | `false` | true=不输出 cmd_vel |
| `use_rviz` | `true` | 启动 RViz |

启动后在 RViz 中使用 "2D Nav Goal" 工具点击地图设置目标。

## 5. 数据流

```
D435 RGB+Depth ──> umap_extract ──> coarse_match ──> [SAM+CLIP+aggregate] ──> detailed_match
                                                                                   │
                                                                 /rematch/tracker_details
                                                                                   │
                                                                        doa_to_tmpc.py
                                                                                   │
T265 Odometry ───────────────────────────────────────────> /doa_obstacles_tmpc (GMM)
                                                                    │
rviz Goal ──────────────────────────> mpc_planner_jackalsimulator ──> /cmd_vel ──> 机器人
```

**T-MPC 关键参数 (settings.yaml):** Horizon N=30, dt=0.2s (6s 预测时域), 控制频率 20Hz, 最大障碍物 12 个, 并行求解器 4 个 (T-MPC++), 机器人半径 0.18m, acados SQP-RTI 求解器.

## 6. 故障排查 (Troubleshooting)

**CUDA 不可用:**
```bash
nvidia-smi
conda activate fastsam && python -c "import torch; print(torch.cuda.is_available())"
```
检查驱动版本 (>= 470) 与 PyTorch CUDA 版本匹配。

**SAM/CLIP 推理过慢 (感知 < 5Hz):**
- 降低 SAM 分辨率：`<param name="sam_image_size" value="48"/>`
- 启用 passthrough 消除同步等待：`<param name="passthrough_clip" value="true"/>`
- 确认 GPU 推理：`python -c "import torch; print(torch.cuda.is_available())"`

**MPC 求解器失败 (机器人不移动):**
```bash
echo $LD_LIBRARY_PATH | grep acados          # 库路径
rostopic echo /move_base_simple/goal -n 1    # 目标是否收到
```
确认 settings.yaml 中 `debug_output: true` 以查看详细错误。

**感知无输出 (障碍物为空):**
```bash
# 逐级检查
rostopic hz /d400/color/image_raw                          # 相机
rostopic echo /umap/2D_3D_bounding_boxes -n 1              # 检测
rostopic echo /tracker/dynamic_obstacles -n 1              # coarse 追踪
rostopic echo /rematch/tracker_details -n 1                # detailed_match
rostopic echo /doa_obstacles_tmpc -n 1                     # 桥接
rostopic echo /tf | grep frame_id                          # TF
```

**TF 错误 (camera_pose.py 报 "frame does not exist"):**
```bash
rosrun rqt_tf_tree rqt_tf_tree                             # 可视化 TF 树
rosrun tf tf_echo odom d400_color_optical_frame            # 检查特定链路
```
确保 map->odom 静态 TF 已发布，且 `odom_frame` 参数与 T265 输出一致。

## 7. 与仿真差异 (Sim-to-Real)

| 维度 | 仿真 | 真实环境 |
|------|------|----------|
| D435 深度 | 无噪声 | 随机噪声，边缘抖动 |
| 光照 | 恒定 | 强光/暗光影响 SAM 掩码和 CLIP 特征 |
| 延迟 | 固定步长 | 变长 (曝光+传输+GPU) |
| TF 坐标系 | `odom` 理想 | T265 自定义 frame，需 remap |
| 地面 | 平整 | 不平，self_filter 需调大 |
| self_filter | 0.0025 | 0.25 |
| T265 | 理想 | 快速旋转/白墙/暗光下漂移 |

**真实环境推荐参数调整 (相对仿真默认值):**
- `pointcloud_generate/dynamic_only`: `false` (获取完整点云便于调试)
- `camera/self_filter`: `0.25` (仿真用 0.0025)
- `MIN_OBSTACLE_Z`: `0.1` (过滤地面以下误检)
- `max_dist`: `6.0` (最大有效深度)

## 8. 实验前检查清单

**硬件:**
- [ ] D435 和 T265 USB 连接正常 (`lsusb`)
- [ ] 电池充足，急停可触及

**软件:**
- [ ] ROS Master 运行中
- [ ] 相机话题正常发布 (`rostopic hz /d400/color/image_raw`)
- [ ] TF 树完整 (`rosrun tf view_frames`)
- [ ] GPU 可用 (`nvidia-smi`)
- [ ] acados 可加载 (`ldconfig -p | grep acados`)
- [ ] conda 环境已激活

**管线:**
- [ ] 所有节点无报错
- [ ] 障碍物有输出 (`rostopic echo /doa_obstacles_tmpc -n 1`)
- [ ] rviz 显示障碍物和机器人位置

**安全:**
- [ ] 首次测试使用 `plan_only:=true`
- [ ] 低速启动，操作员可随时物理介入

## 9. 关键文件

| 用途 | 路径 |
|------|------|
| 真实世界启动 | `src/mpc_nav/launch/real_world_tmpc.launch` |
| DOA 感知参数 | `src/DOA/src/umap/config/simulation.yaml` |
| 点云生成参数 | `src/DOA/src/pointcloud_generator/param/default_param.yaml` |
| T-MPC 求解器 | `src/mpc_planner/mpc_planner_jackalsimulator/config/settings.yaml` |
| DOA->T-MPC 桥接 | `src/mpc_nav/scripts/doa_to_tmpc.py` |
| 全局规划器参数 | `src/guidance_planner/config/params.yaml` |
| RViz 配置 | `src/mpc_nav/launch/rviz/mpc.rviz` |

## 10. 常用命令

```bash
# 启动
conda activate fastsam && source devel/setup.bash && roslaunch mpc_nav real_world_tmpc.launch

# 调试
roslaunch mpc_nav real_world_tmpc.launch plan_only:=true

# 查看障碍物
rostopic echo /doa_obstacles_tmpc -n 1

# 重新编译
conda deactivate && catkin build mpc_planner_jackalsimulator && source devel/setup.bash

# 录制 bag
rosbag record -O experiment.bag \
  /d400/color/image_raw /d400/aligned_depth_to_color/image_raw \
  /t265/odom/sample /doa_obstacles_tmpc /cmd_vel /tf /tf_static
```
