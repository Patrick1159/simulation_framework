# mpc_nav

基于 CasADi + IPOPT 的移动机器人 MPC 规划器（差速/独轮车模型），支持动态障碍物软约束、控制平滑约束，并通过 `/cmd_vel` 与底盘对接。

## MPC 算法概述

### 状态与控制
- 状态 $x=[p_x, p_y, \psi]$
- 控制 $u=[v, \omega]$
- 离散模型（采样周期 $dt$）

$$
\begin{aligned}
 p_x^{k+1} &= p_x^{k} + dt\, v^k \cos(\psi^k)\\
 p_y^{k+1} &= p_y^{k} + dt\, v^k \sin(\psi^k)\\
 \psi^{k+1} &= \psi^{k} + dt\, \omega^k
\end{aligned}
$$

### 目标函数（软约束）
代价函数在每个预测步长 $k$ 由以下项组成，并在末端加入更强的终端代价。设目标为 $g=[g_x,g_y,g_\psi]$，控制为 $u_k=[v_k,\omega_k]$：

**运行代价**（代码实现见 `scripts/mpc_solver.py` 的 [running cost 段](scripts/mpc_solver.py#L68-L105)）
$$
\begin{aligned}
J_k &= w_{p}\,\|p_k-g_{xy}\|^2 + w_{y}\,\sin^2(\psi_k-g_\psi) \\
&\quad + w_{u}(v_k^2+\omega_k^2) + w_{\Delta u}\,\|u_k-u_{k-1}\|^2 \\
&\quad + w_{obs}\,\sum_{i=1}^{M}\left[\max\left(0,\,r_{safe,i}^2-d_{i,k}^2\right)\right]^2
\end{aligned}
$$

其中：
- $p_k=[p_x^k,p_y^k]$，$g_{xy}=[g_x,g_y]$
- $u_{k-1}$ 为上一控制（$k=0$ 时用上一周期控制 $u_{prev}$）
- $d_{i,k}^2=(p_x^k-o_{x,i}^k)^2+(p_y^k-o_{y,i}^k)^2$
- $r_{safe,i}=r_{robot}+r_{i}+margin$
- 障碍物预测：$o_{x,i}^k=o_{x,i}+k\,dt\,v_{x,i}$，$o_{y,i}^k=o_{y,i}+k\,dt\,v_{y,i}$

**终端代价**（代码中更强的目标牵引，见 [terminal cost 段](scripts/mpc_solver.py#L107-L111)）
$$
J_N = 5\,w_{p}\,\|p_N-g_{xy}\|^2 + 2\,w_{y}\,\sin^2(\psi_N-g_\psi)
$$

关键权重由 `config/mpc.yaml` 中参数控制：
- `w_goal_pos`, `w_goal_yaw`, `w_u`, `w_du`, `w_obs`

### 约束
- **动力学等式约束**
- **控制边界**：`v_min ≤ v ≤ v_max`，`|w| ≤ w_max`
- **障碍物软避障**：基于安全距离侵入量的惩罚（不做硬约束）

### 动态障碍物建模
- 每个障碍物用 $(x,y,v_x,v_y,r)$ 表示
- 在预测步长 $k$ 采用 **匀速外推**
- 只取最近 `obstacle_num` 个障碍物参与优化

### 求解器与热启动
- 使用 **CasADi** 生成 NLP
- 使用 **IPOPT** 求解
- 使用上一周期控制序列作为热启动

## 输入输出

### 订阅
- `/odom`（机器人位姿）
- `/move_base_simple/goal`（目标点，可通过参数修改）
- `/doa_obstacles`（障碍物列表，来自 `doa_to_obstacles.py`）

### 发布
- `/cmd_vel`（底盘控制）
- `/mpc/pred_path`（预测轨迹）
- `/mpc/status`（求解状态）

## 配置文件
默认参数位于 `config/mpc.yaml`，包括：
- 预测步长 `N`、采样周期 `dt`
- 速度/角速度上限
- 机器人半径、安全距离
- 代价权重
- 话题与 TF 帧

## 使用方式

### 方式一：仿真启动（Gazebo + 动态障碍物）
该方式会启动仿真环境、障碍物、RViz：

```bash
roslaunch mpc_nav mpc.launch cmd_vel_topic:=/cmd_vel
```

可选参数：
- `dynamic_scene`：选择动态场景（默认 1）
- `cmd_vel_topic`：规划器输出的控制话题

### 方式二：仅规划器启动（实机实验）
只启动 MPC 与障碍物适配器，不包含仿真环境：

```bash
roslaunch mpc_nav mpc_real.launch cmd_vel_topic:=/cmd_vel
```

如不需要 DOA 障碍物输入，可关闭适配器：

```bash
roslaunch mpc_nav mpc_real.launch cmd_vel_topic:=/cmd_vel use_doa_adapter:=false
```

## 常见注意事项
- **实机**请确保底盘驱动订阅 `/cmd_vel`，并且 `/odom` 正常发布
- 若控制抖动，可降低 `w_du` 或调小 `w_obs`
- 若跟踪不够快，可适当提高 `w_goal_pos`

## 目录结构
- `scripts/mpc_node.py`：ROS 节点入口
- `scripts/mpc_solver.py`：CasADi/IPOPT MPC 求解器
- `launch/mpc.launch`：仿真启动
- `launch/mpc_real.launch`：实机仅规划器启动
