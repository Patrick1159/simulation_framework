#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_exp_1.py — Experiment Scene 1: Crossing Dynamic Obstacles
              + ID Jitter + Observation/Tracking Layer

架构说明 (三层分离):
    ┌─────────────────────────────────────────────────────────┐
    │  Truth Layer     真实物理世界                             │
    │  o["x/y/vx/vy"]  → Gazebo set_model_state               │
    ├─────────────────────────────────────────────────────────┤
    │  Association Layer   ID 跳变 + 观测位置 + 速度估计         │
    │  IdJitter            → published_id                     │
    │  TrackEstimator      → observed position + velocity     │
    ├─────────────────────────────────────────────────────────┤
    │  Publish Layer   发给 MPC 的 ObstacleArray               │
    │  msg.id       = published_id  (跳变后)                  │
    │  msg.position = observed_position  (含噪声)             │
    │  msg.velocity = estimated_velocity  (按 id 历史差分)    │
    └─────────────────────────────────────────────────────────┘

ID 跳变模式 (~id_mode):
    "stable"          — 不跳变（基准对照组）
    "random_reassign" — 每隔 id_flip_period 秒全局随机重映射
    "random_flip"     — 每帧按 id_flip_prob 概率独立跳变

速度估计来源 (~velocity_source):
    "truth"       — 直接发真值速度（对照组，ID 跳变无影响）
    "per_id_diff" — 按 published_id 历史差分估计（默认）
    "per_id_ema"  — 差分 + EMA 滤波（更平滑，但跳变恢复更慢）

新生 ID 速度初始化 (~new_id_velocity_mode):
    "zero"    — 新 id 第一帧速度为 0（模拟 track birth，推荐）
    "hold"    — 保持上一次该 id 的缓存值（若无历史则归零）

可调 ROS 参数 (运动场景):
    ~radius / ~height / ~mass / ~x_offset / ~y_start / ~speed
    ~y_reset / ~start_delay / ~stagger_delay / ~rate
    ~model_prefix / ~frame_id

可调 ROS 参数 (观测层):
    ~velocity_source        default: "per_id_diff"
    ~new_id_velocity_mode   default: "zero"
    ~pos_noise_std          default: 0.0   (m)
    ~track_timeout          default: 5.0   (s)
    ~ema_alpha              default: 0.6

可调 ROS 参数 (ID 跳变):
    ~id_mode          default: "stable"
    ~id_flip_period   default: 2.0   (s)
    ~id_flip_prob     default: 0.1
    ~id_pool_size     default: 5
"""

import math
import random
import sys
import termios
import tty
import threading

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from mpc_nav.msg import Obstacle, ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray

from sdf import cylinder_sdf


# ──────────────────────────────────────────────────────────────────────────────
# 工具函数
# ──────────────────────────────────────────────────────────────────────────────

def quat_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def make_pose(x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> Pose:
    p = Pose()
    p.position = Point(x, y, z)
    p.orientation = quat_from_yaw(yaw)
    return p


# ──────────────────────────────────────────────────────────────────────────────
# Layer 1: ID 跳变器
# ──────────────────────────────────────────────────────────────────────────────

class IdJitter:
    """
    将障碍物「真实 ID」映射为「发布 ID」，实现可控的 ID 跳变。

    random_reassign: 每隔 flip_period 秒对所有 ID 做随机全排列洗牌。
                     保证每个真实 ID 都被唯一分配（无重复）。
    random_flip:     每帧每个 ID 以 flip_prob 独立地随机换为 pool 内某值。
                     不保证唯一性，可出现两个障碍物 ID 相同的情况。
    """

    VALID_MODES = ("stable", "random_reassign", "random_flip")

    def __init__(self, mode: str, n_obs: int, pool_size: int,
                 flip_period: float, flip_prob: float):
        if mode not in self.VALID_MODES:
            raise ValueError(f"[IdJitter] Unknown id_mode='{mode}'. "
                             f"Valid: {self.VALID_MODES}")
        self.mode        = mode
        self.n_obs       = n_obs
        self.pool        = list(range(pool_size))
        self.flip_period = flip_period
        self.flip_prob   = flip_prob
        self._mapping    = list(range(n_obs))
        self._last_flip  = 0.0

    def get_published_ids(self, now_sec: float) -> list:
        if self.mode == "stable":
            return list(range(self.n_obs))

        if self.mode == "random_reassign":
            if now_sec - self._last_flip >= self.flip_period:
                if len(self.pool) >= self.n_obs:
                    new_ids = random.sample(self.pool, self.n_obs)
                else:
                    rospy.logwarn_throttle(
                        10.0, "[IdJitter] pool_size (%d) < n_obs (%d): "
                              "sampling with replacement.", len(self.pool), self.n_obs)
                    new_ids = [random.choice(self.pool) for _ in range(self.n_obs)]
                rospy.loginfo("[IdJitter] random_reassign: %s → %s",
                              self._mapping, new_ids)
                self._mapping   = new_ids
                self._last_flip = now_sec
            return list(self._mapping)

        if self.mode == "random_flip":
            for i in range(self.n_obs):
                if random.random() < self.flip_prob:
                    old = self._mapping[i]
                    self._mapping[i] = random.choice(self.pool)
                    rospy.logdebug("[IdJitter] random_flip obs[%d]: %d → %d",
                                   i, old, self._mapping[i])
            return list(self._mapping)

        return list(range(self.n_obs))

    def summary(self) -> str:
        if self.mode == "stable":
            return "id_mode=stable"
        if self.mode == "random_reassign":
            return (f"id_mode=random_reassign | pool={self.pool} | "
                    f"period={self.flip_period:.2f}s")
        return (f"id_mode=random_flip | pool={self.pool} | "
                f"prob={self.flip_prob:.3f}")


# ──────────────────────────────────────────────────────────────────────────────
# Layer 2: 观测 + 跟踪估计器
# ──────────────────────────────────────────────────────────────────────────────

class TrackEstimator:
    """
    按 published_id 维护历史观测，模拟感知层 tracking 行为。

    这是 ID 不稳定影响 MPC 的关键传导层：
      - ID 稳定  → 历史连续 → 速度估计准确
      - ID 跳变  → 新 id 无历史 (newborn) → 速度归零
      - ID swap  → 继承错误历史 → 速度方向可能反向

    Attributes:
        velocity_source (str):  速度估计模式。
        new_id_vel_mode (str):  新生 ID 速度初始化方式。
        pos_noise_std (float):  位置观测噪声标准差 (m)。
        track_timeout (float):  track 缓存超时时间 (s)。
        ema_alpha (float):      EMA 平滑系数。
        _memory (dict):         published_id → 历史状态缓存。
    """

    VALID_VEL_SOURCES   = ("truth", "per_id_diff", "per_id_ema")
    VALID_NEWBORN_MODES = ("zero", "hold")

    def __init__(self, velocity_source: str, new_id_vel_mode: str,
                 pos_noise_std: float, track_timeout: float, ema_alpha: float):
        if velocity_source not in self.VALID_VEL_SOURCES:
            raise ValueError(
                f"[TrackEstimator] Unknown velocity_source='{velocity_source}'. "
                f"Valid: {self.VALID_VEL_SOURCES}")
        if new_id_vel_mode not in self.VALID_NEWBORN_MODES:
            raise ValueError(
                f"[TrackEstimator] Unknown new_id_velocity_mode='{new_id_vel_mode}'. "
                f"Valid: {self.VALID_NEWBORN_MODES}")

        self.velocity_source  = velocity_source
        self.new_id_vel_mode  = new_id_vel_mode
        self.pos_noise_std    = pos_noise_std
        self.track_timeout    = track_timeout
        self.ema_alpha        = ema_alpha

        # key: published_id (int)
        # value: {"x", "y", "t", "vx", "vy", "initialized": bool}
        self._memory: dict = {}

        rospy.loginfo(
            "[TrackEstimator] vel_source='%s' | newborn='%s' | "
            "noise=%.3fm | timeout=%.1fs | ema_alpha=%.2f",
            velocity_source, new_id_vel_mode,
            pos_noise_std, track_timeout, ema_alpha,
        )

    # ── 内部工具 ──────────────────────────────────────────────────────────────

    def _purge_stale(self, now_sec: float):
        """清除超时未更新的 track（模拟 track death）。"""
        stale = [k for k, v in self._memory.items()
                 if now_sec - v["t"] > self.track_timeout]
        for k in stale:
            rospy.loginfo("[TrackEstimator] track id=%d timed out, purged.", k)
            del self._memory[k]

    def _update_mem(self, pid: int, x: float, y: float,
                    vx: float, vy: float, t: float):
        if pid not in self._memory:
            self._memory[pid] = {"initialized": False}
        m = self._memory[pid]
        m["x"] = x;  m["y"] = y;  m["t"] = t
        m["vx"] = vx; m["vy"] = vy
        m["initialized"] = True

    # ── 对外接口 ──────────────────────────────────────────────────────────────

    def observe(self, published_id: int,
                true_x: float, true_y: float,
                true_vx: float, true_vy: float,
                now_sec: float,
                is_waiting: bool):
        """
        根据真值和 published_id，生成观测层的位置和速度估计。

        Returns:
            obs_x, obs_y (float):    发布位置（含噪声）
            obs_vx, obs_vy (float):  发布速度（按估计模式）
            is_newborn (bool):       本帧是否为新生 track
        """
        self._purge_stale(now_sec)

        # 等待 / 暂停：发布真值位置，速度清零
        if is_waiting:
            return true_x, true_y, 0.0, 0.0, False

        # 位置：加测量噪声
        if self.pos_noise_std > 0.0:
            obs_x = true_x + random.gauss(0.0, self.pos_noise_std)
            obs_y = true_y + random.gauss(0.0, self.pos_noise_std)
        else:
            obs_x, obs_y = true_x, true_y

        # 速度来源：truth 直接返回真值
        if self.velocity_source == "truth":
            self._update_mem(published_id, obs_x, obs_y, true_vx, true_vy, now_sec)
            return obs_x, obs_y, true_vx, true_vy, False

        # 速度来源：per_id_diff / per_id_ema（按 published_id 历史差分）
        mem = self._memory.get(published_id, None)

        if mem is None:
            # ── 新生 ID：首次出现，没有历史 ───────────────────────────────────
            self._memory[published_id] = {
                "x": obs_x, "y": obs_y, "t": now_sec,
                "vx": 0.0,  "vy": 0.0, "initialized": False,
            }
            # "hold" 模式此处也只能给零（无历史可继承）
            return obs_x, obs_y, 0.0, 0.0, True

        # ── 有历史：差分计算原始速度 ──────────────────────────────────────────
        dt     = max(1e-3, now_sec - mem["t"])
        raw_vx = (obs_x - mem["x"]) / dt
        raw_vy = (obs_y - mem["y"]) / dt

        if self.velocity_source == "per_id_ema" and mem["initialized"]:
            a  = self.ema_alpha
            vx = a * raw_vx + (1.0 - a) * mem["vx"]
            vy = a * raw_vy + (1.0 - a) * mem["vy"]
        else:
            vx, vy = raw_vx, raw_vy

        self._update_mem(published_id, obs_x, obs_y, vx, vy, now_sec)
        return obs_x, obs_y, vx, vy, False

    def summary(self) -> str:
        return (f"vel_source={self.velocity_source} | newborn={self.new_id_vel_mode} | "
                f"noise={self.pos_noise_std:.3f}m | timeout={self.track_timeout:.1f}s")


# ──────────────────────────────────────────────────────────────────────────────
# 主类
# ──────────────────────────────────────────────────────────────────────────────

class CrossingObstacleScene:
    """
    两个以恒定速度交叉运动的动态障碍物场景节点。

    障碍物 A: 左前 → 右后 (yaw = -π/4)
    障碍物 B: 右前 → 左后 (yaw = -3π/4)
    两轨迹在机器人正前方 (0, y_cross) 处相交，形成交叠遮挡。
    """

    _YAW_RIGHT_BACK = -math.pi / 4
    _YAW_LEFT_BACK  = -3.0 * math.pi / 4

    def __init__(self):
        rospy.init_node("sim_exp_1_crossing_obstacles")

        # ── 运动场景参数 ──────────────────────────────────────────────────────
        self.model_prefix = rospy.get_param("~model_prefix", "exp1_obs")
        self.world_frame  = rospy.get_param("~frame_id",     "world")

        self.radius = float(rospy.get_param("~radius",  0.3))
        self.height = float(rospy.get_param("~height",  1.0))
        self.mass   = float(rospy.get_param("~mass",    5.0))

        self.x_offset    = float(rospy.get_param("~x_offset",    2.0))
        self.y_start     = float(rospy.get_param("~y_start",     4.0))
        self.speed       = float(rospy.get_param("~speed",       0.5))
        self.y_reset     = float(rospy.get_param("~y_reset",    -2.0))
        self.start_delay = float(rospy.get_param("~start_delay",  3.0))
        self.rate_hz     = float(rospy.get_param("~rate",        30.0))

        _default_stagger = math.sqrt(2.0) * self.x_offset / self.speed + 2.0
        self.stagger_delay = float(rospy.get_param("~stagger_delay", _default_stagger))

        # ── Layer 1: ID 跳变器 ────────────────────────────────────────────────
        _N_OBS = 2
        self.id_jitter = IdJitter(
            mode        = str(rospy.get_param("~id_mode",        "stable")),
            n_obs       = _N_OBS,
            pool_size   = int(rospy.get_param("~id_pool_size",   5)),
            flip_period = float(rospy.get_param("~id_flip_period", 2.0)),
            flip_prob   = float(rospy.get_param("~id_flip_prob",  0.1)),
        )

        # ── Layer 2: 观测 / 跟踪估计器 ───────────────────────────────────────
        self.track_estimator = TrackEstimator(
            velocity_source = str(rospy.get_param("~velocity_source",      "per_id_diff")),
            new_id_vel_mode = str(rospy.get_param("~new_id_velocity_mode", "zero")),
            pos_noise_std   = float(rospy.get_param("~pos_noise_std",      0.0)),
            track_timeout   = float(rospy.get_param("~track_timeout",      5.0)),
            ema_alpha       = float(rospy.get_param("~ema_alpha",          0.6)),
        )

        # ── 状态控制 ──────────────────────────────────────────────────────────
        self.is_paused = False

        # ── 固定配置 ──────────────────────────────────────────────────────────
        self._configs = [
            {
                "name": f"{self.model_prefix}_A",
                "id":   0,
                "x0":  -self.x_offset,
                "y0":   self.y_start,
                "yaw":  self._YAW_RIGHT_BACK,
                "material": "Gazebo/Blue",
                "label": "A (left-front → right-back)",
            },
            {
                "name": f"{self.model_prefix}_B",
                "id":   1,
                "x0":   self.x_offset,
                "y0":   self.y_start,
                "yaw":  self._YAW_LEFT_BACK,
                "material": "Gazebo/Yellow",
                "label": "B (right-front → left-back)",
            },
        ]
        self.obstacles = []

        # ── ROS 发布者 ────────────────────────────────────────────────────────
        self.obs_pub    = rospy.Publisher("obstacles",        ObstacleArray, queue_size=1)
        self.marker_pub = rospy.Publisher("obstacles_marker", MarkerArray,   queue_size=1)

        # ── Gazebo 服务 ───────────────────────────────────────────────────────
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.spawn_srv     = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # ── 启动日志 ──────────────────────────────────────────────────────────
        t_cross = self.x_offset / (math.cos(math.pi / 4) * self.speed)
        y_cross = self.y_start + math.sin(self._YAW_RIGHT_BACK) * self.speed * t_cross
        rospy.loginfo(
            "[sim_exp_1] Scene initialised.\n"
            "  Obs A: (%.2f, %.2f) → right-back | Obs B: (%.2f, %.2f) → left-back\n"
            "  Speed: %.2f m/s | Crossing ≈ (0.00, %.2f) at t≈%.1f s\n"
            "  Stagger delay: %.1f s\n"
            "  ID jitter:  %s\n"
            "  Obs layer:  %s",
            -self.x_offset, self.y_start,
             self.x_offset, self.y_start,
            self.speed, y_cross, t_cross,
            self.stagger_delay,
            self.id_jitter.summary(),
            self.track_estimator.summary(),
        )

    # ──────────────────────────────────────────────────────────────────────────
    # 状态管理
    # ──────────────────────────────────────────────────────────────────────────

    def _make_state(self, cfg: dict, move_time=None) -> dict:
        yaw = cfg["yaw"]
        return {
            "name":      cfg["name"],
            "id":        cfg["id"],
            "x":         cfg["x0"],
            "y":         cfg["y0"],
            "yaw":       yaw,
            "vx":        self.speed * math.cos(yaw),
            "vy":        self.speed * math.sin(yaw),
            "move_time": move_time if move_time is not None else rospy.Time(0),
        }

    # ──────────────────────────────────────────────────────────────────────────
    # Truth Layer: Gazebo 同步
    # ──────────────────────────────────────────────────────────────────────────

    def spawn_obstacles(self):
        for cfg in self._configs:
            sdf = cylinder_sdf(
                cfg["name"],
                radius=self.radius, height=self.height, mass=self.mass,
                material_name=cfg.get("material"),
            )
            pose = make_pose(cfg["x0"], cfg["y0"],
                             z=self.height * 0.5, yaw=cfg["yaw"])
            req = SpawnModelRequest()
            req.model_name      = cfg["name"]
            req.model_xml       = sdf
            req.robot_namespace = ""
            req.initial_pose    = pose
            req.reference_frame = self.world_frame
            try:
                resp = self.spawn_srv(req)
                if not resp.success:
                    rospy.logwarn("[sim_exp_1] Spawn failed for %s: %s",
                                  cfg["name"], resp.status_message)
                else:
                    rospy.loginfo("[sim_exp_1] Spawned %s: %s",
                                  cfg["name"], cfg["label"])
            except Exception as exc:
                rospy.logwarn("[sim_exp_1] Spawn exception for %s: %s",
                              cfg["name"], str(exc))
            self.obstacles.append(self._make_state(cfg))

    def _teleport(self, o: dict, moving: bool = True):
        """将真实物理状态同步到 Gazebo（始终使用真值，与观测层无关）。"""
        st = ModelState()
        st.model_name      = o["name"]
        st.reference_frame = self.world_frame
        st.pose  = make_pose(o["x"], o["y"], z=self.height * 0.5, yaw=o["yaw"])
        st.twist = Twist()
        if moving:
            st.twist.linear.x = o["vx"]
            st.twist.linear.y = o["vy"]
        try:
            self.set_state_srv(st)
        except Exception as exc:
            rospy.logwarn_throttle(2.0,
                "[sim_exp_1] set_model_state failed for %s: %s", o["name"], str(exc))

    # ──────────────────────────────────────────────────────────────────────────
    # Truth Layer: 主更新步
    # ──────────────────────────────────────────────────────────────────────────

    def step(self, dt: float):
        now = rospy.Time.now()

        if self.is_paused:
            for o in self.obstacles:
                self._teleport(o, moving=False)
            self._publish()
            return

        for idx, o in enumerate(self.obstacles):
            if now < o["move_time"]:
                rospy.loginfo_throttle(
                    3.0, "[sim_exp_1] %s waiting (%.1f s remaining)",
                    o["name"], (o["move_time"] - now).to_sec())
                self._teleport(o, moving=False)
                continue

            o["x"] += o["vx"] * dt
            o["y"] += o["vy"] * dt

            if o["y"] < self.y_reset:
                delay = 0.0 if idx == 0 else self.stagger_delay
                new_move_time = now + rospy.Duration(delay)
                rospy.loginfo("[sim_exp_1] %s reset (departs in %.1f s)",
                              o["name"], delay)
                self.obstacles[idx] = self._make_state(self._configs[idx],
                                                       move_time=new_move_time)
                o = self.obstacles[idx]

            self._teleport(o, moving=True)

        self._publish()

    # ──────────────────────────────────────────────────────────────────────────
    # Association + Publish Layer
    # ──────────────────────────────────────────────────────────────────────────

    def _publish(self):
        now     = rospy.Time.now()
        now_sec = now.to_sec()

        # Layer 1: 获取本帧各障碍物的发布 ID（暂停时不推进跳变计时器）
        if not self.is_paused:
            pub_ids = self.id_jitter.get_published_ids(now_sec)
        else:
            pub_ids = list(self.id_jitter._mapping)

        obs_array    = ObstacleArray()
        obs_array.header.frame_id = self.world_frame
        obs_array.header.stamp    = now
        marker_array = MarkerArray()

        for idx, o in enumerate(self.obstacles):
            is_waiting   = (now < o["move_time"]) or self.is_paused
            published_id = pub_ids[idx]
            id_jittered  = (published_id != o["id"])

            # Layer 2: 观测位置 + 按 published_id 估计速度
            obs_x, obs_y, obs_vx, obs_vy, is_newborn = \
                self.track_estimator.observe(
                    published_id = published_id,
                    true_x       = o["x"],
                    true_y       = o["y"],
                    true_vx      = o["vx"],
                    true_vy      = o["vy"],
                    now_sec      = now_sec,
                    is_waiting   = is_waiting,
                )

            # Layer 3: 构建并发布 ObstacleArray 消息
            msg          = Obstacle()
            msg.id       = published_id          # ← 跳变后的 ID
            msg.position = Point(obs_x, obs_y, 0.0)
            msg.velocity = Vector3(obs_vx, obs_vy, 0.0)
            msg.radius   = self.radius
            msg.type     = 1
            obs_array.obstacles.append(msg)

            # RViz 标记
            self._make_markers(
                marker_array, idx, o,
                obs_x, obs_y, obs_vx, obs_vy,
                published_id, id_jittered,
                is_waiting, is_newborn, now,
            )

        self.obs_pub.publish(obs_array)
        self.marker_pub.publish(marker_array)

    def _make_markers(self, marker_array, idx, o,
                      obs_x, obs_y, obs_vx, obs_vy,
                      published_id, id_jittered,
                      is_waiting, is_newborn, stamp):
        """
        生成三类 RViz 标记，便于直观对比真值与观测层差异：
          - 圆柱体：颜色编码状态（正常/跳变/等待）
          - 真值速度箭头（黄色）：真实运动方向
          - 观测速度箭头（白色/橙色）：MPC 实际收到的估计速度
          - 文字标签：显示真实 → 发布 ID 及当前状态
        """

        # ── 圆柱体 ─────────────────────────────────────────────────────────
        cyl = Marker()
        cyl.header.frame_id = self.world_frame
        cyl.header.stamp    = stamp
        cyl.ns     = "exp1_cylinder"
        cyl.id     = idx
        cyl.type   = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.pose   = make_pose(o["x"], o["y"], z=self.height * 0.5, yaw=o["yaw"])
        cyl.scale.x = self.radius * 2
        cyl.scale.y = self.radius * 2
        cyl.scale.z = self.height
        cyl.color.a = 0.4 if is_waiting else 0.85
        if id_jittered and not is_waiting:
            cyl.color.r, cyl.color.g, cyl.color.b = 1.0, 0.2, 0.2   # 红：ID 跳变
        elif idx == 0:
            cyl.color.r, cyl.color.g, cyl.color.b = 0.2, 0.4, 1.0   # 蓝：Obs A
        else:
            cyl.color.r, cyl.color.g, cyl.color.b = 0.1, 0.85, 0.85 # 青：Obs B
        marker_array.markers.append(cyl)

        # ── 真值速度箭头（黄色） ───────────────────────────────────────────
        true_arrow = Marker()
        true_arrow.header.frame_id = self.world_frame
        true_arrow.header.stamp    = stamp
        true_arrow.ns     = "exp1_vel_truth"
        true_arrow.id     = idx + 100
        true_arrow.type   = Marker.ARROW
        true_arrow.action = Marker.ADD
        true_arrow.pose   = make_pose(o["x"], o["y"],
                                      z=self.height * 0.5 + 0.05, yaw=o["yaw"])
        s = 0.0 if self.is_paused else self.speed * 2.0
        true_arrow.scale.x = s
        true_arrow.scale.y = 0.07 if s > 0 else 0.0
        true_arrow.scale.z = 0.07 if s > 0 else 0.0
        true_arrow.color.a = 0.9
        true_arrow.color.r, true_arrow.color.g, true_arrow.color.b = 1.0, 0.9, 0.0
        marker_array.markers.append(true_arrow)

        # ── 观测速度箭头（白色/橙色，表示 MPC 实际收到的估计速度） ──────────
        if not is_waiting and not self.is_paused:
            obs_spd = math.hypot(obs_vx, obs_vy)
            obs_yaw = math.atan2(obs_vy, obs_vx)
            obs_arrow = Marker()
            obs_arrow.header.frame_id = self.world_frame
            obs_arrow.header.stamp    = stamp
            obs_arrow.ns     = "exp1_vel_obs"
            obs_arrow.id     = idx + 200
            obs_arrow.type   = Marker.ARROW
            obs_arrow.action = Marker.ADD
            obs_arrow.pose   = make_pose(obs_x, obs_y,
                                         z=self.height * 0.5 + 0.15, yaw=obs_yaw)
            obs_arrow.scale.x = obs_spd * 2.0
            obs_arrow.scale.y = 0.07 if obs_spd > 1e-3 else 0.0
            obs_arrow.scale.z = 0.07 if obs_spd > 1e-3 else 0.0
            obs_arrow.color.a = 0.95
            if is_newborn:
                obs_arrow.color.r, obs_arrow.color.g, obs_arrow.color.b = 1.0, 0.5, 0.0  # 橙：newborn
            else:
                obs_arrow.color.r, obs_arrow.color.g, obs_arrow.color.b = 1.0, 1.0, 1.0  # 白：正常估计
            marker_array.markers.append(obs_arrow)

        # ── 文字标签 ─────────────────────────────────────────────────────────
        text = Marker()
        text.header.frame_id = self.world_frame
        text.header.stamp    = stamp
        text.ns     = "exp1_label"
        text.id     = idx + 300
        text.type   = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = o["x"]
        text.pose.position.y = o["y"]
        text.pose.position.z = self.height + 0.3
        text.pose.orientation = quat_from_yaw(0.0)
        text.scale.z = 0.28
        text.color.a = 1.0

        true_label = 'A' if idx == 0 else 'B'
        if self.is_paused:
            status = " (PAUSED)"
        elif is_waiting:
            status = " (waiting)"
        elif is_newborn:
            status = " (newborn!)"
        elif id_jittered:
            status = " [ID!]"
        else:
            status = ""

        if id_jittered and not is_waiting:
            text.text = f"Obs {true_label} {o['id']}→{published_id}{status}"
            text.color.r, text.color.g, text.color.b = 1.0, 0.3, 0.3
        else:
            text.text = f"Obs {true_label} id:{published_id}{status}"
            text.color.r = text.color.g = text.color.b = 1.0
        marker_array.markers.append(text)

    # ──────────────────────────────────────────────────────────────────────────
    # 主循环
    # ──────────────────────────────────────────────────────────────────────────

    def run(self):
        self.spawn_obstacles()

        def keyboard_listener():
            rospy.loginfo("[sim_exp_1] Press SPACE to pause/resume.")
            while not rospy.is_shutdown():
                fd  = sys.stdin.fileno()
                old = termios.tcgetattr(fd)
                try:
                    tty.setraw(fd)
                    ch = sys.stdin.read(1)
                    if ch == ' ':
                        self.is_paused = not self.is_paused
                        rospy.loginfo("[sim_exp_1] %s",
                                      "PAUSED" if self.is_paused else "RESUMED")
                    elif ch == '\x03':
                        rospy.signal_shutdown("Ctrl+C")
                        break
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old)

        threading.Thread(target=keyboard_listener, daemon=True).start()

        if self.start_delay > 0.0:
            rospy.loginfo("[sim_exp_1] Waiting %.1f s ...", self.start_delay)
            t0 = rospy.Time.now()
            while (rospy.Time.now() - t0).to_sec() < self.start_delay \
                    and not rospy.is_shutdown():
                rospy.sleep(0.1)

        t0 = rospy.Time.now()
        self.obstacles[0]["move_time"] = t0
        self.obstacles[1]["move_time"] = t0 + rospy.Duration(self.stagger_delay)
        rospy.loginfo(
            "[sim_exp_1] Started. A departs NOW | B departs in %.1f s\n"
            "  ID jitter: %s\n"
            "  Obs layer: %s",
            self.stagger_delay,
            self.id_jitter.summary(),
            self.track_estimator.summary(),
        )

        rate = rospy.Rate(self.rate_hz)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt  = max(1.0 / (5.0 * self.rate_hz), min(0.2, (now - last).to_sec()))
            last = now
            if self.is_paused:
                for o in self.obstacles:
                    o["move_time"] += rospy.Duration(dt)
            self.step(dt)
            rate.sleep()


if __name__ == "__main__":
    CrossingObstacleScene().run()