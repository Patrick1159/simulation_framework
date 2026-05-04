#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive experiment TUI for the MPC navigation stack.

Pinned panels:
  - Robot pose in `odom` (from /odom)
  - MPC status (from /mpc/status) + active goal
  - DOA obstacles summary (from /doa_obstacles)

Commands:
  goto X Y [YAW_DEG]   publish a 2D nav goal (matches RViz "2D Nav Goal")
  stop                 freeze the robot at its current pose
  r | reset            same as stop, then prompt for a fresh goal
  q | quit | exit      exit the TUI
  Ctrl+C               also exits

The TUI is a developer tool — it never publishes /cmd_vel directly. Stop
and reset both work by publishing the robot's current odom pose as the
new goal, letting the MPC decelerate the robot naturally without fighting
for the velocity topic.

Dependencies (install with pip in the user's env):
  rich              -- panels / tables / colors
  prompt_toolkit    -- non-blocking command input with history
"""

import math
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations as tft

from mpc_nav.msg import MPCStatus, ObstacleArray

try:
    from rich.console import Console
    from rich.live import Live
    from rich.layout import Layout
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
except ImportError:
    sys.stderr.write(
        "ERROR: rich not installed. Run: pip install rich prompt_toolkit\n"
    )
    sys.exit(1)

try:
    from prompt_toolkit import PromptSession
    from prompt_toolkit.history import InMemoryHistory
    from prompt_toolkit.patch_stdout import patch_stdout
except ImportError:
    sys.stderr.write(
        "ERROR: prompt_toolkit not installed. Run: pip install rich prompt_toolkit\n"
    )
    sys.exit(1)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def quat_to_yaw(q) -> float:
    """Extract yaw (rad) from a geometry_msgs/Quaternion."""
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def yaw_to_quat(yaw: float):
    """Build a (x, y, z, w) tuple from a yaw angle in radians."""
    return tft.quaternion_from_euler(0.0, 0.0, yaw)


# ---------------------------------------------------------------------------
# Shared state — written by ROS callbacks, read by the renderer
# ---------------------------------------------------------------------------
@dataclass
class ObstacleRow:
    oid: int
    x: float
    y: float
    speed: float
    distance: float


@dataclass
class State:
    # Robot pose
    have_odom: bool = False
    rx: float = 0.0
    ry: float = 0.0
    ryaw: float = 0.0
    rv: float = 0.0
    rw: float = 0.0
    odom_stamp: float = 0.0

    # Goal currently believed to be active (echoed from our own publishes)
    goal_x: Optional[float] = None
    goal_y: Optional[float] = None
    goal_yaw: Optional[float] = None
    goal_stamp: float = 0.0

    # MPC status
    mpc_have: bool = False
    mpc_success: bool = False
    mpc_solve_ms: float = 0.0
    mpc_cost: float = 0.0
    mpc_text: str = ""
    mpc_stamp: float = 0.0

    # Obstacles
    obstacles: List[ObstacleRow] = field(default_factory=list)
    obs_stamp: float = 0.0
    obs_dt_samples: deque = field(default_factory=lambda: deque(maxlen=10))

    # User feedback (last command echo, errors)
    last_message: str = ""
    last_message_stamp: float = 0.0

    lock: threading.Lock = field(default_factory=threading.Lock)


# ---------------------------------------------------------------------------
# ROS I/O
# ---------------------------------------------------------------------------
class RosBridge:
    def __init__(self, state: State):
        self.state = state

        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.obstacles_topic = rospy.get_param("~obstacles_topic", "/doa_obstacles")
        self.mpc_status_topic = rospy.get_param("~mpc_status_topic", "/mpc/status")
        self.frame_id = rospy.get_param("~frame_id", "odom")

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)

        rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber(self.obstacles_topic, ObstacleArray, self._on_obstacles, queue_size=1)
        rospy.Subscriber(self.mpc_status_topic, MPCStatus, self._on_mpc_status, queue_size=1)

    def _on_odom(self, msg: Odometry):
        with self.state.lock:
            self.state.have_odom = True
            self.state.rx = msg.pose.pose.position.x
            self.state.ry = msg.pose.pose.position.y
            self.state.ryaw = quat_to_yaw(msg.pose.pose.orientation)
            # Twist is in the child_frame, often base_footprint.
            self.state.rv = msg.twist.twist.linear.x
            self.state.rw = msg.twist.twist.angular.z
            self.state.odom_stamp = time.time()

    def _on_obstacles(self, msg: ObstacleArray):
        # Need robot pose to compute distances; if odom missing yet, skip dists.
        with self.state.lock:
            rx = self.state.rx
            ry = self.state.ry
            have_odom = self.state.have_odom

        rows: List[ObstacleRow] = []
        for o in msg.obstacles:
            speed = math.hypot(o.velocity.x, o.velocity.y)
            if have_odom:
                dist = math.hypot(o.position.x - rx, o.position.y - ry)
            else:
                dist = float("nan")
            rows.append(ObstacleRow(
                oid=int(o.id),
                x=float(o.position.x),
                y=float(o.position.y),
                speed=speed,
                distance=dist,
            ))
        rows.sort(key=lambda r: (math.isnan(r.distance), r.distance))

        now = time.time()
        with self.state.lock:
            if self.state.obs_stamp > 0.0:
                self.state.obs_dt_samples.append(now - self.state.obs_stamp)
            self.state.obs_stamp = now
            self.state.obstacles = rows

    def _on_mpc_status(self, msg: MPCStatus):
        with self.state.lock:
            self.state.mpc_have = True
            self.state.mpc_success = bool(msg.success)
            self.state.mpc_solve_ms = float(msg.solve_time_ms)
            self.state.mpc_cost = float(msg.cost)
            self.state.mpc_text = msg.status_text or ""
            self.state.mpc_stamp = time.time()

    def publish_goal(self, x: float, y: float, yaw: float):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_pub.publish(msg)
        with self.state.lock:
            self.state.goal_x = x
            self.state.goal_y = y
            self.state.goal_yaw = yaw
            self.state.goal_stamp = time.time()


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def _stale(stamp: float, threshold: float) -> bool:
    """A timestamp is "stale" if it never updated or is older than threshold."""
    return stamp <= 0.0 or (time.time() - stamp) > threshold


def render(state: State, obs_topic: str) -> Layout:
    with state.lock:
        s = State(
            have_odom=state.have_odom,
            rx=state.rx, ry=state.ry, ryaw=state.ryaw,
            rv=state.rv, rw=state.rw, odom_stamp=state.odom_stamp,
            goal_x=state.goal_x, goal_y=state.goal_y, goal_yaw=state.goal_yaw,
            goal_stamp=state.goal_stamp,
            mpc_have=state.mpc_have, mpc_success=state.mpc_success,
            mpc_solve_ms=state.mpc_solve_ms, mpc_cost=state.mpc_cost,
            mpc_text=state.mpc_text, mpc_stamp=state.mpc_stamp,
            obstacles=list(state.obstacles), obs_stamp=state.obs_stamp,
            last_message=state.last_message,
            last_message_stamp=state.last_message_stamp,
        )
        obs_dt_samples = list(state.obs_dt_samples)

    layout = Layout()
    layout.split(
        Layout(name="robot", size=4),
        Layout(name="mpc", size=4),
        Layout(name="obstacles", ratio=1),
        Layout(name="footer", size=3),
    )

    # ---- Robot panel ----
    if not s.have_odom:
        robot_text = Text("waiting for /odom...", style="yellow")
    else:
        stale = _stale(s.odom_stamp, 1.0)
        color = "red" if stale else "green"
        robot_text = Text.assemble(
            ("x: ", "bold"), (f"{s.rx:+.3f} ", color),
            ("y: ", "bold"), (f"{s.ry:+.3f} ", color),
            ("yaw: ", "bold"),
            (f"{math.degrees(s.ryaw):+.1f}°  ", color),
            ("v: ", "bold"), (f"{s.rv:+.2f} m/s  ", color),
            ("ω: ", "bold"), (f"{s.rw:+.2f} rad/s", color),
        )
    layout["robot"].update(Panel(robot_text, title="Robot (odom)", border_style="cyan"))

    # ---- MPC panel ----
    mpc_lines = []
    if s.goal_x is None:
        mpc_lines.append(Text("Goal: <none — type `goto X Y [YAW_DEG]`>", style="dim"))
    else:
        reach = (math.hypot(s.goal_x - s.rx, s.goal_y - s.ry)
                 if s.have_odom else float("nan"))
        mpc_lines.append(Text.assemble(
            ("Goal: ", "bold"),
            (f"x={s.goal_x:+.2f}  y={s.goal_y:+.2f}  yaw="
             f"{math.degrees(s.goal_yaw):+.1f}°  ",
             "white"),
            ("dist: ", "bold"),
            (f"{reach:.2f} m" if not math.isnan(reach) else "?", "white"),
        ))
    if not s.mpc_have:
        mpc_lines.append(Text("MPC: waiting for /mpc/status...", style="yellow"))
    else:
        stale = _stale(s.mpc_stamp, 1.0)
        color = "red" if stale else ("green" if s.mpc_success else "yellow")
        mpc_lines.append(Text.assemble(
            ("MPC: ", "bold"),
            (f"{s.mpc_text:<6} ", color),
            ("solve: ", "bold"), (f"{s.mpc_solve_ms:5.1f} ms  ", color),
            ("cost: ", "bold"), (f"{s.mpc_cost:8.2f}", color),
        ))
    layout["mpc"].update(Panel(Text("\n").join(mpc_lines), title="MPC", border_style="magenta"))

    # ---- Obstacles panel ----
    table = Table(show_header=True, header_style="bold", expand=True, pad_edge=False)
    table.add_column("ID", justify="right", width=6)
    table.add_column("pos (odom)", justify="left")
    table.add_column("|v| m/s", justify="right", width=8)
    table.add_column("dist m", justify="right", width=8)

    for row in s.obstacles[:10]:
        dist_str = f"{row.distance:.2f}" if not math.isnan(row.distance) else "?"
        table.add_row(
            str(row.oid),
            f"({row.x:+.2f}, {row.y:+.2f})",
            f"{row.speed:.2f}",
            dist_str,
        )

    if obs_dt_samples:
        avg_dt = sum(obs_dt_samples) / len(obs_dt_samples)
        rate_str = f"{(1.0 / avg_dt):.1f} Hz" if avg_dt > 1e-6 else "? Hz"
    else:
        rate_str = "?"
    # Build title as a Text object so brackets in `obs_topic` (e.g.
    # "/doa_obstacles") are not interpreted as rich markup tags.
    obs_title = Text(
        f"Obstacles  count: {len(s.obstacles)}   [{obs_topic}  {rate_str}]",
        style="bold",
    )
    if not s.obstacles:
        obs_body = Text("no obstacles received yet", style="dim")
    else:
        obs_body = table
    layout["obstacles"].update(Panel(obs_body, title=obs_title, border_style="blue"))

    # ---- Footer (commands + last message) ----
    cmd_help = Text(
        "Commands: goto X Y [YAW_DEG]   stop   r/reset   q/quit  (Ctrl+C exits)",
        style="dim",
    )
    if s.last_message and (time.time() - s.last_message_stamp) < 5.0:
        msg_text = Text(s.last_message, style="bold green")
        body = Text("\n").join([cmd_help, msg_text])
    else:
        body = cmd_help
    layout["footer"].update(Panel(body, border_style="white"))

    return layout


# ---------------------------------------------------------------------------
# Command loop
# ---------------------------------------------------------------------------
class CommandHandler:
    def __init__(self, state: State, bridge: RosBridge):
        self.state = state
        self.bridge = bridge

    def _set_message(self, text: str):
        with self.state.lock:
            self.state.last_message = text
            self.state.last_message_stamp = time.time()

    def _parse_goto(self, args: List[str]) -> Optional[tuple]:
        """Returns (x, y, yaw_rad) or None on parse error (already messaged)."""
        if len(args) < 2 or len(args) > 3:
            self._set_message("usage: goto X Y [YAW_DEG]")
            return None
        try:
            x = float(args[0])
            y = float(args[1])
            yaw_deg = float(args[2]) if len(args) == 3 else 0.0
        except ValueError:
            self._set_message(f"goto: cannot parse {' '.join(args)!r} as numbers")
            return None
        return x, y, math.radians(yaw_deg)

    def handle(self, line: str) -> bool:
        """Returns False to request exit."""
        line = line.strip()
        if not line:
            return True
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("q", "quit", "exit"):
            return False

        if cmd == "goto":
            parsed = self._parse_goto(parts[1:])
            if parsed is None:
                return True
            x, y, yaw = parsed
            self.bridge.publish_goal(x, y, yaw)
            self._set_message(
                f"→ goal published: x={x:+.2f} y={y:+.2f} "
                f"yaw={math.degrees(yaw):+.1f}°"
            )
            return True

        if cmd == "stop":
            self._stop_at_current()
            return True

        if cmd in ("r", "reset"):
            self._stop_at_current()
            self._set_message(
                "goal cleared — robot held at current pose. "
                "type `goto X Y [YAW_DEG]` to plan a new one."
            )
            return True

        self._set_message(f"unknown command: {cmd!r}")
        return True

    def _stop_at_current(self):
        """Re-publish the robot's current odom pose as the goal so the MPC
        decelerates the robot naturally — we never touch /cmd_vel."""
        with self.state.lock:
            if not self.state.have_odom:
                self.state.last_message = "stop: no /odom yet"
                self.state.last_message_stamp = time.time()
                return
            x, y, yaw = self.state.rx, self.state.ry, self.state.ryaw
        self.bridge.publish_goal(x, y, yaw)
        self._set_message(f"◼ stopped at current pose ({x:+.2f}, {y:+.2f})")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    rospy.init_node("mpc_exp_tui", anonymous=True, disable_signals=True)
    state = State()
    bridge = RosBridge(state)
    handler = CommandHandler(state, bridge)

    console = Console()
    obs_topic_label = bridge.obstacles_topic

    # Background thread: refresh the live display at ~10 Hz independent of
    # ROS message timing, so the TUI stays alive even if topics go quiet.
    stop_evt = threading.Event()

    def render_loop(live: Live):
        while not stop_evt.is_set():
            try:
                live.update(render(state, obs_topic_label))
            except Exception as e:  # noqa: BLE001
                # Never let a render hiccup take down the TUI.
                rospy.logwarn_throttle(5.0, f"[exp_tui] render error: {e}")
            stop_evt.wait(0.1)

    session = PromptSession(history=InMemoryHistory())

    with Live(render(state, obs_topic_label), console=console, refresh_per_second=10,
              screen=True) as live:
        render_thread = threading.Thread(target=render_loop, args=(live,), daemon=True)
        render_thread.start()
        try:
            while not rospy.is_shutdown():
                with patch_stdout():
                    try:
                        line = session.prompt("> ")
                    except EOFError:
                        break  # Ctrl-D
                    except KeyboardInterrupt:
                        break  # Ctrl-C
                if not handler.handle(line):
                    break
        finally:
            stop_evt.set()
            render_thread.join(timeout=0.5)

    console.print("[dim]exp_tui shutting down.[/dim]")
    rospy.signal_shutdown("exp_tui exit")


if __name__ == "__main__":
    main()
