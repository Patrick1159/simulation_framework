#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import select
import termios
import tty
import rospy
from geometry_msgs.msg import Twist


HELP = """
Keyboard Teleop (ROS1 /cmd_vel)
--------------------------------
W/S : increase/decrease linear x
A/D : increase/decrease angular z
Space: STOP (set both to 0)
Q   : quit

Tips:
- Hold keys to ramp up speed
- Release keys: keeps last command (like cruise). Press Space to stop.
"""

class KeyboardTeleop:
    def __init__(self):
        rospy.init_node("keyboard_teleop_cmdvel")

        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.lin_step = float(rospy.get_param("~lin_step", 0.05))
        self.ang_step = float(rospy.get_param("~ang_step", 0.10))
        self.lin_max = float(rospy.get_param("~lin_max", 0.5))
        self.ang_max = float(rospy.get_param("~ang_max", 1.5))
        self.rate_hz = float(rospy.get_param("~rate", 20.0))

        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        self.v = 0.0
        self.w = 0.0

        self.settings = termios.tcgetattr(sys.stdin)

        rospy.loginfo("Publishing Twist to: %s", self.cmd_vel_topic)
        rospy.loginfo(HELP)

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def get_key(self, timeout=0.0):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                key = self.get_key(timeout=0.02)

                if key in ("w", "W"):
                    self.v += self.lin_step
                elif key in ("s", "S"):
                    self.v -= self.lin_step
                elif key in ("a", "A"):
                    self.w += self.ang_step
                elif key in ("d", "D"):
                    self.w -= self.ang_step
                elif key == " ":
                    self.v = 0.0
                    self.w = 0.0
                elif key in ("q", "Q"):
                    break

                self.v = self.clamp(self.v, -self.lin_max, self.lin_max)
                self.w = self.clamp(self.w, -self.ang_max, self.ang_max)

                msg = Twist()
                msg.linear.x = self.v
                msg.angular.z = self.w
                self.pub.publish(msg)

                rate.sleep()
        finally:
            # stop on exit
            msg = Twist()
            self.pub.publish(msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            rospy.loginfo("Teleop exited, robot stopped.")

if __name__ == "__main__":
    KeyboardTeleop().run()
