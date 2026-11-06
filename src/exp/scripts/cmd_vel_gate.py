#!/usr/bin/env python3
"""Gate /cmd_vel for the demonstration experiment.

Forwards /mpc/cmd_vel_raw to /cmd_vel when ~apply is true; otherwise either
publishes a zero Twist at 10 Hz (active brake) or publishes nothing.
"""

import rospy
from geometry_msgs.msg import Twist


class CmdVelGate(object):
    def __init__(self):
        self.apply = rospy.get_param('~apply', True)
        self.zero_when_blocked = rospy.get_param('~zero_when_blocked', True)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber(
            '/mpc/cmd_vel_raw', Twist, self.cmd_cb, queue_size=1)

        if self.apply:
            rospy.loginfo(
                "cmd_vel_gate: apply=True - forwarding /mpc/cmd_vel_raw to /cmd_vel")
        elif self.zero_when_blocked:
            rospy.loginfo(
                "cmd_vel_gate: apply=False, zero_when_blocked=True - "
                "publishing zero Twist at 10 Hz")
            rospy.Timer(rospy.Duration(0.1), self.zero_cb)
        else:
            rospy.loginfo(
                "cmd_vel_gate: apply=False, zero_when_blocked=False - "
                "publishing nothing")

    def cmd_cb(self, msg):
        if self.apply:
            self.pub.publish(msg)

    def zero_cb(self, _event):
        self.pub.publish(Twist())


if __name__ == '__main__':
    rospy.init_node('cmd_vel_gate')
    CmdVelGate()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
