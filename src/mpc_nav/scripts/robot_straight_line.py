#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("robot_straight_line")
    speed = rospy.get_param("~speed", 0.4)
    duration = rospy.get_param("~duration", 20.0)
    topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

    pub = rospy.Publisher(topic, Twist, queue_size=1)
    rate = rospy.Rate(30)

    twist = Twist()
    twist.linear.x = speed

    start = rospy.Time.now()
    rospy.loginfo("Robot moving at %.1f m/s for %.1f s", speed, duration)

    while not rospy.is_shutdown():
        if (rospy.Time.now() - start).to_sec() >= duration:
            twist.linear.x = 0.0
            pub.publish(twist)
            rospy.loginfo("Duration reached, stopping")
            break
        pub.publish(twist)
        rate.sleep()


if __name__ == "__main__":
    main()
