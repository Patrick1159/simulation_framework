#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def pose_callback(msg):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        state = ModelState()
        state.model_name = 'my_tb3'   # 改成你的机器人模型名
        state.pose = msg.pose.pose              # 提取 Pose（去掉协方差）
        state.reference_frame = 'world'
        
        set_state(state)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('pose_relay')
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, pose_callback)
    rospy.spin()