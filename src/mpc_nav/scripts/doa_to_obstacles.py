#!/usr/bin/env python3

import rospy
import math
from umap.msg import TrackerDetailArray
from mpc_nav.msg import Obstacle, ObstacleArray

class DOAAdapter:
    def __init__(self):
        # Parameters
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        self.radius_padding = rospy.get_param('~radius_padding', 0.1)
        self.max_age = rospy.get_param('~max_age', 0.5)
        self.velocity_limit = rospy.get_param('~velocity_limit', 5.0)
        
        # Publisher and Subscriber
        self.pub = rospy.Publisher('/doa_obstacles', ObstacleArray, queue_size=1)
        self.sub = rospy.Subscriber('/rematch/tracker_details', TrackerDetailArray, self.callback, queue_size=1)
        
        rospy.loginfo("DOA to MPC adapter started. Target frame: %s", self.target_frame)
        rospy.loginfo("Subscribing: /rematch/tracker_details -> Publishing: /doa_obstacles (observations)")
    
    def callback(self, msg):
        # Check frame
        if msg.header.frame_id != self.target_frame:
            rospy.warn_throttle(5.0, "Frame mismatch: got %s, expected %s", 
                              msg.header.frame_id, self.target_frame)
        
        out_msg = ObstacleArray()
        out_msg.header = msg.header
        out_msg.header.frame_id = self.target_frame
        
        for tracker in msg.trackers:
            # Filter by age
            if tracker.age > self.max_age:
                continue
            
            obs = Obstacle()
            obs.id = tracker.tracker_id
            obs.position = tracker.position
            
            # Velocity limiting
            v_mag = math.sqrt(tracker.velocity.x**2 + tracker.velocity.y**2 + tracker.velocity.z**2)
            if v_mag > self.velocity_limit:
                scale = self.velocity_limit / v_mag
                obs.velocity.x = tracker.velocity.x * scale
                obs.velocity.y = tracker.velocity.y * scale
                obs.velocity.z = tracker.velocity.z * scale
            else:
                obs.velocity = tracker.velocity
            
            # Radius: use diagonal of box + padding
            obs.radius = 0.5 * math.sqrt(tracker.box_dimensions.x**2 + tracker.box_dimensions.y**2) + self.radius_padding
            obs.type = 1  # cylinder
            
            out_msg.obstacles.append(obs)
        
        self.pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('doa_to_obstacles', anonymous=False)
    adapter = DOAAdapter()
    rospy.spin()
