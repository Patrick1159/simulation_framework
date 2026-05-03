#!/usr/bin/env python3

import rospy
import math
from umap.msg import TrackerDetailArray
from mpc_nav.msg import Obstacle, ObstacleArray
from geometry_msgs.msg import Point, PointStamped, Vector3, Vector3Stamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf.transformations as tft

class DOAAdapter:
    def __init__(self):
        # Parameters
        self.target_frame = rospy.get_param('~target_frame', 'real_baselink') # odom
        self.radius_padding = rospy.get_param('~radius_padding', 0.1)
        self.max_age = rospy.get_param('~max_age', 0.5)
        self.velocity_limit = rospy.get_param('~velocity_limit', 5.0)
        self.tf_timeout = rospy.get_param('~tf_timeout', 0.2)
        self.drop_on_tf_fail = rospy.get_param('~drop_on_tf_fail', True)
        self.marker_scale = rospy.get_param('~marker_scale', 1.0)
        self.marker_lifetime = rospy.get_param('~marker_lifetime', 0.2)
        
        # Publisher and Subscriber
        self.pub = rospy.Publisher('/doa_obstacles', ObstacleArray, queue_size=1)
        self.marker_pub = rospy.Publisher('/doa_obstacles_markers', MarkerArray, queue_size=1)
        self.sub = rospy.Subscriber('/rematch/tracker_details', TrackerDetailArray, self.callback, queue_size=1)

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("DOA to MPC adapter started. Target frame: %s", self.target_frame)
        rospy.loginfo("Subscribing: /rematch/tracker_details -> Publishing: /doa_obstacles (observations)")
    
    def callback(self, msg):
        # After the frame-honest refactor (Phase 7), rematch publishes
        # tracker_details directly in `odom` (REMATCH_OUTPUT_FRAME = "odom").
        # The TF lookup branch below is kept as a safety net: if a future
        # config sets a non-odom output frame, we still transform here so
        # MPC always receives obstacles in `target_frame`. In the common
        # case `source_frame == target_frame == "odom"`, no TF call is made.
        if msg.header.frame_id != self.target_frame:
            rospy.logwarn_throttle(5.0, "Frame mismatch: got %s, expected %s — will transform via TF",
                                   msg.header.frame_id, self.target_frame)
        
        out_msg = ObstacleArray()
        out_msg.header = msg.header
        out_msg.header.frame_id = self.target_frame

        marker_array = MarkerArray()
        marker_ns = "doa_obstacles"

        source_frame = msg.header.frame_id if msg.header.frame_id else self.target_frame
        transform = None
        if source_frame != self.target_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    rospy.Time(0),
                    rospy.Duration(self.tf_timeout)
                )
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
                rospy.logwarn_throttle(2.0, "TF lookup failed %s->%s: %s",
                                       source_frame, self.target_frame, str(e))
                if self.drop_on_tf_fail:
                    return
        for idx, tracker in enumerate(msg.trackers):
            # Filter by age
            if tracker.age > self.max_age:
                continue
            
            obs = Obstacle()
            obs.id = tracker.tracker_id
            if transform is not None:
                try:
                    obs.position = self._transform_point(tracker.position, transform)
                except Exception as e:
                    rospy.logwarn_throttle(2.0, "Point transform failed: %s", str(e))
                    if self.drop_on_tf_fail:
                        continue
                    obs.position = tracker.position
            else:
                obs.position = tracker.position
            
            # Velocity limiting
            v_mag = math.sqrt(tracker.velocity.x**2 + tracker.velocity.y**2 + tracker.velocity.z**2)
            if v_mag > self.velocity_limit:
                scale = self.velocity_limit / v_mag
                vel = tracker.velocity
                vel.x *= scale
                vel.y *= scale
                vel.z *= scale
            else:
                vel = tracker.velocity

            if transform is not None:
                try:
                    obs.velocity = self._transform_vector(vel, transform)
                except Exception as e:
                    rospy.logwarn_throttle(2.0, "Velocity transform failed: %s", str(e))
                    if self.drop_on_tf_fail:
                        continue
                    obs.velocity = vel
            else:
                obs.velocity = vel
            
            # Radius: use diagonal of box + padding
            obs.radius = 0.5 * math.sqrt(tracker.box_dimensions.x**2 + tracker.box_dimensions.y**2) + self.radius_padding
            # obs.radius = min(tracker.box_dimensions.x, tracker.box_dimensions.y) * 0.5 + self.radius_padding
            obs.type = 1  # cylinder
            # Forward upstream confidence; default 1.0 if upstream did not populate
            # the field (e.g. older bag files, back-compat).
            obs.confidence = getattr(tracker, 'confidence', 1.0) or 1.0

            out_msg.obstacles.append(obs)

            # Visualization markers (position sphere + velocity arrow)
            sphere = Marker()
            sphere.header = out_msg.header
            sphere.ns = f"{marker_ns}/position"
            sphere.id = idx * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = obs.position
            sphere.pose.orientation.w = 1.0
            radius = max(obs.radius, 0.05)
            sphere.scale.x = radius * 2.0 * self.marker_scale
            sphere.scale.y = radius * 2.0 * self.marker_scale
            sphere.scale.z = radius * 2.0 * self.marker_scale
            sphere.color.r = 0.2
            sphere.color.g = 0.8
            sphere.color.b = 1.0
            sphere.color.a = 0.6
            sphere.lifetime = rospy.Duration(self.marker_lifetime)
            marker_array.markers.append(sphere)

            arrow = Marker()
            arrow.header = out_msg.header
            arrow.ns = f"{marker_ns}/velocity"
            arrow.id = idx * 2 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.05 * self.marker_scale  # shaft diameter
            arrow.scale.y = 0.1 * self.marker_scale   # head diameter
            arrow.scale.z = 0.1 * self.marker_scale   # head length
            arrow.color.r = 1.0
            arrow.color.g = 0.4
            arrow.color.b = 0.1
            arrow.color.a = 0.9
            arrow.lifetime = rospy.Duration(self.marker_lifetime)

            start = obs.position
            speed = math.sqrt(obs.velocity.x**2 + obs.velocity.y**2 + obs.velocity.z**2)
            arrow_len = max(0.2, speed) * self.marker_scale
            end = PointStamped()
            end.point.x = start.x + (obs.velocity.x * arrow_len if speed > 1e-6 else 0.0)
            end.point.y = start.y + (obs.velocity.y * arrow_len if speed > 1e-6 else 0.0)
            end.point.z = start.z + (obs.velocity.z * arrow_len if speed > 1e-6 else 0.0)
            arrow.points.append(start)
            arrow.points.append(end.point)
            marker_array.markers.append(arrow)
        
        self.pub.publish(out_msg)
        self.marker_pub.publish(marker_array)

    @staticmethod
    def _transform_point(point, transform):
        q = transform.transform.rotation
        t = transform.transform.translation
        rot = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        vec = [point.x, point.y, point.z, 1.0]
        res = rot.dot(vec)
        res[0] += t.x
        res[1] += t.y
        res[2] += t.z
        return Point(x=res[0], y=res[1], z=res[2])

    @staticmethod
    def _transform_vector(vec, transform):
        q = transform.transform.rotation
        rot = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        v = [vec.x, vec.y, vec.z, 0.0]
        res = rot.dot(v)
        return Vector3(x=res[0], y=res[1], z=res[2])

if __name__ == '__main__':
    rospy.init_node('doa_to_obstacles', anonymous=False)
    adapter = DOAAdapter()
    rospy.spin()
