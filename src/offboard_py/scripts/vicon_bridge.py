#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from offboard_py.scripts.utils import numpy_to_pose_stamped, transform_stamped_to_numpy, transform_stamped_to_odometry
from nav_msgs.msg import Odometry

class ViconBridge():

    def __init__(self):
        self.mavros_vision_pose_pub = rospy.Publisher("mavros/odometry/out", Odometry, queue_size=10)
        #local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=print_pose)
        self.last_vicon_update = rospy.Time.now()
        self.vicon_pose_sub = rospy.Subscriber("vicon/ROB498_Drone/ROB498_Drone", TransformStamped, callback = self.vicon_update)
        self.realsense_pose_sub = rospy.Subscriber("camera/odom/sample_throttled", Odometry, callback = self.realsense_update) # fail safe
        self.min_vicon_rate = 20.0

    def vicon_update(self, transform: TransformStamped):
        odom = transform_stamped_to_odometry(transform)
        odom.header.frame_id = "camera_odom_frame"
        odom.child_frame_id = "camera_pose_frame"
        odom.header.stamp=rospy.Time.now()
        self.mavros_vision_pose_pub.publish(odom)
        self.last_vicon_update = odom.header.stamp

    def realsense_update(self, odom: Odometry):
        if rospy.Time.now().to_sec() - self.last_vicon_update.to_sec() > 1.0/self.min_vicon_rate:
            odom.header.frame_id = "camera_odom_frame"
            odom.child_frame_id = "camera_pose_frame"
            odom.pose.covariance = [0] * 36
            # Set linear and angular velocities to zero (unknown)
            odom.twist.twist.linear.x = 0
            odom.twist.twist.linear.y = 0
            odom.twist.twist.linear.z = 0
            odom.twist.twist.angular.x = 0
            odom.twist.twist.angular.y = 0
            odom.twist.twist.angular.z = 0
            # Set twist covariance to zero (unknown)
            odom.twist.covariance = [0] * 36
            odom.header.stamp=rospy.Time.now()
            self.mavros_vision_pose_pub.publish(odom)
            self.last_vicon_update = rospy.Time.now()

if __name__ == "__main__":
    rospy.init_node("vicon_bridge")
    vb = ViconBridge()
    rospy.spin()

    
