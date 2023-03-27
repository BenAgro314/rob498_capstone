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
        self.vicon_pose_sub = rospy.Subscriber("vicon/ROB498_Drone/ROB498_Drone", TransformStamped, callback = self.vicon_update)

    def vicon_update(self, transform: TransformStamped):
        odom = transform_stamped_to_odometry(transform)
        odom.header.frame_id = "camera_odom_frame"
        odom.child_frame_id = "camera_pose_frame"
        odom.header.stamp=rospy.Time.now()
        self.mavros_vision_pose_pub.publish(odom)

if __name__ == "__main__":
    rospy.init_node("vicon_bridge")
    vb = ViconBridge()
    rospy.spin()

    
