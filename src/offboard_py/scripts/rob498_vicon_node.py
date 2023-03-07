#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped

def viconUpdate(transform:TransformStamped):
    piped_pose = PoseStamped()

    piped_pose.header = transform.header
    piped_pose.pose.position.x = transform.transform.translation.x
    piped_pose.pose.position.y = transform.transform.translation.y
    piped_pose.pose.position.z = transform.transform.translation.z
    piped_pose.pose.orientation = transform.transform.rotation
    mavros_vision_pose_pub.publish(piped_pose)


if __name__ == "__main__":
    rospy.init_node("viconEKF")
    mavros_vision_pose_pub = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=10)
    vicon_pose_sub = rospy.Subscriber("vicon/ROB498_Drone/ROB498_Drone", TransformStamped, callback = viconUpdate)
    rospy.spin()

    