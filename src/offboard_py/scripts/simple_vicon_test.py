#!/usr/bin/env python3

from offboard_py.scripts.utils import numpy_to_pose_stamped, transform_stamped_to_numpy
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped

def vicon_update(transform:TransformStamped):
    transform.transform.translation.x += np.random.rand(1)
    matrix = transform_stamped_to_numpy(transform)
    #theta = np.pi
    #T = np.array([
    #    [1, 0, 0, 0],
    #    [0, np.cos(theta), -np.sin(theta), 0],
    #    [0, np.sin(theta), np.cos(theta), 0],
    #    [0, 0, 0, 1],
    #])
    #matrix = T @ matrix
    pose = numpy_to_pose_stamped(matrix, frame_id = "map")
    mavros_vision_pose_pub.publish(pose)

if __name__ == "__main__":
    rospy.init_node("vicon_test")
    mavros_vision_pose_pub = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=10)
    #local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=print_pose)
    vicon_pose_sub = rospy.Subscriber("vicon/ROB498_Drone/ROB498_Drone", TransformStamped, callback = vicon_update)

    #rate = rospy.Rate(10);

    #while not rospy.is_shutdown():
    #    pose = PoseStamped()
    #    pose.pose.position.x = 1
    #    pose.pose.position.y = 2
    #    pose.pose.position.z = 3
    #    pose.header.stamp=rospy.Time.now()
    #    pose.header.frame_id='vision'
    #    mavros_vision_pose_pub.publish(pose)
    #    rate.sleep()

    rospy.spin()

