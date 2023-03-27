#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, TransformStamped
from offboard_py.scripts.utils import pose_to_transform_stamped
import numpy as np

class FakeVicon:

	def __init__(self):
		self.pose_pub = rospy.Publisher("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, queue_size = 10)
		self.start_time=rospy.Time.now()
		self.link_pose = TransformStamped()
		self.link_pose.header.stamp=rospy.Time.now()
		self.link_pose.transform.translation.x = 10.0
		self.link_pose.transform.translation.y = 10.0
		self.link_pose.transform.translation.z = 10.0
		self.link_pose.transform.rotation.x = 0.0
		self.link_pose.transform.rotation.y = 0.0
		self.link_pose.transform.rotation.z = 0.0
		self.link_pose.transform.rotation.w = 1.0
		self.link_pose.header.frame_id="camera_odom_frame"
		self.link_pose.child_frame_id="camera_pose_frame"

	def update(self):
		self.link_pose = TransformStamped()
		self.link_pose.header.stamp=rospy.Time.now()
		self.link_pose.transform.translation.x = 10.0
		self.link_pose.transform.translation.y = 10.0
		self.link_pose.transform.translation.z = 10.0
		self.link_pose.transform.rotation.x = 0.0
		self.link_pose.transform.rotation.y = 0.0
		self.link_pose.transform.rotation.z = 0.0
		self.link_pose.transform.rotation.w = 1.0
		self.link_pose.header.frame_id="camera_odom_frame"
		self.link_pose.child_frame_id="camera_pose_frame"
		self.link_pose.transform.translation.x = 0.2 * (rospy.Time.now().to_sec() - self.start_time.to_sec())
		self.link_pose.transform.translation.y = 0.3 * (rospy.Time.now().to_sec() - self.start_time.to_sec())
		self.link_pose.transform.translation.z = 0.1 * (rospy.Time.now().to_sec() - self.start_time.to_sec())


if __name__ == '__main__':
	rospy.init_node('fake_vicon', anonymous=True)
	fv = FakeVicon()
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		fv.pose_pub.publish(fv.link_pose)
		fv.update()
		rate.sleep()
