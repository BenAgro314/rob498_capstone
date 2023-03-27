#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, TransformStamped
from offboard_py.scripts.utils import pose_to_transform_stamped

class FakeVicon:

	def __init__(self):
		self.pose_pub = rospy.Publisher("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, queue_size = 10)
		self.link_pose = TransformStamped()
		self.link_pose.transform.translation.x = 10.0
		self.link_pose.transform.translation.y = 10.0
		self.link_pose.transform.translation.z = 10.0


if __name__ == '__main__':
	rospy.init_node('fake_vicon', anonymous=True)
	fv = FakeVicon()
	publish_rate = rospy.get_param('~publish_rate', 10)

	rate = rospy.Rate(publish_rate)
	while not rospy.is_shutdown():
		fv.pose_pub.publish(fv.link_pose)
		rate.sleep()
