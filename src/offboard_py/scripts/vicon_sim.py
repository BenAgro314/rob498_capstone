#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, TransformStamped
from offboard_py.scripts.utils import pose_to_transform_stamped

class GazeboLinkPose:
  link_name = ''
  link_pose = TransformStamped()
  def __init__(self, link_name):
    self.link_name = link_name
    self.link_name_rectified = "vicon_pose" #link_name.replace("::", "_")

    if not self.link_name:
      raise ValueError("'link_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
    self.pose_pub = rospy.Publisher("/vicon/ROB498_Drone/ROB498_Drone", TransformStamped, queue_size = 10)

  def callback(self, data):
    try:
        ind = data.name.index(self.link_name)
        self.link_pose = pose_to_transform_stamped(data.pose[ind])
    except ValueError:
        print(f"Invalid link name {self.link_name}, choose one from: {data.name}")

if __name__ == '__main__':
  try:
    rospy.init_node('gazebo_link_pose', anonymous=True)
    gp = GazeboLinkPose("iris::dots_link")
    publish_rate = rospy.get_param('~publish_rate', 10)

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
      gp.pose_pub.publish(gp.link_pose)
      rate.sleep()

  except rospy.ROSInterruptException:
    pass