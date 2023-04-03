#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def pose_callback(pose_msg):
    # Extract position and orientation from the pose message
    position = pose_msg.pose.position
    orientation = pose_msg.pose.orientation

    # Create a transform broadcaster
    br = tf.TransformBroadcaster()

    # Broadcast the transform
    br.sendTransform(
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
        rospy.Time.now(),
        "base_link",
        "map"
    )

def main():
    # Initialize the ROS node
    rospy.init_node('pose_to_tf', anonymous=True)

    # Subscribe to the mavros/local_position/pose topic
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass