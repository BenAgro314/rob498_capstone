#!/usr/bin/env python3

import rospy
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from jetcam.csi_camera import CSICamera
import numpy as np


K = np.array(
    [
        [334.94030171,    0.,         280.0627713],
        [  0., 595.99313333, 245.316628],
        [  0.,           0.,           1.        ]
    ]
)
D = np.array([[-0.36600591, 0.20973317, -0.00181088, 0.00102208, -0.07504754]])

shape = (1080, 1080)
map1, map2 = cv2.initUndistortRectifyMap(K, D, None, K, shape, cv2.CV_32FC1)

def publish_images():
    # Initialize the ROS node
    rospy.init_node('imx219_publisher', anonymous=True)

    # Create a publisher for the image topic
    image_pub = rospy.Publisher('imx219_image', Image, queue_size=10)

    # Create a CvBridge object for converting images
    bridge = CvBridge()

    # Set the capture resolution (modify as needed)
    capture_width = 1080
    capture_height = 1080

    fps = 30
    # Initialize the CSICamera from the jetcam package
    camera = CSICamera(width=capture_width, height=capture_height, capture_fps=fps)
    rate = rospy.Rate(fps)


    # Capture images continuously
    while not rospy.is_shutdown():
        # Capture an image from the camera
        cv_image = camera.read()
        cv_image = cv2.remap(cv_image, map1, map2, interpolation=cv2.INTER_LINEAR)

        # Convert the captured image to a ROS image message
        image_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        # Set the timestamp for the image message
        image_msg.header.stamp = rospy.Time.now()

        # Publish the image message
        image_pub.publish(image_msg)

        # Sleep to control the capture rate
        rate.sleep()

if __name__ == '__main__':
    try:
        time.sleep(15)
        publish_images()
    except rospy.ROSInterruptException:
        pass
