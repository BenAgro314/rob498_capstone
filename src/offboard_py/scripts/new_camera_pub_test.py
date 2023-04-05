#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from jetcam.csi_camera import CSICamera

def publish_images():
    # Initialize the ROS node
    rospy.init_node('imx219_publisher', anonymous=True)

    # Create a publisher for the image topic
    image_pub = rospy.Publisher('imx219_image', Image, queue_size=10)

    # Create a CvBridge object for converting images
    bridge = CvBridge()

    # Set the capture resolution (modify as needed)
    capture_width = 540
    capture_height = 540

    fps = 5
    # Initialize the CSICamera from the jetcam package
    camera = CSICamera(width=capture_width, height=capture_height, capture_fps=fps)
    rate = rospy.Rate(fps)


    # Capture images continuously
    while not rospy.is_shutdown():
        # Capture an image from the camera
        cv_image = camera.read()

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
        publish_images()
    except rospy.ROSInterruptException:
        pass
