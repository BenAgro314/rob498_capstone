#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import picamera
import picamera.array

def publish_images():
    # Initialize the ROS node
    rospy.init_node('imx219_publisher', anonymous=True)

    # Create a publisher for the image topic
    image_pub = rospy.Publisher('imx219_image', Image, queue_size=10)

    # Create a CvBridge object for converting images
    bridge = CvBridge()

    # Set the capture resolution (modify as needed)
    capture_width = 640
    capture_height = 480

    # Initialize the PiCamera and set the resolution
    with picamera.PiCamera() as camera:
        camera.resolution = (capture_width, capture_height)

        # Create a PiRGBArray object to store the captured image
        with picamera.array.PiRGBArray(camera) as output:

            # Capture images continuously
            while not rospy.is_shutdown():
                # Capture an image from the camera
                camera.capture(output, format='bgr')

                # Convert the captured image to an OpenCV image
                cv_image = output.array

                # Convert the OpenCV image to a ROS image message
                image_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

                # Set the timestamp for the image message
                image_msg.header.stamp = rospy.Time.now()

                # Publish the image message
                image_pub.publish(image_msg)

                # Clear the output array for the next capture
                output.truncate(0)

                # Sleep to control the capture rate
                rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        publish_images()
    except rospy.ROSInterruptException:
        pass
