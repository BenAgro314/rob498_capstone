#!/usr/bin/env python3

import rospy
import time
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from jetcam.csi_camera import CSICamera

if __name__ == '__main__':
    # Set the capture resolution (modify as needed)
    capture_width = 540
    capture_height = 540

    fps = 30
    # Initialize the CSICamera from the jetcam package
    camera = CSICamera(width=capture_width, height=capture_height, capture_fps=fps)

    # Capture images continuously
    for i in range(20):
        # Capture an image from the camera
        path = "images/{i:02d}.png"
        print(f"Saving: {path}")
        cv_image = camera.read()
        cv2.imwrite(path, cv_image)
        cv2.imshow('Image', cv_image)
        cv2.waitKey(0)

