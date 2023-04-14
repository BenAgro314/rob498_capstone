#!/usr/bin/env python3
import os
import sys
import cv2
import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if len(sys.argv) != 3:
    print("Usage: python3 extract_images.py /path/to/rosbag.bag /path/to/save/images")
    exit(1)

bag_file = sys.argv[1]
output_path = sys.argv[2]
image_topic = '/imx219_image'

if not os.path.exists(output_path):
    os.makedirs(output_path)

rospy.init_node('extract_images')
bag = rosbag.Bag(bag_file, 'r')
bridge = CvBridge()
count = 0

for topic, msg, _ in bag.read_messages(topics=[image_topic]):
    if rospy.is_shutdown():
        break

    cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    file_path = os.path.join(output_path, f"frame_{count:04d}.png")
    cv2.imwrite(file_path, cv_img)
    count += 1

bag.close()

