#/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from math import tan, pi
import message_filters
from geometry_msgs.msg import PoseStamped
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
import tf.transformations as tf_transformations
import pytesseract
from imutils.object_detection import non_max_suppression

def decode_predictions(scores, geometry):
	# grab the number of rows and columns from the scores volume, then
	# initialize our set of bounding box rectangles and corresponding
	# confidence scores
	(numRows, numCols) = scores.shape[2:4]
	rects = []
	confidences = []
	# loop over the number of rows
	for y in range(0, numRows):
		# extract the scores (probabilities), followed by the
		# geometrical data used to derive potential bounding box
		# coordinates that surround text
		scoresData = scores[0, 0, y]
		xData0 = geometry[0, 0, y]
		xData1 = geometry[0, 1, y]
		xData2 = geometry[0, 2, y]
		xData3 = geometry[0, 3, y]
		anglesData = geometry[0, 4, y]
		# loop over the number of columns
		for x in range(0, numCols):
			# if our score does not have sufficient probability,
			# ignore it
			if scoresData[x] < 0.95:
				continue
			# compute the offset factor as our resulting feature
			# maps will be 4x smaller than the input image
			(offsetX, offsetY) = (x * 4.0, y * 4.0)
			# extract the rotation angle for the prediction and
			# then compute the sin and cosine
			angle = anglesData[x]
			cos = np.cos(angle)
			sin = np.sin(angle)
			# use the geometry volume to derive the width and height
			# of the bounding box
			h = xData0[x] + xData2[x]
			w = xData1[x] + xData3[x]
			# compute both the starting and ending (x, y)-coordinates
			# for the text prediction bounding box
			endX = int(offsetX + (cos * xData1[x]) + (sin * xData2[x]))
			endY = int(offsetY - (sin * xData1[x]) + (cos * xData2[x]))
			startX = int(endX - w)
			startY = int(endY - h)
			# add the bounding box coordinates and probability score
			# to our respective lists
			rects.append((startX, startY, endX, endY))
			confidences.append(scoresData[x])
	# return a tuple of the bounding boxes and associated confidences
	return (rects, confidences)

class CameraHandler:

    def __init__(self):
        # Create a tf2_ros.Buffer and a tf2_ros.TransformListener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # Define the source and target frames
        self.t_1_2 = self.get_relative_transformation_matrix()


        # Start the ROS node and continue listening for synchronized messages
        self.net = cv2.dnn.readNet("/home/agrobenj/catkin_ws/src/t265_stereo/scripts/frozen_east_text_detection.pb")

        self.window_title = 'Realsense'
        cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
        window_size = 5
        self.min_disp = 0
        # must be divisible by 16
        self.num_disp = 112 - self.min_disp
        self.max_disp = self.min_disp + self.num_disp
        self.stereo = cv2.StereoSGBM_create(minDisparity = self.min_disp,
                                    numDisparities = self.num_disp,
                                    blockSize = 16,
                                    P1 = 8*3*window_size**2,
                                    P2 = 32*3*window_size**2,
                                    disp12MaxDiff = 1,
                                    uniquenessRatio = 10,
                                    speckleWindowSize = 100,
                                    speckleRange = 32)

        # Create subscribers for the Image and CameraInfo topics
        left_image_sub = message_filters.Subscriber('/camera/fisheye1/image_raw', Image)
        left_camera_info_sub = message_filters.Subscriber('/camera/fisheye1/camera_info', CameraInfo)
        right_image_sub = message_filters.Subscriber('/camera/fisheye2/image_raw', Image)
        right_camera_info_sub = message_filters.Subscriber('/camera/fisheye2/camera_info', CameraInfo)

        # Create an ApproximateTimeSynchronizer to synchronize the two subscribers
        # The queue size is set to 10, and the slop parameter (in seconds) is set to 0.1
        # The slop parameter allows for some delay between the messages
        ts = message_filters.ApproximateTimeSynchronizer([left_image_sub, right_image_sub, left_camera_info_sub, right_camera_info_sub], queue_size=10, slop=0.05)
        ts.registerCallback(self.synchronized_callback)


        rospy.spin()

    @staticmethod
    def get_relative_transformation_matrix():
        # Initialize the ROS node

        # Create a tf2_ros.Buffer and a tf2_ros.TransformListener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # Define the source and target frames
        source_frame = 'camera_fisheye1_frame'
        target_frame = 'camera_fisheye2_frame'

        # Wait for the transform to become available
        try:
            tf_buffer.can_transform(target_frame, source_frame, rospy.Time(0), timeout=rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logerr('Error waiting for transform: %s', ex)
            return

        # Get the transform from the source frame to the target frame
        try:
            transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))

            # Extract translation and rotation (quaternion) from the transform
            translation = [transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z]
            rotation = [transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w]

            # Convert quaternion to a rotation matrix
            t = tf_transformations.quaternion_matrix(rotation)

            # Set the translation components of the transformation matrix
            t[0:3, 3] = translation

            # Print the relative transformation matrix
            #rospy.loginfo('Relative transformation matrix:')
            #rospy.loginfo('\n' + str(rotation_matrix))
            return t 

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logerr('Error looking up transform: %s', ex)

    @staticmethod
    def get_intrinsic_matrix_from_info(camera_info):
        return np.array(camera_info.K).reshape((3, 3))

    @staticmethod
    def get_distortion_matrix_from_info(camera_info):
        return np.array(camera_info.D)

    def synchronized_callback(self, left_image_msg, right_image_msg, left_camera_info, right_camera_info):
        #rospy.loginfo('Received synchronized messages:')
        #rospy.loginfo('Image timestamp: %s', left_image_msg.header.stamp)
        #rospy.loginfo('CameraInfo timestamp: %s', left_camera_info.header.stamp)
        K_left = self.get_intrinsic_matrix_from_info(left_camera_info)
        K_right = self.get_intrinsic_matrix_from_info(right_camera_info)
        D_left = self.get_distortion_matrix_from_info(left_camera_info)
        D_right = self.get_distortion_matrix_from_info(right_camera_info)
        width, height = left_image_msg.width, left_image_msg.height
        R = self.t_1_2[:3, :3]
        T = self.t_1_2[:3, -1]

        stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
        stereo_height_px = 300          # 300x300 pixel stereo image
        stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)

        # We set the left rotation to identity and the right rotation
        # the rotation between the cameras
        R_left = np.eye(3)
        R_right = R

        # The stereo algorithm needs max_disp extra pixels in order to produce valid
        # disparity on the desired image region. This changes the width, but the
        # center of projection should be on the center of the cropped image
        stereo_width_px = stereo_height_px + self.max_disp
        stereo_size = (stereo_width_px, stereo_height_px)
        stereo_cx = (stereo_height_px - 1)/2 + self.max_disp
        stereo_cy = (stereo_height_px - 1)/2

        # Construct the left and right projection matrices, the only difference is
        # that the right projection matrix should have a shift along the x axis of
        # baseline*focal_length
        P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                        [0, stereo_focal_px, stereo_cy, 0],
                        [0,               0,         1, 0]])
        P_right = P_left.copy()
        P_right[0][3] = T[0]*stereo_focal_px

        # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
        # since we will crop the disparity later
        Q = np.array([[1, 0,       0, -(stereo_cx - self.max_disp)],
                    [0, 1,       0, -stereo_cy],
                    [0, 0,       0, stereo_focal_px],
                    [0, 0, -1/T[0], 0]])

        # Create an undistortion map for the left and right camera which applies the
        # rectification and undoes the camera distortion. This only has to be done
        # once
        m1type = cv2.CV_32FC1
        (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
        (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
        undistort_rectify = {"left"  : (lm1, lm2),
                            "right" : (rm1, rm2)}
        mode = "stack"

        frame_copy = {"left"  : np.frombuffer(left_image_msg.data, dtype=np.uint8).reshape((height, width)),
                        "right" : np.frombuffer(right_image_msg.data, dtype=np.uint8).reshape((height, width))}

        center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                        map1 = undistort_rectify["left"][0],
                                        map2 = undistort_rectify["left"][1],
                                        interpolation = cv2.INTER_LINEAR),
                                "right" : cv2.remap(src = frame_copy["right"],
                                        map1 = undistort_rectify["right"][0],
                                        map2 = undistort_rectify["right"][1],
                                        interpolation = cv2.INTER_LINEAR)}

        # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
        disparity = self.stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0

        # re-crop just the valid part of the disparity
        disparity = disparity[:,self.max_disp:]

        # convert disparity to 0-255 and color it
        disp_vis = 255*(disparity - self.min_disp)/ self.num_disp
        disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis,1), cv2.COLORMAP_JET)
        image = cv2.cvtColor(center_undistorted["left"][:,self.max_disp:], cv2.COLOR_GRAY2RGB)

        orig = image.copy()
        origH, origW = orig.shape[:2]

        newW, newH = (320, 320)
        rW = origW / float(newW)
        rH = origH / float(newH)

        # resize the image and grab the new image dimensions
        image = cv2.resize(image, (newW, newH))
        (H, W) = image.shape[:2]

        layerNames = [
        "feature_fusion/Conv_7/Sigmoid",
        "feature_fusion/concat_3"]

        # construct a blob from the image and then perform a forward pass of
        # the model to obtain the two output layer sets
        blob = cv2.dnn.blobFromImage(image, 1.0, (W, H),
            (123.68, 116.78, 103.94), swapRB=True, crop=False)
        self.net.setInput(blob)
        (scores, geometry) = self.net.forward(layerNames)
        # decode the predictions, then  apply non-maxima suppression to
        # suppress weak, overlapping bounding boxes
        (rects, confidences) = decode_predictions(scores, geometry)
        boxes = non_max_suppression(np.array(rects), probs=confidences)

        # initialize the list of results
        results = []
        # loop over the bounding boxes
        for (startX, startY, endX, endY) in boxes:
            # scale the bounding box coordinates based on the respective
            # ratios
            startX = int(startX * rW)
            startY = int(startY * rH)
            endX = int(endX * rW)
            endY = int(endY * rH)
            # in order to obtain a better OCR of the text we can potentially
            # apply a bit of padding surrounding the bounding box -- here we
            # are computing the deltas in both the x and y directions
            dX = int((endX - startX) * 0.1)
            dY = int((endY - startY) * 0.1)
            # apply padding to each side of the bounding box, respectively
            startX = max(0, startX - dX)
            startY = max(0, startY - dY)
            endX = min(origW, endX + (dX * 2))
            endY = min(origH, endY + (dY * 2))
            # extract the actual padded ROI
            roi = orig[startY:endY, startX:endX]

            # in order to apply Tesseract v4 to OCR text we must supply
            # (1) a language, (2) an OEM flag of 1, indicating that the we
            # wish to use the LSTM neural net model for OCR, and finally
            # (3) an OEM value, in this case, 7 which implies that we are
            # treating the ROI as a single line of text
            config = ("-l eng --oem 1 --psm 7")
            text = pytesseract.image_to_string(roi, config=config)
            # add the bounding box coordinates and OCR'd text to the list
            # of results
            results.append(((startX, startY, endX, endY), text))

        # loop over the results
        output = orig.copy()
        for ((startX, startY, endX, endY), text) in results:
            # display the text OCR'd by Tesseract
            print("OCR TEXT")
            print("========")
            print("{}\n".format(text))
            # strip out non-ASCII text so we can draw the text on the image
            # using OpenCV, then draw the text and a bounding box surrounding
            # the text region of the input image
            text = "".join([c if ord(c) < 128 else "" for c in text]).strip()
            cv2.rectangle(output, (startX, startY), (endX, endY),
                (0, 0, 255), 2)
            cv2.putText(output, text, (startX, startY - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        if mode == "stack":
            cv2.imshow(self.window_title, np.hstack((output, disp_color)))
        if mode == "overlay":
            ind = disparity >= self.min_disp
            image[ind, 0] = disp_color[ind, 0]
            image[ind, 1] = disp_color[ind, 1]
            image[ind, 2] = disp_color[ind, 2]
            cv2.imshow(self.window_title, image)
        key = cv2.waitKey(1)
        
if __name__ == "__main__":
    rospy.init_node('synchronized_listener', anonymous=True)
    h = CameraHandler()
