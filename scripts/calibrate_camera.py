import cv2
import numpy as np
import glob

# Define the size of the checkerboard pattern (number of inner corners)
num_corners_x = 6  # Number of inner corners along the x-axis
num_corners_y = 8  # Number of inner corners along the y-axis

# Define the width of each square in the checkerboard (in meters or any other real-world unit)
square_width = 0.108  # Example value

# Define the object points (3D world points) for the checkerboard pattern
objp = np.zeros((num_corners_x * num_corners_y, 3), np.float32)
objp[:, :2] = np.mgrid[0:num_corners_x, 0:num_corners_y].T.reshape(-1, 2) * square_width

# Arrays to store object points and image points from all the images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load the images of the checkerboard pattern
images = glob.glob('/home/agrobenj/catkin_ws/images/imx219_calib_v2/*.png')

#criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Loop through the images and find the checkerboard corners
img_list = []
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_list.append(img)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (num_corners_x, num_corners_y), None)

    # If corners are found, add object points and image points
    if ret:
        objpoints.append(objp)
        #corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        #corn_img = np.copy(img)
        #cv2.drawChessboardCorners(corn_img, (num_corners_x, num_corners_y), corners, ret)
        #cv2.imshow('img', corn_img)
        #cv2.waitKey(0)

# Calibrate the camera
ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print the camera calibration parameters
print("Intrinsic Parameters Matrix (K):")
print(K)
print("Distortion Coefficients (D):")
print(D)

print("Showing undistored images:")
def undistort_image(img, K, D):
    # Undistort the image

    shape = img.shape[:2]
    #map1, map2 = cv2.initUndistortRectifyMap(K, D, None, K, shape, cv2.CV_32FC1)
    #undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)
    undistorted_img = cv2.undistort(img, K, D)

    # Display the original and undistorted images
    cv2.imshow('Original Image', img)
    cv2.imshow('Undistorted Image', undistorted_img)

    # Wait for a key press and close the windows
    cv2.waitKey(0)
    cv2.destroyAllWindows()

for image in img_list:
    undistort_image(image, K, D)