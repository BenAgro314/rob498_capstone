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

    camera = CSICamera(width=width, height=height, capture_device=0)

    # Start the camera
    camera.running = True

    # Initialize a counter for the number of images captured
    image_counter = 0

    # Create a window to display the camera feed
    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

    try:
        while image_counter < 20:
            # Get the current frame from the camera
            frame = camera.value
            
            # Display the frame in the window
            cv2.imshow("Camera Feed", frame)
            
            # Wait for a key press (1 millisecond)
            key = cv2.waitKey(1) & 0xFF
            
            # If the "c" key is pressed, capture and save the image
            if key == ord("c"):
                # Save the image to disk
                cv2.imwrite(f"images/{image_counter}.png", frame)
                
                # Increment the image counter
                image_counter += 1
                
                # Print a message indicating the image was captured
                print(f"Image {image_counter} captured.")
                
                # Add a delay between captures (optional)
                time.sleep(0.5)
            
            # If the "q" key is pressed, exit the loop
            if key == ord("q"):
                break
    finally:
        # Release the camera and close the window
        camera.running = False
        cv2.destroyAllWindows()

    print("Finished capturing images.")