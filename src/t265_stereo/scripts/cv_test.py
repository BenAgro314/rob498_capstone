import cv2
import numpy as np

WINDOW_TITLE = 'OpenCV Test Window'
cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

# Create a simple black image
image = np.zeros((300, 300, 3), dtype=np.uint8)

while True:
    cv2.imshow(WINDOW_TITLE, image)
    key = cv2.waitKey(1)

    if key == ord('q'):
        break


print("hi")
cv2.destroyAllWindows()

