import cv2
import time

def capture_and_save_images(num_images=10, delay=1, output_prefix='image'):
    cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
    if not cap.isOpened():
        print("Error")
        return
    for i in range(num_images):
        ret,frame = cap.read()
        if not ret:
            print("Error")
            break
        output_filename=f"/home/jetson/image_data/{output_prefix}_{i}.jpg"
        cv2.imwrite(output_filename, frame)
        print(f"Saved image {i+1}")
        time.sleep(delay)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_and_save_images()

