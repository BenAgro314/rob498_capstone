#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include "include/flow_opencv.hpp"

void drawOpticalFlow(cv::Mat& img, float flow_x, float flow_y, int scale, const cv::Scalar& color) {
    cv::Point2f p1(img.cols/2, img.rows/2);
    cv::Point2f p2(p1.x + flow_x*scale, p1.y + flow_y*scale);
    cv::arrowedLine(img, p1, p2, color, 2);
}

int main(int argc, char *argv[]) {
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera" << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cv::Mat frame, frame_gray, flow_image;
    std::vector<cv::Mat> images;
    cv::Mat prev_frame_gray;
    OpticalFlowOpenCV optical_flow(100, 100, 30, 640, 480, 100, 1.5); // create OpticalFlowOpenCV object

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Convert the current frame to grayscale
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        // If this isn't the first frame, calculate optical flow
        if (!prev_frame_gray.empty()) {
            int dt_us;
            float flow_x, flow_y;
            int flow_quality = optical_flow.calcFlow(prev_frame_gray.data, 0, dt_us, flow_x, flow_y);

            // Create an image with the same dimensions as the current frame for displaying flow vectors
            if (flow_image.empty()) {
                flow_image = cv::Mat::zeros(frame_gray.size(), CV_8UC3);
            }

            // Draw the flow vectors on the flow image
            cv::Mat flow_image = cv::Mat::zeros(frame.size(), CV_8UC3);
            drawOpticalFlow(flow_image, flow_x, flow_y, 500, cv::Scalar(0, 255, 0));

            // Superimpose the flow image on the camera image
            cv::add(frame, flow_image, frame);
        }

        images.push_back(frame.clone());
        cv::imshow("Camera", frame.clone());
        cv::waitKey(1);

        // Remember the previous frame for the next iteration
        prev_frame_gray = frame_gray;

    }
    return 0;
}