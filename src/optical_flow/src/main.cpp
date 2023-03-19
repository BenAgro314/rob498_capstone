#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <ros/ros.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include "../include/flow_opencv.hpp"

void drawOpticalFlow(cv::Mat& img, float flow_x, float flow_y, int scale, const cv::Scalar& color) {
    cv::Point2f p1(img.cols/2, img.rows/2);
    cv::Point2f p2(p1.x + flow_x*scale, p1.y + flow_y*scale);
    cv::arrowedLine(img, p1, p2, color, 2);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	printf("xgyro: [%f]\nygyro: [%f]\n zgyro: [%f]\n", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "optical_flow_node");
	ros::NodeHandle nh;

	// Create publisher
	ros::Publisher flow_pub = nh.advertise<mavros_msgs::OpticalFlowRad>("/mavros/px4flow/raw/optical_flow_rad", 1);
	// Subscribe to imu
	ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 1, imuCallback);
	ros::Rate loop_rate(30);
	std::string pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1, ! nvvidconv flip-method=0 ! video/x-raw, width=1280, height=720, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera" << std::endl;
        return -1;
    }
    printf("opened camera\n");
    cv::Mat frame, frame_gray, flow_image;
//    std::vector<cv::Mat> images;
    cv::Mat prev_frame_gray;
    OpticalFlowOpenCV optical_flow(100, 100, 30, 640, 480, 100, 1.5); // create OpticalFlowOpenCV object
    printf("Entering while loop...\n");
    while (ros::ok()) {
        cap >> frame;
	printf("Frame captured\n");
        if (frame.empty()) break;

        // Convert the current frame to grayscale
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        // If this isn't the first frame, calculate optical flow
        if (!prev_frame_gray.empty()) {
            int dt_us;
            float flow_x, flow_y;
            int flow_quality = optical_flow.calcFlow(prev_frame_gray.data, 0, dt_us, flow_x, flow_y);

	    mavros_msgs::OpticalFlowRad flow_msg;
	    flow_msg.header.stamp = ros::Time::now();
	    flow_msg.integration_time_us = dt_us;
	    flow_msg.integrated_x = flow_x;
	    flow_msg.integrated_y = flow_y;
	    flow_msg.quality = flow_quality;
	    flow_pub.publish(flow_msg);
            // Create an image with the same dimensions as the current frame for displaying flow vectors
            // if (flow_image.empty()) {
               //  flow_image = cv::Mat::zeros(frame_gray.size(), CV_8UC3);
            // }

            // Draw the flow vectors on the flow image
            // cv::Mat flow_image = cv::Mat::zeros(frame.size(), CV_8UC3);
            // drawOpticalFlow(flow_image, flow_x, flow_y, 500, cv::Scalar(0, 255, 0));

            // Superimpose the flow image on the camera image
            // cv::add(frame, flow_image, frame);
        }

//        images.push_back(frame.clone());
	//cv::imshow("Camera", frame.clone());
        cv::waitKey(1);

        // Remember the previous frame for the next iteration
        prev_frame_gray = frame_gray;
	ros::spinOnce();
    }
    return 0;
}
