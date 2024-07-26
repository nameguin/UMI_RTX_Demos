/**
 * @file node_camera.hpp
 * @brief Implementation of the Camera class for handling RealSense camera data and ROS communication.
 * 
 * This file contains the implementation of the Camera class, which initializes the camera,
 * handles image processing, and communicates with ROS topics.
 */

#ifndef __CAMERA__
#define __CAMERA__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64.hpp"
#include "umi_rtx_interfaces/msg/board.hpp"
#include "umi_rtx_interfaces/msg/game_data.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>
#include <iostream>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc.hpp>
#include <librealsense2/rs.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

/**
 * @file node_camera.cpp
 * @brief Implementation of the Camera class for controlling the UMI-RTX camera system.
 */

 struct GridSquare {
    cv::Point2f center;
    std::vector<cv::Point> contours;
    int area;
};

/**
 * @class Camera
 * @brief A ROS 2 node for managing and processing data from a RealSense camera.
 * 
 * The `Camera` class is responsible for capturing images and depth data from a RealSense camera,
 * processing this data to detect colored objects and their positions, and publishing relevant information
 * to various ROS 2 topics. The class also handles initialization of ROS 2 interfaces and the RealSense camera.
 * 
 * @details
 * The class includes methods to initialize ROS 2 publishers and subscribers, configure the RealSense camera,
 * process image and depth data, and handle capture step commands. It also provides functionality to detect
 * colored objects and determine box indices based on coordinates.
 * 
 * The `Camera` constructor sets up the node, initializes interfaces, and configures the camera.
 */
class Camera : public rclcpp::Node{
public:
    /**
    * @brief Construct a new Camera object.
    * 
    * The constructor also performs the following initializations:
    * - Calls `init_interfaces()` to set up publishers, subscribers, and timers for the camera node.
    * - Calls `init_camera()` to configure and start the RealSense camera, including setting up streams and configuring sensor options.
    * 
    * @details
    * The `init_interfaces()` method creates and initializes ROS 2 publishers and subscribers required for image and pose data, board state, and capture step commands.
    * The `init_camera()` method checks for available RealSense cameras, initializes the first available device, and configures its depth and color streams. It also sets sensor options for high accuracy.
    * 
    * @note 
    * Ensure that the RealSense camera is properly connected and that the RealSense SDK is correctly installed before using this constructor.
    */
    Camera() : Node("control"), align_to_color(RS2_STREAM_COLOR), turn(1){
        init_interfaces();
        init_camera();
    };

private:
    /**
    * @brief Callback function for processing frames and publishing data.
    * 
    * This function is called periodically by a timer and performs the following tasks:
    * - Waits for the next set of frames from the RealSense camera.
    * - Aligns the depth frame to the color frame for improved accuracy.
    * - Retrieves and processes the depth frame, colored depth frame, and color frame.
    * - Converts the frames to OpenCV matrices for further processing.
    * - If the `capture_step` flag is set, processes the frames to detect colored objects, updates the pose message
    *   with the detected object's position, and draws rectangles on the color frame to indicate regions of interest.
    * - Publishes the board state, processed pose message, and images (depth and color) to respective ROS 2 topics.
    * - Resets the `capture_step` flag after processing.
    * 
    * @details
    * The function utilizes the RealSense `rs2::pipeline` to capture and process frames. The depth frame is aligned with
    * the color frame to enhance accuracy. The `color_map` filter is applied to the depth frame to create a colored
    * depth frame for visualization. Both the depth and color frames are converted to OpenCV matrices, which are then
    * used to detect colored objects and draw rectangles on the color frame. The resulting images and pose data are
    * published to the appropriate ROS 2 topics.
    */
    void timer_callback();

    /**
    * @brief Initializes the ROS 2 interfaces for the Camera class.
    * 
    * This function initializes the various ROS 2 publishers, subscribers, and timers needed
    * for the Camera class to function correctly.
    */
    void init_interfaces();

    /**
    * @brief Initializes the camera.
    * 
    * This function checks for available cameras, selects the first detected RealSense camera, and sets up the configuration for the depth and color streams.
    */
    void init_camera();

    /**
    * @brief Detects colored objects in the frame.
    * 
    * This function processes the captured frames to detect colored objects. It identifies the position of the objects,
    * determines their coordinates, and updates the board state accordingly.
    * 
    * @param pose_msg The pose message to be populated with object coordinates.
    * @param depth The depth frame for distance measurement.
    */
    void get_colored_objects(geometry_msgs::msg::Pose pose_msg, rs2::depth_frame depth);

    /**
     * @brief Finds the fittest line with respect to the contour of the target and
     * computes its orientation.
     * 
     * @param contours The longest contours in the binarized image.
     */
    void get_angles(vector<cv::Point> &contours);

    /**
    * @brief Callback function to capture a step.
    * 
    * This function is called when a message is received on the "capture_step" topic.
    * It sets the capture_step flag based on the received message.
    * 
    * @param msg The received message.
    */
    void get_capture_step(const std_msgs::msg::Bool::SharedPtr msg);

    /**
    * @brief Determines the box index based on the pawn coordinates.
    * 
    * This function calculates the box index based on the provided coordinates.
    * 
    * @param cx The x-coordinate.
    * @param cy The y-coordinate.
    * @return The box index or -1 if the coordinates are out of bounds.
    */
    int find_box(double cx, double cy);

    std::chrono::milliseconds loop_dt_ = 40ms;
    rclcpp::TimerBase::SharedPtr timer_;

    int board_state[9];

    int turn;
    bool capture_step;
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align_to_color;
    rs2::colorizer color_map;
    int available_device;

    cv::Mat colorFrameCV, depthFrameCV;

    int m_frame_width, m_frame_height;
    int m_depth_frame_width, m_depth_frame_height;

    double m_cx, m_cy, m_cz, yaw, pitch, roll;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher; 
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr processed_pose_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_to_play_publisher;

    rclcpp::Publisher<umi_rtx_interfaces::msg::Board>::SharedPtr board_state_publisher; 

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr capture_step_subscriber;
};


#endif