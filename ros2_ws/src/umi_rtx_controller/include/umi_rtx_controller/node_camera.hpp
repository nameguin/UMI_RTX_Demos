/**
 * @file node_camera.hpp
 * @author Guillaume GARDE (guillaume.garde@ensta-bretagne.org)
 * @brief This node is dedicated to the computer vision part of this project.
 * It allows to detect the targeted banana, gets its centroid's 2D coordinates 
 * and tries to compute the depth maps of the scene.
 * @version 0.1
 * @date 2023-08-15
 * 
 * 
 */
#ifndef __CAMERA__
#define __CAMERA__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64.hpp"
#include "umi_rtx_interfaces/msg/board.hpp"
#include "umi_rtx_interfaces/msg/game_data.hpp"
#include "umi_rtx_interfaces/msg/board_coordinates.hpp"
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
 * @brief This class uses OpenCV
 * 
 */
 struct GridSquare {
    cv::Point2f center;
    std::vector<cv::Point> contours;
    int area;
};

class Camera : public rclcpp::Node{
public:
    /**
     * @brief Sets up the block-matching algorithm and creates useful publishers.
     * Tries to open the stereo camera, calibrates it and rectifies the images.
     * 
     */
    Camera() : Node("control"), align_to_color(RS2_STREAM_COLOR){
        init_interfaces();
        init_camera();
    };

private:
    /**
     * @brief Work loop. Splits the views, computes the banana's pose(position and orientation),
     * computes the disparity map of the scene, computes the depth map of the scene and publishes this data.
     * 
     */
    void timer_callback();

    /**
     * @brief Creates an object to use the block-matching algorithm and publishers to display
     * useful images and coordinates.
     * 
     */
    void init_interfaces();

    /**
     * @brief Tries to open the stereo camera, then calibrates it and rectifies the views.
     * 
     */
    void init_camera();

    /**
     * @brief Binarizes the image to find the banana, finds it, computes its area,
     * computes its centroid's coordinates, gets its orientation and publish this data.
     * 
     * @param coord_msg Message to publish the position of the banana.
     * @param angles_msg Message to publish the orientation of the banana.
     */
    void get_colored_objects(geometry_msgs::msg::Pose pose_msg, rs2::depth_frame depth);

    /**
     * @brief Finds the fittest line with respect to the contour of the target and
     * computes its orientation.
     * 
     * @param contours The set of detected contours in the binarized image.
     */
    void get_angles(vector<vector<cv::Point>> &contours);

    /**
     * @brief Uses stereo images with easy-to-detect points on a chessboard and applies correspondence
     * algorithms to compute the intrinsic and extrinsic parameters of the stereo camera. It will look for 
     * an inner pattern. 
     * 
     */
    void stereo_calibration();

    /**
     * @brief Computes rectification parameters. Computes the rotation
     * matrices for each camera that (virtually) make both camera image planes the same plane.
     * Computes the joint undistortion and rectification transformation.
     * 
     */
    void stereo_rectification();

    /**
     * @brief Splits the stereo image (concatenation of the left and right views) into 
     * two separate images.
     * 
     */
    void stereo_split_views();

    /**
     * @brief Performs horizontal block matching between the views to compute disparity.
     * 
     */
    void stereo_get_disparity();

    /**
     * @brief Computes depth from disparity.
     * 
     */
    void stereo_get_depth();

    void get_grid_position();

    std::vector<GridSquare> board_coordinates;

    std::chrono::milliseconds loop_dt_ = 40ms;//! Work loop timer.

    rclcpp::TimerBase::SharedPtr timer_;//! Pointer to the timer.

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher; //! Image publisher of the left view with the contour of the target.
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr processed_pose_publisher; //! Pose publisher of the position and orientation of the target.
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher; //! Image publisher of the depth map of the scene.
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr edges_publisher;

    rclcpp::Publisher<umi_rtx_interfaces::msg::Board>::SharedPtr board_state_publisher; 
    rclcpp::Publisher<umi_rtx_interfaces::msg::BoardCoordinates>::SharedPtr board_coordinates_publisher; 

    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align_to_color;
    rs2::colorizer color_map;
    int available_device;

    cv::Mat colorFrameCV, depthFrameCV;
    cv::Mat depth_normalized;
    cv::Mat m_R, m_T, m_E, m_F;//! Extrinsic parameters.
    cv::Mat m_R1, m_R2, m_P1, m_P2, m_Q;//! Extrinsic parameters.

    cv::Ptr<cv::StereoSGBM> stereo;//! Block-matching algorithm instance.

    bool hasPlayed;
    int m_frame_width, m_frame_height;//! Size of the stereo images.
    int m_depth_frame_width, m_depth_frame_height;
    int blockSize = 7;//!
    int min_disp = 0;//!
    int max_disp = 80;//!
    int num_disp = max_disp - min_disp;//!
    int uniquenessRatio = 10;//!
    int speckleWindowSize = 200;//!
    int speckleRange = 2;//!
    int disp12MaxDiff = 0;//!
    int iP1 = 8 * 1 * blockSize * blockSize;//!
    int iP2 = 16 * 1 * blockSize * blockSize;//!

    //cv::Size m_patternSize(7,5);
    float m_squareSize = 3.1;//! Size of the pattern used to calibrate.

    double m_baseline = 6.3; // centimeters //! Distance between the optical axes.
    double m_focalLength = 2.8;// millimeters //! Focal length.
    double m_rms_error;//! Calibration root-mean-square error.
    double m_cx, m_cy, m_cz, yaw, pitch, roll;//! Pose coordinates of the target (position and orientation).
    double depth_factor = 1.5;//! Proportionality factor to get depth from disparity.


};


#endif