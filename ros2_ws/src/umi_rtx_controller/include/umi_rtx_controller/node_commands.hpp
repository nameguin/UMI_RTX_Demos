/**
 * @file node_commands.hpp
 * @brief Node for managing and sending commands to a robotic arm based on image and game data.
 * 
 * This file defines the `Objective_node` class which handles the publication of target poses,
 * grip parameters, and step capture signals. It also subscribes to image data and game data,
 * processes this information, and adjusts the arm's actions accordingly. The class is designed
 * to break down the robot's movements into multiple steps to ensure precise and accurate control.
 */

#ifndef __OBJ_H__
#define __OBJ_H__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "umi_rtx_interfaces/msg/game_data.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include <QApplication>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QObject>

#include <iostream>
#include <math.h>
#include <string>
#include <deque>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

struct Point3D {
    double x;
    double y;
    double z;
};

/**
 * @class Objective_node
 * @brief Manages and sends commands to a robotic arm based on image and game data.
 * 
 * The Objective_node class handles the publication of target poses, grip parameters,
 * and step capture signals. It subscribes to image data and game data, processes this
 * information, and adjusts the arm's actions accordingly. The class is designed to break
 * down the robot's movements into multiple steps to ensure precise and accurate control.
 */
class Objective_node : public rclcpp::Node{
public:

    /**
    * @brief Construct a new Objective_node object.
    * 
    * Initializes the Objective_node with default values, sets up interfaces, and
    * initializes the game board and moves history that will be used for the interface.
    */
    Objective_node() : Node("objective"), count(1), need_update(false) {
        init_interfaces();
        for(int i = 0; i < 3; ++i){
            for(int j = 0;j < 3; ++j) {
                board[i][j] = 0;
                moves_history[i * 3 + j] = "";
            }
        }
    };

    /**
    * @brief Function that will be used at each iteration in the GUI to update the target that will be communicated.
    * 
    * This function updates the target state parameters including position, orientation,
    * and grip, which will be communicated to the robotic arm.
    * 
    * @param new_x New x-coordinate of the target position.
    * @param new_y New y-coordinate of the target position.
    * @param new_z New z-coordinate of the target position.
    * @param new_yaw New yaw angle of the target orientation.
    * @param new_pitch New pitch angle of the target orientation.
    * @param new_roll New roll angle of the target orientation.
    * @param new_grip New grip parameter for the robotic arm.
    */
    void update_state(double new_x, double new_y, double new_z, double new_yaw, double new_pitch, double new_roll, double new_grip);

    string mode="manual";
    cv::Mat processed_frame, depth_frame;
    Point3D target_object;
    Point3D target_place;
    double x=0., y=0.6, z=0.6, yaw=0.,pitch=0.,roll=0., grip=0.8;
    float t=0,dt=0.04;
    bool is_robot_turn = false;

    int board[3][3];

    std::string moves_history[9];
    std::string primary_msg = "";
    std::string secondary_msg = "";
    bool need_update;

private :
    /**
     * @brief Initialize the timer, subscribers and publishers
     */
    void init_interfaces();

    /**
    * @brief Timer callback function for controlling the robotic arm.
    * 
    * This function is executed at regular intervals and handles the state updates
    * and movements of the robotic arm. It adjusts the arm's position, orientation,
    * and grip based on the current mode and game state. The function also publishes
    * updated pose, grip, and capture step messages.
    * 
    * The function performs the following steps:
    * - Moves the robotic arm to a capture position.
    * - Waits for a brief period.
    * - If it is the robot's turn and the game has started:
    *   - Moves the arm to the detected pawn position.
    *   - Closes the grip to grab the pawn.
    *   - Raises the arm.
    *   - Moves the arm to the target placement position.
    *   - Lowers the arm.
    *   - Opens the grip to place the pawn.
    *   - Returns the arm to the capture position.
    * - If the arm is in manual mode, it adapts the origin pose for automatic procedures.
    * 
    * @note This function is part of the state machine controlling the robotic arm.
    */
    void timer_callback();

    /**
    * @brief Get the processed images published.
    * 
    * This function is a callback that processes the processed image received from a ROS topic.
    * It converts the ROS image message to an OpenCV image and stores it in the `processed_frame` member variable.
    * 
    * @param msg The ROS image message containing the processed image.
    */
    void get_processed_image(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
    * @brief Get the depth images published.
    * 
    * This function is a callback that processes the depth image received from a ROS topic.
    * It converts the ROS image message to an OpenCV image and stores it in the `depth_frame` member variable.
    * 
    * @param msg The ROS image message containing the depth image.
    */
    void get_depth_image(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
    * @brief Get the next move for the robot.
    * 
    * This function is a callback that processes the next move data received from a ROS topic.
    * It updates the `robot_next_move` member variable with the data from the received message.
    * 
    * @param msg The ROS message containing the box where the robot will play.
    */
    void get_robot_next_move(const std_msgs::msg::Int32::SharedPtr msg);

    /**
    * @brief Get the game data published.
    * 
    * This function is a callback that processes the game data received from a ROS topic.
    * It updates the internal state of the `Objective_node` based on the received game data.
    * 
    * @param msg The ROS message containing the game data.
    */
    void get_game_data(const umi_rtx_interfaces::msg::GameData::SharedPtr msg);
    
    std::chrono::milliseconds loop_dt_ = 40ms;

    int robot_next_move;
    
    bool is_game_started = false;
    int count;
    double x0,y0,z0,yaw0,pitch0,roll0,t0;

    double processed_x,processed_y,processed_z,processed_yaw,processed_pitch,processed_roll;
    double target_x,target_y,target_z, final_x,final_y,final_z;

    bool is_initialized=false;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr grip_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr capture_step_publisher;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr processed_image_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscriber;

    rclcpp::Subscription<umi_rtx_interfaces::msg::GameData>::SharedPtr game_data_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_next_move_subscriber;
};

#endif