/**
 * @file node_commands.hpp
 * @author Théo MASSA (theo.massa@ensta-bretagne.org)
 * @brief Node associated with the GUI
 * @version 0.1
 * @date 2023-07-19
 * 
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
#include "umi_rtx_interfaces/msg/board_coordinates.hpp"

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
 * @brief Node that works in pair with the GUI, manage the targeted pose and the command we send to the arm. 
 */
class Objective_node : public rclcpp::Node{
public:
    /**
     * @brief Construct a new Objective_node object
     */
    Objective_node() : Node("objective"), count(1), need_update(false), turn(1){
        init_interfaces();
        for(int i = 0; i < 3; ++i)
            for(int j = 0;j < 3; ++j)
                board[i][j] = 0;
    };

    /**
     * @brief Function that will be used at each iteration in the GUI to update the target that will be communicated.
     * 
     * @param new_x 
     * @param new_y 
     * @param new_z 
     * @param new_yaw 
     * @param new_pitch 
     * @param new_roll 
     * @param new_grip 
     */
    void update_state(double new_x, double new_y, double new_z, double new_yaw, double new_pitch, double new_roll, double new_grip);

    string mode="manual";
    cv::Mat processed_frame, depth_frame, edges_frame;
    Point3D target_object;
    Point3D target_place;
    double x=0., y=0.6, z=0.6, yaw=0.,pitch=0.,roll=0., grip=0.8;
    int board[3][3];
    int turn;
    int player_turn;
    int result;
    std::deque<std::string> recent_msgs;
    bool need_update;

private :
    /**
     * @brief Initialize the timer, subscribers and publishers
     */
    void init_interfaces();
    /**
     * @brief Timer callback, actions that will be done at every iterations
     */
    void timer_callback();

    /**
     * @brief Get the targeted position processed by the camera.
     * 
     * @param msg 
     */
    void get_processed_pose(const geometry_msgs::msg::Pose::SharedPtr msg);
    /**
     * @brief Get the processed images published
     * 
     * @param msg 
     */
    void get_processed_image(const sensor_msgs::msg::Image::SharedPtr msg);
    /**
     * @brief Get the depth images published
     * 
     * @param msg 
     */
    void get_depth_image(const sensor_msgs::msg::Image::SharedPtr msg);

    void get_edges_image(const sensor_msgs::msg::Image::SharedPtr msg);

    void get_robot_next_move(const std_msgs::msg::Int32::SharedPtr msg);

    void get_game_data(const umi_rtx_interfaces::msg::GameData::SharedPtr msg);

    void get_move_history(const std_msgs::msg::String::SharedPtr msg);

    void get_board_coordinates(const umi_rtx_interfaces::msg::BoardCoordinates::SharedPtr msg);

    bool are_valid_points();
    
    std::chrono::milliseconds loop_dt_ = 40ms;

    int robot_next_move;
    bool has_played;
    int count;
    double x0,y0,z0,yaw0,pitch0,roll0,t0;
    float t=0,dt=0.04;
    Point3D board_coordinates[9];

    double processed_x,processed_y,processed_z,processed_yaw,processed_pitch,processed_roll;
    double target_x,target_y,target_z, final_x,final_y,final_z;

    bool is_initialized=false;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr grip_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr has_played_publisher;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr processed_image_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr edges_image_subscriber;
    
    rclcpp::Subscription<umi_rtx_interfaces::msg::BoardCoordinates>::SharedPtr board_coordinates_subscriber;

    rclcpp::Subscription<umi_rtx_interfaces::msg::GameData>::SharedPtr game_data_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_next_move_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr last_move_subscriber;
};

#endif