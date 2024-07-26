/**
 * @file node_simu.cpp
 * @brief Implementation of the Simu_node class for simulating joint states in ROS.
 * 
 * This file contains the implementation of the Simu_node class, which is responsible for:
 * - Initializing ROS interfaces (subscriptions and publications).
 * - Handling timer callbacks to publish joint states.
 * - Parsing a URDF file to initialize joint properties.
 * - Processing incoming joint commands and updating the joint states accordingly.
 * 
 * The main function initializes ROS, creates an instance of Simu_node, and starts spinning
 * to process incoming messages.
 */

#ifndef __SIMU_H__
#define __SIMU_H__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "umi_rtx_controller/umi-drivers/rtx.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>

#include "rapidxml.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;
using namespace rapidxml;

/**
 * @class Simu_node
 * @brief ROS 2 node for simulating a robotic arm.
 * 
 * This class represents a ROS 2 node designed for simulating the operation of a robotic arm. It handles the
 * initialization of the node's interfaces, processes commands for joint movements, and manages the simulation
 * of the robotic arm based on the URDF description.
 * 
 * @details
 * The `Simu_node` class inherits from `rclcpp::Node` and provides functionality to:
 * - Initialize ROS 2 publishers and subscribers.
 * - Read the URDF description to extract joint information.
 * - Process commands for joint states and simulate their effects.
 */
class Simu_node : public rclcpp::Node{
public:
    /**
     * @brief Construct a new Simu_node object
     * 
     */
    Simu_node() : Node("simulation"){
        init_interfaces();
        init_urdf();
    };

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
     * @brief Read the URDF description of the arm, to get the joints informations
     */
    void init_urdf();
    /**
     * @brief Get the commands that will be sent to the arm
     * 
     * @param msg States of the joints required to reach the desired position sent through 
     */
    void get_commands(const sensor_msgs::msg::JointState::SharedPtr msg);

    std::chrono::milliseconds loop_dt_ = 40ms;

    map<string,map<string, double>> dependent_joints;
    map<string,map<string, double>> free_joints;
    map<string,double> zeros;

    vector<string> joint_list;
    vector<string> names;

    string urdf_file = ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/urdf/umi_rtx.urdf";

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr invkin_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr simu_publisher;
    
};

#endif