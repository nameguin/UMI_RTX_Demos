/**
 * @file node_invkin.hpp
 * @brief Déclaration des fonctions et méthodes pour le nœud d'inverse kinematics (IK).
 */

#ifndef __INVKIN_H__
#define __INVKIN_H__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "umi_rtx_controller/umi-drivers/rtx.h"

#include <map>
#include <math.h>
#include <vector>
#include <cmath>
#include <string>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

/**
 * @class InvKin_node
 * @brief ROS 2 node for performing inverse kinematics.
 * 
 * This class represents a ROS 2 node that performs inverse kinematics to compute joint states
 * required for a robotic arm to reach a specified target pose and grip. It uses the Pinocchio
 * library to handle kinematics calculations based on the URDF model of the robotic arm.
 * 
 * @details
 * The `InvKin_node` class inherits from `rclcpp::Node` and provides functionality to:
 * - Initialize ROS 2 interfaces including publishers and subscribers.
 * - Read the URDF description of the robotic arm for kinematics calculations.
 * - Compute the required joint angles to achieve a given target pose and grip using inverse kinematics.
 * - Publish the computed joint states.
 * 
 * It includes methods for:
 * - Handling incoming target pose and grip commands.
 * - Calculating joint states through inverse kinematics.
 * - Correcting joint angles to be within a specified range.
 */
class InvKin_node : public rclcpp::Node{
public:
    /**
     * @brief Construct a new InvKin_node object
     * 
     */
    InvKin_node() : Node("inverse_kinematics") {
        init_interfaces();
        pinocchio::urdf::buildModel(urdf_file,model);
        data = pinocchio::Data(model);
        J = pinocchio::Data::Matrix6x(6,model.nv);
    };

private:
    /**
     * @brief Initialize the timer, subscribers and publishers
     */
    void init_interfaces();
    /**
     * @brief Timer callback, actions that will be done at every iterations
     */
    void timer_callback();
    /**
     * @brief Get the targeted position
     *
     * @param msg 
     */
    void get_pose(const geometry_msgs::msg::Pose::SharedPtr msg);
    /**
     * @brief Get the targeted grip
     * 
     * @param msg 
     */
    void get_grip(const std_msgs::msg::Float32::SharedPtr msg);
    
    /**
     * @brief Processed the joints' states required to reach the desired pose, using an inverse kinematics algorithm
     * 
     * @param x Targeted x
     * @param y Targeted y
     * @param z Targeted z
     */
    void get_state(double x, double y, double z);

    /**
     * @brief Put joints' angles in [-pi,pi]
     * 
     * @param q 
     */
    void correct_angle(Eigen::VectorXd &q);

    std::chrono::milliseconds loop_dt_ = 40ms; // Timer of the node
    map<int,double> state;

    float L = 0.15; // Length of the hand

    double last_x,last_y,last_z;
    int ROLL=6, PITCH=7;

    double target_yaw, last_yaw;
    double target_pitch, last_pitch;
    double target_roll, last_roll;
    double target_grip, lats_grip;

    string urdf_file = ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/urdf/umi_rtx.urdf";

    // pinocchio variables and constants for inverse kinematics
    pinocchio::Model model;
    pinocchio::Data data;
    const int JOINT_ID = 6;
    const double eps  = 1e-2;
    const int IT_MAX  = 1000;
    const double DT   = 1e-2;
    const double damp = 1e-12;
    pinocchio::Data::Matrix6x J;
    Eigen::VectorXd q;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr grip_subscription;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr angles_publisher;

};



#endif