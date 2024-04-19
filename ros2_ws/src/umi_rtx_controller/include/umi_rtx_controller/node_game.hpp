#ifndef NODE_GAME_HPP
#define NODE_GAME_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "umi_rtx_interfaces/msg/grid.hpp"
#include "umi_rtx_interfaces/msg/game_data.hpp"


#include <iostream>
#include <math.h>
#include <string>

using namespace std;
using namespace std::chrono_literals;
using namespace std::placeholders;

class Game_node : public rclcpp::Node{
public:
    /**
     * @brief Construct a new Game_node object
     */
    Game_node() : Node("game"){
        init_interfaces();
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
     * @brief Get the turn number of the game.
     *
     * @param msg
     */
    void get_grid(const umi_rtx_interfaces::msg::Grid::SharedPtr msg);

    void get_has_played(const std_msgs::msg::Bool::SharedPtr msg);

    std::chrono::milliseconds loop_dt_ = 40ms;

    bool robot_has_played;
    int robot_next_move;
    int turn;
    int grid[3][3];

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<umi_rtx_interfaces::msg::GameData>::SharedPtr game_data_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr robot_next_move_publisher;

    rclcpp::Subscription<umi_rtx_interfaces::msg::Grid>::SharedPtr grid_state_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr has_played_subscription;

};
#endif //NODE_GAME_HPP
