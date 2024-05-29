#ifndef NODE_GAME_HPP
#define NODE_GAME_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "umi_rtx_interfaces/msg/board.hpp"
#include "umi_rtx_interfaces/msg/game_data.hpp"


#include <iostream>
#include <math.h>
#include <string>
#include <random>
#include <climits>
#include <chrono>
#include <thread>

using namespace std;
using namespace std::chrono_literals;
using namespace std::placeholders;

// Définition de la taille de la grille
const int BOARD_SIZE = 3;
const int ROBOT = 1;
const int HUMAN = 2;

// Structure représentant une position sur la grille
struct Position {
    int row;
    int col;
};

class Game_node : public rclcpp::Node{
public:
    /**
     * @brief Construct a new Game_node object
     */
    Game_node() : Node("game"), turn(1), human_has_played(true), robot_has_played(true), count(0){
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(1, 2);
        starter = dis(gen);
        player_turn = 2;
        data_msg.result = -1;

        for(int i = 0; i < 3; ++i)
            for(int j = 0;j < 3; ++j)
                board[i][j] = 0;

        std::this_thread::sleep_for(std::chrono::seconds(2));
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
    void get_board(const umi_rtx_interfaces::msg::Board::SharedPtr msg);

    void get_has_played(const std_msgs::msg::Bool::SharedPtr msg);

    Position getAIMove();

    vector<Position> getAvailablePositions();

    bool hasWinner(int player);

    int get_player_turn();

    bool isBoardFull();

    void displayBoard();

    Position findBestMove();

    int minimax(int depth, bool isMaximizer);

    bool isGameOver();

    int evaluate();

    std::chrono::milliseconds loop_dt_ = 40ms;

    bool robot_has_played;
    bool human_has_played;
    int starter;
    bool is_finished;
    Position robot_next_move;
    int turn;
    int player_turn;
    int board[3][3];
    int count;
    std::vector<std::string> msgs;
    std_msgs::msg::String last_move_msg;
    umi_rtx_interfaces::msg::GameData data_msg;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<umi_rtx_interfaces::msg::GameData>::SharedPtr game_data_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr robot_next_move_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr last_move_publisher;

    rclcpp::Subscription<umi_rtx_interfaces::msg::Board>::SharedPtr board_state_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr has_played_subscription;

};
#endif //NODE_GAME_HPP
