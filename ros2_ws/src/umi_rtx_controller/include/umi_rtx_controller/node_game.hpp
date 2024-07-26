/**
 * @file node_game.hpp
 * @brief Node for managing the game logic and interactions for a tic-tac-toe game.
 * 
 * This file defines the `Game_node` class which handles game initialization, game state updates,
 * move management, and communication with other ROS nodes.
 * 
 * The node subscribes to board state updates, and publishes game data and the robot's next move.
 */

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

// Definition of the board size
const int BOARD_SIZE = 3;
const int ROBOT = 1;
const int HUMAN = 2;

struct Position {
    int row;
    int col; 
};

struct Move_msg {
    std::string msg;
    int box; 
};

/**
 * @class Game_node
 * @brief ROS 2 node for managing and controlling a game.
 * 
 * This class handles the logic for a game of Tic-Tac-Toe, including managing game state, processing moves, and 
 * interacting with other components via ROS 2 messages and services.
 * 
 * @details
 * The `Game_node` class inherits from `rclcpp::Node` and provides functionality to:
 * - Initialize game settings and ROS 2 interfaces.
 * - Handle game logic such as determining moves for AI player.
 * - Publish game state and move information.
 * - Subscribe to game state updates and readiness signals.
 * 
 * It includes mechanisms for:
 * - Randomly selecting the starting player.
 * - Performing game moves and evaluating game state.
 * - Using the minimax algorithm for AI decision-making.
 * - Publishing and subscribing to relevant ROS 2 topics.
 */
class Game_node : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Game_node object.
     */
    Game_node() : Node("game"), turn(1), game_is_started(true), is_initialized(false) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(1, 2);
        starter = 1;
        player_turn = starter;

        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j){
                board[i][j] = 0;
                last_board[i][j] = 0;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
        init_interfaces();
    };

private:
    /**
     * @brief Initialize the timer, subscribers, and publishers.
     */
    void init_interfaces();

    /**
     * @brief Timer callback, actions that will be done at every iteration.
     */
    void timer_callback();

    /**
     * @brief Callback to get the board state from the subscription.
     * 
     * @param msg Shared pointer to the received board state message.
     */
    void get_board(const umi_rtx_interfaces::msg::Board::SharedPtr msg);

    /**
     * @brief Callback to get the has_played state from the subscription.
     * 
     * @param msg Shared pointer to the received boolean message.
     */
    void get_ready_to_play(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Get a random move for the AI.
     * 
     * @return Position The chosen position.
     */
    Position getAIMove();

    /**
     * @brief Get available positions on the board.
     * 
     * @return vector<Position> A vector of available positions.
     */
    vector<Position> getAvailablePositions();

    /**
     * @brief Check if a player has won.
     * 
     * @param player The player to check for victory (ROBOT or HUMAN).
     * @return true If the player has won.
     * @return false If the player has not won.
     */
    bool hasWinner(int player);

    /**
     * @brief Determine the current player's turn.
     * 
     * @return int The current player's turn (ROBOT or HUMAN).
     */
    void update_turn();

    /**
     * @brief Check if the board is full.
     * 
     * @return true If the board is full.
     * @return false If the board is not full.
     */
    bool isBoardFull();

    /**
     * @brief Display the board on the console.
     */
    void displayBoard();

    /**
     * @brief Find the best move for the robot using the minimax algorithm.
     * 
     * @return Position The best position for the robot's next move.
     */
    Position findBestMove();

    /**
     * @brief Minimax algorithm to evaluate the board state.
     * 
     * @param depth Current depth in the minimax tree.
     * @param isMaximizer Boolean flag to indicate if the current node is maximizing or minimizing.
     * @return int The evaluation score.
     */
    int minimax(int depth, bool isMaximizer);

    /**
     * @brief Check if the game is over.
     * 
     * @return true If the game is over.
     * @return false If the game is not over.
     */
    bool isGameOver();

    /**
     * @brief Evaluate the board state to determine the score.
     * 
     * @return int The evaluation score.
     */
    int evaluate();

    /**
    * @brief Check for endgame conditions.
    * 
    * @return true if the game has ended, false otherwise.
    */
    bool check_endgame();

    /**
    * @brief Check for changes in the board and update move history accordingly.
    * 
    * @return true if changes are detected, false otherwise.
    */
    bool check_changes();

    /**
    * @brief Add a move to the history.
    * 
    * @param box The position on the board (0-8) where the move occurred.
    * @param player The player making the move (ROBOT or HUMAN).
    */
    void add_move(int box, int player);

    /**
    * @brief Delete a move from the history.
    * 
    * @param box The position on the board (0-8) of the move to be deleted.
    */
    void delete_move(int box);


    // Timer loop duration
    std::chrono::milliseconds loop_dt_ = 40ms; 

    // Game information
    bool game_is_started;
    bool is_initialized;
    int board[3][3];
    int last_board[3][3];
    Position robot_next_move;
    int turn; 
    int player_turn;
    int starter;
    Move_msg moves_history[9];
    std::string primary_msg;
    std::string secondary_msg;

    // Game controllers
    bool ready_to_play;
    bool is_finished;

    // Messages
    std_msgs::msg::String last_move_msg;
    umi_rtx_interfaces::msg::GameData data_msg; 

    // Publishers
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Publisher<umi_rtx_interfaces::msg::GameData>::SharedPtr game_data_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr robot_next_move_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr last_move_publisher;

    rclcpp::Subscription<umi_rtx_interfaces::msg::Board>::SharedPtr board_state_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_to_play_subscription;
};

#endif // NODE_GAME_HPP
