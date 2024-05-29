#include "umi_rtx_controller/node_game.hpp"

void Game_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Game_node::timer_callback, this));

    board_state_subscription = this->create_subscription<umi_rtx_interfaces::msg::Board>("board_state",10,
        std::bind(&Game_node::get_board, this, _1));

    has_played_subscription = this->create_subscription<std_msgs::msg::Bool>("has_played",10,
        std::bind(&Game_node::get_has_played, this, _1));

    game_data_publisher  = this->create_publisher<umi_rtx_interfaces::msg::GameData>("game_data",10);
    last_move_publisher  = this->create_publisher<std_msgs::msg::String>("last_move",10);
    robot_next_move_publisher  = this->create_publisher<std_msgs::msg::Int32>("robot_next_move",10);
}

void Game_node::timer_callback(){
    std_msgs::msg::Int32 robot_next_move_msg;
    if(robot_has_played && human_has_played){
        player_turn = get_player_turn();
        data_msg.playerturn = player_turn;
        data_msg.turn = turn;
        game_data_publisher->publish(data_msg);

        if(!is_finished){
            last_move_publisher->publish(last_move_msg);
            if(player_turn == ROBOT){
                robot_next_move = findBestMove();
                robot_next_move_msg.data = robot_next_move.row * 3 + robot_next_move.col;
                robot_next_move_publisher->publish(robot_next_move_msg);

                board[robot_next_move.row][robot_next_move.col] = 1;
                last_move_msg.data = "[Turn " +  std::to_string(turn) + "] Robot played in ( " + std::to_string(robot_next_move.row) + " ; " + std::to_string(robot_next_move.col) + " )";

                for(int i = 0; i < 9; ++i)
                    data_msg.board.data[i] = board[i/3][i%3];

                if(hasWinner(ROBOT)){
                    data_msg.result = ROBOT;
                    is_finished = true;
                }
                else if(isBoardFull()){
                    data_msg.result = 0;
                    is_finished = true;
                }
                robot_has_played = false;
            }
            else {
                Position move = getAIMove();
                board[move.row][move.col] = 2;
                last_move_msg.data = "[Turn " +  std::to_string(turn) + "] Human played in ( " + std::to_string(move.row) + " ; " + std::to_string(move.col) + " )";

                for(int i = 0; i < 9; ++i)
                    data_msg.board.data[i] = board[i/3][i%3];

                if(hasWinner(HUMAN)){
                    data_msg.result = HUMAN;
                    is_finished = true;
                }
                else if(isBoardFull()){
                    data_msg.result = 0;
                    is_finished = true;
                }
            }
        }
    }
    count++;
}

bool Game_node::isBoardFull() {
    for (int i = 0; i < BOARD_SIZE; ++i) {
        for (int j = 0; j < BOARD_SIZE; ++j) {
            if (board[i][j] == 0) {
                return false;
            }
        }
    }
    return true;
}

int Game_node::get_player_turn(){
    int count[2] = {0,0};
    for (int i = 0; i < BOARD_SIZE; ++i) {
        for (int j = 0; j < BOARD_SIZE; ++j) {
            if (board[i][j] != 0) {
                count[board[i][j] - 1]++;
            }
        }
    }
    turn = count[0] + count[1] + 1;
    if(count[0] == count[1])
        return starter;
    else if(count[0] > count[1])
        return 2;
    else 
        return 1;
}

bool Game_node::hasWinner(int player) {
    
    for (int i = 0; i < BOARD_SIZE; ++i) {
        bool win = true;
        for (int j = 0; j < BOARD_SIZE; ++j) {
            if (board[i][j] != player) {
                win = false;
                break;
            }
        }
        if (win) return true;
    }

    for (int j = 0; j < BOARD_SIZE; ++j) {
        bool win = true;
        for (int i = 0; i < BOARD_SIZE; ++i) {
            if (board[i][j] != player) {
                win = false;
                break;
            }
        }
        if (win) return true;
    }

    bool win = true;
    for (int i = 0; i < BOARD_SIZE; ++i) {
        if (board[i][i] != player) {
            win = false;
            break;
        }
    }
    if (win) return true;

    win = true;
    for (int i = 0; i < BOARD_SIZE; ++i) {
        if (board[i][BOARD_SIZE - i - 1] != player) {
            win = false;
            break;
        }
    }
    return win;
}

vector<Position> Game_node::getAvailablePositions() {
    vector<Position> availablePositions;
    for (int i = 0; i < BOARD_SIZE; ++i) {
        for (int j = 0; j < BOARD_SIZE; ++j) {
            if (board[i][j] == 0) {
                availablePositions.push_back({i, j});
            }
        }
    }
    return availablePositions;
}

Position Game_node::getAIMove() {
    vector<Position> availablePositions = getAvailablePositions();
    srand(time(0));
    int index = rand() % availablePositions.size();
    return availablePositions[index];
}

int Game_node::evaluate() {
    for (int i = 0; i < BOARD_SIZE; ++i) {
        if (board[i][0] == board[i][1] && board[i][1] == board[i][2]) {
            if (board[i][0] == ROBOT) return 1;
            else if (board[i][0] == HUMAN) return -1;
        }
    }
    for (int j = 0; j < BOARD_SIZE; ++j) {
        if (board[0][j] == board[1][j] && board[1][j] == board[2][j]) {
            if (board[0][j] == ROBOT) return 1;
            else if (board[0][j] == HUMAN) return -1;
        }
    }
    if (board[0][0] == board[1][1] && board[1][1] == board[2][2]) {
        if (board[0][0] == ROBOT) return 1;
        else if (board[0][0] == HUMAN) return -1;
    }
    if (board[0][2] == board[1][1] && board[1][1] == board[2][0]) {
        if (board[0][2] == ROBOT) return 1;
        else if (board[0][2] == HUMAN) return -1;
    }
    return 0;
}

bool Game_node::isGameOver() {
    for (int i = 0; i < BOARD_SIZE; ++i) {
        if (board[i][0] != 0 && board[i][0] == board[i][1] && board[i][1] == board[i][2]) {
            return true;
        }
    }
    for (int j = 0; j < BOARD_SIZE; ++j) {
        if (board[0][j] != 0 && board[0][j] == board[1][j] && board[1][j] == board[2][j]) {
            return true;
        }
    }
    if (board[0][0] != 0 && board[0][0] == board[1][1] && board[1][1] == board[2][2]) {
        return true;
    }
    if (board[0][2] != 0 && board[0][2] == board[1][1] && board[1][1] == board[2][0]) {
        return true;
    }
    for (int i = 0; i < BOARD_SIZE; ++i) {
        for (int j = 0; j < BOARD_SIZE; ++j) {
            if (board[i][j] == 0) {
                return false;
            }
        }
    }
    return true;
}

int Game_node::minimax(int depth, bool isMaximizing) {
    if (isGameOver()) {
        int score = evaluate();
        return score;
    }

    if (isMaximizing) {
        int bestScore = -1000;
        for (int i = 0; i < BOARD_SIZE; ++i) {
            for (int j = 0; j < BOARD_SIZE; ++j) {
                if (board[i][j] == 0) {
                    int temp = board[i][j];
                    board[i][j] = ROBOT;
                    int score = minimax(depth + 1, false);
                    board[i][j] = 0;
                    bestScore = max(score, bestScore);
                }
            }
        }
        return bestScore;
    } else {
        int bestScore = 1000;
        for (int i = 0; i < BOARD_SIZE; ++i) {
            for (int j = 0; j < BOARD_SIZE; ++j) {
                if (board[i][j] == 0) {
                    board[i][j] = HUMAN;
                    int score = minimax(depth + 1, true);
                    board[i][j] = 0;
                    bestScore = min(score, bestScore);
                }
            }
        }
        return bestScore;
    }
}

Position Game_node::findBestMove() {
    int bestVal = -1000;
    Position bestMove;
    bestMove.row = -1;
    bestMove.col = -1;

    for (int i = 0; i < BOARD_SIZE; ++i) {
        for (int j = 0; j < BOARD_SIZE; ++j) {
            if (board[i][j] == 0) {
                board[i][j] = ROBOT;
                int moveVal = minimax(0, false);
                board[i][j] = 0;
                if (moveVal > bestVal) {
                    bestMove.row = i;
                    bestMove.col = j;
                    bestVal = moveVal;
                }
            }
        }
    }
    return bestMove;
}

void Game_node::get_has_played(const std_msgs::msg::Bool::SharedPtr msg){
    robot_has_played = msg->data;
}

void Game_node::get_board(const umi_rtx_interfaces::msg::Board::SharedPtr msg){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++) {
            //board[i][j] = msg->data[i * 3 + j];
        }
    }
}

void Game_node::displayBoard() {
    for (int i = 0; i < BOARD_SIZE; ++i) {
        for (int j = 0; j < BOARD_SIZE; ++j) {
            char symbol = ' ';
            switch (board[i][j]) {
                case 0:
                    symbol = '-';
                    break;
                case 1:
                    symbol = 'X';
                    break;
                case 2:
                    symbol = 'O';
                    break;
            }
            cout << symbol << " ";
        }
        cout << endl;
    }
}

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    shared_ptr<rclcpp::Node> node = make_shared<Game_node>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}