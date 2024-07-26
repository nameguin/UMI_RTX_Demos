#include "umi_rtx_controller/node_game.hpp"

void Game_node::init_interfaces(){
    // Initialize the timer to call `timer_callback` at regular intervals
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Game_node::timer_callback, this));

    // Create a subscription to receive board state messages
    board_state_subscription = this->create_subscription<umi_rtx_interfaces::msg::Board>("board_state",10,
        std::bind(&Game_node::get_board, this, _1));

    // Create publishers for game data and robot's next move
    game_data_publisher  = this->create_publisher<umi_rtx_interfaces::msg::GameData>("game_data",10);
    robot_next_move_publisher  = this->create_publisher<std_msgs::msg::Int32>("robot_next_move",10);
}

void Game_node::timer_callback(){
    std_msgs::msg::Int32 robot_next_move_msg;
    data_msg.isgamestarted = false;
    if(game_is_started){
        if(!is_initialized){
            // Initialize the game state and clear the moves history
            std::memset(moves_history, 0, 9 * sizeof(int));
            if(!check_endgame()){
                update_turn();
                primary_msg = "Turn " +  std::to_string(turn);

                // Determine the next move based on the player's turn
               if(player_turn == ROBOT){
                    data_msg.isrobotturn = true;
                    secondary_msg = "Robot";
                    robot_next_move = findBestMove();
                    robot_next_move_msg.data = robot_next_move.row * 3 + robot_next_move.col;
                    robot_next_move_publisher->publish(robot_next_move_msg);
                }
                else {
                    data_msg.isrobotturn = false;
                    secondary_msg = "Human";
                }
            }
            is_initialized = true;
        }
        else if(!check_changes()){
            // If there are no changes, update the turn
            update_turn();

            primary_msg = "Turn " +  std::to_string(turn);
            if(player_turn == ROBOT){
                data_msg.isrobotturn = true;
                secondary_msg = "Robot";
                robot_next_move = findBestMove();
                robot_next_move_msg.data = robot_next_move.row * 3 + robot_next_move.col;
                robot_next_move_publisher->publish(robot_next_move_msg);
            }
            else {
                data_msg.isrobotturn = false;
                secondary_msg = "Human";
            }
        }
        data_msg.isgamestarted = true;
    }

    // Populate the data message with the current game state
    for(int i = 0; i < 9; ++i){
        data_msg.board.data[i] = board[i/3][i%3];
        data_msg.moveshistory[i] = moves_history[i].msg;
    }

    // Publish the updated game data
    data_msg.primarymsg = primary_msg;
    data_msg.secondarymsg = secondary_msg;

    game_data_publisher->publish(data_msg);
}

bool Game_node::check_changes(){
    for(int i = 0; i < BOARD_SIZE; i++){
        for(int j = 0; j < BOARD_SIZE; j++){
            if(board[i][j] != last_board[i][j]){
                // Handle addition and deletion of moves
                if(board[i][j] == 0){
                    delete_move(i * 3 + j);
                }
                else if(last_board[i][j] == 0){
                    add_move(i * 3 + j, board[i][j]);
                }
                else {
                    delete_move(i * 3 + j);
                    add_move(i * 3 + j, board[i][j]);
                }
                if(check_endgame())
                    return true;
            }
        }
    }  
    return false;   
}

void Game_node::add_move(int box, int player){
    if(player == ROBOT)
        moves_history[turn - 1].msg = "[Turn " +  std::to_string(turn) + "] Robot played in ( " + std::to_string(box / 3) + " ; " + std::to_string(box % 3) + " )";
    else 
        moves_history[turn - 1].msg = "[Turn " +  std::to_string(turn) + "] Human played in ( " + std::to_string(box / 3) + " ; " + std::to_string(box % 3) + " )";
    moves_history[turn - 1].box = box;
    turn++;
}

void Game_node::delete_move(int box){
    int index = -1;
    for(int i = 0; i < 9; ++i)
        if(moves_history[i].box == box) {
            index = i;
            break;
        }
    moves_history[turn - 1].box = -1;
    moves_history[turn - 1].msg = "";

    if (index != -1) {
        // Shift the remaining moves down in the history
        for (int i = index; i < turn; ++i) {
            moves_history[i].msg = moves_history[i + 1].msg;
            if(moves_history[i].msg.size() > 6)
                moves_history[i].msg.at(6) -= 1;
            moves_history[i].box = moves_history[i + 1].box;
        }
    }
    turn--;
}

bool Game_node::check_endgame(){
    if(hasWinner(ROBOT)){
        is_initialized = false;
        game_is_started = false;

        primary_msg = "Defeat !!!";
        secondary_msg = "You lost";
        return true;
    }   
    else if(hasWinner(HUMAN)){
        is_initialized = false;
        game_is_started = false;

        primary_msg = "Victory !!!";
        secondary_msg = "You won";
        return true;
    }   
    else if(isBoardFull()){
        is_initialized = false;
        game_is_started = false;

        primary_msg = "Draw !!!";
        secondary_msg = "Nobody won";
        return true;
    }
    return false;
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

void Game_node::update_turn(){
    int count[2] = {0,0};
    for (int i = 0; i < BOARD_SIZE; ++i) {
        for (int j = 0; j < BOARD_SIZE; ++j) {
            if (board[i][j] != 0) {
                count[board[i][j] - 1]++;
            }
        }
    }
    turn = count[0] + count[1] + 1;

    // Determine whose turn it is based on move counts
    if(count[0] == count[1])
        player_turn = starter;
    else if(count[0] > count[1])
        player_turn = 2;
    else 
        player_turn = 1;
}

bool Game_node::hasWinner(int player) {
    
    // Check rows for a win
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

    // Check main diagonal for a win
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

    // Check anti-diagonal for a win
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
    // Check rows for a win or loss
    for (int i = 0; i < BOARD_SIZE; ++i) {
        if (board[i][0] == board[i][1] && board[i][1] == board[i][2]) {
            if (board[i][0] == ROBOT) return 1;
            else if (board[i][0] == HUMAN) return -1;
        }
    }
    // Check columns for a win or loss
    for (int j = 0; j < BOARD_SIZE; ++j) {
        if (board[0][j] == board[1][j] && board[1][j] == board[2][j]) {
            if (board[0][j] == ROBOT) return 1;
            else if (board[0][j] == HUMAN) return -1;
        }
    }
    // Check main diagonal for a win or loss
    if (board[0][0] == board[1][1] && board[1][1] == board[2][2]) {
        if (board[0][0] == ROBOT) return 1;
        else if (board[0][0] == HUMAN) return -1;
    }
    // Check anti-diagonal for a win or loss
    if (board[0][2] == board[1][1] && board[1][1] == board[2][0]) {
        if (board[0][2] == ROBOT) return 1;
        else if (board[0][2] == HUMAN) return -1;
    }
    return 0;
}

bool Game_node::isGameOver() {
    // Check rows for a win
    for (int i = 0; i < BOARD_SIZE; ++i) {
        if (board[i][0] != 0 && board[i][0] == board[i][1] && board[i][1] == board[i][2]) {
            return true;
        }
    }
    // Check columns for a win
    for (int j = 0; j < BOARD_SIZE; ++j) {
        if (board[0][j] != 0 && board[0][j] == board[1][j] && board[1][j] == board[2][j]) {
            return true;
        }
    }
    // Check diagonals for a win
    if (board[0][0] != 0 && board[0][0] == board[1][1] && board[1][1] == board[2][2]) {
        return true;
    }
    if (board[0][2] != 0 && board[0][2] == board[1][1] && board[1][1] == board[2][0]) {
        return true;
    }
    // Check if the board is full
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

void Game_node::get_board(const umi_rtx_interfaces::msg::Board::SharedPtr msg){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++) {
            last_board[i][j] = board[i][j];
            board[i][j] = msg->data[i * 3 + j];
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