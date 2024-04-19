#include "umi_rtx_controller/node_game.hpp"

void Game_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Game_node::timer_callback, this));

    grid_state_subscription = this->create_subscription<umi_rtx_interfaces::msg::Grid>("grid_state",10,
        std::bind(&Game_node::get_grid, this, _1));

    has_played_subscription = this->create_subscription<std_msgs::msg::Bool>("has_played",10,
        std::bind(&Game_node::get_has_played, this, _1));

    game_data_publisher  = this->create_publisher<umi_rtx_interfaces::msg::GameData>("game_data",10);
    robot_next_move_publisher  = this->create_publisher<std_msgs::msg::Int32>("robot_next_move",10);
}

void Game_node::timer_callback(){
    umi_rtx_interfaces::msg::GameData data_msg;

    if(robot_has_played){
        cout << "Oui" << endl;
        std_msgs::msg::Int32 robot_next_move_msg;
        robot_next_move++;
        robot_next_move_msg.data = robot_next_move;
        robot_next_move_publisher->publish(robot_next_move_msg);
    }
    cout << "Non" << endl;
        

    //cout << "grid[1][2] = " << grid[1][2] << endl;


}

void Game_node::get_has_played(const std_msgs::msg::Bool::SharedPtr msg){
    robot_has_played = msg->data;
}

void Game_node::get_grid(const umi_rtx_interfaces::msg::Grid::SharedPtr msg){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++) {
            grid[i][j] = msg->data[i * 3 + j];
        }
    }
}

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    shared_ptr<rclcpp::Node> node = make_shared<Game_node>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}