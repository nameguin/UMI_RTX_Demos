#include "umi_rtx_controller/node_commands.hpp"

void Objective_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Objective_node::timer_callback, this));

    pose_publisher  = this->create_publisher<geometry_msgs::msg::Pose>("target_pose",10);
    grip_publisher  = this->create_publisher<std_msgs::msg::Float32>("target_grip",10);
    has_played_publisher  = this->create_publisher<std_msgs::msg::Bool>("has_played",10);

    pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose>("processed_pose",10,
        std::bind(&Objective_node::get_processed_pose, this, _1));
    processed_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("processed_image",10,
        std::bind(&Objective_node::get_processed_image, this, _1));
    depth_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("depth_image",10,
        std::bind(&Objective_node::get_depth_image, this, _1));
    edges_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("edges_image",10,
        std::bind(&Objective_node::get_edges_image, this, _1));

    game_data_subscriber = this->create_subscription<umi_rtx_interfaces::msg::GameData>("game_data",10,
        std::bind(&Objective_node::get_game_data, this, _1));

    robot_next_move_subscriber = this->create_subscription<std_msgs::msg::Int32>("robot_next_move",10,
        std::bind(&Objective_node::get_robot_next_move, this, _1));

    last_move_subscriber = this->create_subscription<std_msgs::msg::String>("last_move",10,
        std::bind(&Objective_node::get_move_history, this, _1));

    board_coordinates_subscriber = this->create_subscription<umi_rtx_interfaces::msg::BoardCoordinates>("board_coordinates",10,
        std::bind(&Objective_node::get_board_coordinates, this, _1));


}

void Objective_node::timer_callback(){
    std_msgs::msg::Bool has_played_msg;
    has_played_msg.data = false;
    /*
    We follow a simple trajectory between each step :
        - Open the grip and goes to the target
        - Close the grip to grab the object
        - Go back to initial place
        - Wait
        - We put the object to a predifined position
    */
    //dt*=2;
    if (mode != "manual"){  
        // Lissajou();
        // x = processed_x;
        // y = processed_y;
        // z = processed_z;
        // yaw = atan2(y,x)*180/M_PI;
        
        if (!is_initialized){
            //target_object.x = processed_x;
            //target_object.y = processed_y;
            //target_object.z = processed_z;

            target_object.x = -0.3;
            target_object.y = 0.25;
            target_object.z = 0.2;

            is_initialized=true;
        }
        target_place.x = board_coordinates[robot_next_move].x;
        target_place.y = board_coordinates[robot_next_move].y;
        target_place.z = board_coordinates[robot_next_move].z;

        if(player_turn == 1 and result == -1){
            bool valid_points = are_valid_points();

            if ((t-t0) < 8){
                double t_duration = 8;

                x = x0 + (0.-x0)*(t-t0) / t_duration;
                y = y0 + (0.45-y0)*(t-t0) / t_duration;
                z = z0 + (0.7-z0)*(t-t0) / t_duration;

                pitch = pitch0 + (90.-pitch0)*(t-t0) / t_duration;
                roll = roll0 + (0.-roll0)*(t-t0) / t_duration;
                grip = 0.02;
            }
            else if ((t-t0) < 14){
                x0 = x;
                y0 = y;
                z0 = z;

                roll0 = roll;
                pitch0 = pitch;
            }

            else if ((t-t0) < 20 and valid_points){
                double t_delta = t - t0 - 14;
                double t_duration = 6;

                x = x0 + (target_object.x-x0) * t_delta / t_duration;
                y = y0 + (target_object.y-y0) * t_delta / t_duration;

                pitch = pitch0 + (90.-pitch0) * t_delta / t_duration;
                roll = roll0 + (0.-roll0) * t_delta / t_duration;
            } 

            else if ((t-t0) < 26 and valid_points){
                double t_delta = t - t0 - 20;
                double t_duration = 6;

                z = z0 + (target_object.z-z0) * t_delta / t_duration;
            } 

            else if ((t-t0) < 32){
                x0 = x;
                y0 = y;
                z0 = z;

                roll0 = roll;
                pitch0 = pitch;
                grip = 0.02 + (0.08-0.02)*(t-t0-26)/6;
            } 

            else if ((t-t0) < 34 and valid_points){
                double t_delta = t - t0 - 32;
                double t_duration = 2;
                z = z0 + (target_place.z + 0.2 - z0) * t_delta / t_duration;
            } 

            else if ((t-t0) < 46 and valid_points){
                double t_delta = t - t0 - 34;
                double t_duration = 12;

                x = x0 + (target_place.x-x0) * t_delta / t_duration;
                y = y0 + (target_place.y-y0) * t_delta / t_duration;
                z0 = z;

                pitch = pitch0 + (90.-pitch0) * t_delta / t_duration;
                roll = roll0 + (0.-roll0) * t_delta / t_duration;
            } 
            else if ((t-t0) < 48 and valid_points){
                double t_delta = t - t0 - 46;
                double t_duration = 2;
                z = z0 + (target_place.z -z0) * t_delta / t_duration;
            } 

            else if ((t-t0)<54){
                x0 = x;
                y0 = y;
                z0 = z;

                roll0 = roll;
                pitch0 = pitch;
                grip = 0.08 + (0.02-0.08)*(t-t0-48)/6;
            }

            else if ((t-t0) < 60){
                double t_delta = t - t0 - 54;
                double t_duration = 6;

                x = x0 + (0.-x0) * t_delta / t_duration;
                y = y0 + (0.45-y0) * t_delta / t_duration;
                z = z0 + (0.7-z0) * t_delta / t_duration;

                pitch = pitch0 + (90.-pitch0) * t_delta / t_duration;
                roll = roll0 + (0.-roll0) * t_delta / t_duration;
            } 
            else if ((t-t0) >= 60){
                has_played_msg.data = true;
            }
        }
        else {
            x0 = x;
            y0 = y;
            z0 = z;

            roll0 = roll;
            pitch0 = pitch;
            t0 = t;    
            has_played_msg.data = true;  
        }
    }

    else {
        is_initialized=false;
        /*
        If the arm is controlled manually we adapt the origin pose of the automatical procedure of grab mode
        */
        x0 = x;
        y0 = y;
        z0 = z;

        roll0 = roll;
        pitch0 = pitch;

        t0 = t;
    }
    t+=dt;

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = x;
    pose_msg.position.y = y;
    pose_msg.position.z = z;

    pose_msg.orientation.x = yaw; 
    pose_msg.orientation.y = pitch;
    pose_msg.orientation.z = roll;

    pose_publisher->publish(pose_msg);

    std_msgs::msg::Float32 grip_msg;
    grip_msg.data = grip;
    grip_publisher->publish(grip_msg);

    has_played_publisher->publish(has_played_msg);
}

bool Objective_node::are_valid_points(){
    if(target_object.x < -0.6 || target_object.x > 0.6
        || target_object.y < 0.2 || target_object.y > 0.7
        || target_object.z < 0.1 || target_object.z > 0.5)
        return false;

    if(target_place.x < -0.6 || target_place.x > 0.6
        || target_place.y < 0.2 || target_place.y > 0.7)
        return false;
    return true;
    
}

void Objective_node::get_board_coordinates(const umi_rtx_interfaces::msg::BoardCoordinates::SharedPtr msg){
    for(int i = 0; i < 9; i++){
        board_coordinates[i] = {msg->points[i].x / 100, 
                                msg->points[i].y / 100, 
                                msg->points[i].z / 100};
    }
}

void Objective_node::get_robot_next_move(const std_msgs::msg::Int32::SharedPtr msg){
    robot_next_move = msg->data;
}

void Objective_node::get_move_history(const std_msgs::msg::String::SharedPtr msg){
    recent_msgs.push_back(msg->data);
}

void Objective_node::get_game_data(const umi_rtx_interfaces::msg::GameData::SharedPtr msg){
    need_update = true;
    player_turn = msg->playerturn;
    result = msg->result;
    turn = msg->turn;
    for(int i = 0; i < 9; i++)
        board[i/3][i%3] = msg->board.data[i];
}

void Objective_node::get_processed_pose(const geometry_msgs::msg::Pose::SharedPtr msg){
    if((t-t0)>=8 and (t-t0)<14) {
        processed_x = msg->position.x;
        processed_y = msg->position.y;
        processed_z = msg->position.z;

        processed_yaw = msg->orientation.x;
        processed_pitch = msg->orientation.y;
        processed_roll = msg->orientation.z;
    }
}

void Objective_node::get_processed_image(const sensor_msgs::msg::Image::SharedPtr msg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    processed_frame = cv_ptr->image;
}

void Objective_node::get_depth_image(const sensor_msgs::msg::Image::SharedPtr msg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    depth_frame = cv_ptr->image;
}

void Objective_node::get_edges_image(const sensor_msgs::msg::Image::SharedPtr msg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    edges_frame = cv_ptr->image;
}

void Objective_node::update_state(double new_x, double new_y, double new_z, double new_yaw, double new_pitch, double new_roll, double new_grip){
    x = new_x;
    y = new_y;
    z = new_z;
    yaw = new_yaw;
    pitch = new_pitch;
    roll = new_roll;
    grip = new_grip;
}