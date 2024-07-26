#include "umi_rtx_controller/node_commands.hpp"

void Objective_node::init_interfaces(){
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&Objective_node::timer_callback, this));

    pose_publisher  = this->create_publisher<geometry_msgs::msg::Pose>("target_pose",10);
    grip_publisher  = this->create_publisher<std_msgs::msg::Float32>("target_grip",10);
    capture_step_publisher  = this->create_publisher<std_msgs::msg::Bool>("capture_step",10);

    /*pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose>("processed_pose",10,
        std::bind(&Objective_node::get_processed_pose, this, _1));*/
    processed_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("processed_image",10,
        std::bind(&Objective_node::get_processed_image, this, _1));
    depth_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>("depth_image",10,
        std::bind(&Objective_node::get_depth_image, this, _1));

    game_data_subscriber = this->create_subscription<umi_rtx_interfaces::msg::GameData>("game_data",10,
        std::bind(&Objective_node::get_game_data, this, _1));

    robot_next_move_subscriber = this->create_subscription<std_msgs::msg::Int32>("robot_next_move",10,
        std::bind(&Objective_node::get_robot_next_move, this, _1));
}

void Objective_node::timer_callback(){
    std_msgs::msg::Bool capture_step_msg;
    capture_step_msg.data = false;
    if (mode != "manual"){  
        // Go to capture position
        if(t < 8){
            double t_duration = 8;

            x = x0 + (0.-x0)*t / t_duration;
            y = y0 + (0.45-y0)*t / t_duration;
            z = z0 + (0.7-z0)*t / t_duration;

            pitch = pitch0 + (90. -pitch0)*t / t_duration;
            roll = roll0 + (0.-roll0)*t / t_duration;
            grip = 0.02;

            if(t >= 8 - dt){
                x = 0.;
                y = 0.45;
                z = 0.7;

                pitch = 90.;
                roll = 0.;              
            }

        } 
        // Wait
        else if(t < 10){
            x0 = x;
            y0 = y;
            z0 = z;
            t0 = t; 

            roll0 = roll;
            pitch0 = pitch;

            target_object.x = 0.188;
            target_object.y = 0.545;
            target_object.z = 0.1;

            capture_step_msg.data = true;
        }
        else if(is_robot_turn and is_game_started){
            target_place.x = -0.188 + ((robot_next_move) % 3) * 0.120;
            target_place.y = 0.425 + ((robot_next_move) / 3) * 0.120;
            target_place.z = 0.1;

            // Go to the detected pawn position on x-axis and y-axis
            if ((t-t0) < 10){
                double t_delta = t - t0;
                double t_duration = 10;

                x = x0 + (target_object.x-x0) * t_delta / t_duration;
                y = y0 + (target_object.y-y0) * t_delta / t_duration;

                pitch = pitch0 + (0.-pitch0) * t_delta / t_duration;
                roll = roll0 + (0.-roll0) * t_delta / t_duration;

                if((t-t0) >= 10 - dt){
                    x = target_object.x;
                    y = target_object.y;
                    pitch = 0.;
                    roll = 0.;
                }
            } 
            
            // Go to the detected pawn position on z-axis
            else if ((t-t0) < 18){
                double t_delta = t - t0 - 10;
                double t_duration = 8;

                z = z0 + (target_object.z-z0) * t_delta / t_duration;

                if((t-t0) >= 18 - dt){
                    z = target_object.z;
                }
            } 

            // Close the grip
            else if ((t-t0) < 28){
                x0 = x;
                y0 = y;
                z0 = z;

                roll0 = roll;
                pitch0 = pitch;
                grip = 0.02 + (0.08-0.02)*(t-t0-18)/10;
            } 

            // Rise 20 cm
            else if ((t-t0) < 32){
                double t_delta = t - t0 - 28;
                double t_duration = 4;
                z = z0 + (target_place.z + 0.2 - z0) * t_delta / t_duration;

                if((t-t0) >= 32 - dt){
                    z = target_place.z + 0.2;
                }
            } 

            // Go to the box coordinate on x-axis and y-axis
            else if ((t-t0) < 44){
                double t_delta = t - t0 - 32;
                double t_duration = 12;

                x = x0 + (target_place.x-x0) * t_delta / t_duration;
                y = y0 + (target_place.y-y0) * t_delta / t_duration;
                z0 = z;

                pitch = pitch0 + (0.-pitch0) * t_delta / t_duration;
                roll = roll0 + (0.-roll0) * t_delta / t_duration;

                if((t-t0) >= 44 - dt){
                    x = target_place.x;
                    y = target_place.y;
                    pitch = 0.;
                    roll = 0.;
                }
            } 

            // Go to the box coordinate on z-axis
            else if ((t-t0) < 48){
                double t_delta = t - t0 - 44;
                double t_duration = 4;
                z = z0 + (target_place.z -z0) * t_delta / t_duration;

                if((t-t0) >= 48 - dt){
                    z = target_place.z;
                }
            } 

            // Open the grip
            else if ((t-t0) < 58){
                x0 = x;
                y0 = y;
                z0 = z;

                roll0 = roll;
                pitch0 = pitch;
                grip = 0.08 + (0.02-0.08)*(t-t0-48)/10;
            }

            // Go back to the capture position
            else if ((t-t0) < 66){
                double t_delta = t - t0 - 58;
                double t_duration = 8;

                x = x0 + (0.-x0) * t_delta / t_duration;
                y = y0 + (0.45-y0) * t_delta / t_duration;
                z = z0 + (0.7-z0) * t_delta / t_duration;

                pitch = pitch0 + (90.-pitch0) * t_delta / t_duration;
                roll = roll0 + (0.-roll0) * t_delta / t_duration; 

                if((t-t0) >= 66 - dt){
                    x = 0.;
                    y = 0.45;
                    z  = 0.7;
                    pitch = 90.;
                    roll = 0.;
                }
            } 

            // Wait
            else if ((t-t0) < 76){
                x0 = x;
                y0 = y;
                z0 = z;

                roll0 = roll;
                pitch0 = pitch;

                //grip = 0.02 + (0.08-0.02)*(t-t0-66)/10;
                capture_step_msg.data = true;
            } 
            else if ((t-t0) < 84){ 
                capture_step_msg.data = true;
            }
            else {
                t0 = t; 
            }
        }
        else {
            x0 = x;
            y0 = y;
            z0 = z;

            roll0 = roll;
            pitch0 = pitch;
            t0 = t;  

            capture_step_msg.data = true;      
        }
    }

    else {
        /*
        If the arm is controlled manually we adapt the origin pose of the automatical procedure of grab mode
        */
        x0 = x;
        y0 = y;
        z0 = z;

        roll0 = roll;
        pitch0 = pitch;

        t0 = t;
        capture_step_msg.data = true;
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

    capture_step_publisher->publish(capture_step_msg);
}

void Objective_node::get_robot_next_move(const std_msgs::msg::Int32::SharedPtr msg){
    robot_next_move = msg->data;
}

void Objective_node::get_game_data(const umi_rtx_interfaces::msg::GameData::SharedPtr msg){
    need_update = true;

    is_robot_turn = msg->isrobotturn;
    is_game_started = msg->isgamestarted;
    primary_msg = msg->primarymsg;
    secondary_msg = msg->secondarymsg;

    for(int i = 0; i < 9; i++){
        board[i/3][i%3] = msg->board.data[i];
        moves_history[i] = msg->moveshistory[i];
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

void Objective_node::update_state(double new_x, double new_y, double new_z, double new_yaw, double new_pitch, double new_roll, double new_grip){
    x = new_x;
    y = new_y;
    z = new_z;
    yaw = new_yaw;
    pitch = new_pitch;
    roll = new_roll;
    grip = new_grip;
}