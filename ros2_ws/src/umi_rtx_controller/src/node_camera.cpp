#include "umi_rtx_controller/node_camera.hpp"

void Camera::init_interfaces(){
    // Create a timer that calls the timer_callback function at regular intervals defined by loop_dt_
    timer_ = this->create_wall_timer(loop_dt_,std::bind(&Camera::timer_callback,this));

    // Initialize the publisher for processed images
    image_publisher = this->create_publisher<sensor_msgs::msg::Image>("processed_image",10);
    
    // Initialize the publisher for processed pose data
    processed_pose_publisher = this->create_publisher<geometry_msgs::msg::Pose>("processed_pose",10);
    
    // Initialize the publisher for depth images
    depth_publisher = this->create_publisher<sensor_msgs::msg::Image>("depth_image",10);
    
    // Initialize the publisher to signal readiness to play
    ready_to_play_publisher = this->create_publisher<std_msgs::msg::Bool>("ready_to_play",10);

    // Initialize the publisher for board state information
    board_state_publisher = this->create_publisher<umi_rtx_interfaces::msg::Board>("board_state",10);

    // Initialize the subscriber to capture step commands
    capture_step_subscriber = this->create_subscription<std_msgs::msg::Bool>("capture_step",10,
        std::bind(&Camera::get_capture_step, this, _1));
}

void Camera::init_camera(){
    std::cout << "Checking for available cameras... " << std::endl;

    // Create a context for querying connected devices
    rs2::context ctx;
    auto device_list = ctx.query_devices(); // Query the list of connected devices
    available_device = device_list.size(); // Store the number of available devices

    // Output the number of connected RealSense devices
    if (available_device == 0)
        std::cout << "No RealSense camera detected!" << std::endl << std::endl;
    else if(available_device == 1)
        std::cout << "There is " << available_device << " connected RealSense device." << std::endl << std::endl;
    else
        std::cout << "There are " << available_device << " connected RealSense devices." << std::endl << std::endl;

    // Initialize the first available device if any are found
    if(available_device > 0) {
        auto dev = device_list[0]; 
        std::cout << "Using device 0 " << std::endl;
        std::cout << "Name: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        std::cout << "Serial number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        std::cout << "Firmware version: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl; 

        // Set frame dimensions
        m_frame_width = 1280;
        m_frame_height = 720;

        // Configure the depth and color streams
        cfg.enable_stream(RS2_STREAM_DEPTH, m_frame_width/2, m_frame_height/2, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, m_frame_width, m_frame_height, RS2_FORMAT_BGR8, 30);
    }

    try {
        // Start the pipeline with the configured settings
        pipe.start(cfg);
        auto sensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>(); // Get the depth sensor
        sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY); // Set visual preset to high accuracy
    } catch (const rs2::error& e) {
        std::cerr << "Librealsense error: " << e.what() << std::endl;
    }

    // Configure the color map options
    color_map.set_option(RS2_OPTION_MIN_DISTANCE, 0.0f); // Minimum distance for depth color mapping
    color_map.set_option(RS2_OPTION_MAX_DISTANCE, 6.0f); // Maximum distance for depth color mapping
    color_map.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, false); // Disable histogram equalization
}

void Camera::timer_callback(){
    geometry_msgs::msg::Pose pose_msg; // Pose message to be published

    // Check if there is at least one available device
    if(available_device > 0) {
        // Wait for the next set of frames from the camera
        rs2::frameset frames = pipe.wait_for_frames();

        // Align the depth frame to the color frame for better accuracy
        frames = align_to_color.process(frames);

        // Retrieve the depth frame, colored depth frame, and color frame
        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::frame coloredDepth = frames.get_depth_frame().apply_filter(color_map); // Apply color map filter to depth frame
        rs2::frame color = frames.get_color_frame();

        // Get the width and height of the frames
        const int depth_w = depth.as<rs2::video_frame>().get_width();
        const int depth_h = depth.as<rs2::video_frame>().get_height();
        const int color_w = color.as<rs2::video_frame>().get_width();
        const int color_h = color.as<rs2::video_frame>().get_height();

        // Convert depth and color frames to OpenCV matrices
        depthFrameCV = cv::Mat(cv::Size(depth_w, depth_h), CV_8UC3, (void*)coloredDepth.get_data(), cv::Mat::AUTO_STEP);
        colorFrameCV = cv::Mat(cv::Size(color_w, color_h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

        // Check if the capture step is active
        if(capture_step) {
            get_colored_objects(pose_msg, depth); // Process the frames to detect colored objects
            pose_msg.position.x = m_cx; // Set the detected object's X position
            pose_msg.position.y = m_cy; // Set the detected object's Y position
            pose_msg.position.z = m_cz; // Set the detected object's Z position

            // Draw rectangles on the color frame to indicate regions of interest
            int top_left_x = 333;
            int top_left_y = m_frame_height - 444;
            int bottom_right_x = m_frame_width - 333;
            int bottom_right_y = m_frame_height;

            cv::rectangle(colorFrameCV, cv::Point(top_left_x + 167, top_left_y), cv::Point(bottom_right_x, bottom_right_y), cv::Scalar(0, 0, 0), 2);
            cv::rectangle(colorFrameCV, cv::Point(top_left_x, top_left_y), cv::Point(top_left_x + 148, top_left_y + 444), cv::Scalar(0, 0, 0), 2);

            // Publish the board state and processed pose messages
            umi_rtx_interfaces::msg::Board board_msg;
            for(int i = 0; i < 9; i++)
                board_msg.data[i] = board_state[i];

            board_state_publisher->publish(board_msg);
            processed_pose_publisher->publish(pose_msg);

            // Reset capture step flag
            capture_step = false;
        }

        // Publish the depth and color images
        sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",depthFrameCV).toImageMsg();
        depth_publisher->publish(*depth_msg);

        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colorFrameCV).toImageMsg();
        image_publisher->publish(*img_msg);
    }
}


void Camera::get_capture_step(const std_msgs::msg::Bool::SharedPtr msg) {
    capture_step = msg->data;
}

void Camera::get_colored_objects(geometry_msgs::msg::Pose msg, rs2::depth_frame depth){

    // Reset the board state
    for(auto& value : board_state) {
        value = 0;
    }

    // Define the region of interest (ROI) in the color frame
    int x1 = 333;
    int y1 = m_frame_height - 444;
    int x2 = m_frame_width - 333;
    int y2 = m_frame_height;

    // Create a rectangle around the region of interest with some padding
    cv::Rect roi(x1 - 100, y1 - 100, x2 - x1 + 200, y2 - y1 + 100);
    cv::Mat croppedImage = colorFrameCV(roi); // Crop the image to the ROI

    cv::Mat hsv_img;
    cv::cvtColor(croppedImage, hsv_img, cv::COLOR_BGR2HSV); // Convert cropped image to HSV color space

    // Define color ranges for detection
    cv::Scalar lower_bound_robot = cv::Scalar(25, 100, 100); // Lower bound for robot color
    cv::Scalar upper_bound_robot = cv::Scalar(60, 255, 255); // Upper bound for robot color

    cv::Scalar lower_bound_human = cv::Scalar(15, 100, 100); // Lower bound for human color
    cv::Scalar upper_bound_human = cv::Scalar(100, 255, 255); // Upper bound for human color

    cv::Mat bin_hsv_img_robot;
    cv::inRange(hsv_img, lower_bound_robot, upper_bound_robot, bin_hsv_img_robot); // Binary image for robot detection

    cv::Mat bin_hsv_img_human;
    cv::inRange(hsv_img, lower_bound_human, upper_bound_human, bin_hsv_img_human); // Binary image for human detection

    // Find contours in the binary images
    std::vector<std::vector<cv::Point>> contours_robot;
    cv::findContours(bin_hsv_img_robot, contours_robot, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<std::vector<cv::Point>> contours_human;
    cv::findContours(bin_hsv_img_human, contours_human, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    double minArea = 5000.0; // Minimum contour area to be considered
    m_cx = 0, m_cy = 0, m_cz = 0; 

    // Process robot contours
    for (size_t i = 0; i < contours_robot.size(); i++) {
        double area = cv::contourArea(contours_robot[i]);
        if(area < minArea) // Skip small contours
            continue;

        cv::Moments moments = cv::moments(contours_robot[i]); // Compute moments of the contour
        double cx = moments.m10 / moments.m00; // X coordinate of the contour center
        double cy = moments.m01 / moments.m00; // Y coordinate of the contour center
        double cz = depth.get_distance(cx, cy); // Depth (Z) coordinate from the depth frame

        cv::Point2f robotCenter(cx, cy);
        cv::circle(croppedImage, robotCenter, 4, cv::Scalar(0, 255, 255), -1); // Draw circle at the target center

        int box = find_box(cx, cy); // Determine which box the object is in
        if(box >= 0 && box < 9){
            cv::circle(croppedImage, cv::Point(cx, cy), 8, cv::Scalar(0, 0, 255), -1); // Draw circle for valid objects
            cv::drawContours(croppedImage, contours_robot, i, cv::Scalar(255, 255, 255), 2); // Draw contour

            board_state[box] = 1; // Mark box as containing a robot
        }
        else if(box == 9) { // Special case for the pick up box
            get_angles(contours_robot[i]); // Compute angles

            // Draw circles and contours on both cropped and depth frames
            cv::circle(croppedImage, cv::Point(cx, cy), 8, cv::Scalar(0, 255, 255), -1);
            cv::circle(depthFrameCV, cv::Point(cx, cy), 8, cv::Scalar(0, 255, 255), -1);
            cv::drawContours(croppedImage, contours_robot, i, cv::Scalar(255, 0, 0), 2);
            cv::drawContours(depthFrameCV, contours_robot, i, cv::Scalar(255, 0, 0), 2);

            // Update center coordinates
            m_cx = cx;
            m_cy = cy;
            m_cz = cz;
        }
    }

    // Process human contours
    for (size_t i = 0; i < contours_human.size(); i++) {
        double area = cv::contourArea(contours_human[i]);
        if(area < minArea) // Skip small contours
            continue;

        cv::Moments moments = cv::moments(contours_human[i]); // Compute moments of the contour
        double cx = moments.m10 / moments.m00; // X coordinate of the contour center
        double cy = moments.m01 / moments.m00; // Y coordinate of the contour center
        double cz = depth.get_distance(cx, cy); // Depth (Z) coordinate from the depth frame

        cv::Point2f humanCenter(cx, cy);
        cv::circle(croppedImage, humanCenter, 4, cv::Scalar(0, 255, 255), -1); // Draw circle at the human center

        int box = find_box(cx, cy); // Determine which box the object is in
        if(box >= 0 && box < 9){
            cv::circle(croppedImage, cv::Point(cx, cy), 8, cv::Scalar(0, 0, 255), -1); // Draw circle for valid objects
            cv::drawContours(croppedImage, contours_human, i, cv::Scalar(255, 255, 255), 2); // Draw contour

            board_state[box] = 2; // Mark box as containing a human
        }
    }

    // Indicate the absence of detected objects
    if(m_cx == 0 && m_cy == 0 && m_cz == 0) {
        cv::circle(colorFrameCV, cv::Point(m_frame_width - 40, 40), 20, cv::Scalar(0, 0, 255), -1); // Draw a red circle if no object is detected
    }

    // Draw crosshair in the center of the color frame
    cv::line(colorFrameCV, cv::Point(m_frame_width / 2 - 25, m_frame_height / 2), cv::Point(m_frame_width / 2 + 25, m_frame_height / 2), cv::Scalar(255, 255, 255), 2);
    cv::line(colorFrameCV, cv::Point(m_frame_width / 2, m_frame_height / 2 - 25), cv::Point(m_frame_width / 2, m_frame_height / 2 + 25), cv::Scalar(255, 255, 255), 2);

    // Update the pose message with the computed orientation
    msg.orientation.x = yaw;
    msg.orientation.y = pitch;
    msg.orientation.z = roll;
}


void Camera::get_angles(vector<cv::Point> &longest_contour){

    cv::Vec4f line_params;
    cv::fitLine(longest_contour, line_params, cv::DIST_L2, 0, 0.01, 0.01);

    float vx = line_params[0];
    float vy = line_params[1];
    float theta = atan2(vy, vx) + M_PI/2;

    yaw = 90.;
    pitch = 90.;
    roll = theta*180/M_PI;
}

int Camera::find_box(double cx, double cy){

    // Check if the coordinates are outside the grid
    if(cx < 100 || cy < 100 || cy >= 544)
        return -1;
    else if(cx < 248)
        return 9;
    else if(cx < 267)
        return -1;

    // Calculate the width of each box dynamically based on frame width
    // Box coordinates are defined relative to frame dimensions and padding
    else if(cx < 267 + (m_frame_width - 833) / 3 && cy < 100 + 444 / 3)
        return 0;
    else if (cx < 267 + 2 * (m_frame_width - 833) / 3 && cy <  100 + 444 / 3)
        return 1;
    else if (cx < 267 + m_frame_width - 833 && cy < 100 + 444 / 3)
        return 2;
    else if(cx < 267 + (m_frame_width - 833) / 3 && cy < 100 + 2 * 444 / 3)
        return 3;
    else if (cx < 267 + 2 * (m_frame_width - 833) / 3 && cy < 100 + 2 * 444 / 3)
        return 4;
    else if (cx < 267 + m_frame_width - 833 && cy < 100 +2 * 444 / 3)
        return 5;
    else if(cx < 267 + (m_frame_width - 833) / 3 && cy < 100 + 444)
        return 6;
    else if (cx < 267 + 2 * (m_frame_width - 833) / 3 && cy < 100 + 444)
        return 7;
    else if (cx < 267 + m_frame_width - 833 && cy < 100 + 444)
        return 8;
    else 
        return -1;
} 

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Camera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}