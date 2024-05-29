#include "umi_rtx_controller/node_camera.hpp"

void Camera::init_interfaces(){
    stereo = cv::StereoSGBM::create(min_disp,num_disp,blockSize,iP1,iP2,disp12MaxDiff,0,uniquenessRatio,speckleWindowSize,speckleRange);

    timer_ = this->create_wall_timer(loop_dt_,std::bind(&Camera::timer_callback,this));

    image_publisher = this->create_publisher<sensor_msgs::msg::Image>("processed_image",10);
    processed_pose_publisher = this->create_publisher<geometry_msgs::msg::Pose>("processed_pose",10);
    depth_publisher = this->create_publisher<sensor_msgs::msg::Image>("depth_image",10);
    edges_publisher = this->create_publisher<sensor_msgs::msg::Image>("edges_image",10);


    board_state_publisher = this->create_publisher<umi_rtx_interfaces::msg::Board>("board_state",10);
    board_coordinates_publisher = this->create_publisher<umi_rtx_interfaces::msg::BoardCoordinates>("board_coordinates",10);

    //double_publisher = this->create_publisher<std_msgs::msg::Float64>("target_depth",10);
}

void Camera::init_camera(){
    std::cout << "Checking for available cameras... " << std::endl;

    rs2::context ctx;
    auto device_list = ctx.query_devices();
    available_device = device_list.size();

    if (available_device == 0)
        std::cout << "No RealSense camera detected!" << std::endl << std::endl;
    else if(available_device == 1)
        std::cout << "There is " << available_device << " connected RealSense device." << std::endl << std::endl;
    else
        std::cout << "There are " << available_device << " connected RealSense devices." << std::endl << std::endl;

    if(available_device > 0) {
        auto dev = device_list[0];
        std::cout << "Using device 0 " << std::endl;
        std::cout << "Name: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        std::cout << "Serial number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        std::cout << "Firmware version: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;

        m_frame_width = 1280;
        m_frame_height = 720;

        cfg.enable_stream(RS2_STREAM_DEPTH, m_frame_width/2, m_frame_height/2, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, m_frame_width, m_frame_height, RS2_FORMAT_BGR8, 30);
    }

    try {
        pipe.start(cfg);
        auto sensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>();
        sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    } catch (const rs2::error& e) {
        std::cerr << "Erreur librealsense : " << e.what() << std::endl;
    }
    //color_map.set_option(RS2_OPTION_COLOR_SCHEME, 0)
    color_map.set_option(RS2_OPTION_MIN_DISTANCE, 0.0f); // Minimum distance (0.0 meters)
    color_map.set_option(RS2_OPTION_MAX_DISTANCE, 6.0f); // Maximum distance (6.0 meters)
    color_map.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, false); // Disable histogram equalization

    //std::cout << "init done" << std::endl;
}

void Camera::timer_callback(){
    geometry_msgs::msg::Pose pose_msg;
    umi_rtx_interfaces::msg::BoardCoordinates board_coordinates_msg;

    if(available_device > 0) {
        rs2::frameset frames = pipe.wait_for_frames();

        // Align depth frame to color frame
        frames = align_to_color.process(frames);

        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::frame coloredDepth = frames.get_depth_frame().apply_filter(color_map);
        rs2::frame color = frames.get_color_frame();

        // Query frame size (width and height)
        const int depth_w = depth.as<rs2::video_frame>().get_width();
        const int depth_h = depth.as<rs2::video_frame>().get_height();
        const int color_w = color.as<rs2::video_frame>().get_width();
        const int color_h = color.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        depthFrameCV = cv::Mat(cv::Size(depth_w, depth_h), CV_8UC3, (void*)coloredDepth.get_data(), cv::Mat::AUTO_STEP);

        // Create OpenCV matrix of size (w,h) from the color data
        colorFrameCV = cv::Mat(cv::Size(color_w, color_h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

        std::vector<GridSquare> board_coordinates;
        get_grid_position();
        if(board_coordinates.size() > 0) {
            geometry_msgs::msg::Point points[9];
            auto it = board_coordinates.begin();

            for (size_t i = 0; it != board_coordinates.end(); ++it, ++i) {
                points[i].x = it->center.x;
                points[i].y = it->center.y;
                points[i].z = 20.0;
            }
            board_coordinates_publisher->publish(board_coordinates_msg);
        }
        /*for(int i = 0; i < 3; ++i){
            for(int j = 0; j < 3; ++j){
                board_coordinates_msg.points[i*3+j].x = -40 + j*40;
                board_coordinates_msg.points[i*3+j].y = 60 - i*10;
                board_coordinates_msg.points[i*3+j].z = 20;
            }
        }*/


        get_colored_objects(pose_msg, depth);

        pose_msg.position.x = m_cx;
        pose_msg.position.y = m_cy;
        pose_msg.position.z = m_cz;

        processed_pose_publisher->publish(pose_msg);

        sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",depthFrameCV).toImageMsg();
        depth_publisher->publish(*depth_msg);
    }

    umi_rtx_interfaces::msg::Board board_msg;
    for(int i = 0; i < 9; i++){
        board_msg.data[i] = 0;
    }
    board_state_publisher->publish(board_msg); 

        //std_msgs::msg::Float64 target_depth_msg;
    //target_depth_msg.data = depthMap.at<float>(depthMap.rows/2,depthMap.cols/2);
       //target_depth_msg.data = cv::mean(depthMap)[0];
    //target_depth_msg.data = (0.5*m_frame_width_left/tan(90*0.5*M_PI/180)) * m_baseline / disparityMap.at<float>(m_frame_width_left/2,m_frame_height_left/2);
    //target_depth_msg.data = disparityMap.at<float>(m_frame_width_left/2,m_frame_height_left/2);
    //target_depth_msg.data = depthMap.at<uchar>(300,300);
       //double_publisher->publish(target_depth_msg);
    
    board_coordinates.clear();

}

void Camera::get_grid_position(){

    cv::Mat blurred;
    cv::GaussianBlur(colorFrameCV, blurred, cv::Size(5, 5), 0);

    cv::Mat edges;
    cv::Canny(blurred, edges, 50, 150);

    cv::Mat dilated;
    cv::dilate(edges, dilated, cv::Mat(), cv::Point(-1, -1), 2);

    cv::Mat eroded;
    cv::erode(dilated, eroded, cv::Mat(), cv::Point(-1, -1), 2);

    cv::Mat thresh_image;
    cv::threshold(eroded, thresh_image, 150, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> grid_contours;
    std::vector<std::vector<cv::Point>> board_contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh_image, grid_contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    double minArea = 1000.0; // minimum contour area to consider

    for (size_t i = 0; i < grid_contours.size(); i++) {
        double epsilon = 0.01 * cv::arcLength(grid_contours[i], true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(grid_contours[i], approx, epsilon, true);

        double contourArea = cv::contourArea(approx);

        if (contourArea < minArea)
            continue;

        if (approx.size() == 4) {
            cv::Moments moments = cv::moments(grid_contours[i]);
            double cx = moments.m10 / moments.m00;
            double cy = moments.m01 / moments.m00;
            
            GridSquare current_box;
            current_box.contours = grid_contours[i];
            current_box.area = contourArea;
            current_box.center = cv::Point2f(cx, cy);

            cv::Point p1 = approx[0];
            cv::Point p2 = approx[1];
            double length = cv::norm(p1 - p2);
            if(min(length / sqrt(contourArea), sqrt(contourArea) / length) < 0.7)
                continue;

            bool farEnough = true;
            for (const auto& box : board_coordinates) {
                double distance = cv::norm(current_box.center - box.center);
                if (distance < 1.0) {
                    farEnough = false;
                    cv::circle(colorFrameCV, current_box.center, 6, cv::Scalar(0,0,255), -1);
                    break;
                }
            }
            //cv::drawContours(colorFrameCV, grid_contours, i, cv::Scalar(255, 255, 255), 2);
            //std::cout << "Area : " << current_box.area << std::endl;
            if(farEnough)  
                board_coordinates.push_back(current_box);          

        }
    }

    if(board_coordinates.size() == 10){
        std::sort(board_coordinates.begin(), board_coordinates.end(), [](const GridSquare& a, const GridSquare& b) {
            return a.area < b.area;
        });
        board_coordinates.pop_back();
        for (const auto& square : board_coordinates) {
            //cv::circle(colorFrameCV, square.center, 6, cv::Scalar(0,0,255), -1);
        }
        
    }

    sensor_msgs::msg::Image::SharedPtr edges_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"mono8",thresh_image).toImageMsg();
    edges_publisher->publish(*edges_msg);
}

void Camera::get_colored_objects(geometry_msgs::msg::Pose msg, rs2::depth_frame depth){

    cv::Mat hsv_img;
    cv::cvtColor(colorFrameCV,hsv_img,cv::COLOR_BGR2HSV);

    cv::Scalar lower_bound = cv::Scalar(25,100,100);
    cv::Scalar upper_bound = cv::Scalar(60,255,255);

    cv::Mat bin_hsv_img;
    cv::inRange(hsv_img, lower_bound, upper_bound, bin_hsv_img);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_hsv_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    double maxArea = 0;
    int maxAreaIdx = -1;
    double minArea = 5.0;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if(area < minArea)
            continue;

        cv::Moments moments = cv::moments(contours[i]);
        double cx = moments.m10 / moments.m00;
        double cy = moments.m01 / moments.m00;
        double cz = depth.get_distance(cx, cy);

        cv::Point2f yellowCenter(cx, cy);
        cv::circle(colorFrameCV, yellowCenter, 6, cv::Scalar(0, 255, 255), -1);

        for (const auto& square : board_coordinates) {
            if (cv::pointPolygonTest(square.contours, yellowCenter, false) >= 0) {
                cv::drawContours(colorFrameCV, std::vector<std::vector<cv::Point>>{square.contours}, -1, cv::Scalar(0, 255, 0), 2);
                break;
            }
            else {
                if (area > maxArea)
                {
                    maxArea = area;
                    maxAreaIdx = i;
                }
                get_angles(contours);
                double cz = depth.get_distance(cx, cy);

                if(cz < 2 && cz != 0) {
                    //std::cout << "Centroid : (" << cx << ", " << cy << ")" << std::endl;
                    cv::drawContours(colorFrameCV, contours, maxAreaIdx, cv::Scalar(255, 255, 255), 2);
                    cv::drawContours(depthFrameCV, contours, maxAreaIdx, cv::Scalar(255, 255, 255), 2);
                    m_cx = cx;
                    m_cy = cy;
                    m_cz = cz;

                    cv::circle(colorFrameCV,cv::Point(m_cx,m_cy),8,cv::Scalar(0,0,255),-1);
                    cv::circle(depthFrameCV,cv::Point(m_cx,m_cy),8,cv::Scalar(0,0,255),-1);
                } 
                else {
                    std::cout << "Cannot detect the target" << std::endl;

                    cv::circle(colorFrameCV,cv::Point(m_frame_width-40,40),20,cv::Scalar(0,0,255),-1);
                }
            }
        }
        if(maxAreaIdx == -1) {
            std::cout << "Cannot detect the target" << std::endl;

            cv::circle(colorFrameCV,cv::Point(m_frame_width-40,40),20,cv::Scalar(0,0,255),-1);
        }
    }
    cv::line(colorFrameCV,cv::Point (m_frame_width/2 - 25,m_frame_height/2),cv::Point (m_frame_width/2 + 25,m_frame_height/2),cv::Scalar(255,255,255),2);
    cv::line(colorFrameCV,cv::Point (m_frame_width/2,m_frame_height/2 - 25),cv::Point (m_frame_width/2,m_frame_height/2 + 25),cv::Scalar(255,255,255),2);
        
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colorFrameCV).toImageMsg();
    image_publisher->publish(*img_msg);

    msg.orientation.x = yaw;
    msg.orientation.y = pitch;
    msg.orientation.z = roll;
}

void Camera::get_angles(vector<vector<cv::Point>> &contours){
    vector<cv::Point> longest_contour;
    double max_area = 0.0;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > max_area) {
            max_area = area;
            longest_contour = contour;
        }
    }

    cv::Vec4f line_params;
    cv::fitLine(longest_contour, line_params, cv::DIST_L2, 0, 0.01, 0.01);

    float vx = line_params[0];
    float vy = line_params[1];
    float theta = atan2(vy, vx) + M_PI/2;

    yaw = 90.;
    pitch = 90.;
    roll = theta*180/M_PI;

}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<Camera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}