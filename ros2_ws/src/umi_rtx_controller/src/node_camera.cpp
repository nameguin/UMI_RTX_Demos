#include "umi_rtx_controller/node_camera.hpp"

void Camera::init_interfaces(){
    stereo = cv::StereoSGBM::create(min_disp,num_disp,blockSize,iP1,iP2,disp12MaxDiff,0,uniquenessRatio,speckleWindowSize,speckleRange);

    timer_ = this->create_wall_timer(loop_dt_,std::bind(&Camera::timer_callback,this));

    image_publisher = this->create_publisher<sensor_msgs::msg::Image>("processed_image",10);
    coord_publisher = this->create_publisher<geometry_msgs::msg::Point>("processed_position",10);
    angles_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("processed_angles",10);
    disparity_publisher = this->create_publisher<sensor_msgs::msg::Image>("disparity_image",10);
    depth_publisher = this->create_publisher<sensor_msgs::msg::Image>("depth_image",10);
    double_publisher = this->create_publisher<std_msgs::msg::Float64>("target_depth",10);
}

void Camera::init_camera(){
    std::cout << "Opening webcam... " << std::endl;

    m_frame_width = 1280;
    m_frame_height = 720;
    cfg.enable_stream(RS2_STREAM_DEPTH, m_frame_width/2, m_frame_height/2, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, m_frame_width, m_frame_height, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    auto sensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>();
    sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);

    //std::cout << "init done" << std::endl;
}

void Camera::timer_callback(){
    geometry_msgs::msg::Point coord_msg;
    geometry_msgs::msg::Vector3 angles_msg;

    rs2::frameset frames = pipe.wait_for_frames();

    // Align depth frame to color frame
    frames = align_to_color.process(frames);

    //color_map.set_option(RS2_OPTION_COLOR_SCHEME, 0);
    color_map.set_option(RS2_OPTION_MIN_DISTANCE, 0.0f); // Minimum distance (0.0 meters)
    color_map.set_option(RS2_OPTION_MAX_DISTANCE, 6.0f); // Maximum distance (6.0 meters)
    color_map.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, false); // Disable histogram equalization
    //color_map.set_option(RS2_OPTION_THRESHOLD, 0.0f); // Disable threshold filter

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

    get_banana_and_angles(coord_msg,angles_msg, depth);

    //sensor_msgs::msg::Image::SharedPtr disp_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"mono8",disparityMap).toImageMsg();
    //disparity_publisher->publish(*disp_msg);

    sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",depthFrameCV).toImageMsg();
    depth_publisher->publish(*depth_msg);

        //std_msgs::msg::Float64 target_depth_msg;
    //target_depth_msg.data = depthMap.at<float>(depthMap.rows/2,depthMap.cols/2);
       //target_depth_msg.data = cv::mean(depthMap)[0];
    //target_depth_msg.data = (0.5*m_frame_width_left/tan(90*0.5*M_PI/180)) * m_baseline / disparityMap.at<float>(m_frame_width_left/2,m_frame_height_left/2);
    //target_depth_msg.data = disparityMap.at<float>(m_frame_width_left/2,m_frame_height_left/2);
    //target_depth_msg.data = depthMap.at<uchar>(300,300);
       //double_publisher->publish(target_depth_msg);

}

void Camera::get_banana_and_angles(geometry_msgs::msg::Point coord_msg, geometry_msgs::msg::Vector3 angles_msg, rs2::depth_frame depth){

    cv::Mat hsv_img;
    //cv::cvtColor(frame,hsv_img,cv::COLOR_BGR2HSV);
    cv::cvtColor(colorFrameCV,hsv_img,cv::COLOR_BGR2HSV);


    cv::Scalar lower_bound = cv::Scalar(20,100,100);
    cv::Scalar upper_bound = cv::Scalar(60,255,255);

    cv::Mat bin_hsv_img;
    cv::inRange(hsv_img, lower_bound, upper_bound, bin_hsv_img);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_hsv_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    if(contours.empty()){
        std::cout << "Cannot detect the target" << std::endl;

        cv::circle(colorFrameCV,cv::Point(m_frame_width-40,40),20,cv::Scalar(0,0,255),-1);
        //cv::circle(frameLeft,cv::Point(m_frame_width-40,40),20,cv::Scalar(0,0,255),-1);

        cv::line(colorFrameCV,cv::Point (m_frame_width/2 - 25,m_frame_height/2),cv::Point (m_frame_width/2 + 25,m_frame_height/2),cv::Scalar(255,255,255),2);
        cv::line(colorFrameCV,cv::Point (m_frame_width/2,m_frame_height/2 - 25),cv::Point (m_frame_width/2,m_frame_height/2 + 25),cv::Scalar(255,255,255),2);
        //cv::line(frameLeft,cv::Point (m_frame_width_left/2 - 25,m_frame_height_left/2),cv::Point (m_frame_width_left/2 + 25,m_frame_height_left/2),cv::Scalar(255,255,255),2);
        //cv::line(frameLeft,cv::Point (m_frame_width_left/2,m_frame_height_left/2 - 25),cv::Point (m_frame_width_left/2,m_frame_height_left/2 + 25),cv::Scalar(255,255,255),2);

        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",colorFrameCV).toImageMsg();
        //sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",frameLeft).toImageMsg();
        image_publisher->publish(*img_msg);
    }

    else{

        double maxArea = 0;
        int maxAreaIdx = -1;

        for (int i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);

            if (area > maxArea)
            {
                maxArea = area;
                maxAreaIdx = i;
            }
        }

        //std::cout << "indice : " << maxAreaIdx << std::endl;


        if(maxAreaIdx > -1) {
            get_angles(contours);
            cv::drawContours(colorFrameCV, contours, maxAreaIdx, cv::Scalar(255, 255, 255), 2);
            //cv::drawContours(frameLeft, contours, maxAreaIdx, cv::Scalar(255, 255, 255), 2);

            cv::Moments moments = cv::moments(contours[maxAreaIdx]);

            if (moments.m00 != 0) {
                double cx = moments.m10 / moments.m00;
                double cy = moments.m01 / moments.m00;
                //std::cout << "Centroid : (" << cx << ", " << cy << ")" << std::endl;

                coord_msg.x = cx;
                coord_msg.y = cy;
                m_cx = cx;
                m_cy = cy;
            }

            else {
                std::cout << "Impossible centroid calculation" << std::endl;

                coord_msg.x = m_cx;
                coord_msg.y = m_cy;

                cv::circle(colorFrameCV,cv::Point(m_frame_width-40,40),20,cv::Scalar(100,50,100),-1);
                //cv::circle(frameLeft,cv::Point(m_frame_width_left-40,40),20,cv::Scalar(100,50,100),-1);
            }
        }

        else{
            coord_msg.x = m_cx;
            coord_msg.y = m_cy;
        }

        std::cout << "Yellow object detected at (" << m_cx <<";"<< m_cy << ")" << std::endl;
        std::cout << "Depth: " << depth.get_distance(m_cx, m_cy) << std::endl;
        cv::circle(colorFrameCV,cv::Point(m_cx,m_cy),8,cv::Scalar(0,0,255),-1);

        //cv::circle(colorFrameCV,cv::Point(m_frame_width-40,40),20,cv::Scalar(0,255,0),-1);
        //cv::circle(frameLeft,cv::Point(m_frame_width_left-40,40),20,cv::Scalar(0,255,0),-1);

        cv::line(colorFrameCV,cv::Point (m_frame_width/2 - 25,m_frame_height/2),cv::Point (m_frame_width/2 + 25,m_frame_height/2),cv::Scalar(255,255,255),2);
        cv::line(colorFrameCV,cv::Point (m_frame_width/2,m_frame_height/2 - 25),cv::Point (m_frame_width/2,m_frame_height/2 + 25),cv::Scalar(255,255,255),2);
        //cv::line(frameLeft,cv::Point (m_frame_width_left/2 - 25,m_frame_height_left/2),cv::Point (m_frame_width_left/2 + 25,m_frame_height_left/2),cv::Scalar(255,255,255),2);
        //cv::line(frameLeft,cv::Point (m_frame_width_left/2,m_frame_height_left/2 - 25),cv::Point (m_frame_width_left/2,m_frame_height_left/2 + 25),cv::Scalar(255,255,255),2);

        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colorFrameCV).toImageMsg();
        //sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frameLeft).toImageMsg();
        image_publisher->publish(*img_msg);
    }
    coord_publisher->publish(coord_msg);

    angles_msg.x = yaw;
    angles_msg.y = pitch;
    angles_msg.z = roll;
    angles_publisher->publish(angles_msg);
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