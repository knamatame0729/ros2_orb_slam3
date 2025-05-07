// Includes
#include "ros2_orb_slam3/stereo_common.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

// Constructor
StereoMode::StereoMode() : Node("stereo_node_cpp")
{
    //* Find path to home directory
    homeDir = getenv("HOME");

    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 STEREO NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given");
    this->declare_parameter("voc_file_arg", "file_not_set");
    this->declare_parameter("settings_file_path_arg", "file_path_not_set");

    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Stereo/";
    }

    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

    subexperimentconfigName = "/stereo_py_driver/experiment_settings";    // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/stereo_py_driver/exp_settings_ack";              // send an acknowledgement to the python node
    subLeftImgMsgName = "/stereo_py_driver/left_img_msg";                 // topic to receive left image messages
    subRightImgMsgName = "/stereo_py_driver/right_img_msg";               // topic to receive right image messages
    subTimestepMsgName = "/stereo_py_driver/timestep_msg";                // topic to receive timestamp message
    subSemanticMaskName = "/segmented_image";                             // Semantic mask topic as mono8

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&StereoMode::experimentSetting_callback, this, std::placeholders::_1));
    
    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);
    
    //* subscrbite to the left image messages coming from the Python driver node
    subLeftImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(subLeftImgMsgName, 1, std::bind(&StereoMode::leftImg_callback, this, std::placeholders::_1));
    
    //* subscrbite to the right left image messages coming from the Python driver node
    subRightImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(subRightImgMsgName, 1, std::bind(&StereoMode::rightImg_callback, this, std::placeholders::_1));
    
    //* subscribe to receive the timestep
    subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&StereoMode::Timestep_callback, this, std::placeholders::_1));
    
    //* subscribe to receive semntic image
    subSemantic_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(subSemanticMaskName, 10, std::bind(&StereoMode::semanticCallback, this, std::placeholders::_1));

    //* publish pointcloud
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/map_points", 10);

    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

// Destructor
StereoMode::~StereoMode() {
    pAgent->Shutdown();
}

// Semantic Mask Callback
void StereoMode::semanticCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // Convert ros msg to cv img
        latest_semantic_mask_ = cv_bridge::toCvCopy(msg, "mono8")->image;
        latest_mask_stamp_ = msg->header.stamp;

        RCLCPP_INFO(this->get_logger(), "Received semantic mask with stamp: %f, size: %dx%d, encoding: %s",
                    latest_mask_stamp_.seconds(), latest_semantic_mask_.cols, latest_semantic_mask_.rows, msg->encoding.c_str());

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CvBridge Error: %s", e.what());
    }
}

// Experiment Settings Callback
void StereoMode::experimentSetting_callback(const std_msgs::msg::String& msg)
{
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig;

    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";

    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);
}

// Initialize VSLAM
void StereoMode::initializeVSLAM(std::string& configString) {
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set") {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");
        rclcpp::shutdown();
    }

    settingsFilePath = settingsFilePath + configString + ".yaml";
    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());

    sensorType = ORB_SLAM3::System::STEREO;
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "StereoMode node initialized successfully");
}

//* Callback that processes timestep sent over ROS
void StereoMode::Timestep_callback(const std_msgs::msg::Float64& time_msg) {
    timeStep = time_msg.data;
}

// Left Image Callback
void StereoMode::leftImg_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  
    latest_left_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    latest_left_stamp_ = msg->header.stamp;
    RCLCPP_DEBUG(this->get_logger(), "Left image received, size: %dx%d", latest_left_image_.cols, latest_left_image_.rows);
    processStereoImages();

}

// Right Image Callback
void StereoMode::rightImg_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    
    latest_right_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    latest_right_stamp_ = msg->header.stamp;

    RCLCPP_DEBUG(this->get_logger(), "Right image received, size: %dx%d", latest_right_image_.cols, latest_right_image_.rows);
    processStereoImages();
}

// Process Stereo Images
void StereoMode::processStereoImages() {
    if (!latest_left_image_.empty() && !latest_right_image_.empty()) {
        double time_diff = std::abs(latest_left_stamp_.seconds() - latest_right_stamp_.seconds());
        if (time_diff < 0.05) {
            RCLCPP_INFO(this->get_logger(), "Processing stereo pair, left stamp: %f, right stamp: %f",
                        latest_left_stamp_.seconds(), latest_right_stamp_.seconds());

            try {
                // Get transform matrix fron world frame to camera frame
                latest_Tcw_ = pAgent->TrackStereo(latest_left_image_, latest_right_image_, timeStep);

                RCLCPP_INFO(this->get_logger(), "Tracking state: %d", pAgent->GetTrackingState());
                publishPointCloud();
            } catch (std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error in TrackStereo: %s", e.what());
            }

            latest_left_image_ = cv::Mat();
            latest_right_image_ = cv::Mat();
        }
    }
}

// Publish Semantic Point Cloud
void StereoMode::publishPointCloud() {
    
    if (!pAgent || latest_semantic_mask_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Skipping point cloud publish: pAgent or semantic mask is null");
        return;
    }

    // Get Map Points that is been tracking
    std::vector<ORB_SLAM3::MapPoint*> map_points = pAgent->GetTrackedMapPoints();
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    for (const auto& mp : map_points)
    {
        if (mp && !mp->isBad())
        {
            //ORB_SLAM3::MapPoint* non_const_mp = const_cast<ORB_SLAM3::MapPoint*>(mp);

            // Get Map Point coordinate in world frame
            auto pos = mp->GetWorldPos();

            // Project outo image coordinate
            cv::Point2f uv = projectMapPointToImage(mp, latest_Tcw_);
            int class_id = 0;

            // Check the uv is in the image
            if (uv.x >= 0 && uv.x < latest_semantic_mask_.cols &&
                uv.y >= 0 && uv.y < latest_semantic_mask_.rows &&
                latest_mask_stamp_.seconds() > 0)
            {
                // Get class ID for each pixel from semantic mask
                class_id = latest_semantic_mask_.at<uint8_t>(uv.y, uv.x);
            }

            pcl::PointXYZRGB pt;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);

            // BGR thresholders corresponding classes using switch-case
            switch (class_id) {
                case 0: pt.r = 0; pt.g = 0; pt.b = 0; break;       // Rover
                case 1: pt.r = 81; pt.g = 0; pt.b = 81; break;     // Ground
                case 2: pt.r = 42; pt.g = 59; pt.b = 108; break;   // Rocks
                case 3: pt.r = 180; pt.g = 130; pt.b = 70; break;  // Earth
                case 4: pt.r = 160; pt.g = 190; pt.b = 110; break; // Lander
                case 5: pt.r = 30; pt.g = 170; pt.b = 250; break;  // Fiducials
            }

            cloud.push_back(pt);
        }
    }

    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(cloud, pointcloud_msg);
    pointcloud_msg.header.stamp = this->get_clock()->now();
    pointcloud_msg.header.frame_id = "map";
    pointcloud_publisher_->publish(pointcloud_msg);
    RCLCPP_INFO(this->get_logger(), "Published %ld semantic points", cloud.size());
}

// Project Map Point to Image
cv::Point2f StereoMode::projectMapPointToImage(const ORB_SLAM3::MapPoint* mp, const Sophus::SE3f& Tcw)
{
    // Camera Parameters
    float fx = 329.2, fy = 329.2, cx = 240.0, cy = 240.0;

    ORB_SLAM3::MapPoint* non_const_mp = const_cast<ORB_SLAM3::MapPoint*>(mp);

    // Get Point in World Coordinate 
    Eigen::Vector3f pos = non_const_mp->GetWorldPos();

    // Conmute camera frame from world frame using transform matrix
    Eigen::Vector3f pos_c = Tcw * pos;

    if (pos_c(2) <= 0) return cv::Point2f(-1, -1);
    
    // Compute points corrdinate in pixel frame
    float u = fx * pos_c(0) / pos_c(2) + cx;
    float v = fy * pos_c(1) / pos_c(2) + cy;

    if (u < 0 || u >= 480 || v < 0 || v >= 480) return cv::Point2f(-1, -1);

    return cv::Point2f(u, v);
}
