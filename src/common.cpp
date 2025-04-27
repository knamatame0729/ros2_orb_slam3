/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
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

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages
    // Semantic mask topic as mono8
    subSemanticMaskName = "/segmented_image";

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    //* subscribe to receive semntic image
    subSemantic_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(subSemanticMaskName, 10, std::bind(&MonocularMode::semanticCallback, this, _1));

    //* publish pointcloud
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/map_points", 10);

    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();

}

//* Semantic Mask Callback
void MonocularMode::semanticCallback(const sensor_msgs::msg::Image::SharedPtr msg){

    try
    {
        // Convert ros msg to cv img
        latest_semantic_mask_ = cv_bridge::toCvCopy(msg, "mono8")->image;

        // Sync
        latest_mask_stamp_ = msg->header.stamp;

        // Logger
        RCLCPP_INFO(this->get_logger(), "Received semantic mask with stamp: %f, size: %dx%d, encoding: %s",
                    latest_mask_stamp_.seconds(), latest_semantic_mask_.cols, latest_semantic_mask_.rows, msg->encoding.c_str());
    }

    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "CvBridge Error: %s", e.what());
    }
}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // Logger for debug
        RCLCPP_INFO(this->get_logger(), "Received image, size: %dx%d, encoding: %s",
                    cv_ptr->image.cols, cv_ptr->image.rows, msg.encoding.c_str());
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    //Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep);

    // 
    latest_Tcw_ = pAgent->TrackMonocular(cv_ptr->image, timeStep);
    
    RCLCPP_INFO(this->get_logger(), "Tracking state: %d", pAgent->GetTrackingState());

    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

    publishPointCloud(); // Publish semantic point cloud
}

//* Publish semantic point cloud
void MonocularMode::publishPointCloud(){

    // Logger
    if (!pAgent)
    {
        RCLCPP_ERROR(this->get_logger(), "Skipping point cloud publish: pAgent is null");
        return;
    }
    if (latest_semantic_mask_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Skipping point cloud publish: Semantic mask is empty");
        return;
    }

    // Get Map Points that is been tracking
    std::vector<ORB_SLAM3::MapPoint*> map_points = pAgent->GetTrackedMapPoints();
    pcl::PointCloud<pcl::PointXYZRGB> cloud; // Store with itensity for class ID

    for (const auto& mp : map_points)
    {
        if (mp && !mp->isBad())
        {
            ORB_SLAM3::MapPoint* non_const_mp = const_cast<ORB_SLAM3::MapPoint*>(mp);

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
    pointcloud_msg.header.frame_id = "world";
    pointcloud_publisher_->publish(pointcloud_msg);
    RCLCPP_INFO(this->get_logger(), "Published %ld semantic points", cloud.size());
}

//* Project Map Point to Image Coordinate
cv::Point2f MonocularMode::projectMapPointToImage(const ORB_SLAM3::MapPoint* mp, const Sophus::SE3f& Tcw){
    // Camera parameters (Perfect pinhole with FOV 1.22 rads)
    float fx = 329.2, fy = 329.2, cx = 240.0, cy = 240.0;

    ORB_SLAM3::MapPoint* non_const_mp = const_cast<ORB_SLAM3::MapPoint*>(mp);

    // Get Point in World Coordinate 
    Eigen::Vector3f pos = non_const_mp->GetWorldPos();

    // Transform to camera coordinate
    Eigen::Vector3f pos_c = Tcw * pos; // pos_c = Rcw * pos + t_cw

    // Check if point is in front of camera
    if (pos_c(2) <= 0) return cv::Point2f(-1, -1);

    // Pixel coordinate
    float u = fx * pos_c(0) / pos_c(2) + cx;
    float v = fy * pos_c(1) / pos_c(2) + cy;

    // Check if projection is in image
    if (u < 0 || u >= 480 || v < 0 || v >= 480) return cv::Point2f(-1, -1);

    return cv::Point2f(u, v);

}