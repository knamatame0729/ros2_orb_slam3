// Include file
#ifndef STEREO_COMMON_HPP
#define STEREO_COMMON_HPP

// C++ includes
#include <iostream> // The iostream library is an object-oriented library that provides input and output functionality using streams
#include <algorithm> // The header <algorithm> defines a collection of functions especially designed to be used on ranges of elements.
#include <fstream> // Input/output stream class to operate on files.
#include <chrono> // c++ timekeeper library
#include <vector> // vectors are sequence containers representing arrays that can change in size.
#include <queue>
#include <thread> // class to represent individual threads of execution.
#include <mutex> // A mutex is a lockable object that is designed to signal when critical sections of code need exclusive access, preventing other threads with the same protection from executing concurrently and access the same memory locations.
#include <cstdlib> // to find home directory

#include <cstring>
#include <sstream> // String stream processing functionalities

//* ROS2 includes
//* std_msgs in ROS 2 https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
#include <rclcpp/rclcpp.hpp>

// #include "your_custom_msg_interface/msg/custom_msg_field.hpp" // Example of adding in a custom message
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
using std::placeholders::_1; //* TODO why this is suggested in official tutorial

// Include Eigen
// Quick reference: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense> // Includes Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header file

// Include cv-bridge
#include <cv_bridge/cv_bridge.h>

// Include OpenCV computer vision library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> // Image processing tools
#include <opencv2/highgui/highgui.hpp> // GUI tools
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.h>

//* ORB SLAM 3 includes
#include "System.h" //* Also imports the ORB_SLAM3 namespace

//* Gobal defs
#define pass (void)0 // Python's equivalent of "pass" i.e. no operation

// Node-specific definitions
class StereoMode : public rclcpp::Node
{
    public:
        std::string experimentConfig = "";
        double timeStep;
        std::string receivedConfig = "";

        StereoMode();
        ~StereoMode();

    private:
        // Class internal variables
        std::string homeDir = "";
        std::string packagePath = "ros2_ws/src/ros2_orb_slam3/"; // Change to match your workspace
        std::string OPENCV_WINDOW = "";
        std::string nodeName = "";
        std::string vocFilePath = "";
        std::string settingsFilePath = "";
        bool bSettingsFromPython = false;

        std::string subexperimentconfigName = "";
        std::string pubconfigackName = "";
        std::string subLeftImgMsgName = ""; 
        std::string subRightImgMsgName = ""; 
        std::string subTimestepMsgName = "";
        std::string subSemanticMaskName = "";

        Sophus::SE3f latest_Tcw_;
        cv::Mat latest_semantic_mask_;
        rclcpp::Time latest_mask_stamp_;
        cv::Mat latest_left_image_, latest_right_image_;
        rclcpp::Time latest_left_stamp_, latest_right_stamp_;

        // Publishers and subscribers
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subLeftImgMsg_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subRightImgMsg_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subSemantic_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

        // ORB-SLAM3 variables
        ORB_SLAM3::System* pAgent;
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = false;
        bool enableOpenCVWindow = false;

        // ROS callbacks
        void experimentSetting_callback(const std_msgs::msg::String& msg);
        void Timestep_callback(const std_msgs::msg::Float64& time_msg);
        void leftImg_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void rightImg_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void semanticCallback(const sensor_msgs::msg::Image::SharedPtr msg);

        // Helper functions
        void initializeVSLAM(std::string& configString);
        void processStereoImages();
        void publishPointCloud();
        cv::Point2f projectMapPointToImage(const ORB_SLAM3::MapPoint* mp, const Sophus::SE3f& Tcw);
};

#endif