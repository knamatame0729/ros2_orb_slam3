#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError
import time

class StereoDriver(Node):
    def __init__(self, node_name="stereo_py_node"):
        super().__init__(node_name)

        self.declare_parameter("settings_name", "Lunar_Stereo_Cam")
        self.declare_parameter("left_image_topic", "/frontleft_camera/image")
        self.declare_parameter("right_image_topic", "/frontcamera_right/image")

        self.settings_name = self.get_parameter('settings_name').value
        self.left_image_topic = self.get_parameter('left_image_topic').value
        self.right_image_topic = self.get_parameter('right_image_topic').value
        
        self.bridge = CvBridge()
        
        self.pub_exp_config_name = "/stereo_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/stereo_py_driver/exp_settings_ack"
        self.pub_left_img_name = "/stereo_py_driver/left_img_msg"
        self.pub_right_img_name = "/stereo_py_driver/right_img_msg"
        self.pub_timestep_name = "/stereo_py_driver/timestep_msg"
        self.send_config = True
        
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)
        self.subscribe_exp_ack_ = self.create_subscription(
            String, self.sub_exp_ack_name, self.ack_callback, 10)
        self.publish_left_img_ = self.create_publisher(Image, self.pub_left_img_name, 1)
        self.publish_right_img_ = self.create_publisher(Image, self.pub_right_img_name, 1)
        self.publish_timestep_ = self.create_publisher(Float64, self.pub_timestep_name, 1)
        
        self.left_image_sub = self.create_subscription(
            Image, self.left_image_topic, self.left_image_callback, 10)
        self.right_image_sub = self.create_subscription(
            Image, self.right_image_topic, self.right_image_callback, 10)
        
        self.exp_config_msg = self.settings_name
        self.get_logger().info(f"Configuration to be sent: {self.exp_config_msg}")

        self.frame_id = 0
        self.get_logger().info("StereoDriver initialized, attempting handshake with CPP node")

    def ack_callback(self, msg):
        if msg.data == "ACK":
            self.send_config = False
            self.get_logger().info("Received ACK from CPP node")

    def handshake_with_cpp_node(self):
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

    def left_image_callback(self, msg):
        try:
            self.frame_id += 1
            timestep = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            timestep_msg = Float64()
            timestep_msg.data = timestep
            self.publish_timestep_.publish(timestep_msg)

            self.publish_left_img_.publish(msg)
            self.get_logger().info(f"Published left image frame {self.frame_id}, timestamp: {timestep}")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def right_image_callback(self, msg):
        try:
            self.frame_id += 1
            timestep = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            timestep_msg = Float64()
            timestep_msg.data = timestep
            self.publish_timestep_.publish(timestep_msg)

            self.publish_right_img_.publish(msg)
            self.get_logger().info(f"Published right image frame {self.frame_id}, timestamp: {timestep}")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoDriver("stereo_py_node")

    while node.send_config:
        node.handshake_with_cpp_node()
        rclpy.spin_once(node)
        if not node.send_config:
            break

    node.get_logger().info("Handshake Complete")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()