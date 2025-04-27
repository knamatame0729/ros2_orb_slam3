#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge, CvBridgeError
import time

class MonoDriver(Node):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)

        # Initialize parameters to be passed from the command line (or launch file)
        self.declare_parameter("settings_name", "Lunar_Cam")

        # ================= Change this topic name to active camera topic =====================
        self.declare_parameter("image_topic", "/front_camera/image")

        #* Parse values sent by command line
        self.settings_name = self.get_parameter('settings_name').value
        self.image_topic = self.get_parameter('image_topic').value
        
        # Define a CvBridge object
        self.bridge = CvBridge()
        
        #* ROS2 publisher/subscriber variables [HARDCODED]
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"  # Camera image
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.send_config = True # Set False once handshake is completed with the cpp node
        
        #* Setup ROS2 publishers and subscribers
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)

        #* Subscriber to get acknowledgement from CPP node that it received experimetn settings
        self.subscribe_exp_ack_ = self.create_subscription(String, self.sub_exp_ack_name, self.ack_callback, 10)
        
        # Publisher to send RGB image
        self.publish_img_msg_ = self.create_publisher(Image, self.pub_img_to_agent_name, 1)

        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)
        
        # Subscribe camera image
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        
        #* Build the configuration string to be sent out
        self.exp_config_msg = self.settings_name
        self.get_logger().info(f"Configuration to be sent: {self.exp_config_msg}")

        self.frame_id = 0

        self.get_logger().info("MonoDriver initialized, attempting handshake with CPP node")

    # ****************************************************************************************
    def ack_callback(self, msg):
        """
            Callback function
        """
        print(f"Got ack: {msg.data}")
        
        if(msg.data == "ACK"):
            self.send_config = False
            # self.subscribe_exp_ack_.destory() # TODO doesn't work 
    # ****************************************************************************************

    # ****************************************************************************************
    def handshake_with_cpp_node(self):
        """
            Send and receive acknowledge of sent configuration settings
        """
        if (self.send_config == True):
            # print(f"Sent mesasge: {self.exp_config_msg}")
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)
    # ****************************************************************************************
    
    # ****************************************************************************************
    def image_callback(self, msg):
        """
        Callback that sends the RGB image message to the CPP node
        """
        try:
            # Increment frame id
            self.frame_id += 1

            # Timestamp
            timestep = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            timestep_msg = Float64()
            timestep_msg.data = timestep
            
            # Publish image
            self.publish_timestep_msg_.publish(timestep_msg)

            # Publish timestamp
            self.publish_img_msg_.publish(msg)

            self.get_logger().info(f"Published image frame {self.frame_id}, timestamp: {timestep}")

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
    # ****************************************************************************************

def main(args=None):
    rclpy.init(args=args) # Initialize node
    node = MonoDriver("mono_py_node")

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


            