#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.declare_parameter("number_to_publish", 2)
        self.declare_parameter("publish_rate", 1.0)
        
        
        self.number = self.get_parameter("number_to_publish").value
        self.publish_rate = self.get_parameter("publish_rate").value
        
        self.publisher_ = self.create_publisher(Int64,"number",10)
        timer_ = self.create_timer(1.0 / self.publish_rate ,self.timerCallback)
        self.get_logger().info("number_publisher has been started")

    def timerCallback(self):
        msg = Int64()
        msg.data = self.number
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()