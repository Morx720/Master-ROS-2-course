#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.create_subscription(Int64,"number",self.subCallback,10)
        self.pub_ = self.create_publisher(Int64,"number_counter",10)
        self.sum_ =0

        self.get_logger().info("number_counter has been started")

    def subCallback(self,msg:Int64):
        self.sum_ += msg.data
        pubMsg = Int64()
        pubMsg.data = self.sum_
        self.pub_.publish(pubMsg)
        


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()