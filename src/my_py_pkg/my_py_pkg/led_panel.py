#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStates
from my_robot_interfaces.srv import SetLed


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")

        self.leds = [False,False,False]
        
        self.pub_ = self.create_publisher(LedStates, "led_panel_state",10)
        self.server_ = self.create_service(SetLed, "set_led", self.callbackSetLed)
        self.get_logger().info("led_panel has been started")

    def publishLedState(self):
        msg = LedStates()
        msg.led_state = self.leds
        self.pub_.publish(msg=msg)
        
    def callbackSetLed(self, request:SetLed.Request, response:SetLed.Response):
        
        led = request.led_number
        state = request.state
       
        self.leds[led-1] = state
        self.publishLedState()
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
