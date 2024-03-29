#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStates
from my_robot_interfaces.srv import SetLed


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")

        self.declare_parameter("led_states", [False,False,False])
        self.leds = self.get_parameter("led_states").value
        
        self.pub_ = self.create_publisher(LedStates, "led_panel_state",10)
        self.server_ = self.create_service(SetLed, "set_led", self.callbackSetLed)
        self.get_logger().info("led_panel has been started")
        self.publishLedState()

    def publishLedState(self):
        msg = LedStates()
        msg.led_state = self.leds
        self.pub_.publish(msg=msg)
        
    def callbackSetLed(self, request:SetLed.Request, response:SetLed.Response):
        
        led = request.led_number
        state = request.state
        if not (led <4 and led >0):
            self.get_logger().error(f"Service call denied. got invaled led number:{led}. allowed number 1-3")
            response.success = False
            return response
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
    
