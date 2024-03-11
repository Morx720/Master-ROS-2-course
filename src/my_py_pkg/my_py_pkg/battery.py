#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from functools import partial

class BatteryNode(Node):
   def __init__(self):
      super().__init__("battery")

      self.battery_last_state_change = self.getTime()
      self.battery_empty = True
      self.callSetLedServer(3,True)
      timer_ = self.create_timer(0.1, self.simulateBattery)
      

      self.get_logger().info("battery has been started")


   def getTime(self):
      sec, nsec = self.get_clock().now().seconds_nanoseconds()
      
      return sec + nsec /1000000000.0
      
   def simulateBattery(self):
      now = self.getTime()
      if self.battery_empty:
         if now - self.battery_last_state_change > 6.0:
            self.battery_last_state_change = now
            self.battery_empty = False
            self.callSetLedServer(3, False)
            self.get_logger().info("Battery full")
      else:
         if now -self.battery_last_state_change > 4.0:
            self.battery_last_state_change = now
            self.battery_empty = True
            self.callSetLedServer(3, True)
            self.get_logger().info("Battery empty")
      
      
   def callSetLedServer(self, led:int, state:bool):
      cli_ = self.create_client(SetLed, "set_led")
      
      while not cli_.wait_for_service(timeout_sec=1.0):
         self.get_logger().warn("waiting for server...")
        
      request = SetLed.Request()
      request.led_number = led
      request.state = state

      future = cli_.call_async(request=request)
      future.add_done_callback(partial(self.callbackSetLed, led=led, state=state))
        
   def callbackSetLed(self, future, led, state):
      try:
         response = future.result()
         
         if response.success:
            self.get_logger().info(f"Led {led} has been set to {state}")
            
         else:
            self.get_logger().warn("Service call was successful but request not satisfied")
         
      except Exception as e:
         self.get_logger().error(f"Servce call faild with exception {e}") 

      


def main(args=None):
   rclpy.init(args=args)
   node = BatteryNode()
   rclpy.spin(node)
   rclpy.shutdown()

if __name__ == "__main__":
   main()