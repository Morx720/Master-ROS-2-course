#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import AliveTurtles
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.srv import CatchTurtle
from my_robot_interfaces.srv import TunePid
from my_robot_interfaces.srv import GetPidValues
from functools import partial

import math
from simple_pid import PID

class turtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
    
        self.declare_parameter("catch_closest_turtle_first", True)
        self.catch_closest_turtle_first = self.get_parameter("catch_closest_turtle_first").value
        self.pose_ = None
        self.pose_sub = self.create_subscription(Pose, "turtle1/pose", self.callbackTurtlePose,10)
        self.alive_turtles_sub = self.create_subscription(AliveTurtles, "alive_turtles", self.updateTarget,10)
        self.cmd_Vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.tune_pid_server = self.create_service(TunePid, "tune_pid", self.callbackTunePid)
        self.get_pid_server = self.create_service(GetPidValues, "get_pid_values", self.callbackGetPidValues)
        self.control_timer = self.create_timer(0.001,self.callbackControllLoop)

        self.turtue_to_catch = None

        self.pid_lin_vel = PID(2.0, 0.0, 0.0, 0.0)
        self.pid_ang_vel = PID(10.0, 0.0, 0.0, 0.0)
        
        

        self.get_logger().info("turtle_controller has been started")
        
    def updateTarget(self, msg:AliveTurtles):
        if len(msg.trutles) >0:
            closest_turtle = None
            closest_distance = None
            if self.catch_closest_turtle_first:
                for turtle in msg.trutles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)
                    if closest_turtle == None or distance < closest_distance:
                        closest_turtle = turtle
                        closest_distance = distance
                        
                self.turtue_to_catch = closest_turtle
            else:
                self.turtue_to_catch = msg.trutles[0]

    def callbackTurtlePose(self,msg):
        self.pose_ = msg
        
    def callbackControllLoop(self):
        if self.pose_ == None or self.turtue_to_catch == None:
            return
        msg = Twist()
        
        dist_x = self.turtue_to_catch.x - self.pose_.x
        dist_y = self.turtue_to_catch.y  - self.pose_.y
        dist = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        
        if dist > 0.5:
            
            x_vel = self.pid_lin_vel(-dist)
            msg.linear.x = x_vel

        
            target_theta = math.atan2(dist_y, dist_x)
            diff_theta = target_theta - self.pose_.theta
                    
            clipped_theta = pi_clip(-diff_theta)
            
            msg.angular.z = self.pid_ang_vel(clipped_theta)
            
        else: #target reached
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.callCatchTurtle (self.turtue_to_catch.name)
            self.turtue_to_catch = None
                    
        self.cmd_Vel_pub.publish(msg)
        
    def callCatchTurtle(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server....")

        request = CatchTurtle.Request()
        
        request.name = turtle_name
        
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_turtle_catch, turtle_name=request.name))
    
    def callback_call_turtle_catch(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().info(f"Turtle {turtle_name} could not be caught")
                
        except Exception as e:
            self.get_logger().error(f"Servce call faild with exception {e}") 

    def callbackTunePid(self, request:TunePid.Request, response:TunePid.Response):
        self.pid_lin_vel.Kp = request.linear_velocity.p
        self.pid_lin_vel.Ki = request.linear_velocity.i
        self.pid_lin_vel.Kd = request.linear_velocity.d
        
        self.pid_ang_vel.Kp = request.angular_velocity.p
        self.pid_ang_vel.Ki = request.angular_velocity.i
        self.pid_ang_vel.Kd = request.angular_velocity.d
       
        response.success = True
        return response
    
    def callbackGetPidValues(self, request:GetPidValues.Request, response:GetPidValues.Response):
        response.linear_velocity.p = self.pid_lin_vel.Kp
        response.linear_velocity.i = self.pid_lin_vel.Ki
        response.linear_velocity.d = self.pid_lin_vel.Kd
        
        response.angular_velocity.p = self.pid_ang_vel.Kp
        response.angular_velocity.i = self.pid_ang_vel.Ki
        response.angular_velocity.d = self.pid_ang_vel.Kd
        
        return response


def pi_clip(angle):
    if angle > 0:
        if angle > math.pi:
            return angle - 2*math.pi
    else:
        if angle < -math.pi:
            return angle + 2*math.pi
    return angle


        
    

def main(args=None):
    rclpy.init(args=args)
    node = turtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    