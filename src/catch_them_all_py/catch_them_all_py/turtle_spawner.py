#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from my_robot_interfaces.msg import AliveTurtles
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.srv import CatchTurtle
from functools import partial
import random
import math

class turtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        
        self.declare_parameter("spawn_frequency", 1.0)
        self.spawnRate = self.get_parameter("spawn_frequency").value
        self.declare_parameter("turtle_name_prefix", "Turtle")
        self.name_prefix = self.get_parameter("turtle_name_prefix").value
        
        self.turtle_counter = 1 
        self.alive_turtles = []
        
        self.alive_turtle_pup_ = self.create_publisher(AliveTurtles, "alive_turtles", 10)
        self.timer_ = self.create_timer(1.0 / self.spawnRate ,self.spawnRandomTurtle)
        self.catch_turtle_server_ = self.create_service(CatchTurtle, "catch_turtle", self.catchTurtleCallback)

        self.get_logger().info("turtle_spawner has been started")
        
    def publishAliveTurtle(self):
        msg = AliveTurtles()
        msg.trutles = self.alive_turtles
        self.alive_turtle_pup_.publish(msg)

    def catchTurtleCallback(self, request:CatchTurtle.Request, response:CatchTurtle.Response):
        self.killTurtle(request.name)
        response.success = True
        return response


    def spawnRandomTurtle(self):
        client = self.create_client(Spawn, "spawn")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server....")

        request = Spawn.Request()
        
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(0.0, 10.0)
        request.theta = random.uniform(0, 2*math.pi)
        request.name = self.name_prefix + str(self.turtle_counter)
        self.turtle_counter = self.turtle_counter +1 
        
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_turtle_spawn, turtle_name=request.name , x=request.x, y=request.y, theta=request.theta))
    
    def callback_call_turtle_spawn(self, future, turtle_name, x, y, theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(f"turtlesim has spawned {response.name}")
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.alive_turtles.append(new_turtle)
                self.publishAliveTurtle()

        except Exception as e:
            self.get_logger().error(f"Servce call faild with exception {e}") 


    def killTurtle(self, turtle_name):
        client = self.create_client(Kill, "kill")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server....")

        request = Kill.Request()
        
        request.name = turtle_name
        
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_turtle_kill, turtle_name=request.name))
    
    def callback_call_turtle_kill(self, future, turtle_name):
        try:
            future.result()
            for (i, turtle) in enumerate(self.alive_turtles):
                if turtle.name == turtle_name:
                    del self.alive_turtles[i]
                    self.publishAliveTurtle()
                    self.get_logger().info(f"{turtle_name} has been catched and killed")
                    break

        except Exception as e:
            self.get_logger().error(f"Servce call faild with exception {e}") 





def main(args=None):
    rclpy.init(args=args)
    node = turtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()