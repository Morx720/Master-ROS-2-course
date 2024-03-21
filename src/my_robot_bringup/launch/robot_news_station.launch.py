from launch import LaunchDescription
from launch_ros.actions import Node

robotList= ["R1","R2","R3","R4", "R5"]

def generate_launch_description():
    ld = LaunchDescription()
    
    for robot in robotList:
    
        robot_node = Node(
            package="my_py_pkg",
            executable="robot_news_station",
            name=f"{robot}_news_station",
            parameters=[
                {"RobotName": robot}
          ]
        )
        ld.add_action(robot_node)
        
    
    smartphone_node = Node(
        package="my_py_pkg",
        executable="smartphone",
        name="smartphone"
    )
    ld.add_action(smartphone_node)
        
    return ld