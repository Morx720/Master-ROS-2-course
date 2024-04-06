from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
        
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
    )
    
    turtle_controller_node = Node(
        package="catch_them_all_cpp",
        executable="turtle_controller",
        name="controller",
        parameters =[
            {"catch_closest_turtle_first": True}
        ]
    )
    
    turtle_spawner_node = Node(
        package="catch_them_all_cpp",
        executable="turtle_spawner",
        name="spawner",
        parameters=[
            {"spawn_frequency" : 1.0},
            {"turtle_name_prefix" : "Turtle"}
        ]
    )
    
    ld.add_action(turtlesim_node)
    ld.add_action(turtle_controller_node)
    ld.add_action(turtle_spawner_node)
    
    return ld