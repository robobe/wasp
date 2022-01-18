from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    my_node = Node(
        package="my_python_pkg",
        executable="test",
    )
    
    ld.add_action(my_node)
    return ld