import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import logging
from launch.actions import SetEnvironmentVariable

log = logging.get_logger("My_launch")

def generate_launch_description():
    log.info("start launch")
    ld = LaunchDescription()
    log_color_action = SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value="1")
    my_node = Node(
        package="my_python_pkg",
        executable="test",
    )
    
    ld.add_action(log_color_action)
    ld.add_action(my_node)
    return ld