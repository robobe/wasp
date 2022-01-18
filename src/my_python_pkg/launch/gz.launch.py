import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import logging
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

log = logging.get_logger("My_launch")

pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
pkg_share = FindPackageShare(package='wasp_pkg').find('wasp_pkg')
 
# Set the path to the world file
world_file_name = 'camera.world'
world_path = os.path.join(pkg_share, 'worlds', world_file_name)

def generate_launch_description():
    ld = LaunchDescription()
    log.info("start launch")


    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_path, "verbose": "true"}.items(),
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    return ld
