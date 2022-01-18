ROS2 Install

for now install [foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

!!! Note
    Add `source /opt/ros/foxy/setup.bash` to .bashrc


## Gazebo ROS
```
sudo apt install ros-foxy-gazebo-ros
sudo apt install ros-foxy-gazebo-plugins
```

### Test
```bash
# Add gazrbo environment
source /usr/share/gazebo/setup.bash
# Export GAZEBO_PLUGIN_PATH 
export GAZEBO_PLUGIN_PATH=/opt/ros/foxy/lib:$GAZRBO_PLUGIN_PATH
# run
gazebo --verbos /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_camera_demo.world
```