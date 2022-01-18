# Gazebo Plugins

```
ros2 pkg create --build-type ament_cmake wasp_gazebo_pkg --dependencies gazebo gazebo_ros
```

```
colcon build
```

!!! Warning
    If fail on `No module named 'catkin_pkg'`
    Try add `pip install catkin-pkg` package to virtual env
    Install it global `sudo apt install python3-catkin-pkg` not help and use it outside the venv without success, need to understand way