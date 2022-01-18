# Hello launch files

```python
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker",
    )
    
    ld.add_action(listener_node)
    return ld
```

!!! Note
    Your launch file must contain this function: `generate_launch_description()`, and must return a `LaunchDescription` object.

## setup.py
- Add action to data_file copy launch folder to `pkg` folder (install/my_python_pkg/share/my_python_pkg/)

```python
data_files=[
    (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py'))
],

```

!!! Note
    Add import to os and glob
    ```
    import os
    from glob import glob
    ```
---

##  usage
```bash
#ros2 launch <package_name> <launch_file_name>
ros2 launch my_python_pkg simple.launch.py
```

---

# Include other launch file
```python title="parent.launch.py" linenums="1" hl_lines="2-5"
{{include("src/my_python_pkg/launch/parent.launch.py")}}
```
---

# References
- [Launch file](https://roboticsbackend.com/ros2-launch-file-example/)
- [ROS2 launch](https://docs.ros.org/en/rolling/Tutorials/Launch/Creating-Launch-Files.html)