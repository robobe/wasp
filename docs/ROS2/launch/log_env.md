# Launch log, environment variables and ExecuteProcess

- Launch logging
- Add environment variable

## launch logging

```python title="log.launch.py" linenums="1" hl_lines="2 5 8"
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
```


## SetEnvironmentVariable
demo: add colorize log for launched node's

- set environment variable `RCUTILS_COLORIZED_OUTPUT` to 1 to get node colored logging

- See line 3, 10, 16 how to set env. variable from code using launch file

## demo 2
- Run gazebo with execute process

```
start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', world],
        cwd=[launch_dir], output='screen')
```