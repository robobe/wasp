# ROS2 simple python node

- create pkg
- create node
- build source and run


```bash title="ros2 python pkg create"
# cd to src workspace folder
ros2 pkg create my_python_pkg --build-type ament_python
```

```bash title="build"
# cd to workspace root folder
colcon build --packages-select my_python_pkg
```

---
### Node code

- Add python file `my_node` under `my_python_pkg` project/pkg folder
```python title="hello_py_node.py"
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("This node just says 'Hello'")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## setup.py
- Add entry point

```json
entry_points={
        'console_scripts': [
            "test = my_python_pkg.my_node:main"
        ],
    }
```

---

## Build and Run
```bash
# from ws root folder
colcon build --packages-select my_python_pkg

# source
source instell/setup.bash

# run
ros2 run my_python_pkg test

# Result
[INFO] [1642482347.288644110] [my_node]: This node just says 'Hello'
```

---

# References
- [Create a ROS2 Python package](https://roboticsbackend.com/create-a-ros2-python-package/)
- [Launch file](https://roboticsbackend.com/ros2-launch-file-example/)