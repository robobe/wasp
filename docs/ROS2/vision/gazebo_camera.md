# ROS2 Gazebo camera sensor

```bash
ros2 topic echo /camera/image_raw --no-arr
# Command help
ros2 topic echo --help
# --no-arr              Don't print array fields of messages

```

```
header:
  stamp:
    sec: 130
    nanosec: 262000000
  frame_id: camera_link
height: 480
width: 640
encoding: rgb8
is_bigendian: 0
step: 1920
data: '<sequence type: uint8, length: 921600>'
```