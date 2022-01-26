
---
title: gazebo joints PID
tags:
    - pid
    - gazebo
    - pygazebo
---

```cpp title="velodynePlugin" linenums="1" hl_lines="27"
{{include("gz/plugins/VelodynePlugin/velodyne_plugin.cc")}}
```

## Control 
- Send `vector3d` msg to gazebo using python code
- Using pygazebo package

```proto title="gazebo/msgs/vector3d.proto"
# https://github.com/osrf/gazebo/blob/gazebo11/gazebo/msgs/vector3d.proto
syntax = "proto2";
package gazebo.msgs;

message Vector3d
{
  required double x = 1;
  required double y = 2;
  required double z = 3;
}
```

## pygazebo
[pygazebo](https://github.com/robobe/pygazebo)


```python title="pub_vector.py"
import asyncio
from time import sleep
from pygazebo import pygazebo
from pygazebo.msg import vector3d_pb2

async def publish_loop():
    manager = await pygazebo.connect()

    publisher = await manager.advertise(
        '/gazebo/default/velodyne_hdl-32/vel_cmd',
        'gazebo.msgs.Vector3d')

    message = vector3d_pb2.Vector3d()
    message.x = 0.10
    message.y = 0
    message.z = 0
    
    while True:
        await publisher.publish(message)
        await asyncio.sleep(1)
        

loop = asyncio.get_event_loop()
loop.run_until_complete(publish_loop())
```
---

# Reference
- [velodyne_plugin](http://gazebosim.org/tutorials?tut=guided_i5#PluginConfiguration)
- [Physics gazebo API](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Model.html)