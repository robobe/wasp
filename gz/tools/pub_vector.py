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