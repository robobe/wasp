import asyncio
import sys
# sys.path.insert(0,'..')
import pygazebo
from pymavlink import mavutil


loop = asyncio.get_event_loop()
master = mavutil.mavlink_connection("tcp:0.0.0.0:5762")
master.wait_heartbeat()
counter = 0

def cb(data):
    global counter
    counter+=1
    if counter%8 == 0:
        counter=0
    else:
        return
    
    message = pygazebo.msg.OpticalFlow_pb2.OpticalFlow.FromString(data)
    print('Received message:', 
    int(message.time_usec),
        int(1),
        int(message.integration_time_us),
        float(message.integrated_x),
        float(message.integrated_y)
    )
    print(float(message.temperature),
        int(message.quality),
        int(message.time_delta_distance_us),
        float(message.distance))
    #optical_flow_rad_send(
    # self, time_usec,
    #  sensor_id,
    #  integration_time_us,
    #  integrated_x,
    #  integrated_y,
    #  integrated_xgyro,
    #  integrated_ygyro,
    #  integrated_zgyro,
    #  temperature,
    #  quality,
    #  time_delta_distance_us,
    #  distance,
    #  force_mavlink1=False):
    master.mav.optical_flow_rad_send(
        int(message.time_usec),
        int(1),
        int(message.integration_time_us),
        float(message.integrated_x),
        float(message.integrated_y),
        0,# message.integrated_xgyro,
        0,# message.integrated_ygyro,
        0,# message.integrated_zgyro,
        int(message.temperature),
        int(message.quality),
        int(message.time_delta_distance_us),
        float(message.distance)
    )
def handler():
    try:
        print(master.recv_match().to_dict())
    except:
        pass
    finally:
        loop.call_later(1, handler)        

async def publish_loop():
    manager = await pygazebo.connect()

    subscriber = await manager.subscribe("/gazebo/default/iris_demo/px4flow/link/opticalFlow", 
    'sensor_msgs.msgs.OpticalFlow', cb)
        
    
    
    while True:
        await asyncio.sleep(1)



loop.call_later(1, handler)
loop.create_task(publish_loop())
loop.run_forever()