"""
https://www.ardusub.com/developers/pymavlink.html
"""
import os
os.environ['MAVLINK20']="1"
from pymavlink import mavutil
import time
# Connect to SITL serial 2
master = mavutil.mavlink_connection("tcp:0.0.0.0:5762")
master.wait_heartbeat()
while True:
    try:
        msg = master.recv_match()
        print(type(msg))
        print(msg.to_dict())
    except:
        pass
    time.sleep(0.1)