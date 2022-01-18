from __future__ import print_function
from dronekit import connect, VehicleMode
import time
import math
import signal

# Connect to the Vehicle.
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
connection_string = "tcp:0.0.0.0:5762"
print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=False)

vehicle.wait_ready("autopilot_version")

old_x, old_y = 0, 0
old_p, old_r = 0, 0
da_p, da_r = 0, 0
delta_x, delta_y = 0, 0

def debug_cb(x, name, message):
    global old_x, old_y
    global old_p, old_r
    global da_p, da_r
    global delta_x, delta_y

    if name == "ATTITUDE":
        da_p = 10 * math.tan(message.pitch - old_p)
        da_r = 10 * math.tan(message.roll - old_r)

        time_usec = int(time.time()*10**6)
        
        vehicle._master.mav.optical_flow_send(
            time_usec,
            1,
            int(-delta_y + da_r), # flow x
            int(delta_x + da_p), # flow y
            0,
            0,
            51,
            0)
        print(int(-delta_y - da_r), int(delta_x + da_p))
        
    if name == "LOCAL_POSITION_NED":
        
        delta_x = message.x - old_x
        delta_y = message.y - old_y

        old_x, old_y = message.x, message.y
        # def optical_flow_send(self, time_usec, sensor_id, flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, quality, ground_distance, flow_rate_x=0, flow_rate_y=0, force_mavlink1=False):
        
        

    if name == "COMMAND_ACK":
        print(message)

vehicle.add_message_listener("*", debug_cb)

signal.pause()
