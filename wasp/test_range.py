"""
distance_sensor_send #132
RANGEFINDER report ( #173 )
"""
import rclpy
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from pymavlink import mavutil
from dronekit import connect, VehicleMode
import time

RANGE_MIN = 2
RANGE_MAX = 40000
RNGFND1_TYPE_MAVLINK = 10
SENSOR_ID = 1
TOPIC_RANGE = "/range"

TAKEOFF_ALT = 20

class MyNode(Node):
    def __init__(self):
        NODE_NAME="test_range"
        super().__init__(NODE_NAME)
        node_handler_group = ReentrantCallbackGroup()
        self.vehicle = None
        self.create_subscription(LaserScan, TOPIC_RANGE, 
            self.__range_handler, 
            qos_profile=qos_profile_sensor_data,
            callback_group=node_handler_group)
        # t_drone = Thread(target=self.connect, daemon=True)
        # t_drone.start()
        self.connect()
        self.start_time =  int(round(time.time() * 1000))
        self.get_logger().info(NODE_NAME)

    def __range_handler(self, msg: LaserScan):
        range = msg.ranges[2]
        if range != float('inf'):
            self.send_distance_sensor(range)
        else:
            self.send_distance_sensor(0)
        

    def current_milli_time(self):
        current_milli_time = int(round(time.time() * 1000) - self.start_time)
        return current_milli_time

    def connect(self):
        connection_string = "udp:0.0.0.0:14552"
        self.vehicle = connect(connection_string, wait_ready=True)
        self.vehicle.wait_ready('autopilot_version')
        self.get_logger().info("\nGet all vehicle attribute values:")
        return
        self.vehicle.mode = VehicleMode("GUIDED")
        while not self.vehicle.mode.name=='GUIDED':  #Wait until mode has changed
            self.get_logger().info(" Waiting for mode change ...")
            time.sleep(1)


        # Check that vehicle is armable
        while not self.vehicle.is_armable:
            self.get_logger().info(" Waiting for vehicle to initialise...")
            time.sleep(1)

        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            self.get_logger().info(" Waiting for arming...")
            time.sleep(1)

        self.get_logger().info("Taking off!")
        self.vehicle.simple_takeoff(TAKEOFF_ALT)  # Take off to target altitude
        while True:
        # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= 5 * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def send_distance_sensor(self, depth_to_ground):
        """
        time_boot_ms	uint32_t	ms		Timestamp (time since system boot).
        min_distance	uint16_t	cm		Minimum distance the sensor can measure
        max_distance	uint16_t	cm		Maximum distance the sensor can measure
        current_distance	uint16_t	cm		Current distance reading
        type	uint8_t		MAV_DISTANCE_SENSOR	Type of distance sensor.
        id	uint8_t			Onboard ID of the sensor
        orientation	uint8_t		MAV_SENSOR_ORIENTATION	Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270
        covariance	uint8_t	cm^2		Measurement variance. Max standard deviation is 6cm. UINT8_MAX if unknown.
        """
        if not self.vehicle:
            return
        curr_dist = int(depth_to_ground * 100)
        self.vehicle._master.mav.distance_sensor_send(
            self.current_milli_time(),# ms Timestamp (UNIX time or time since system boot) (ignored)
            RANGE_MIN,   # min_distance, uint16_t, cm
            RANGE_MAX,   # max, uint16_t, cm
            int(curr_dist),      # current_distance,	uint16_t, cm
            RNGFND1_TYPE_MAVLINK,
            SENSOR_ID,
            mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,              # orientation
            0               # covariance : 0 (ignored)
        )
        
        self.get_logger().info(f"current distance: {curr_dist}", throttle_duration_sec=2)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    exec.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()