import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode
import time

class MyNode(Node):
    def __init__(self):
        node_name="DK_test"
        super().__init__(node_name)
        self.connect()
        self.get_logger().info("Hello ROS2")

    def connect(self):
        connection_string = "udp:0.0.0.0:14550"
        vehicle = connect(connection_string, wait_ready=True)
        vehicle.wait_ready('autopilot_version')
        print("\nGet all vehicle attribute values:")
        print(" Autopilot Firmware version: %s" % vehicle.version)
        print("   Major version number: %s" % vehicle.version.major)
        print("   Minor version number: %s" % vehicle.version.minor)
        print("   Patch version number: %s" % vehicle.version.patch)
        print("   Release type: %s" % vehicle.version.release_type())
        print("   Release version: %s" % vehicle.version.release_version())
        print("   Stable release?: %s" % vehicle.version.is_stable())

        vehicle.mode = VehicleMode("GUIDED")
        while not vehicle.mode.name=='GUIDED':  #Wait until mode has changed
            print(" Waiting for mode change ...")
            time.sleep(1)


        # Check that vehicle is armable
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        vehicle.simple_takeoff(10)  # Take off to target altitude

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()