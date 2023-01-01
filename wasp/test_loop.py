import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from math import tan, sqrt
import time
from .lib_aruco_pose import *
import os

CAMERA_TOPIC = "/camera/image_raw"
HEIGHT = 640
WIDTH = 480
HFOV =  1.0236 # in rad

class MyNode(Node):
    def __init__(self):
        node_name="test_loop"
        super().__init__(node_name)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, CAMERA_TOPIC, self.image_handler, qos_profile=qos_profile_sensor_data)
        self.init_aruco()
        self.connect()
        self.get_logger().info("test_loop")
        self.start_time =  int(round(time.time() * 1000))

    def find_aruco(self, image):
        corners, ids, rejected = cv2.aruco.detectMarkers(image, self.arucoDict,
                parameters=self.arucoParams)
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()

            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the bounding box of the ArUCo detection
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

                # draw the ArUco marker ID on the image
                cv2.putText(image, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                print("[INFO] ArUco marker ID: {}".format(markerID))
                return cX, cY
        else:
            return int(HEIGHT/2), int(WIDTH/2)

    def pix2rad(self, pixel):
        h_fov = HFOV
        v_fov = HFOV * HEIGHT / WIDTH
        h_center = WIDTH / 2
        v_center = HEIGHT / 2
        
        x, y = pixel
        dx = x - h_center
        dy = y - v_center

        x_rad = dx * h_fov / WIDTH
        y_rad = dy * v_fov / HEIGHT

        return -x_rad, -y_rad

    def current_milli_time(self):
        
        current_milli_time = int(round(time.time() * 1000) - self.start_time)
        return current_milli_time


    def send_distance_sensor(self, depth_to_ground):
    # Average out a portion of the centermost part
        curr_dist = int(depth_to_ground * 100)
        if curr_dist > 0 and curr_dist < 4000:
            self.get_logger().info(f"Height : {curr_dist}")
            self.vehicle._master.mav.distance_sensor_send(
                self.current_milli_time(),# ms Timestamp (UNIX time or time since system boot) (ignored)
                0,   # min_distance, uint16_t, cm
                10000,   # min_distance, uint16_t, cm
                curr_dist,      # current_distance,	uint16_t, cm
                10,	            # type : 0 (ignored)
                0,              # id : 0 (ignored)
                mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,              # orientation
                0               # covariance : 0 (ignored)
            )

    def send_landing_target_message(self, xy, z):
        x_offset_rad, y_offset_rad = xy
        x = z * tan(x_offset_rad)
        y = z * tan(y_offset_rad)
        distance = sqrt(x*x + y*y + z*z)
        self.get_logger().info(f"distance : {distance}")
        self.vehicle._master.mav.landing_target_send(
            self.current_milli_time(),                       # time target data was processed, as close to sensor capture as possible
            0,                                  # target num, not used
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
            x_offset_rad,                       # X-axis angular offset, in radians
            y_offset_rad,                       # Y-axis angular offset, in radians
            distance,                           # distance, in meters
            0,                                  # Target x-axis size, in radians, not used
            0,                                  # Target y-axis size, in radians, not used
            0,                                  # x	float	X Position of the landing target on MAV_FRAME, not used
            0,                                  # y	float	Y Position of the landing target on MAV_FRAME, not used
            0,                                  # z	float	Z Position of the landing target on MAV_FRAME, not used
            (1,0,0,0),      # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0), not used
            2,              # type of landing target: 2 = Fiducial marker, not used
            1,              # position_valid boolean, not used
        )

    def image_handler(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # pixel = self.find_aruco(frame)
            # rad = self.pix2rad(pixel)
            marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(frame, loop=False)
            alt = self.vehicle.location.global_relative_frame.alt
            x_cm, y_cm          = self.camera_to_uav(x_cm, y_cm)
            z_cm                = self.vehicle.location.global_relative_frame.alt*100.0
            angle_x, angle_y    = self.marker_position_to_angle(x_cm, y_cm, z_cm)
            # self.send_landing_target_message(rad, alt)
            time_0 = time.time()
            self.get_logger().info("Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x, angle_y))
            self.send_land_message_v2(x_rad=angle_x, y_rad=angle_y, dist_m=z_cm*0.01, time_usec=time.time()*1e6)
            # self.send_distance_sensor(alt)
            cv2.imshow("image", frame)
            cv2.waitKey(3)
        except CvBridgeError as e:
            self.get_logger().error("load image failed")

    def init_aruco(self):
        pass
        
    def send_land_message_v2(self,x_rad=0, y_rad=0, dist_m=0, x_m=0,y_m=0,z_m=0, time_usec=0, target_num=0):
        msg = self.vehicle.message_factory.landing_target_encode(
            int(time_usec),          # time target data was processed, as close to sensor capture as possible
            target_num,          # target num, not used
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
            x_rad,          # X-axis angular offset, in radians
            y_rad,          # Y-axis angular offset, in radians
            dist_m,          # distance, in meters
            0,          # Target x-axis size, in radians
            0,          # Target y-axis size, in radians
            x_m,          # x	float	X Position of the landing target on MAV_FRAME
            y_m,          # y	float	Y Position of the landing target on MAV_FRAME
            z_m,          # z	float	Z Position of the landing target on MAV_FRAME
            (1,0,0,0),  # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            2,          # type of landing target: 2 = Fiducial marker
            1,          # position_valid boolean
        )
        self.vehicle.send_mavlink(msg)
    
    def marker_position_to_angle(self, x, y, z):
        
        angle_x = math.atan2(x,z)
        angle_y = math.atan2(y,z)
        
        return (angle_x, angle_y)
        
    def camera_to_uav(self, x_cam, y_cam):
        x_uav = x_cam
        y_uav = y_cam
        return(x_uav, y_uav)

    def connect(self):
        connection_string = "udp:0.0.0.0:14550"
        self.vehicle = connect(connection_string, wait_ready=True)
        self.vehicle.wait_ready('autopilot_version')
        self.get_logger().info("\nGet all vehicle attribute values:")
        # self.get_logger().info(" Autopilot Firmware version: %s" % vehicle.version)
        # self.get_logger().info("   Major version number: %s" % vehicle.version.major)
        # self.get_logger().info("   Minor version number: %s" % vehicle.version.minor)
        # self.get_logger().info("   Patch version number: %s" % vehicle.version.patch)
        # self.get_logger().info("   Release type: %s" % vehicle.version.release_type())
        # self.get_logger().info("   Release version: %s" % vehicle.version.release_version())
        # self.get_logger().info("   Stable release?: %s" % vehicle.version.is_stable())

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
        self.vehicle.simple_takeoff(20)  # Take off to target altitude
        while True:
        # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= 5 * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

        self.vehicle.mode = VehicleMode("LAND")
        while not self.vehicle.mode.name=='LAND':  #Wait until mode has changed
            self.get_logger().info(" Waiting for mode change ...")
            time.sleep(1)

calib_path  = os.path.dirname(__file__)
camera_matrix       = np.loadtxt(calib_path+'/cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'/cameraDistortion_raspi.txt', delimiter=',')                                      
aruco_tracker       = ArucoSingleTracker(id_to_find=3, marker_size=5, show_video=False, 
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()