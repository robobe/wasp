import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.br = CvBridge()
        self.prev_frame_time = 0
        self.subscription = self.create_subscription(
            Image,
            "/demo_cam/camera1/image_raw",
            self.listener_callback,
            qos_profile_sensor_data,
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Image):
        new_frame_time = time.time()
        frame: np.ndarray = self.br.imgmsg_to_cv2(msg, "rgb8")
        # self.get_logger().info(str(frame.shape))
        fps = math.floor(1 / (new_frame_time - self.prev_frame_time))
        self.prev_frame_time = new_frame_time
        cv2.putText(
            frame,
            str(fps),
            (7, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            3,
            (100, 255, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.imshow("view", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
