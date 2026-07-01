import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int8, Bool
from sensor_msgs.msg import Image, CompressedImage

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge

import cv2
import numpy as np

base_motion = "still"
speed = 30

class AlignAuto(Node):

    def __init__(self):
        super().__init__('align_auto')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_raw = self.create_publisher(Image, '/align_auto/raw', qos)
        self.pub_masked = self.create_publisher(Image, '/align_auto/masked', qos)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/cameras/raw/camera_2',
            self.listener_callback,
            qos)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()

        self.buffer_val = 15

        self.speed_pub = self.create_publisher(Int8, '/speed', 1)
        self.ms_publisher = self.create_publisher(String, '/motor_states/drive', 1)

        self.auto_pub = self.create_publisher(Bool, '/auto_on', 1)

        auto_msg = Bool()
        auto_msg.data = True
        self.auto_pub.publish(auto_msg)

    def listener_callback(self, msg):
        self.straight(msg)

    def straight(self, msg):
        global base_motion
        global speed

        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cropped = frame[110:135, 185:200, :]

        msg = self.bridge.cv2_to_imgmsg(cropped, "bgr8")
        self.pub_raw.publish(msg)

        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        lower_color = np.array([20, 50, 100])
        upper_color = np.array([60, 255, 255])

        mask = cv2.inRange(hsv, lower_color, upper_color)

        result = cv2.bitwise_and(cropped, cropped, mask=mask)

        msg = self.bridge.cv2_to_imgmsg(result, "bgr8")
        self.pub_masked.publish(msg)

        grayscale = result[:, :, 2]
        non_black_count = cv2.countNonZero(grayscale)
        print(non_black_count)

        if non_black_count > self.buffer_val:
            if base_motion != "forward_right":
                base_motion = "forward_right"
                ms_msg = String()
                ms_msg.data = base_motion
                self.ms_publisher.publish(ms_msg)
            
            if speed != 10:
                speed = 10
                speed_msg = Int8()
                speed_msg.data = speed
                self.speed_pub.publish(speed_msg)
        else:
            if base_motion != "forward_left":
                base_motion = "forward_left"
                ms_msg = String()
                ms_msg.data = base_motion
                self.ms_publisher.publish(ms_msg)

            if speed != 10:
                speed = 10
                speed_msg = Int8()
                speed_msg.data = speed
                self.speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)

    align_auto = AlignAuto()

    rclpy.spin(align_auto)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    align_auto.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()