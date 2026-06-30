import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image, CompressedImage

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge

import cv2

import numpy as np

class IrInterpret(Node):

    def __init__(self):
        super().__init__('ir_interpret')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.declare_parameter('camera_id', 0)
        camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value

        self.declare_parameter('front_id', 0)
        front_id = self.get_parameter('front_id').get_parameter_value().integer_value

        self.publisher = self.create_publisher(CompressedImage, f'/cameras/ir/camera_{camera_id}', qos)
        self.subscription = self.create_subscription(
            CompressedImage,
            f'/cameras/raw/camera_{camera_id}',
            self.listener_callback,
            qos)
        self.subscription  # prevent unused variable warning

        self.raw_sub = self.create_subscription(
            CompressedImage,
            f'/cameras/raw/camera_{front_id}',
            self.listener_callback,
            qos)

        self.bridge = CvBridge()

        self.filter_size = 21

        self.heat_place = (-1, -1)

        self.x_length = 160
        self.y_length = 120
    
    def next_val(self, length, i):
        chunks = length / self.filter_size
        if i < chunks:
            return i * self.filter_size
        
        if length - i * self.filter_size > 0:
            return length - self.filter_size
        
        return -1

    def listener_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        x_i = 0
        y_i = 0

        x = self.next_val(self.x_length, x_i)
        y = self.next_val(self.y_length, y_i)

        max_val = -1
        max_coord = (-1, -1)

        while y != -1:
            while x != -1:
                region = hsv_frame[y:y+self.filter_size, x:x+self.filter_size, 2]
                avg = np.sum(region)/(self.filter_size*self.filter_size)
                if avg > max_val:
                    max_val = avg
                    max_coord = (x,y)
                x_i = x_i + 1
                x = self.next_val(self.x_length, x_i)
            x_i = 0
            x = self.next_val(self.x_length, x_i)
            y_i = y_i + 1
            y = self.next_val(self.y_length, y_i)

        self.heat_place = max_coord


        cv2.rectangle(frame, max_coord, (max_coord[0]+self.filter_size, max_coord[1]+self.filter_size), color=(255, 255, 255), thickness = -1)

        msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')

        msg.header.stamp = self.get_clock().now().to_msg()
        

        self.publisher.publish(msg)

        # thermogram = flyr.unpack(frame)

        # temp_in_celsius = thermogram.celsius
        
        # print(f"Size: {temp_in_celsius.shape()}")

        # print(f"heat array: {temp_in_celsius}")

def main(args=None):
    rclpy.init(args=args)

    ir_interpret = IrInterpret()

    rclpy.spin(ir_interpret)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ir_interpret.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()