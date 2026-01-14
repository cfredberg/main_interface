import rclpy
from rclpy.node import Node

import time
import serial

from std_msgs.msg import Int8
from std_msgs.msg import Bool

class ReadSpeedNode(Node):
    def __init__(self):
        super().__init__("main_interface")
        
        self.speed_pub = self.create_publisher(Int8, '/speed', 1)
        self.rev_pub = self.create_publisher(Bool, '/reverse', 1)

        self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
        self.arduino.flushInput()

        timer_period = 1/120  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        try:
            data_bytes = self.arduino.readline()

            if data_bytes:
                data_string = data_bytes.decode("utf-8").strip()
                if data_string:
                    print(data_string)
                    data_string = data_string.split()
                    speed_percent = int(data_string[0])
                    reverse = bool(int(data_string[1]))

                    self.arduino.flushInput()

                    speed_msg = Int8()
                    speed_msg.data = speed_percent
                    self.speed_pub.publish(speed_msg)

                    rev_msg = Bool()
                    rev_msg.data = reverse
                    self.rev_pub.publish(rev_msg)

        except UnicodeDecodeError as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)

    read_speed_node = ReadSpeedNode()
    try:
        rclpy.spin(read_speed_node)
    except Exception as e:
        print(e)

    read_speed_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()