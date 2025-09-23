import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8

class ReadSpeedNode(Node):
    def __init__(self):
        super().__init__("main_interface")
        
        self.publisher = self.create_publisher(Int8, f'/speed', 1)

        import time
        import serial
        arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)

        while True:
            try:
                if (speed := arduino.read(1)):
                    msg = Int8()
                    msg.data = ord(speed.decode())
                    self.publisher.publish(msg)
                    print(msg.data)
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