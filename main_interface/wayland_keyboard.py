import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int8, Bool
from sensor_msgs.msg import Image
from message_filters import Subscriber, TimeSynchronizer

import numpy as np

import evdev
from evdev import ecodes

import cv2

import json

from cv_bridge import CvBridge

base_motion = "still"
pov_base_motion = "still"
rev_status = False

video_display = "raw"

class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__("keyboard_input")

        self.declare_parameter("keyboard_name", "usb-ITE_Tech._Inc._ITE_Device_8258_-event-kbd")

        kbd_name = self.get_parameter("keyboard_name").get_parameter_value().string_value
        
        self.keyboard_input_publisher = self.create_publisher(String, '/keyboard', 1)
        self.ms_publisher = self.create_publisher(String, '/motor_states/drive', 1)
        self.pov_ms_publisher = self.create_publisher(String, '/motor_states/pov_drive', 1)

        self.reverse_subscription = self.create_subscription(
            Bool,
            f'/reverse',
            self.get_reverse,
            1)

        self.kbd = evdev.InputDevice(f"/dev/input/by-id/{kbd_name}")

        self.keyboard_capturing = True
        
        timer_period = 1/60  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def get_reverse(self, msg):
        global rev_status
        rev_status = msg.data

    def timer_callback(self):
        raw_keys = self.kbd.active_keys()
        keys_down = [ecodes.KEY[k] for k in raw_keys]

        if "KEY_F11" in keys_down:
            self.keyboard_capturing = False
        elif "KEY_F12" in keys_down:
            self.keyboard_capturing = True

        if self.keyboard_capturing:
            # if keyboard.Key.esc in keys_down:
            #     # quit
            #     raise PeacefulExit()

            if not rev_status:
                if "KEY_W" in keys_down and "KEY_A" in keys_down:
                    base_motion = "forward_left"
                elif "KEY_W" in keys_down and "KEY_D" in keys_down:
                    base_motion = "forward_right"
                elif "KEY_S" in keys_down and "KEY_A" in keys_down:
                    base_motion = "reverse_left"
                elif "KEY_S" in keys_down and "KEY_D" in keys_down:
                    base_motion = "reverse_right"
                elif "KEY_W" in keys_down:
                    base_motion = "forward"
                elif "KEY_S" in keys_down:
                    base_motion = "reverse"
                elif "KEY_A" in keys_down:
                    base_motion = "left"
                elif "KEY_D" in keys_down:
                    base_motion = "right"
                else:
                    base_motion = "still"
                pov_base_motion = base_motion
            else:
                if "KEY_W" in keys_down and "KEY_A" in keys_down:
                    pov_base_motion = "forward_left"
                    base_motion = "reverse_right"
                elif "KEY_W" in keys_down and "KEY_D" in keys_down:
                    pov_base_motion = "forward_right"
                    base_motion = "reverse_left"
                elif "KEY_S" in keys_down and "KEY_A" in keys_down:
                    pov_base_motion = "reverse_left"
                    base_motion = "forward_right"
                elif "KEY_S" in keys_down and "KEY_D" in keys_down:
                    pov_base_motion = "reverse_right"
                    base_motion = "forward_left"
                elif "KEY_W" in keys_down:
                    pov_base_motion = "forward"
                    base_motion = "reverse"
                elif "KEY_S" in keys_down:
                    pov_base_motion = "reverse"
                    base_motion = "forward"
                elif "KEY_A" in keys_down:
                    pov_base_motion = "left"
                    base_motion = "right"
                elif "KEY_D" in keys_down:
                    pov_base_motion = "right"
                    base_motion = "left"
                else:
                    pov_base_motion = "still"
                    base_motion = "still"


            current_keys = String()
            current_keys.data = str(keys_down)

            print(f"Keys Down: {str(keys_down)}")

            self.keyboard_input_publisher.publish(current_keys)

        ms_msg = String()
        ms_msg.data = base_motion
        self.ms_publisher.publish(ms_msg)

        ms_msg.data = pov_base_motion
        self.pov_ms_publisher.publish(ms_msg)


def main(args=None):
    global video_display
    rclpy.init(args=args)

    keyboard_input_node = KeyboardInputNode()

    rclpy.spin(keyboard_input_node)

    keyboard_input_node.destroy_node()
    rclpy.shutdown()
    
    print("Exited program successfully.  Have a nice day!")


if __name__ == '__main__':
    main()