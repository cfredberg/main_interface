import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from message_filters import Subscriber, TimeSynchronizer

import numpy as np

from pynput import keyboard

import cv2

import json

from cv_bridge import CvBridge

base_motion_states = {"base_motion":"still", "speed":0.5}

video_display = "raw"

keyboard_capturing = True

def key_down(key):
    try:
        keys_down.add(key.char)
    except AttributeError:
        keys_down.add(key)

def key_up(key):
    try:
        keys_down.remove(key.char)
    except AttributeError:
        keys_down.remove(key)

keys_down = set()

keyboard_listener = keyboard.Listener(on_press=key_down, on_release=key_up)
keyboard_listener.start()

class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__("keyboard_input")
        
        self.keyboard_input_publisher = self.create_publisher(String, '/keyboard', 1)
        self.bms_publisher = self.create_publisher(String, '/motor_states/drive', 1)
        
        timer_period = 1/60  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        global keyboard_capturing

        if keyboard.Key.f11 in keys_down:
            keyboard_capturing = False
        elif keyboard.Key.f12 in keys_down:
            keyboard_capturing = True

        if keyboard_capturing:
            # if keyboard.Key.esc in keys_down:
            #     # quit
            #     raise PeacefulExit()
            
            if "w" in keys_down and "a" in keys_down:
                base_motion_states["base_motion"] = "forward_left"
            elif "w" in keys_down and "d" in keys_down:
                base_motion_states["base_motion"] = "forward_right"
            elif "s" in keys_down and "a" in keys_down:
                base_motion_states["base_motion"] = "reverse_left"
            elif "s" in keys_down and "d" in keys_down:
                base_motion_states["base_motion"] = "reverse_right"
            elif "w" in keys_down:
                base_motion_states["base_motion"] = "forward"
            elif "s" in keys_down:
                base_motion_states["base_motion"] = "reverse"
            elif "a" in keys_down:
                base_motion_states["base_motion"] = "left"
            elif "d" in keys_down:
                base_motion_states["base_motion"] = "right"
            else:
                base_motion_states["base_motion"] = "still"

            # if "1" in keys_down:
            #     self.camera_view = 0
            # elif "2" in keys_down:
            #     self.camera_view = 1
            # elif "3" in keys_down:
            #     self.camera_view = 2
            # elif "4" in keys_down:
            #     self.camera_view = 3
            # elif "5" in keys_down:
            #     self.camera_view = -1
            
            # if "h" in keys_down:
            #     video_display = "hazmat"
            #     raise ChangeCamerasException
            # elif "m" in keys_down:
            #     video_display = "motion"
            #     raise ChangeCamerasException
            # elif "q" in keys_down:
            #     video_display = "qr"
            #     raise ChangeCamerasException
            # elif "r" in keys_down:
            #     video_display = "raw"
            #     raise ChangeCamerasException

            # if "c" in keys_down:
            #     qr_codes_strings = []
            #     hazmat_strings = []

            current_keys = String()
            current_keys.data = str(keys_down)

            self.keyboard_input_publisher.publish(current_keys)

        bms_msg = String()
        bms_msg.data = json.dumps(base_motion_states)
        self.bms_publisher.publish(bms_msg)


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