import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from message_filters import Subscriber, TimeSynchronizer

import numpy as np

from pynput import keyboard

import cv2

import json

from cv_bridge import CvBridge

base_motion_states = {"base_motion":"still", "speed":0.5}
'''
base_motion states:
    still
    forward
    reverse
    turn_left
    turn_right
    forward_left
    forward_right
    reverse_left
    reverse_right
speed is a multiplier (just a percentage)
'''

video_display = "raw"

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

class PeacefulExit(Exception):
    def __str__(self):
        return "Closed node successfully"

class ChangeCamerasException(Exception):
    def __str__(self):
        return "Changing the camera subscription now"

class MainInterfaceNode(Node):
    def __init__(self, file0, file1, file2, file3):
        super().__init__("main_interface")
        self.declare_parameter('use_webcam', False)
        self.declare_parameter('camera_0', "")
        self.declare_parameter('camera_1', "")
        self.declare_parameter('camera_2', "")
        self.declare_parameter('camera_3', "")

        self.file0 = file0
        self.file1 = file1
        self.file2 = file2
        self.file3 = file3

        self.count0 = self.count1 = self.count2 = self.count3 = 0
        
        use_webcam = self.get_parameter('use_webcam').get_parameter_value().bool_value
        if not use_webcam:
            self.camera_0 = self.get_parameter('camera_0').get_parameter_value().string_value
            self.camera_1 = self.get_parameter('camera_1').get_parameter_value().string_value
            self.camera_2 = self.get_parameter('camera_2').get_parameter_value().string_value
            self.camera_3 = self.get_parameter('camera_3').get_parameter_value().string_value
        else:
            self.camera_0 = self.camera_1 = self.camera_2 = self.camera_3 = 0

        self.bridge = CvBridge()

        # bms_publisher -> base motion states publisher
        self.bms_publisher = self.create_publisher(String, '/motor_states/drive', 10)

        self.camera_view = -1

        global video_display

        self.ts = None

        self.camera_0_subscription = self.create_subscription(Image, "/cameras/raw/camera_0", self.listener_callback0, 10)
        self.camera_1_subscription = self.create_subscription(Image, "/cameras/raw/camera_0", self.listener_callback1, 10)
        self.camera_2_subscription = self.create_subscription(Image, "/cameras/raw/camera_0", self.listener_callback2, 10)
        self.camera_3_subscription = self.create_subscription(Image, "/cameras/raw/camera_0", self.listener_callback3, 10)

        # self.setup_subscription()
        '''
        tells which video frames to display.  can be:
            raw
            hazmat
            motion
            qr
        '''

    

    def listener_callback0(self, msg_0):
        global video_display
        global keys_down

        print(f"time: {self.get_clock().now()} frame: {self.count0}", file=self.file0)
        # Get frames and display them
        frame_0 = self.bridge.imgmsg_to_cv2(msg_0, "bgr8")
        

        window_height = 1000

        
        camera_display_height = int(window_height/2)
        (h, w) = frame_0.shape[:2]
        new_w_0 = int((w/h)*camera_display_height)

        frame_0 = cv2.resize(frame_0, (new_w_0, camera_display_height))



        cv2.imshow("Camera Output0", frame_0)
        cv2.waitKey(1)
        # Handle keyboard input
        self.count0 += 1
        
        
        # self.get_logger().info(f'I heard: {four_frames.dtype}\n\n\n\n\n')

    def listener_callback1(self, msg_0):
        global video_display
        global keys_down
        print(f"time: {self.get_clock().now()} frame: {self.count1}", file=self.file1)
        # Get frames and display them
        frame_0 = self.bridge.imgmsg_to_cv2(msg_0, "bgr8")
        

        window_height = 1000

        
        camera_display_height = int(window_height/2)
        (h, w) = frame_0.shape[:2]
        new_w_0 = int((w/h)*camera_display_height)

        frame_0 = cv2.resize(frame_0, (new_w_0, camera_display_height))



        cv2.imshow("Camera Output1", frame_0)
        cv2.waitKey(1)
        # Handle keyboard input
        self.count1 += 1
        
        
        # self.get_logger().info(f'I heard: {four_frames.dtype}\n\n\n\n\n')
    
    def listener_callback2(self, msg_0):
        global video_display
        global keys_down
        print(f"time: {self.get_clock().now()} frame: {self.count2}", file=self.file2)
        # Get frames and display them
        frame_0 = self.bridge.imgmsg_to_cv2(msg_0, "bgr8")
        

        window_height = 1000

        
        camera_display_height = int(window_height/2)
        (h, w) = frame_0.shape[:2]
        new_w_0 = int((w/h)*camera_display_height)

        frame_0 = cv2.resize(frame_0, (new_w_0, camera_display_height))



        cv2.imshow("Camera Output2", frame_0)
        cv2.waitKey(1)
        # Handle keyboard input
        self.count2 += 1
        
        
        # self.get_logger().info(f'I heard: {four_frames.dtype}\n\n\n\n\n')

    def listener_callback3(self, msg_0):
        global video_display
        global keys_down
        print(f"time: {self.get_clock().now()} frame: {self.count3}", file=self.file3)
        # Get frames and display them
        frame_0 = self.bridge.imgmsg_to_cv2(msg_0, "bgr8")
        

        window_height = 1000

        
        camera_display_height = int(window_height/2)
        (h, w) = frame_0.shape[:2]
        new_w_0 = int((w/h)*camera_display_height)

        frame_0 = cv2.resize(frame_0, (new_w_0, camera_display_height))



        cv2.imshow("Camera Output3", frame_0)
        cv2.waitKey(1)
        # Handle keyboard input
        self.count3 += 1
        
        
        # self.get_logger().info(f'I heard: {four_frames.dtype}\n\n\n\n\n')

def main(args=None):
    global video_display
    rclpy.init(args=args)
    with open("cam0reclog.txt", "w") as f0:
        with open("cam1reclog.txt", "w") as f1:
            with open("cam2reclog.txt", "w") as f2:
                with open("cam3reclog.txt", "w") as f3:
                    main_interface_node = MainInterfaceNode(f0, f1, f2, f3)

                    rclpy.spin(main_interface_node)
                        
                    main_interface_node.destroy_node()
                    rclpy.shutdown()
    
    print("Exited program successfully.  Have a nice day!")


if __name__ == '__main__':
    main()