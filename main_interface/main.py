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

qr_codes_strings = []

class PeacefulExit(Exception):
    def __str__(self):
        return "Closed node successfully"

class ChangeCamerasException(Exception):
    def __str__(self):
        return "Changing the camera subscription now"

class MainInterfaceNode(Node):
    def __init__(self):
        super().__init__("main_interface")
        self.declare_parameter('use_webcam', False)
        self.declare_parameter('camera_0', "")
        self.declare_parameter('camera_1', "")
        self.declare_parameter('camera_2', "")
        self.declare_parameter('camera_3', "")
        
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

        self.camera_0_subscriber = Subscriber(
            self,
            Image,
            f'/cameras/{video_display}/camera_{self.camera_0}')
        self.camera_1_subscriber = Subscriber(
            self,
            Image,
            f'/cameras/{video_display}/camera_{self.camera_1}')
        self.camera_2_subscriber = Subscriber(
            self,
            Image,
            f'/cameras/{video_display}/camera_{self.camera_2}')
        self.camera_3_subscriber = Subscriber(
            self,
            Image,
            f'/cameras/{video_display}/camera_{self.camera_3}')

        self.ts = TimeSynchronizer([self.camera_0_subscriber, self.camera_1_subscriber, self.camera_2_subscriber, self.camera_3_subscriber], 1)
        self.ts.registerCallback(self.listener_callback)
        '''
        tells which video frames to display.  can be:
            raw
            hazmat
            motion
            qr
        '''

        self.qr_camera_0_string_subscription = self.create_subscription(
            String,
            f'/qr/string/camera_{self.camera_0}',
            self.get_qr_strings,
            1)
        self.qr_camera_1_string_subscription = self.create_subscription(
            String,
            f'/qr/string/camera_{self.camera_1}',
            self.get_qr_strings,
            1)
        self.qr_camera_2_string_subscription = self.create_subscription(
            String,
            f'/qr/string/camera_{self.camera_2}',
            self.get_qr_strings,
            1)
        self.qr_camera_3_string_subscription = self.create_subscription(
            String,
            f'/qr/string/camera_{self.camera_3}',
            self.get_qr_strings,
            1)


    def listener_callback(self, msg_0, msg_1, msg_2, msg_3):
        global video_display
        global keys_down
        global keyboard_capturing

        # Get frames and display them
        frame_0 = self.bridge.imgmsg_to_cv2(msg_0, "bgr8")
        frame_1 = self.bridge.imgmsg_to_cv2(msg_1, "bgr8")
        frame_2 = self.bridge.imgmsg_to_cv2(msg_2, "bgr8")
        frame_3 = self.bridge.imgmsg_to_cv2(msg_3, "bgr8")

        window_height = 1000

        if self.camera_view == 0 or self.camera_view == 1 or self.camera_view == 2 or self.camera_view == 3:
            top_camera_width = 333
            top_camera_height = 250
            new_frame_0 = cv2.resize(frame_0, (top_camera_width, top_camera_height))
            new_frame_1 = cv2.resize(frame_1, (top_camera_width, top_camera_height))
            new_frame_2 = cv2.resize(frame_2, (top_camera_width, top_camera_height))
            new_frame_3 = cv2.resize(frame_3, (top_camera_width, top_camera_height))
            top_row_cameras = cv2.hconcat([new_frame_0, new_frame_1, new_frame_2, new_frame_3])

            vert_bar = np.full((750, 166, 3), 150, dtype=np.uint8)

            if self.camera_view == 0:
                new_frame_0 = cv2.resize(frame_0, (1000, 750))
                bottom_section = cv2.hconcat([vert_bar, new_frame_0, vert_bar])
                all_frames = cv2.vconcat([top_row_cameras, bottom_section])
            if self.camera_view == 1:
                new_frame_1 = cv2.resize(frame_1, (1000, 750))
                bottom_section = cv2.hconcat([vert_bar, new_frame_1, vert_bar])
                all_frames = cv2.vconcat([top_row_cameras, bottom_section])
            if self.camera_view == 2:
                new_frame_2 = cv2.resize(frame_2, (1000, 750))
                bottom_section = cv2.hconcat([vert_bar, new_frame_2, vert_bar])
                all_frames = cv2.vconcat([top_row_cameras, bottom_section])
            if self.camera_view == 3:
                new_frame_3 = cv2.resize(frame_3, (1000, 750))
                bottom_section = cv2.hconcat([vert_bar, new_frame_3, vert_bar])
                all_frames = cv2.vconcat([top_row_cameras, bottom_section])
        else:
            camera_display_height = int(window_height/2)
            (h, w) = frame_0.shape[:2]
            new_w_0 = int((w/h)*camera_display_height)
            (h, w) = frame_1.shape[:2]
            new_w_1 = int((w/h)*camera_display_height)
            (h, w) = frame_2.shape[:2]
            new_w_2 = int((w/h)*camera_display_height)
            (h, w) = frame_3.shape[:2]
            new_w_3 = int((w/h)*camera_display_height)

            frame_0 = cv2.resize(frame_0, (new_w_0, camera_display_height))
            frame_1 = cv2.resize(frame_1, (new_w_1, camera_display_height))
            frame_2 = cv2.resize(frame_2, (new_w_2, camera_display_height))
            frame_3 = cv2.resize(frame_3, (new_w_3, camera_display_height))
            frames_0_1 = cv2.hconcat([frame_0, frame_1])
            frames_2_3 = cv2.hconcat([frame_2, frame_3])
            all_frames = cv2.vconcat([frames_0_1, frames_2_3])
        # add black side panel
        black_background = np.zeros((window_height, 480, 3), dtype=np.uint8)
        all_frames = cv2.hconcat([all_frames, black_background])

        # cv2.putText(frame, text, position, font, scale, color, thickness)
        cv2.putText(all_frames, f'Motor Speed: {base_motion_states["speed"]*100}%', (1350, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(all_frames, f'Base Motion: {base_motion_states["base_motion"]}', (1350, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(all_frames, f'Video Display: {video_display}', (1350, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(all_frames, f'QR Labels:', (1350, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        qr_codes_string = ""
        for string in qr_codes_strings:
            qr_codes_string += string + "\n"
        
        print(f"QR Codes String: {qr_codes_string}")

        start_y = 250
        y_inc = 50
        for i, data in enumerate(qr_codes_string.split("\n")):
            cv2.putText(all_frames, data, (1350, start_y+i*y_inc), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        cv2.imshow("Camera Output", all_frames)

        # Handle keyboard input

        cv2.waitKey(1)

        if keyboard.Key.f11 in keys_down:
            keyboard_capturing = False
        elif keyboard.Key.f12 in keys_down:
            keyboard_capturing = True

        if keyboard_capturing:
            if keyboard.Key.esc in keys_down:
                # quit
                raise PeacefulExit()
            
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

            if "1" in keys_down:
                self.camera_view = 0
            elif "2" in keys_down:
                self.camera_view = 1
            elif "3" in keys_down:
                self.camera_view = 2
            elif "4" in keys_down:
                self.camera_view = 3
            elif "5" in keys_down:
                self.camera_view = -1
            
            if "h" in keys_down:
                video_display = "hazmat"
                raise ChangeCamerasException
            elif "m" in keys_down:
                video_display = "motion"
                raise ChangeCamerasException
            elif "q" in keys_down:
                video_display = "qr"
                raise ChangeCamerasException
            elif "r" in keys_down:
                video_display = "raw"
                raise ChangeCamerasException
        
        # bms_msg -> base motion states message
        bms_msg = String()
        bms_msg.data = json.dumps(base_motion_states)
        self.bms_publisher.publish(bms_msg)
        
        # self.get_logger().info(f'I heard: {four_frames.dtype}\n\n\n\n\n')

    def get_qr_strings(self, qr_strings):
        global qr_codes_strings
        qr_strings = eval(qr_strings.data)
        for string in qr_strings:
            if string not in qr_codes_strings:
                qr_codes_strings.append(string)

def main(args=None):
    global video_display
    rclpy.init(args=args)

    main_interface_node = MainInterfaceNode()

    end_run = False
    while not end_run:
        print("start while")
        try:
            print("in try")
            rclpy.spin(main_interface_node)
        except ChangeCamerasException as e:
            print(e)
            main_interface_node.destroy_node()
            main_interface_node = MainInterfaceNode()
            print("after destruction")
        except Exception as e:
            print(e)
            main_interface_node.destroy_node()
            rclpy.shutdown()
            end_run = True
        print("after exceptipns")
    
    print("Exited program successfully.  Have a nice day!")


if __name__ == '__main__':
    main()