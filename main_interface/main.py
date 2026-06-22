import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
import json
from cv_bridge import CvBridge

# ---------------- GLOBALS ----------------
base_motion = "still"
speed = 50
qr_codes_strings = []
hazmat_strings = []
keys_down = []
video_display = "raw"

# -----------------------------------------

class PeacefulExit(Exception):
    pass

class ChangeCamerasException(Exception):
    pass


class MainInterfaceNode(Node):
    def __init__(self):
        super().__init__("main_interface")

        # Parameters
        self.declare_parameter('use_webcam', False)
        self.declare_parameter('camera_0', 0)
        self.declare_parameter('camera_1', 1)
        self.declare_parameter('camera_2', 2)
        self.declare_parameter('camera_3', 3)

        use_webcam = self.get_parameter('use_webcam').value
        if not use_webcam:
            self.camera_0 = self.get_parameter('camera_0').value
            self.camera_1 = self.get_parameter('camera_1').value
            self.camera_2 = self.get_parameter('camera_2').value
            self.camera_3 = self.get_parameter('camera_3').value
        else:
            self.camera_0 = self.camera_1 = self.camera_2 = self.camera_3 = 0

        self.bridge = CvBridge()
        self.camera_view = -1

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher
        self.main_interface_frame = self.create_publisher(Image, '/main_interface/driver_frame', qos)

        # Subscriptions
        self.create_subscription(CompressedImage, f'/cameras/{video_display}/camera_{self.camera_0}', self.camera_0_callback, qos)
        self.create_subscription(CompressedImage, f'/cameras/{video_display}/camera_{self.camera_1}', self.camera_1_callback, qos)
        self.create_subscription(CompressedImage, f'/cameras/{video_display}/camera_{self.camera_2}', self.camera_2_callback, qos)
        self.create_subscription(CompressedImage, f'/cameras/{video_display}/camera_{self.camera_3}', self.camera_3_callback, qos)

        # Default frames
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        blank = cv2.putText(blank, 'No Signal', (0,480), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
        self.camera_0_frame = blank
        self.camera_1_frame = blank
        self.camera_2_frame = blank
        self.camera_3_frame = blank

        # Timer (keep your 60 Hz request)
        self.timer = self.create_timer(1/60, self.timer_callback)

        # Other subscriptions
        self.create_subscription(String, f'/qr/string/camera_{self.camera_0}', self.get_qr_strings, 1)
        self.create_subscription(String, f'/qr/string/camera_{self.camera_1}', self.get_qr_strings, 1)
        self.create_subscription(String, f'/qr/string/camera_{self.camera_2}', self.get_qr_strings, 1)
        self.create_subscription(String, f'/qr/string/camera_{self.camera_3}', self.get_qr_strings, 1)

        self.create_subscription(String, f'/hazmat/string/camera_{self.camera_0}', self.get_hazmat_strings, 1)
        self.create_subscription(String, f'/hazmat/string/camera_{self.camera_1}', self.get_hazmat_strings, 1)
        self.create_subscription(String, f'/hazmat/string/camera_{self.camera_2}', self.get_hazmat_strings, 1)
        self.create_subscription(String, f'/hazmat/string/camera_{self.camera_3}', self.get_hazmat_strings, 1)

        self.create_subscription(String, '/keyboard', self.get_keyboard, 1)
        self.create_subscription(String, '/motor_states/drive', self.get_ms, 1)
        self.create_subscription(Int8, '/speed', self.get_speed, 1)

    # ---------------- CAMERA CALLBACKS ----------------
    def camera_0_callback(self, msg):
        self.camera_0_frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')

    def camera_1_callback(self, msg):
        self.camera_1_frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')

    def camera_2_callback(self, msg):
        self.camera_2_frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')

    def camera_3_callback(self, msg):
        self.camera_3_frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')

    # ---------------- TIMER CALLBACK ----------------
    def timer_callback(self):
        global video_display, qr_codes_strings, hazmat_strings, speed, base_motion

        # Get frames
        f0, f1, f2, f3 = self.camera_0_frame, self.camera_1_frame, self.camera_2_frame, self.camera_3_frame

        # Resize + layout (unchanged logic)
        window_height = 1000

        if self.camera_view in (0,1,2,3):
            small = (333, 250)
            nf0 = cv2.resize(f0, small)
            nf1 = cv2.resize(f1, small)
            nf2 = cv2.resize(f2, small)
            nf3 = cv2.resize(f3, small)
            top = cv2.hconcat([nf0, nf1, nf2, nf3])

            vert = np.full((750, 166, 3), 150, dtype=np.uint8)

            if self.camera_view == 0:
                big = cv2.resize(f0, (1000, 750))
            elif self.camera_view == 1:
                big = cv2.resize(f1, (1000, 750))
            elif self.camera_view == 2:
                big = cv2.resize(f2, (1000, 750))
            else:
                big = cv2.resize(f3, (1000, 750))

            bottom = cv2.hconcat([vert, big, vert])
            all_frames = cv2.vconcat([top, bottom])

        else:
            h = window_height // 2
            def resize_half(img):
                ih, iw = img.shape[:2]
                new_w = int((iw/ih)*h)
                return cv2.resize(img, (new_w, h))

            r0, r1, r2, r3 = map(resize_half, (f0,f1,f2,f3))
            all_frames = cv2.vconcat([cv2.hconcat([r0,r1]), cv2.hconcat([r2,r3])])

        # Side panel
        side = np.zeros((window_height, 480, 3), dtype=np.uint8)
        all_frames = cv2.hconcat([all_frames, side])

        # Draw text
        left_x = 1350
        cv2.putText(all_frames, f'Motor Speed: {speed}%', (left_x, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv2.putText(all_frames, f'Base Motion: {base_motion}', (left_x, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv2.putText(all_frames, f'Video Display: {video_display}', (left_x, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

        cv2.putText(all_frames, 'QR Labels:', (left_x, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        y = 250
        for s in qr_codes_strings:
            cv2.putText(all_frames, s, (left_x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
            y += 50

        cv2.putText(all_frames, 'Hazmat Labels:', (left_x, 550), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        y = 600
        for s in hazmat_strings:
            cv2.putText(all_frames, s, (left_x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
            y += 50

        # Publish
        msg = self.bridge.cv2_to_imgmsg(all_frames, "bgr8")
        self.main_interface_frame.publish(msg)

    # ---------------- OTHER CALLBACKS ----------------
    def get_qr_strings(self, msg):
        global qr_codes_strings
        for s in eval(msg.data):
            if s not in qr_codes_strings:
                qr_codes_strings.append(s)

    def get_hazmat_strings(self, msg):
        global hazmat_strings
        for s in eval(msg.data):
            if s not in hazmat_strings:
                hazmat_strings.append(s)

    def get_keyboard(self, msg):
        global keys_down, video_display, qr_codes_strings, hazmat_strings
        keys_down = eval(msg.data)

        if "KEY_ESC" in keys_down:
            raise PeacefulExit()

        if "KEY_1" in keys_down: self.camera_view = 0
        elif "KEY_2" in keys_down: self.camera_view = 1
        elif "KEY_3" in keys_down: self.camera_view = 2
        elif "KEY_4" in keys_down: self.camera_view = 3
        elif "KEY_5" in keys_down: self.camera_view = -1

        if "KEY_H" in keys_down:
            video_display = "hazmat"
            raise ChangeCamerasException
        elif "KEY_Q" in keys_down:
            video_display = "qr"
            raise ChangeCamerasException
        elif "KEY_R" in keys_down:
            video_display = "raw"
            raise ChangeCamerasException

        if "KEY_C" in keys_down:
            qr_codes_strings.clear()
            hazmat_strings.clear()

    def get_ms(self, msg):
        global base_motion
        base_motion = msg.data

    def get_speed(self, msg):
        global speed
        speed = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = MainInterfaceNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except PeacefulExit:
        pass
    except ChangeCamerasException:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
