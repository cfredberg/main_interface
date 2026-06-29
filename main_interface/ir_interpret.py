import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image, CompressedImage

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge

from flirimageextractor import FlirImageExtractor

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

        self.publisher = self.create_publisher(CompressedImage, f'/cameras/ir/camera_{camera_id}', qos)
        self.subscription = self.create_subscription(
            CompressedImage,
            f'/cameras/raw/camera_{camera_id}',
            self.listener_callback,
            qos)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()

        self.flir = FlirImageExtractor()



    def listener_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.flir.process_image(frame)

        array = self.flir.get_thermal_np()
        
        print(f"Size: {array.shape()}")

        print(f"heat array: {array}")

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