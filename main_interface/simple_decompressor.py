import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge

class SimpleDecompressor(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher = self.create_publisher(Image, '/decompressed', qos)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/cameras/raw/camera_0',
            self.listener_callback,
            qos)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()



    def listener_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    simple_decompressor = SimpleDecompressor()

    rclpy.spin(simple_decompressor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_decompressor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()