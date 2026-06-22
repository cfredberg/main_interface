import rclpy
from rclpy.node import Node
from slam_toolbox.srv import Pause

class SlamController(Node):
    def __init__(self):
        super().__init__('slam_controller')
        # Create a service client matching the slam_toolbox service name
        self.client = self.create_client(Pause, '/slam_toolbox/toggle_new_measurements')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SLAM service not available, waiting...')

    def pause_mapping(self):
        req = Pause.Request()
        req.status = True  # True means pause / stop processing new measurements
        self.send_request(req, "Pausing")

    def resume_mapping(self):
        req = Pause.Request()
        req.status = False  # False means resume / process new measurements
        self.send_request(req, "Resuming")

    def send_request(self, req, action_name):
        future = self.client.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(f'{action_name} complete!'))

def main(args=None):
    rclpy.init(args=args)
    node = SlamController()
    
    # Example: Pause mapping immediately
    node.pause_mapping()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
