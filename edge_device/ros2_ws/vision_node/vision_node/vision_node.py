import rclpy
from rclpy.node import Node

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info("Vision node has been initialized.")
        # TODO: Add your subscriptions, publishers, timers, or other functionality here.
        # For example, to create a timer that logs every second:
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Vision node timer callback triggered.")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Allow clean exit on Ctrl+C
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
