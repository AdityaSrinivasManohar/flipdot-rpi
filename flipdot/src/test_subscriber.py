import rclpy
from rclpy.node import Node

from flipdot.src.utils import FlipDotFrame, FlipDotRosConverter


class FlipDotSubscriber(Node):
    def __init__(self):
        super().__init__('flipdot_subscriber')
        
        self.subscription = self.create_subscription(
            FlipDotFrame,
            '/flipdot/command',
            self.listener_callback,
            10
        )
        self.get_logger().info('Flip-dot subscriber started. Waiting for data...')

    def listener_callback(self, msg):
        grid = FlipDotRosConverter.from_msg(msg)

        self.get_logger().info(f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        self.get_logger().info(f'Received {msg.width}x{msg.height} frame')
        
        print("\n" + "=" * (msg.width + 2))
        for row in grid:
            line = "".join(['●' if dot else '○' for dot in row])
            print(f"|{line}|")
        print("=" * (msg.width + 2))

def main(args=None):
    rclpy.init(args=args)
    node = FlipDotSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()