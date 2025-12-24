import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from flipdot.src.utils import FlipDotFrame, FlipDotRosConverter


class FlipDotPublisher(Node):
    def __init__(self):
        super().__init__('flipdot_publisher')

        # Configure Publisher
        self.publisher = self.create_publisher(FlipDotFrame, '/flipdot/command', 10)

        # Publish at 1 Hz
        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Display Dimensions
        self.width = 28
        self.height = 14
        self.frame_count = 0

        # Header
        self.msg_header = Header()
        self.msg_header.frame_id = "flipdot_display"

        self.get_logger().info(f'Flip-dot publisher started. Grid: {self.width}x{self.height}')

    def timer_callback(self):
        y, x = np.indices((self.height, self.width))
        grid = ((x + y + self.frame_count) % 2).astype(np.uint8)

        msg = FlipDotRosConverter.to_msg(grid)

        # Update header timestamp
        self.msg_header.stamp = self.get_clock().now().to_msg()
        msg.header = self.msg_header

        self.publisher.publish(msg)

        # Increment frame for animation
        self.frame_count += 1

        # Log every 20 frames to keep the terminal clean
        if self.frame_count % 20 == 0:
            self.get_logger().info(f'Published frame {self.frame_count}')


def main(args=None):
    rclpy.init(args=args)
    node = FlipDotPublisher()

    try:
        # Keep the node alive and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down Flip-dot publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
