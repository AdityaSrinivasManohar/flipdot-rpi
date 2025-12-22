import rclpy
from rclpy.node import Node
import numpy as np
from flipdot.src.utils import FlipDotRosConverter, FlipDotFrame

class FlipDotTester(Node):
    def __init__(self):
        super().__init__('flipdot_tester')
        
        # Configure Publisher
        self.publisher_ = self.create_publisher(
            FlipDotFrame, 
            'flipdot_command', 
            10
        )
        
        # Publish at 1 Hz
        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Display Dimensions
        self.width = 28
        self.height = 14
        self.frame_count = 0
        
        self.get_logger().info(f'Flip-dot tester started. Grid: {self.width}x{self.height}')

    def timer_callback(self):
        y, x = np.indices((self.height, self.width))
        grid = ((x + y + self.frame_count) % 2).astype(np.uint8)
        
        msg = FlipDotRosConverter.to_msg(grid)
        
        self.publisher_.publish(msg)
        
        # Increment frame for animation
        self.frame_count += 1
        
        # Log every 20 frames to keep the terminal clean
        if self.frame_count % 20 == 0:
            self.get_logger().info(f'Published frame {self.frame_count}')

def main(args=None):
    rclpy.init(args=args)
    node = FlipDotTester()
    
    try:
        # Keep the node alive and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down Flip-dot tester...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()