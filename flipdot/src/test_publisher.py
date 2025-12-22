import rclpy
from rclpy.node import Node
import numpy as np
from flipdotframe_msg.msg import FlipDotFrame

class FlipDotTester(Node):
    def __init__(self):
        super().__init__('flipdot_tester')
        self.publisher_ = self.create_publisher(FlipDotFrame, 'flipdot_command', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.width = 28 # Change to your actual width
        self.height = 14 # Change to your actual height

    def timer_callback(self):
        msg = FlipDotFrame()
        msg.width = self.width
        msg.height = self.height
        
        # Create a checkerboard pattern (1s and 0s)
        # Using numpy makes bit-packing trivial
        grid = np.indices((self.height, self.width)).sum(axis=0) % 2
        
        # Pack bits: 8 pixels become 1 byte
        # bitorder='little' ensures Bit 0 is the first pixel
        packed_data = np.packbits(grid.flatten(), bitorder='little')
        
        msg.data = packed_data.tobytes()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing flip-dot frame')

def main(args=None):
    rclpy.init(args=args)
    node = FlipDotTester()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()