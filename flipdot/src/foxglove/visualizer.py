import numpy as np
import rclpy
from flipdotframe_msg.msg import FlipDotFrame
from rclpy.node import Node
from sensor_msgs.msg import Image

from flipdot.src.utils import FlipDotRosConverter


class FlipDotVisualizer(Node):
    def __init__(self):
        super().__init__('flipdot_visualizer')
        
        # Subscribe to your custom bit-packed message
        self.subscription = self.create_subscription(
            FlipDotFrame,
            '/flipdot/command',
            self.listener_callback,
            10
        )
        
        self.publisher = self.create_publisher(Image, '/flipdot/image', 10)
        self.get_logger().info('Flip-dot Visualizer started.')

    def listener_callback(self, msg):
        # 1. Decode the bit-packed data into a 2D numpy array (0s and 1s)
        grid = FlipDotRosConverter.from_msg(msg)
        
        # 2. Convert to Grayscale (0 -> Black, 1 -> White)
        vis_grid = (grid * 255).astype(np.uint8)
        
        # 3. Create the Image message
        img_msg = Image()
        img_msg.header = msg.header 
        img_msg.height = msg.height
        img_msg.width = msg.width
        img_msg.encoding = "mono8" 
        img_msg.is_bigendian = False
        img_msg.step = msg.width
        img_msg.data = vis_grid.tobytes()
        
        self.publisher.publish(img_msg)
        
        t = msg.header.stamp
        self.get_logger().info(f'Visualizing frame from T: {t.sec}.{t.nanosec:09d}', once=True)

def main(args=None):
    rclpy.init(args=args)
    node = FlipDotVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()