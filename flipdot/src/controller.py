import sys

import numpy as np
import rclpy
import serial
from flippydot import Panel
from rclpy.node import Node

from flipdot.src.utils import FlipDotFrame, FlipDotRosConverter


class FlipDotController:
    def __init__(self, 
        width=28, 
        height=7, 
        port='/dev/serial0', 
        baud=57600, 
        layout=[[1], [2]], 
        module_rotation=180
    ):
        """Initialize serial and panel connection."""

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=1)
            print(f"SUCCESS: Connected to flipdot on {port}")
        except serial.SerialException as e:
            print(f"WARNING: Could not open {port}: {e}")
            print("Check connection to hardware. exiting")
            sys.exit(1)
        
        self.panel = Panel(layout, 
                width, 
                height, 
                module_rotation=module_rotation, 
                screen_preview=False
            )
        self.width = self.panel.get_total_width()
        self.height = self.panel.get_total_height()
        print(f"Hardware Initialized: {self.width}x{self.height} at {port}")

    def send_frame(self, frame):
        """Internal helper to send NumPy frame to panel."""
        serial_data = self.panel.apply_frame(frame)
        self.ser.write(serial_data)

    def close(self):
        """Close serial connection."""
        if self.ser.is_open:
            self.ser.close()

class FlipDotHardwareNode(Node):
    def __init__(self):
        super().__init__('flipdot_hardware_node')
        self.controller = FlipDotController(
            port='/dev/serial0',
            baud=57600,
            layout=[[1], [2]]
        )

        self.subscription = self.create_subscription(
            FlipDotFrame,
            '/flipdot/command',
            self.command_callback,
            10
        )
        
        self.get_logger().info('FlipDot Hardware Node is listening on /flipdot/command')

    def command_callback(self, msg):
        """Callback triggered when a new FlipDotFrame arrives."""
        try:
            grid = FlipDotRosConverter.from_msg(msg)
            
            # Send the grid directly to the physical hardware
            self.controller.send_frame(grid)
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')

    def destroy_node(self):
        """Clean shutdown: clear display and close serial port."""
        self.get_logger().info('Shutting down: Clearing display...')
        # Optional: Send a zeroed frame to 'turn off' all dots
        blank_frame = np.zeros((self.controller.height, self.controller.width), dtype=np.uint8)
        self.controller.send_frame(blank_frame)
        self.controller.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FlipDotHardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()