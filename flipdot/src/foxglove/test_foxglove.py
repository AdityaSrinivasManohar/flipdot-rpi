import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import foxglove
from foxglove.schemas import RawImage, Timestamp

class FoxgloveBridgeNode(Node):
    def __init__(self):
        super().__init__('foxglove_bridge_python')
        print("Initializing Foxglove Bridge Python Node")
        
        # 1. Initialize Foxglove Server
        self.fox_server = foxglove.start_server(host="0.0.0.0", port=8765)
        self.get_logger().info("Foxglove WebSocket server started on port 8765")

        # 2. Subscribe to ROS Image topic
        self.subscription = self.create_subscription(
            Image,
            '/flipdot/image',
            self.image_callback,
            10  # QoS depth
        )
        print("Foxglove Bridge Python Node initialized")


    def image_callback(self, msg):
        # 3. Map ROS Image data to Foxglove RawImage
        # Note: 'mono8' is standard for Flipdots. Adjust encoding if your ROS topic differs.
        foxglove.log(
            "/flipdot/image",
            RawImage(
                timestamp=Timestamp.now(),
                frame_id=msg.header.frame_id,
                width=msg.width,
                height=msg.height,
                encoding=msg.encoding,
                step=msg.step,
                data=bytes(msg.data)
            )
        )
        self.get_logger().debug(f"Forwarded frame: {msg.width}x{msg.height}")

    # def stop(self):
    #     self.fox_server.stop()

def main(args=None):
    print("Starting Foxglove Bridge Python Node")
    rclpy.init(args=args)
    print("ROS2 initialized")
    node = FoxgloveBridgeNode()
    print("node created")
    
    try:
        # Spin the ROS node to handle incoming messages
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    print("okay")
    main()