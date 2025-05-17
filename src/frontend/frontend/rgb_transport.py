import rclpy
from rclpy.node import Node, QoSProfile
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class RGBTransport(Node):
    def __init__(self):
        super().__init__('rgb_transport')

        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        # Subscribe to compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/rgb/image_compressed',
            self.compressed_callback,
            self.QOS
        )

        # Publisher for raw image
        self.publisher = self.create_publisher(
            Image,
            '/camera/rgb/image_decompressed',
            self.QOS
        )

        self.bridge = CvBridge()
        self.get_logger().info("Initialized")

    def compressed_callback(self, msg: CompressedImage):
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if image_np is None:
                self.get_logger().warn("Decoded image is None")
                return

            # Convert OpenCV image to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
            image_msg.header = msg.header  # Preserve timestamp/frame_id

            # Publish the raw image
            self.publisher.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RGBTransport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
