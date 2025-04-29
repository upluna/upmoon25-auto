import rclpy
from rclpy.node import Node, QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()

        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        self.sub = self.create_subscription(Image, '/camera/rgb/image_raw', self.listener_callback, self.QOS)

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('/home/max/test.png', cv_image)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node=ImageSaver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()