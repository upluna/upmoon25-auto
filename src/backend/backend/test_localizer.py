import rclpy
from rclpy.node import Node, QoSProfile
from std_msgs.msg import Int8
import sys


class TestLocalizer(Node):

    def __init__(self):
        super().__init__('test_localizer')

        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        val = int(sys.argv[1])

        self.PUB_localizer = self.create_publisher(Int8, '/cmd_localizer', self.QOS)

        msg = Int8()
        msg.data = val

        self.PUB_localizer.publish(msg)

        self.get_logger().info('Done')

def main(args=None):
    rclpy.init()

    t = TestLocalizer()

    rclpy.spin_once(t)

    t.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()