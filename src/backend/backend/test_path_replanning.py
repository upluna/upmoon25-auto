import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose, Quaternion

class TestPathReplanning(Node):
    def __init__(self):
        super().__init__('test_path_replanning')

        # This is special for Nav2
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        self.PUB_path = self.create_publisher(Pose, 'path/goal', self.QOS)

        while True:
            self.sendGoal()


    def sendGoal(self):

        goal_msg = Pose()
        self.get_logger().info('Input end coordinates (x y): ')
        end_input = input().strip().split(',')
        x = float(end_input[0])
        y = float(end_input[1])

        goal_msg.position.x = x
        goal_msg.position.y = y
        goal_msg.position.z = 0.0

        goal_msg.orientation = Quaternion()
        goal_msg.orientation.x = 0.9
        goal_msg.orientation.y = 0.0
        goal_msg.orientation.z = 0.0
        goal_msg.orientation.w = 1.0

        self.PUB_path.publish(goal_msg)
        self.get_logger().info('Goal sent')
            

    
def main():
    rclpy.init()

    node = TestPathReplanning()
    node.get_logger().info('Node initialized')

    rclpy.spin(node)

    rclpy.shutdown()
    #node.future = node.cli.call_async(node.req)
    #rclpy.spin_until_future_complete(node, node.future)
    #if node.future.result() is not None:
    #    node.get_logger().info(f'Path found: {node.future.result()}')
    #else:
    #    node.get_logger().error('Service call failed')
    #rclpy.shutdown()

if __name__ == '__main__':
    main()