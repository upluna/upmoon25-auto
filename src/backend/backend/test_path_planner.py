import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from interfaces.srv import FindPath
from interfaces.srv import GoTo
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Path

class TestPathPlanner(Node):
    def __init__(self):
        super().__init__('test_path_planner')
        self.cli = self.create_client(GoTo, 'goto')
        while not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Service not available, waiting again...')

    

    def sendRequest(self):
        self.req = GoTo.Request()

        self.get_logger().info('Input end coordinates (x y): ')
        end_input = input().strip().split(',')
        x = float(end_input[0])
        y = float(end_input[1])

        self.req.goal_x = x
        self.req.goal_y = y
    
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
        

def main():
    rclpy.init()

    node = TestPathPlanner()
    node.get_logger().info('Node initialized')

    while True:
        response = node.sendRequest()

        node.get_logger().info("Found response!")
    #node.future = node.cli.call_async(node.req)
    #rclpy.spin_until_future_complete(node, node.future)
    #if node.future.result() is not None:
    #    node.get_logger().info(f'Path found: {node.future.result()}')
    #else:
    #    node.get_logger().error('Service call failed')
    #rclpy.shutdown()

if __name__ == '__main__':
    main()