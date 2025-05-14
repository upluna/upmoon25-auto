from interfaces.srv import FindTag
import rclpy
from rclpy.node import Node

TIMEOUT=10

class TagClientAsync(Node):

    def __init__(self):
        super().__init__('tag_client_async')
        self.cli = self.create_client(FindTag, 'find_tag')
        while not self.cli.wait_for_service(timeout_sec=TIMEOUT):
            self.get_logger().info('service not avail....')
        self.req = FindTag.Request()

    def sendRequest(self):
        self.req.timeout_seconds = TIMEOUT
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
def main():
    rclpy.init()

    tca = TagClientAsync()
    response = tca.sendRequest()

    tca.get_logger().info("Found response!")
    tca.get_logger().info(str(response.tag_pose))

    tca.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()