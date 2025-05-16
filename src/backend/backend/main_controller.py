import rclpy
import os

from enum import Enum
from rclpy.node import Node, QoSProfile
from std_msgs.msg import Int8, Header
from interfaces.srv import FindTag


'''
    The main controller for the entire autonomous system.
'''

class Command():

    def __init__(self, name):
        self.name = name
        self.node = None # Reference to MainController node, filled out later

    # Called when the self.node link is filled
    def onLink(self):
        pass

    def execute(self, args):
        pass

class FindTagCommand(Command):

    def __init__(self):
        super().__init__(name='findtag')

        self.cli = None

    def onLink(self):
        self.cli = self.node.create_client(FindTag, 'find_tag')

    def execute(self, args):
        if not self.cli.wait_for_service(timeout_sec=0.01):
            self.node.get_logger().error("Service not available")
            return
        
        timeout_secs = 10
        if len(args) > 0:
            timeout_secs = int(args[0])

        request = FindTag.Request()
        request.timeout_seconds = timeout_secs

        response = self.cli.call(request)

        self.get_logger().info(f'Response: {response}')

class MapCommand(Command):

    def __init__(self):
        super().__init__(name='map')

        self.PUB_mapcmd = None

    def onLink(self):
        self.PUB_mapcmd = self.node.create_publisher(Int8, '/cmd_map', qos_profile=self.node.QOS)

    def execute(self, args):
        if len(args) < 1:
            self.node.get_logger().info("Command 'map' takes 1 argument (0 or 1)")

        val = int(args[0])

        msg = Int8()
        msg.data = val

        self.PUB_mapcmd.publish(msg)

class LocalizeCommand(Command):

    def __init__(self):
        super().__init__(name='localize')

        self.PUB_cmd = None

    def onLink(self):
        self.PUB_cmd = self.node.create_publisher(Int8, '/cmd_localizer', qos_profile = self.node.QOS)

    def execute(self, args):
        if len(args) < 1:
            self.node.get_logger().info("Command 'localize' takes 1 argument (0 or 1)")
            return

        val = int(args[0])

        msg = Int8()
        msg.data = val

        self.PUB_cmd.publish(msg)


class HelloWorldCommand(Command):

    def __init__(self):
        super().__init__(name='helloworld')

    def execute(self, args):
        self.node.get_logger().info(f'Hello world! {args}')


class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
 
        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )       

        self.commands = {}

        self.registerNewCommand(HelloWorldCommand())
        self.registerNewCommand(MapCommand())
        self.registerNewCommand(FindTagCommand())
        self.registerNewCommand(LocalizeCommand())

        self.get_logger().info("Initialized")

        while True:
            try:
                self.onCommand(input("> "))
            except KeyboardInterrupt:
                print('')
                continue
            except Exception as e:
                self.get_logger().error(f'{e}')

    def onCommand(self, str):
        tokens = str.split(' ')

        if len(tokens) == 0:
            return

        cmd = tokens[0]
        args = tokens[1 : ]

        if cmd == 'quit':
            self.destroy_node()
            rclpy.shutdown()
            exit()

        if cmd not in self.commands.keys():
            self.get_logger().info(f'Command \'{cmd}\' not found')
            return
        
        executor = self.commands[cmd]
        executor.execute(args)
        
    def registerNewCommand(self, cmd):
        cmd.node = self
        cmd.onLink()
        self.commands[cmd.name] = cmd
        
def main(args=None):
    rclpy.init(args=args)
    mc = MainController()

    rclpy.spin(mc)

    mc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()