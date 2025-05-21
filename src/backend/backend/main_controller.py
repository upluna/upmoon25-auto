import rclpy
import os

from enum import Enum
from rclpy.node import Node, QoSProfile
from std_msgs.msg import Int8, Header
from interfaces.srv import FindTag, GoTo
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Point


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

class PointCloudCommand(Command):

    def __init__(self):
        super().__init__(name='pc')

        self.PUB_pc = None

    def onLink(self):
        self.PUB_pc = self.node.create_publisher(Int8, '/cmd/pointcloud', 1)

    def execute(self, args):
        msg = Int8()
        msg.data = 1

        self.PUB_pc.publish(msg)

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

class Miner(Command):

    def __init__(self):
        super().__init__(name='miner')

        self.cmd_pub = None

    def onLink(self):
        qos = QoSProfile(
            depth=3,
            reliability=1,
            history=1,
            durability=2
        )
        self.cmd_pub = self.node.create_publisher(PoseStamped, '/cmd/miner', qos_profile=qos)

    def execute(self, args):
        if len(args) < 1:
            self.node.get_logger().error("Missing Arguments")
            return
        
        msg = PoseStamped()
        msg.header.frame_id = args[0]
        msg.pose.position = Point()
        
        if (args[0] == 'mark' or args[0] == 'recdump' or args[0] == 'recinit'):
            if len(args) < 2:
                self.node.get_logger().error(f'Missing distance arg')
                return
            
            dist = float(args[1])
            msg.pose.position.x = dist
        
        self.cmd_pub.publish(msg)

class GoToCommand(Command):

    def __init__(self):
        super().__init__(name='goto')

        self.cli = None
        self.PUB_abort = None

    def onLink(self):
        self.cli = self.node.create_client(GoTo, 'goto')

        qos = QoSProfile(
            depth=3,
            reliability=1, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        self.PUB_abort = self.node.create_publisher(Int8, '/cmd_mp', qos)

    def execute(self, args):
        if len(args) < 2:
            self.node.get_logger().error("Requires at least 2 arguments (x, y)")
            return
        
        if not self.cli.wait_for_service(timeout_sec=0.04):
            self.node.get_logger().error('Service not available')
            return

        if (args[1] == 'abort'):
            msg = Int8()
            msg.data = 1
            self.PUB_abort.publish(msg)

            self.get_logger().info("Aborting")
            return
        
        x = float(args[0])
        y = float(args[1])
        ang = 0.0

        if len(args) > 2:
            ang = float(args[2])

        req = GoTo.Request()

        req.goal_x = x
        req.goal_y = y
        req.goal_yaw = ang

        self.node.get_logger().info('Sent request')
        response = self.cli.call(req)
        self.node.get_logger().info(f'Response: {response}')


class MapCommand(Command):

    def __init__(self):
        super().__init__(name='map')

        self.PUB_mapcmd = None
        self.PUB_mapmarker = None

    def onLink(self):
        self.PUB_mapcmd = self.node.create_publisher(Int8, '/cmd_map', qos_profile=self.node.QOS)
        self.PUB_map_marker = self.node.create_publisher(Point, '/cmd_map_marker', qos_profile=self.node.QOS)


    def execute(self, args):
        if len(args) < 1:
            self.node.get_logger().info("Command 'map' takes 1 argument (0 or 1)")
            return

        if (args[0] == 'mark'):
            
            if len(args) < 3:
                self.node.get_logger().info("Command 'map mark' takes 2 arguments (x y)")
                return
            
            x = float(args[1])
            y = float(args[2])

            msg = Point()
            msg.x = x
            msg.y = y
            msg.z = 0.0

            self.PUB_map_marker.publish(msg)

            return

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
        self.registerNewCommand(GoToCommand())
        self.registerNewCommand(Miner())
        self.registerNewCommand(PointCloudCommand())


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