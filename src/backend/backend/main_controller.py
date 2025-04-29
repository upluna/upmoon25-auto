import rclpy
import os

from enum import Enum
from rclpy.node import Node

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        self.declare_parameter('fsm_file', '/home/max/Documents/robotics/sim/src/sim/resource/fsm.txt')
        self.fsm = None

        try:
            self.fsm = FSM(self.get_parameter('fsm_file').value)
        except Exception as e:
            self.get_logger().error(str(e))
            self.destroy_node()

        self.get_logger().info('Successful initialization')

# Finite State Machine for modeling autonomous robot behavior
class FSM():

    def __init__(self, file):

        # Dictionary of node names to nodes
        self.nodes = {}
        self.state = None

        self.parseFile(file)

    def transition(self, edge_name):
        self.state = self.state.transition(edge_name)
        return self.state

    def parseFile(self, file):

        node = False
        first_node = True
        with open(file=file, mode='r') as f:
            for line_num, line in enumerate(f):
                # Clean string
                line = line.rstrip()
                if (len(line) < 1):
                    continue

                # Check what section we're in
                if (line[0] == '$'):
                    if (line == '$node'):
                        node = True
                    elif (line == '$edge'):
                        node = False
                    else:
                        raise Exception(f'Fatal exception: Invalid line at {line_num} in {file}')
                    continue

                # Parse section $node
                if (node):
                    self.nodes[line] = FSMNode(line)

                    if (first_node):
                        self.state = self.nodes[line]
                        first_node = False

                    continue
                # Parse section $edge
                line_split = [s.strip() for s in line.split('->')]
                if (len(line_split) != 3):
                    raise Exception(f'Fatal exception: Invalid line at {line_num} in {file}')

                self.nodes[line_split[0]].addEdge(line_split[1], line_split[2])

class FSMNode():

    def __init__(self, name):
        self.name = name

        # Dictionary of edge name : node name
        self.edges = {}

    def addEdge(self, edge_name, node_name):
        self.edges[edge_name] = node_name

    def transition(self, edge_name):
        return self.edges[edge_name]

def main(args=None):
    rclpy.init(args=args)
    mc = MainController()

    rclpy.spin(mc)

    mc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()