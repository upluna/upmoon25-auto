from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Header
from rclpy.qos import QoSProfile

import numpy as np
import quaternion
import rclpy

# TODO: Make a shared paramater file for all nodes to lookup
# TODO: Just have this be a separate node? This isn't even really multithreaded
ROBOT_RAD = 0.24
ROBOT_BUFFER = 1.66

class GlobalCostmapper(Node):

    # 'costmap' color schemes:
    # 1-98: blue to red
    # 99: cyan
    # 100: purple
    def __init__(self):
        super().__init__('global_costmapper')

        # This is special for Nav2
        self.NAV_QOS = QoSProfile(
            depth=1,
            reliability=1, # Reliable
            history=1,     # Keep last
            durability=1   # Transient local
        )

        self.costmap = None
        self.map_x = None
        self.map_y = None
        self.map_res = None
        self.map_initialized = False
        self.PUB_costmap = self.create_publisher(OccupancyGrid, 'map/global_costmap', self.NAV_QOS)
        self.SUB_obstmap = self.create_subscription(OccupancyGrid, 'map/global', self.onMapInit, self.NAV_QOS)
        self.SUB_obstmap_update = self.create_subscription(OccupancyGridUpdate, 'map/global_updates', self.onMapUpdate, self.NAV_QOS)

    def onMapUpdate(self, msg):

        if (not self.map_initialized):
            return
        
        data = np.frombuffer(msg.data, dtype=np.int8)
        if (data.size == 0):
            self.get_logger().info('No data in update')
            return
            
        self.updateCostmap(
            data.reshape((msg.width, msg.height), order='C'),
            x_min = msg.x,
            x_max = msg.x + msg.width,
            y_min = msg.y,
            y_max = msg.y + msg.height,
            stamp = msg.header.stamp
        )

    def onMapInit(self, msg):
        self.get_logger().info('Received map')
        # Get the map data
        self.map_x = msg.info.width
        self.map_y = msg.info.height
        self.map_res = 1 / float(msg.info.resolution)
        self.costmap = np.zeros((self.map_x, self.map_y), dtype=np.int8)
        self.map_initialized = True

        
        self.updateCostmap(
            np.frombuffer(msg.data, dtype=np.int8).reshape((self.map_x, self.map_y), order='C'),
            x_min = 0,
            x_max = self.map_x,
            y_min = 0,
            y_max = self.map_y,
            stamp = msg.header.stamp
            )

    def updateCostmap(self, obst_map, x_min, x_max, y_min, y_max, stamp):
        points_of_interest = np.where(obst_map == 100.0) # Obstacles

        self.get_logger().info(f'Message dimensions: {x_min}, {x_max}, {y_min}, {y_max}')

        circle_rad = int(ROBOT_RAD * self.map_res)
        buffer_rad = int(circle_rad * ROBOT_BUFFER)
        for (x, y) in zip(points_of_interest[0], points_of_interest[1]):
            
            # Generate a circular area around the point
            for xi in range(-buffer_rad, buffer_rad + 1):
                for yi in range(-buffer_rad, buffer_rad + 1):
                    radial = xi**2 + yi**2
                    if (radial <= buffer_rad ** 2):
                        color = 50
                        # Check if the point is within the circle
                        if (radial <= circle_rad ** 2):
                            color = 99

                        x_index = int(xi + x + x_min)
                        y_index = int(yi + y + y_min)

                        # Check bounds
                        if (x_index >= 0 and x_index < self.map_x and
                            y_index >= 0 and y_index < self.map_y):

                            # Don't overwrite the cyan or purple spots
                            if self.costmap[x_index][y_index] > 50:
                                continue
                            self.costmap[x_index][y_index] = color
            
            self.costmap[x + x_min][y + y_min] = 100

        self.publishCostMap(stamp)

    def publishCostMap(self, stamp):

        msg = OccupancyGrid()
        info = MapMetaData()
        info.height = self.map_y
        info.width = self.map_x
        info.resolution = 1.0 / self.map_res
        
        info.origin.position.x = -(self.map_x) / (2.0 * self.map_res)
        info.origin.position.y = -(self.map_y) / (2.0 * self.map_res)
        info.origin.position.z = 0.0

        quat = quaternion.from_euler_angles(-np.pi / 2, np.pi, 0)

        info.origin.orientation.x = quat.x
        info.origin.orientation.y = quat.y
        info.origin.orientation.z = quat.z
        info.origin.orientation.w = quat.w

        data = self.costmap.flatten(order='C')
        
        msg.data = np.round(data).astype(np.int8).tolist()
        msg.info = info
        msg.header = Header(frame_id='map')
        msg.header.stamp = stamp

        self.PUB_costmap.publish(msg)    

def main(args=None):
    rclpy.init()
    node = GlobalCostmapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()