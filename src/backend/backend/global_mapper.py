import rclpy
import numpy as np
import quaternion
import tf2_ros

from rclpy.node import Node, QoSProfile
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Header, Int8, Float32, ColorRGBA
from visualization_msgs.msg import Marker
from scipy.interpolate import griddata

OBS_MAX_Z = -0.1
OBS_MIN_Z = -0.8
DEBUG = True
HEIGHTMAP = False

'''
    Generates an occupancy map of the arena based on incoming pointcloud data.

    Subscriptions:
    /camera/depth/points: PointCloud2
    /cmd_map:             Int8, 1 to turn map on, 0 to turn it off

    Publishes:
    /map/global:         OccupancyGrid
    /map/global_updates: OccupancyGridUpdates

    Params:
    use_sim_data: Set to true (default is true) if using simulator pointcloud

    TODO: CUDA
'''
class GlobalMapper(Node):

    def __init__(self):
        super().__init__('mapper')

        self.HZ = 0.66            # How often to update the map in seconds
        self.NEAR_CLIP = 2.5      # How far out the depth data should go
        self.TOP_CLIP = 5.0       # How far above should points get cut off (ignore ceil)
        self.BOTTOM_CLIP = -2.0   # How far below should points get cut off
        self.CAM_FOV = np.pi / 2  # These parameters are used to calculate the
        self.CAM_DIST = 4.0       # camera's view cone for interpolation

        self.gen_map = False
        self.first_gen = True # We don't publish the whole map everytime, just once
        self.update = True


        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )
        # This is special for Nav2
        self.NAV_QOS = QoSProfile(
            depth=1,
            reliability=1, # Reliable
            history=1,     # Keep last
            durability=1   # Transient local
        )


        # Sim parameter
        self.declare_parameter('use_sim_data', True)
        self.declare_parameter('map_on', True)

        self.use_sim_data = self.get_parameter('use_sim_data').value
        self.map_on = self.get_parameter('map_on').value

        # TF Buffer for looking up transforms
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize map and obstacle map
        self.grid_resolution = 25 # Pixels per meter
        self.grid_x  = 16 * self.grid_resolution
        self.grid_y = 16 * self.grid_resolution
        self.grid = np.zeros((self.grid_x, self.grid_y), dtype=np.float32)

        # Obstacle grid setup
        self.obstacle_map = np.zeros((self.grid_x, self.grid_y), dtype=np.float32)
        self.obstacle_map = self.obstacle_map - 1.0 # Unknown spots initialized to -1

        # Setup callback groups for sub group - we do this to prevent the transform buffer
        # from being blocked
        sub_group = MutuallyExclusiveCallbackGroup()

        # Setup subscribers and publishers
        self.SUB_depth  = self.create_subscription(PointCloud2, '/camera/depth/points', callback=self.handleDepth, qos_profile=self.QOS, callback_group=sub_group)
        self.SUB_cmd    = self.create_subscription(Int8, '/cmd_map', callback=self.handleCmd, qos_profile=self.QOS, callback_group=sub_group)
        self.SUB_marker = self.create_subscription(Point, '/cmd_map_marker', callback=self.handleMarkerCmd, qos_profile=self.QOS, callback_group=sub_group)
        self.SUB_roboz  = self.create_subscription(Float32, '/robo_z', callback=self.handleRoboZ, qos_profile=self.QOS, callback_group=sub_group)
        #self.SUB_mapcmd = self.create_subscription(MapCmd, '/cmd_map', callback=self.handleCmd, qos_profile=self.QOS, callback_group=sub_group)

        self.PUB_global = self.create_publisher(OccupancyGrid, '/map/global', qos_profile=self.NAV_QOS)
        self.PUB_update = self.create_publisher(OccupancyGridUpdate, '/map/global_updates', qos_profile=self.NAV_QOS)
        self.PUB_marker = self.create_publisher(Marker, '/map_marker', qos_profile=self.QOS)

        # Map update clock
        self.timer = self.create_timer(self.HZ, self.clk)

        # Robot's z position in the world relative to the tag
        self.robot_z = 0.0
        self.static_map_odom_tf = None

        self.get_logger().info("Initialized")

    def handleMarkerCmd(self, cmd):
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.type=0
        msg.id = 1
        msg.action = 0
        msg.ns = "fuck"

        msg.color = ColorRGBA()
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 1.0

        msg.pose = Pose()
        msg.pose.position.x = cmd.x
        msg.pose.position.y = cmd.y
        msg.pose.position.z = 1.5

        msg.pose.orientation = Quaternion()
        np_quat = quaternion.from_euler_angles(0, np.pi / 2, 0)
        msg.pose.orientation.x = np_quat.x
        msg.pose.orientation.y = np_quat.y
        msg.pose.orientation.z = np_quat.z
        msg.pose.orientation.w = np_quat.w

    def clk(self):
        self.update = True

    def handleRoboZ(self, msg):
        self.robot_z = msg.data

    # Returns the transform from the depth frame to the specified frame
    def getTransform(self, frame, stamp):
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                'depth_link_optical',
                frame,
                stamp,
                rclpy.duration.Duration(seconds=1.0)    
            )
        except Exception as e:
            self.get_logger().error(f'Unable to get transform from depth_optical to {frame}')
            self.get_logger().error(str(e))
            
        return transform
    
    def filterSnapPoints(self, points):
        self.grid.fill(0.0)

        # Filter points
                
        valid_mask =  (points[: , 2] < self.TOP_CLIP) & \
                      (points[: , 2] > self.BOTTOM_CLIP)
        points = points[valid_mask]

        # Convert to grid indices
        x =  points[: , 0]
        y =  points[: , 1]
        z =  points[: , 2]

        x_grid = np.round((x * self.grid_resolution) + self.grid_x / 2).astype(int)
        y_grid = np.round((y * self.grid_resolution) + self.grid_y / 2).astype(int)
    
        # Mask for valid grid positions
        valid_indices = (x_grid >= 0) & (x_grid < self.grid_x) & \
                        (y_grid >= 0) & (y_grid < self.grid_y)
        x_grid, y_grid, z = x_grid[valid_indices], y_grid[valid_indices], z[valid_indices]

        self.grid[x_grid, y_grid] = z

        # Return the region of the grid which was modified for interpolation
        return (x_grid.max(), x_grid.min(), y_grid.max(), y_grid.min())

    
    def interpolate(self, x_max, x_min, y_max, y_min):
        # We only interpolate the region of the graph in the provided section,
        # and only 0 values (since those are unknowns)
        grid = self.grid[x_min : x_max, y_min : y_max]

        known_coords = np.where(grid != 0.0)
        unknown_coords = np.where(grid == 0.0)
        known_grid = grid[grid != 0.0]

        interp = griddata(known_coords, known_grid, unknown_coords, method='nearest')
        
        # Return back into arr
        self.grid[unknown_coords[0] + x_min, unknown_coords[1] + y_min] = interp

    def detectObstacles(self):
        # We compare the obstacles relative to the robot position
        #grid_mask = (self.grid == 0.0)
        #self.grid[grid_mask] = 0.0

        if DEBUG:
            gridmax = self.grid.max()
            gridmin = self.grid.min()
            self.get_logger().info(f'MIN: {gridmin}, MAX: {gridmax}')            
            self.get_logger().info(f'ROBO Z: {self.robot_z}')

        if HEIGHTMAP:
            gridmax = self.grid.max()
            gridmin = self.grid.min()
            self.get_logger().info(f'MIN: {gridmin}, MAX: {gridmax}')
            range = abs(gridmax - gridmin)

            self.grid += abs(gridmin)
            self.grid *= (100.0 / range)
            
            return

        # Mark known spots
        self.obstacle_map[(self.grid != 0.0) & (self.obstacle_map != 100.0)] = 0.0

        obstacle_mask = (self.grid > OBS_MAX_Z)
        self.obstacle_map[obstacle_mask] = 100.0

        obstacle_mask = (self.grid < OBS_MIN_Z) & (self.grid != 0.0)
        self.obstacle_map[obstacle_mask] = 100.0

    def transformPoints(self, points, stamp):
        tf = self.getTransform('map', stamp)
        
        if (tf == None):
            return (points, False)
        
        valid_mask = None

        if self.use_sim_data:
            valid_mask = (points[:, 2] != 0) & (points[:, 2] != np.inf)
        else:
            valid_mask = (points[:, 2] != 0) & (points[:, 2] != np.inf) & (points[:, 2] <= 3)

        points = points[valid_mask]

        tf_rot = tf.transform.rotation
        tf_rot = np.quaternion(tf_rot.w, tf_rot.x, tf_rot.y, tf_rot.z)
        tf_rot = quaternion.as_rotation_matrix(tf_rot)

        tf_pos = tf.transform.translation

        def apply(pts):
            pts[:, 0] = pts[:, 0] - tf_pos.x
            pts[:, 1] = pts[:, 1] - tf_pos.y
            pts[:, 2] = pts[:, 2] - tf_pos.z
            return pts @ tf_rot

        points = apply(points)

        return (points, True)


    # Generates the global occupancy map based on the provided points
    def genMap(self, points, stamp):

        points, success = self.transformPoints(points, stamp)

        if (success == False):
            return

        x_max, x_min, y_max, y_min = self.filterSnapPoints(points)

        self.detectObstacles()
        
        
        #if (self.first_gen):
        #    self.first_gen = False
        #    self.publishObstacleMap(stamp)
        #else:
        #    self.publishObstacleMapUpdate(stamp, x_min, x_max, y_min, y_max)
        

        self.publishObstacleMap(stamp)

        self.update = False

    def handleCmd(self, msg):
        data = int(msg.data)
        if (data > 0):
            self.map_on = True
        else:
            self.map_on = False

    def handleDepth(self, msg):
        if (not self.update or not self.map_on):
            return
        
        points = np.frombuffer(msg.data, dtype=np.float32)
        if (not self.use_sim_data):
            points = points.reshape((msg.width, 3))
        else:
            points = points.reshape(-1, 8)
            points = points[:, :3]

        self.genMap(points, msg.header.stamp)

    def debugPublishPoint(self, x, y, z, frame, id=0):
        if (not hasattr(self, 'PUB_debug')):
            self.PUB_debug = self.create_publisher(Marker, '/debug/map', self.QOS)

        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        self.PUB_debug.publish(marker)

    def publishObstacleMapUpdate(self, stamp, x_min, x_max, y_min, y_max):
        msg = OccupancyGridUpdate()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp

        msg.x = int(x_min)
        msg.y = int(y_min)
        msg.width  = int(x_max - x_min)
        msg.height = int(y_max - y_min)

        data = None
        if HEIGHTMAP:
            data = self.grid[x_min : x_max, y_min : y_max].flatten(order='C')
        else:
            data = self.obstacle_map[x_min : x_max, y_min : y_max].flatten(order='C')

        msg.data = np.round(data).astype(np.int8).tolist()
        self.PUB_update.publish(msg)

    def publishObstacleMap(self, stamp):
        msg = OccupancyGrid()
        info = MapMetaData()
        info.height = self.grid_y
        info.width = self.grid_x
        info.resolution = 1.0 / self.grid_resolution
        
        info.origin.position.x = -(self.grid_x) / (2.0 * self.grid_resolution)
        info.origin.position.y = -(self.grid_y) / (2.0 * self.grid_resolution)
        info.origin.position.z = 0.0

        quat = quaternion.from_euler_angles(-np.pi / 2, np.pi, 0)

        info.origin.orientation.x = quat.x
        info.origin.orientation.y = quat.y
        info.origin.orientation.z = quat.z
        info.origin.orientation.w = quat.w

        data = None
        if HEIGHTMAP:
            data = self.grid.flatten(order='C')
        else:
            data = self.obstacle_map.flatten(order='C')
        
        msg.data = np.round(data).astype(np.int8).tolist()
        msg.info = info
        msg.header = Header(frame_id='map')
        msg.header.stamp = stamp

        self.PUB_global.publish(msg)        
        
def main(args=None):
    rclpy.init(args=args)

    mapper = GlobalMapper()

    executor = MultiThreadedExecutor()
    executor.add_node(mapper)

    executor.spin()

    mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()