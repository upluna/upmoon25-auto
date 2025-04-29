import rclpy
import numpy as np
import quaternion
import tf2_geometry_msgs

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.action import ActionServer
from interfaces.srv import FindPath
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        # This is special for Nav2
        self.NAV_QOS = QoSProfile(
            depth=1,
            reliability=1, # Reliable
            history=1,     # Keep last
            durability=1   # Transient local
        )

        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        self.SUB_costmap = self.create_subscription(OccupancyGrid, 'map/global_costmap', self.onCostmap, self.NAV_QOS)

        # Where path destinations are published
        self.SUB_goal = self.create_subscription(Pose, 'path/goal', self.onGoal, self.QOS)
        self.SUB_odom = self.create_subscription(Odometry, 'odom', self.onOdom, self.QOS)

        # Where our current path is published
        self.PUB_path = self.create_publisher(Path, 'path/path', self.QOS)

        self.service_ready = False
        self.serving = False
        self.odom = None
        self.goal = None
        self.curr_path = None

        # For the transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Path planner initialized')

    # Gets the robots position in the map frame
    def getRobotMapPose(self):
        if (self.odom == None):
            return None

        # Get the transform from odom to map
        transform = None
        while (transform == None):
            transform = self.lookupTransform('map', 'odom')

        # Transform the odom pose to the map frame
        robot_pose = Pose()
        robot_pose.position.x = self.odom.pose.position.x
        robot_pose.position.y = self.odom.pose.position.y
        robot_pose.position.z = self.odom.pose.position.z

        transformed_pose = tf2_geometry_msgs.do_transform_pose(robot_pose, transform) 
        return transformed_pose


    def lookupTransform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except TransformException as e:
            self.get_logger().error(f'Error looking up transform: {e}')
            return None

    def onOdom(self, msg):
        self.odom = msg.pose

    def onGoal(self, msg):
        self.get_logger().info('Received goal')
        self.serving = True
        self.goal = msg

        self.curr_path = None
        self.updateCurrPath()

    def getNeighbors(self, x, y):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if (dx == 0 and dy == 0):
                    continue
                if (x + dx < 0 or x + dx >= self.costmap_x or y + dy < 0 or y + dy >= self.costmap_y):
                    continue
                neighbors.append((x + dx, y + dy))
        return neighbors

    def worldToMap(self, point):
        x = int(point[0] / self.costmap_res) + self.costmap_x // 2
        y = int(point[1] / self.costmap_res) + self.costmap_x // 2
        return (x, y)
    
    def mapToWorld(self, point):
        x = (point[0] - self.costmap_x // 2) * self.costmap_res
        y = (point[1] - self.costmap_y // 2) * self.costmap_res
        return (x, y)
    

    # Resulting path is in world coordinates
    def aStar(self, start, end):
        
        def distToNearestObstacle(point):
            obstacle_indices = np.array(np.where(self.costmap == 100)) 
            obstacle_points = obstacle_indices - np.array([[point[0]], [point[1]]]) 
            distances = np.sqrt(obstacle_points[0]**2 + obstacle_points[1]**2)

            return np.min(distances)  
        
        def heuristic(point):
            min_dist = distToNearestObstacle(point)
            if (min_dist < 0.5):
                min_dist = float('inf')
            if (min_dist > self.costmap_x):
                min_dist = 0
            
            # Score increases exponentially as distance to nearest obstacle increases
            min_dist = (self.costmap_x) / min_dist
            return ((point[0] - end[0])**2 + (point[1] - end[1])**2)**0.5 + min_dist

        start = self.worldToMap(start)
        end   = self.worldToMap(end)

        open_set = {start}
        came_from = {}

        g_score = { start: 0 }
        f_score = { start : heuristic(start) }

        stamp = self.get_clock().now().to_msg()

        def getGScore(point):
            if point in g_score:
                return g_score[point]
            else:
                return float('inf')
            
        def getFScore(point):
            if point in f_score:
                return f_score[point]
            else:
                return float('inf')   
            

        def getPoseFromPoint(point):
            pose = PoseStamped()
            pose.pose = Pose()
            pose.header.frame_id = 'map'
            pose.header.stamp = stamp

            point_to_world = self.mapToWorld(point)
            pose.pose.position.x = point_to_world[0]
            pose.pose.position.y = point_to_world[1]
            pose.pose.position.z = 0.0

            # TODO pose orientation
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            return pose
            
        while (len(open_set) > 0):
            current = min(open_set, key=lambda point: getFScore(point))
            if (current == end):
                path = []
                # Reconstruct the path
                while (current in came_from):
                    path.append(getPoseFromPoint(current))
                    current = came_from[current]
                return path[::-1]

            open_set.remove(current)

            for neighbor in self.getNeighbors(current[0], current[1]):
                # >= 99 is either the exact obstacle position, or the area around the obstacle
                # that the center of the robot cannot enter
                if (self.costmap[neighbor] >= 99):
                    continue

                tentative_g_score = g_score[current] + self.costmap[neighbor]
                if (tentative_g_score < getGScore(neighbor)):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor)

                    if neighbor not in open_set:
                        open_set.add(neighbor)

        # No path found
        return []
    
    # Fills in the orientations for each of the poses in the path
    # Also removes intermediate poses that are too close together
    def simplifyPath(self, path, final_orientation):
        if (len(path) == 0):
            return path
        
        last_pose = path[len(path) - 1]
        simplified_path = [path[0]]
        prev_pose = simplified_path[0]
        prev_slope = 0.0

        # First pass: check if consecutive poses are colinear
        first = True
        for pose in path:
            if (first):
                first = False
                continue

            # Check if consecutive poses are colinear
            x_dist = pose.pose.position.x - prev_pose.pose.position.x
            if (x_dist == 0):
                prev_pose = pose
                prev_slope = 0.0
                continue
            slope = (pose.pose.position.y - prev_pose.pose.position.y) / x_dist
            if (slope == prev_slope):
                prev_pose = pose
                continue

            simplified_path.append(pose)
            prev_slope = slope
            prev_pose = pose

        # Setup for second pass
        path = simplified_path
        simplified_path = [path[0]]
        
        # Second pass: check if consecutive poses are too close together
        prev_pose = simplified_path[0]
        first = True
        for pose in path:
            if (first):
                first = False
                continue

            # Check if consecutive poses are too close together
            x_dist = pose.pose.position.x - prev_pose.pose.position.x
            y_dist = pose.pose.position.y - prev_pose.pose.position.y
            dist = (x_dist**2 + y_dist**2)**0.5

            # TODO: Make configurable
            if (dist < 0.24):
                prev_pose = pose
                continue

            simplified_path.append(pose)
            prev_pose = pose

        # Add the last pose
        if (simplified_path[len(simplified_path) - 1] != last_pose):
            simplified_path.append(last_pose)
        
        # Final pass: set orientations
        for i in range(len(simplified_path) - 1):
            p1 = simplified_path[i]
            p2 = simplified_path[i + 1]

            x_dist = p2.pose.position.x - p1.pose.position.x
            y_dist = p2.pose.position.y - p1.pose.position.y
            yaw = np.arctan2(y_dist, x_dist)

            quat = quaternion.from_euler_angles(0.0, 0.0, yaw)
            simplified_path[i].pose.orientation.x = quat.x
            simplified_path[i].pose.orientation.y = quat.y
            simplified_path[i].pose.orientation.z = quat.z
            simplified_path[i].pose.orientation.w = quat.w

        # Final thing: set orientation of last pose
        simplified_path[len(simplified_path) - 1].pose.orientation = final_orientation

        self.get_logger().info(f'Original path length: {len(path)}')
        self.get_logger().info(f'Simplified path length: {len(simplified_path)}')
        return simplified_path

    def serveFindPath(self, request, response):
        self.get_logger().info('Received find path request')

        path = []
        try:
            path = self.aStar(
                (request.start.position.x, request.start.position.y),
                (request.end.position.x, request.end.position.y),
                )
            path = self.simplifyPath(path, request.end.orientation)
        except Exception as e:
            self.get_logger().error(f'Error finding path: {e}')        
            response.success = False
            response.path = Path()
            return response

        if (path == []):
            self.get_logger().error('No path found')
            response.success = False
            response.path = Path()
            return response
        
        response.success = True
        response.path = Path()
        response.path.header.frame_id = 'map'
        response.path.header.stamp = self.get_clock().now().to_msg()
        response.path.poses = path

        return response
    
    def publishPath(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = path

        self.PUB_path.publish(path_msg)

    # TODO costmap update function?
    def onCostmap(self, msg):
        # Store costmap data for future processing
        self.costmap_x = msg.info.width
        self.costmap_y = msg.info.height
        self.costmap_res = float(msg.info.resolution)
        self.costmap = np.frombuffer(msg.data, dtype=np.int8).reshape((self.costmap_x, self.costmap_y), order='C')

        if (not self.service_ready):
            self.service_ready = True
            self.srv = self.create_service(FindPath, 'find_path', self.serveFindPath)

            self.get_logger().info('Service ready')

        if (not self.serving):
            return
        
        # If we are serving a path, we need to check that our path doesn't run through an obstacle
        self.updateCurrPath()

    # Checks that our current path doesn't run through an obstacle
    # Also checks that the end pose of the path is not an obstacle
    def updateCurrPath(self):
        if (self.goal == None):
            return
        
        if (self.curr_path == None):
            # TODO This is repeated code which should be refactored
            robo_pose = self.getRobotMapPose()
            # We keep current path unsimplified because it makes checking if it runs through an obstacle easier
            self.curr_path = self.aStar((robo_pose.position.x, robo_pose.position.y), (self.goal.position.x, self.goal.position.y))

            if (self.curr_path == []):
                self.get_logger().error('No path found')
                return
            
            self.publishPath(self.simplifyPath(self.curr_path, self.goal.orientation))
            
            return
        
        # Check if the end pose of the path is an obstacle
        end_pose = self.curr_path[len(self.curr_path) - 1]
        end_pose = self.worldToMap((end_pose.pose.position.x, end_pose.pose.position.y))
        if (self.costmap[end_pose] >= 50):
            self.get_logger().error('End pose is an obstacle')
            return
        
        # Check that the path doesn't run through an obstacle
        for pose in self.curr_path:
            pose = self.worldToMap((pose.pose.position.x, pose.pose.position.y))

            if (self.costmap[pose] >= 50):
                self.get_logger().error('Path runs through an obstacle. Replanning...')
                robo_pose = self.getRobotMapPose()
                self.curr_path = self.aStar((robo_pose.position.x, robo_pose.position.y), (self.goal.position.x, self.goal.position.y))

                if (self.curr_path == []):
                    self.get_logger().error('No path found')
                    return
                self.get_logger().info('Replan succesful')

                self.publishPath(self.simplifyPath(self.curr_path, self.goal.orientation))
                return



        



        
def main():
    rclpy.init()
    node = PathPlanner()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()