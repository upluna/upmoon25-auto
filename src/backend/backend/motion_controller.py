import rclpy
import tf2_geometry_msgs
import quaternion
import math
import numpy as np

from rclpy.node import Node, QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped, Twist
from std_msgs.msg import Header, String, Int8

from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from threading import Condition

from interfaces.srv import GoTo
from enum import Enum

class State(Enum):
    WAITING_SERV = 0 # We are waiting for the required services to be initialized
    WAITING_CMD  = 1 # We are waiting for a request
    WAITING_PATH = 2 # We are waiting for a path to be published
    ROTATE       = 3 # We are currently rotating to the required orientation
    LINEAR       = 4 # We are currently moving to a node
    FINISHED     = 5 # We have completed the path
    ABORTED      = 6 # We have aborted the path

'''
    Handles controlling the autonomous motion of the robot. Provides a service, GoTo,
    which delivers the robot to the target destination. 

    Subscriptions:
    /path/path: Path, path to the goal provided by path_planner. Dynamically re-plans
                      based on updated map information.
    /odom_true:      Odometry, odom information provided by T265 or simulator

    Publishes:
    /path/goal: Pose, path_planner uses this as its goal when plotting paths.
    /cmd_vel:   Twist, for telling the motor driver what to do

    Services:
    GoTo: takes an x and y position on the map, and returns if succesful or aborted
'''

class MotionController(Node):
    
    def __init__(self):
        super().__init__('motion_controller')

        self.YAW_TOLERANCE = 0.05
        self.XZ_TOLERANCE = 0.25
        self.ROT_SPEED = 1.0
        self.LINEAR_SPEED = 35.0

        self.state = State.WAITING_SERV
        # We wait for certain nodes to be ready first
        self.get_logger().info("Waiting for nodes...")
        #self.waitForServices()
        self.get_logger().info("Found nodes!")
        self.state = State.WAITING_CMD

        # Now we initialize properly: subscribers, publishers, broadcasters service
        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        self.reliable_QOS = QoSProfile(
            depth=3,
            reliability=1, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Serivce is in a different thread, so it can wait for
        # us to finish path before returning message
        pubsub_cb = MutuallyExclusiveCallbackGroup()
        srv_cb    = MutuallyExclusiveCallbackGroup()

        self.srv = self.create_service(GoTo, 'goto', self.serveGoTo, callback_group=srv_cb)

        self.SUB_path = self.create_subscription(Path, 'path/path', self.onPath, self.QOS, callback_group=pubsub_cb)
        self.SUB_odom = self.create_subscription(Odometry, '/odom', self.onOdom, self.QOS, callback_group=pubsub_cb)
        self.SUB_cmd = self.create_subscription(Int8, '/cmd_mp', self.onCmd, self.reliable_QOS, callback_group=pubsub_cb)


        self.PUB_goal = self.create_publisher(Pose, 'path/goal', self.QOS, callback_group=pubsub_cb)
        self.PUB_vel = self.create_publisher(Twist, '/cmd/velocity', 10, callback_group=pubsub_cb)

        self.curr_path = []
        self.goal_idx = -1

        self.do_log = True

        self.transform = None # map -> odom

        self.srv_cond = Condition()

    # abort msg
    def onCmd(self, msg):
        self.sendStop()

        with self.srv_cond:
            self.state = State.FINISHED
            self.srv_cond.notify()

    def sendStop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.PUB_vel.publish(vel_msg)
        self.get_logger().info('Stopping robot')

    def getLookRotation(self, vec):
        length = math.sqrt(vec[0]**2 + vec[1]**2)
        vec = vec / length

        forward = np.array([1.0, 0.0 ,0.0])

        dot = np.dot(forward, vec)

        rot_ang = math.acos(dot)
        rot_axis = np.cross(forward, vec)

        return quaternion.from_rotation_vector(rot_ang * rot_axis)  

    # Gets the robots position in the map frame
    def getRobotMapPose(self, odom):
        # Get the transform from odom to map
        while (self.transform == None):
            self.transform = self.lookupTransform('map', 'odom')

        transformed_pose = tf2_geometry_msgs.do_transform_pose(odom.pose.pose, self.transform) 
        return transformed_pose          

    def onOdom(self, msg):
        try:
            if (self.state != State.ROTATE and self.state != State.LINEAR):
                return
                            
            goal_pose = self.curr_path[self.goal_idx]


            robo_map_pose = self.getRobotMapPose(msg)
            curr_orientation = robo_map_pose.orientation
            curr_position = robo_map_pose.position

            ang_z = 0.0
            lin_x = 0.0

            goal_quat = None

            curr_quat = quaternion.from_float_array([curr_orientation.x, curr_orientation.y, curr_orientation.z, curr_orientation.w])
            curr_euler = quaternion.as_euler_angles(curr_quat)
            curr_yaw = curr_euler[1]

            if curr_quat.y < 0:
                curr_yaw = - curr_yaw


            if (self.state == State.ROTATE):
                goal_quat = quaternion.from_float_array([goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w])
                goal_euler = quaternion.as_euler_angles(goal_quat)
                goal_yaw = goal_euler[1]

                if goal_quat.y < 0:
                    goal_yaw = -goal_yaw
        
                lin_x = 0.0

                dx = math.sin(goal_yaw)
                dy = math.cos(goal_yaw)

                bx = math.sin(curr_yaw)
                by = math.cos(curr_yaw)  

                theta = math.atan2(dx*by - dy*bx, dx*bx + dy*by)    
                    
                # Otherwise, rotate accordingly
                if (abs(theta) > 0.05):
                    if (theta < 0):
                        ang_z = self.ROT_SPEED
                    else:
                        ang_z = -self.ROT_SPEED
                else:
                    ang_z = 0.0

                # Check if we are within the tolerance
                if (abs(theta) < self.YAW_TOLERANCE):
                    self.get_logger().info('Rotated to goal orientation')
                    self.sendStop()
                    self.nextPose()
            elif (self.state == State.LINEAR):
                # Find the header from our current position to the goal position
                dx = goal_pose.position.x - curr_position.x
                dy = goal_pose.position.y - curr_position.y

                v_length = math.sqrt(dx**2 + dy**2)
                dx /= v_length
                dy /= v_length

                # Convert our header to a vector
                bx = math.cos(curr_yaw)
                by = math.sin(curr_yaw)

                cross = dx*by - dy*bx

                theta = cross
                if abs(theta) > 0.1:
                    if (theta < 0):
                        ang_z = -self.ROT_SPEED
                    else:
                        ang_z = self.ROT_SPEED
                else:
                    ang_z = 0.0
                    
                # Check if we are within the tolerance
                if (abs(curr_position.x - goal_pose.position.x) < self.XZ_TOLERANCE and
                    abs(curr_position.y - goal_pose.position.y) < self.XZ_TOLERANCE):
                    self.get_logger().info('Reached goal position')
                    self.sendStop()
                    self.nextPose()
                    return
                else:
                    # Otherwise, move accordingly. We assume the robot is always facing the goal, so just go forward
                    lin_x = self.LINEAR_SPEED


            msg = Twist()
            msg.angular.z = ang_z
            msg.linear.x = lin_x
            self.PUB_vel.publish(msg)
        except Exception as e:
            self.get_logger().error(str(e))
        
            
    def nextPose(self):
        self.do_log = True
        # Check if path was just initialized
        if (self.goal_idx == -1):
            self.goal_idx = 0
            self.get_logger().info('Starting path')
            self.state = State.ROTATE
            return self.curr_path[self.goal_idx]
            
        # Intermediate poses
        if (self.state == State.ROTATE):
            self.state = State.LINEAR
            self.goal_idx += 1

            # Check if at end of path
            if (self.goal_idx >= len(self.curr_path)):
                self.get_logger().info('Finished path')
                self.state = State.FINISHED
                self.goal_idx = -1
                self.curr_path = []

                self.sendStop()

                with self.srv_cond:
                    self.state = State.FINISHED
                    self.srv_cond.notify()
                return
            
            self.get_logger().info('Moving to next pose')
            return self.curr_path[self.goal_idx]
        
        self.state = State.ROTATE
        return self.curr_path[self.goal_idx]   

    def updatePath(self, poses):
        self.curr_path = []
        self.goal_idx = -1
        # Poses are in the map frame, so we need to transform them to the odom frame
        transform = None
        while (transform == None):
            transform = self.lookupTransform('map', 'odom')
            if (transform == None):
                self.get_logger().info('Waiting for transform')
                rclpy.sleep(0.1)

        # Transform the poses to the odom frame
        for pose in poses:
            #self.curr_path.append(tf2_geometry_msgs.do_transform_pose(pose.pose, transform))
            self.curr_path.append(pose.pose)


    def onPath(self, msg):
        if (self.state == State.WAITING_SERV or self.state == State.WAITING_CMD):
            return
        
        self.updatePath(msg.poses)
        self.get_logger().info('Updated path')

        # We have a path, so we can start moving
        self.nextPose()    


    def serveGoTo(self, request, response):
        self.get_logger().info('Received request')

        # Publish goal and wait for path
        self.state = State.WAITING_PATH

        goal_pose = Pose()
        goal_pose.position.x = request.goal_x
        goal_pose.position.y = request.goal_y
        goal_pose.position.z = 0.0
        goal_pose.orientation = Quaternion()

        # Rotation about z quaternion
        quat = quaternion.from_euler_angles(0.0, 0.0, request.goal_yaw)
        goal_pose.orientation.x = quat.x
        goal_pose.orientation.y = quat.y
        goal_pose.orientation.z = quat.z
        goal_pose.orientation.w = quat.w

        self.PUB_goal.publish(goal_pose)
        
        with self.srv_cond:
            self.srv_cond.wait_for(lambda: self.state == State.FINISHED or self.state == State.ABORTED)

        response.error_msg = String()

        if self.state == State.FINISHED:
            self.get_logger().info('Path finished')
            response.success = True
            response.error_msg.data = 'Path finished'
        elif self.state == State.ABORTED:
            self.get_logger().info('Path aborted')
            response.success = False
            response.error_msg.data = 'Path aborted'

        self.state = State.WAITING_CMD

        return response

    def lookupTransform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except TransformException as e:
            self.get_logger().error(f'Error looking up transform: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)

    mc = MotionController()
    executor = MultiThreadedExecutor()
    executor.add_node(mc)
    try:
        executor.spin()
    except KeyboardInterrupt:
        mc.get_logger().info('Keyboard interrupt, shutting down')
    except Exception as e:
        mc.get_logger().error(f'Error: {e}')
    finally:
        mc.destroy_node()

if __name__ == '__main__':
    main()