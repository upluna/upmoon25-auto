import rclpy
import numpy as np
import quaternion
import math
import tf2_ros
import tf2_geometry_msgs


from rclpy.node import Node, QoSProfile
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int8, ColorRGBA, Float32
from visualization_msgs.msg import Marker
from enum import Enum


# PoseStamped topic, /cmd/miner
# header: frame_id, command name
# abort - stop
# recinit - record initial pose (pose)
# recdump - record dump distance (pose.x distance away)
# mark - place marker pose.x distance away
# start - begin run

BUCKET_CHAIN_ON = True

# States:
class MinerState(Enum):
    STOPPED = 0
    DRIVE_TO_START = 1
    DIG = 2
    DRIVE_TO_DUMP = 3
    DUMP = 4
    DRIVE_TO_DIG = 5
    LOWER_BUCKET = 6
    RAISE_BUCKET = 7
    BUCKET_ALARM = 8

# SETPOINTS:
CLK = 0.05
BUCKET_LOWERED_POS = 45 #TODO:
DIG_SPEED = -15.0
MOVE_SPEED = 45.0
BUCKET_INCREMENTAL_LOWER = 2 #TODO
IR_THRESHOLD = 400 #<-Real Value = 400
BUCKET_SLOW_SPEED = 30

'''
    This node controls autonomous mining. You might want to gut this and rebuild it
    for 2026. It was scrapped together in a few hours. 

    The user first marks a position for the robot to start at, which it will use as its digging position,
    with 'recinit'. Then, the user marks a position for the robot to dump its load with 'recdump <distance>'.
    Upon receiving the 'start' command, the robot will drive to the initial position, lower its bucket,
    and begin digging. After some time, it will raise the bucket and drive to the dump position, dump, and repeat.

    Subscriptions:
    /cmd/miner - PoseStamped, command to execute. See main_controller for detailed command info
    /sensor/ir - Int16, IR distance sensor reading
    /sensor/bucket_alarm - Int16, bucket chain alarm reading, 1 if alarm is on, 0 if off
    /camera/rgb/tag_pose - PoseStamped, pose of the tag in the camera frame, used to determine position of the robot
    /odom - Odometry, used to determine the robot's position and orientation in the world

    Publishes:
    /miner_marker - Marker, used to visualize markers in Rviz for lining up the robot
    /cmd/velocity - Twist, linear.x for forward/backward speed, angular.z for rotation speed
    /cmd/conveyor - Int16, 1 to turn on the conveyor, 0 to turn off
    /cmd/bucket_vel - Int16, speed of the bucket chain
    /cmd/bucket_pos - Int16, position of the bucket chain, 0 for fully raised, 100 for fully lowered
'''
class MiningController(Node):

    def __init__(self):
        super().__init__('mining_controller')

        self.get_logger().info("Initialized")

        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        cmd_QOS = QoSProfile(
            depth=3,
            reliability=1, # Reliable
            history=1,
            durability=2
        )

        self.create_subscription(PoseStamped, '/cmd/miner', self.onCmd, cmd_QOS)
        #self.create_subscription(Odometry, '/odom', self.onOdom, self.QOS)
        self.create_subscription(PoseStamped, '/camera/rgb/tag_pose', self.onTagPose, self.QOS)

        self.ir_distance = 0
        self.create_subscription(Int16, '/sensor/ir', self.onIR, self.QOS)

        self.bucket_alarm = 0
        self.prev_bucket_alarm = 0
        self.create_subscription(Int16, '/sensor/bucket_alarm', self.onBucketAlarm, self.QOS)

        self.PUB_marker = self.create_publisher(Marker, '/miner_marker', self.QOS)
        self.timer = self.create_timer(CLK, self.state_check)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.velocity_pub = self.create_publisher(Twist, 'cmd/velocity', 10)
        self.conveyor_pub = self.create_publisher(Int16, 'cmd/conveyor', 10)
        self.bucket_vel_pub = self.create_publisher(Int16, 'cmd/bucket_vel', 10)
        self.bucket_pos_pub = self.create_publisher(Int16, 'cmd/bucket_pos', 10)

        self.velocity = Twist()
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0

        self.conveyor = Int16()
        self.conveyor.data = 0

        self.bucket_vel = Int16()
        self.bucket_vel.data = 0

        self.bucket_pos = Int16()
        self.bucket_pos.data = 0

        self.rec_init = None
        self.rec_dump = None
        self.run_init = np.array([0.0, 0.0, 0.0])
        self.back_dist = 0.0

        self.abort = False
        self.first = True

        self.prev_state = MinerState.STOPPED

        self.clock = 0
        self.clock_start = 0

        self.num_iterations = 0

        self.forward = np.array([1.0, 0.0, 0.0])
        self.pos     = np.array([0.0 ,0.0 ,0.0])

        self.state = MinerState.STOPPED

    def getTransform(self, stamp):
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'rgb_link_optical', rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().error("Could not find transform!")
            self.get_logger().error(f'{e}')

        return transform

    def onTagPose(self, msg):

        transform = self.getTransform(msg.header.stamp)
        if transform == None: return

        transform_pose = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)

        # Need to transform to base_link
        self.pos[0] = transform_pose.position.x
        self.pos[1] = transform_pose.position.y
        self.pos[2] = transform_pose.position.z

        #self.get_logger().info(f'POSE: {self.pos[0]}, {self.pos[1]}, {self.pos[2]}')

    def onOdom(self, msg):
        msg = msg.pose.pose
        odom_quat = quaternion.from_float_array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

        self.forward = quaternion.rotate_vectors(odom_quat, [1.0, 0.0, 0.0])

        self.pos[0] = msg.position.x
        self.pos[1] = msg.position.y
        self.pos[2] = msg.position.z

    def onCmd(self, msg):
        cmd_name = msg.header.frame_id

        self.get_logger().info(f'Received command {cmd_name}')

        if cmd_name == 'mark':
            self.onMark(msg.pose)
        elif cmd_name == 'recinit':
            self.onRecInit(msg.pose)
        elif cmd_name == 'recdump':
            self.onRecDump(msg.pose)
        elif cmd_name == 'run':
            self.state = MinerState.DRIVE_TO_START
            self.setRunInit()
        elif cmd_name == 'abort':
            self.abort = True
            self.state = MinerState.STOPPED

    def onIR(self, msg):
        self.ir_distance = msg.data

    def onBucketAlarm(self, msg):
        self.prev_bucket_alarm = self.bucket_alarm
        self.bucket_alarm = msg.data

    def onRecInit(self, pose):
        self.rec_init = pose.position.x


    def onRecDump(self, pose):
        self.rec_dump = pose.position.x


    def onMark(self, pose):
        dist = pose.position.x

        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

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
        msg.pose.position.x = float(dist)
        msg.pose.position.y = 0.0
        msg.pose.position.z = 1.5

        msg.pose.orientation = Quaternion()
        np_quat = quaternion.from_euler_angles(0, np.pi / 2, 0)
        msg.pose.orientation.x = np_quat.x
        msg.pose.orientation.y = np_quat.y
        msg.pose.orientation.z = np_quat.z
        msg.pose.orientation.w = np_quat.w


        self.get_logger().info(f'Marker pos: {msg.pose.position}')

        msg.scale = Vector3()
        msg.scale.x = 1.5
        msg.scale.y = 0.075
        msg.scale.z = 0.075

        msg.frame_locked = True

        self.PUB_marker.publish(msg)

    def setRunInit(self):
        self.run_init[0] = self.pos[0]
        self.run_init[1] = self.pos[1]
        self.run_init[2] = self.pos[2]

    # p is a pose
    def getDist(self, p):
        return math.sqrt(
            (self.pos[0] - p[0])**2 +
            (self.pos[1] - p[1])**2 +
            (self.pos[2] - p[2])**2
        )

    def state_check(self):
        self.clock += CLK
        if (self.bucket_alarm == 1 and self.prev_bucket_alarm == 0): # Rising edge case
            self.state = MinerState.BUCKET_ALARM
            self.get_logger().error("Bucket Chain Stall Detected! Attempting Recovery")
            self.clock_start = self.clock
        
        if self.state == MinerState.STOPPED:
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = 0.0

            self.conveyor.data = 0

            self.bucket_pos.data = 0

            self.bucket_vel.data = 0

            if self.first:
                self.velocity_pub.publish(self.velocity)
                self.conveyor_pub.publish(self.conveyor)
                self.bucket_vel_pub.publish(self.bucket_vel)
                self.bucket_pos_pub.publish(self.bucket_pos)

            self.first = False

        elif self.state == MinerState.DRIVE_TO_START:
            distance = self.getDist(self.run_init)

            self.get_logger().info(f'run_init {self.run_init}, pos {self.pos}')
            self.get_logger().info(f'Distance: {distance}')

            self.velocity.linear.x = 1.0 * MOVE_SPEED

            if distance >= self.rec_init:
                self.get_logger().info("Completed start drive")
                self.state = MinerState.LOWER_BUCKET
                self.velocity.linear.x = 0.0
                self.clock_start = self.clock
                self.setRunInit()
                self.bucket_pos.data = 0

        elif self.state == MinerState.LOWER_BUCKET:
            if (BUCKET_CHAIN_ON):
                self.bucket_vel.data = 100 # full speed !!

            if ((self.clock - self.clock_start >= 0.65) and self.ir_distance < IR_THRESHOLD):
                self.clock_start = self.clock
                self.bucket_pos.data += 2

            if (self.ir_distance >= IR_THRESHOLD):
                self.state = MinerState.DIG # presumably we are lowered and spinning, move on the the DIG state
                self.clock_start = self.clock
                self.setRunInit()
            
        elif self.state == MinerState.DIG:
            self.velocity.linear.x = DIG_SPEED

            self.bucket_vel.data = 100

            if (self.clock - self.clock_start >= 15):
                self.state = MinerState.RAISE_BUCKET
                self.clock_start = self.clock
                self.velocity.linear.x = 0.0

                self.get_logger().info("Raising bucket....")

                # Calc distance backwards we've travelled
                self.back_dist = self.getDist(self.run_init)
                self.get_logger().info(f'Travelled backwards {self.back_dist}')

                self.setRunInit()


        elif self.state == MinerState.RAISE_BUCKET:
            self.bucket_pos.data = 0

            self.bucket_vel.data = 100

            if (self.clock - self.clock_start >= 5):
                self.state = MinerState.DRIVE_TO_DUMP
                self.clock_start = self.clock
                self.bucket_vel.data = BUCKET_SLOW_SPEED
                self.get_logger().info("Driving to dump")
            elif (self.clock - self.clock_start >= 10):
                self.bucket_vel.data = -BUCKET_SLOW_SPEED
            elif (self.clock -self.clock_start >= 5):
                self.bucket_vel.data = BUCKET_SLOW_SPEED

        elif self.state == MinerState.DRIVE_TO_DUMP:
            distance = self.getDist(self.run_init)
            #self.get_logger().info(f'Distance: {distance}')

            self.velocity.linear.x = 1.0 * MOVE_SPEED

            if (self.clock - self.clock_start >= 10):
                self.bucket_vel.data = 0
            elif (self.clock -self.clock_start >= 5):
                self.bucket_vel.data = -BUCKET_SLOW_SPEED

            if distance >= self.rec_dump + self.back_dist:
                self.get_logger().info("Completed dump drive")
                self.state = MinerState.DUMP
                self.velocity.linear.x = 0.0
                self.clock_start = self.clock

        elif self.state == MinerState.DUMP:
            self.conveyor.data = 1
            self.bucket_vel.data = 0

            if (self.clock - self.clock_start >= 10):
                self.state = MinerState.DRIVE_TO_DIG
                self.conveyor.data = 0
                self.setRunInit()
                self.get_logger().info("Driving to dig")

        elif self.state == MinerState.DRIVE_TO_DIG:
            distance = self.getDist(self.run_init)
            self.get_logger().info(f'Distance: {distance}')

            self.velocity.linear.x = -1.0 * MOVE_SPEED

            if distance >= self.rec_dump:
                self.num_iterations += 1
                self.get_logger().info("Completed dig drive, driving to dump")
                self.state = MinerState.LOWER_BUCKET
                self.setRunInit()
                self.clock_start = self.clock
                self.velocity.linear.x = 0.0

        elif self.state == MinerState.BUCKET_ALARM:
            self.bucket_pos.data = 0

            self.bucket_vel.data = 0

            if (self.clock - self.clock_start >= 15):
                if (self.bucket_alarm == 0):
                    self.state = MinerState.DRIVE_TO_START
                    self.clock_start = self.clock
                    self.get_logger().warn("Recovered successfully! Restarting Cycle")
                else:
                    self.state = MinerState.STOPPED
                    self.abort = True
                    self.get_logger().error("Failed to Recover! Aborting...")
            elif (self.clock - self.clock_start >= 10):
                self.bucket_vel.data = +BUCKET_SLOW_SPEED
            elif (self.clock -self.clock_start >= 5):
                self.bucket_vel.data = -BUCKET_SLOW_SPEED


        if self.state != MinerState.STOPPED:
            self.velocity_pub.publish(self.velocity)
            self.conveyor_pub.publish(self.conveyor)
            self.bucket_vel_pub.publish(self.bucket_vel)
            self.bucket_pos_pub.publish(self.bucket_pos)

        if self.abort:
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = 0.0

            self.conveyor.data = 0

            self.bucket_vel.data = 0

            self.velocity_pub.publish(self.velocity)
            self.conveyor_pub.publish(self.conveyor)
            self.bucket_vel_pub.publish(self.bucket_vel)
            self.bucket_pos_pub.publish(self.bucket_pos)

            rclpy.shutdown()
            self.destroy_node()

def main(args=None):
    rclpy.init()

    mc = MiningController()

    rclpy.spin(mc)

    rclpy.shutdown()
    mc.destroy_node()

if __name__ == '__main__':
    main()