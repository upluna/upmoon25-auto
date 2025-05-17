import rclpy
import numpy as np
import quaternion
import math

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
    WAIT = 8

CLK = 0.05
BUCKET_LOWERED_POS = 60 #TODO: figure this out!!
DIG_SPEED = 5

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

        self.create_subscription(PoseStamped, '/cmd/miner', self.onCmd, self.QOS)
        self.create_subscription(Odometry, '/odom', self.onOdom, self.QOS)

        self.ir_distance = 0.0
        self.create_subscription(Float32, '/sensor/ir', self.onIR, self.QOS)

        self.PUB_marker = self.create_publisher(Marker, '/miner_marker', self.QOS)
        self.timer = self.create_timer(CLK, self.state_check)

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
        self.run_init = None

        self.abort = False

        self.clock = 0

        self.forward = np.array([1.0, 0.0, 0.0])
        self.pos     = np.array([0.0 ,0.0 ,0.0])

        self.state = MinerState.STOPPED

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
            self.run_init = self.pos
        elif cmd_name == 'abort':
            self.abort = True
            self.state = MinerState.STOPPED

    def onIR(self, msg):
        self.ir_distance = msg.data

    def onRecInit(self, pose):
        dist = pose.position.x
        new_forward = self.forward * dist

        self.rec_init = self.pos + new_forward


    def onRecDump(self, pose):
        dist = pose.position.x
        new_forward = self.forward * dist

        self.rec_dump = self.pos + new_forward


    def onMark(self, pose):
        dist = pose.position.x
        new_forward = self.forward * dist

        marker_pos = self.pos + new_forward

        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

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
        msg.pose.position.x = marker_pos[0]
        msg.pose.position.y = marker_pos[1]
        msg.pose.position.z = marker_pos[2] + 1.5

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

    # p is a pose
    def getDist(self, p):
        return math.sqrt(
            (self.pos[0] - p[0])**2 +
            (self.pos[1] - p[1])**2 
        )

    def state_check(self):
        self.clock += CLK
        if self.state == MinerState.STOPPED:
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = 0.0

            self.conveyor.data = 0

            self.bucket_pos.data = 0

            self.bucket_vel.data = 0

        elif self.state == MinerState.DRIVE_TO_START:
            distance = self.getDist(self.rec_init)
            self.get_logger().info(f'Distance: {distance}')

            self.velocity.linear.x = 1.0

            if distance <= 0.1:
                self.get_logger().info("Completed start drive")
                self.state = MinerState.LOWER_BUCKET
                self.velocity.linear.x = 0.0
        elif self.state == MinerState.LOWER_BUCKET:
            self.bucket_vel.data = 100 # full speed !!
            self.bucket_pos.data = BUCKET_LOWERED_POS

            self.clock += 1
            #TODO: check the IR distance?
            if (self.clock == 300): # wait 15s
                self.clock = 0
                self.state = MinerState.DIG # presumably we are lowered and spinning, move on the the DIG state
            
        elif self.state == MinerState.DIG:
            self.velocity.linear.x = DIG_SPEED
        elif self.state == MinerState.DRIVE_TO_DUMP:
            distance = self.getDist(self.rec_dump)
            self.get_logger().info(f'Distance: {distance}')

            self.velocity.linear.x = 1.0

            if distance <= 0.1:
                self.get_logger().info("Completed dump drive")
                self.state = MinerState.DUMP
                self.velocity.linear.x = 0.0
        elif self.state == MinerState.DUMP:
            ...
        elif self.state == MinerState.DRIVE_TO_DIG:
            distance = self.getDist(self.rec_init)
            self.get_logger().info(f'Distance: {distance}')

            self.velocity.linear.x = -1.0

            if distance <= 0.1:
                self.get_logger().info("Completed dig drive")
                self.state = MinerState.DRIVE_TO_DUMP
                self.velocity.linear.x = 0.0

        self.velocity_pub.publish(self.velocity)
        self.conveyor_pub.publish(self.conveyor)
        self.bucket_vel_pub.publish(self.bucket_vel)
        self.bucket_pos_pub.publish(self.bucket_pos)

        if self.abort:
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