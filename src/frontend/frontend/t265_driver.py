import rclpy
import pyrealsense2 as rs
import numpy as np
import rclpy.timer
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Pose, PoseWithCovariance, Point, Quaternion, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
import pyrealsense2 as rs

from rclpy.node import Node, QoSProfile

T265_SN = '943222111294'
COV = 0.01

class T265Driver(Node):

    def __init__(self):
        super().__init__('t265_driver')

        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        self.PUB_odom = self.create_publisher(Odometry, '/odom', self.QOS)

        pipe = rs.pipeline()
        
        cfg = rs.config()
        cfg.enable_device(T265_SN)
        cfg.enable_stream(rs.stream.pose)

        pipe.start(cfg)

        self.get_logger().info("T265 Initialized")

        while True:

            frames = pipe.wait_for_frames()

            pose = frames.get_pose_frame()

            if not pose: continue

            time = self.get_clock().now().to_msg()
            self.publishOdom(pose, time)
            #self.broadcastTransform(pose, time)

    def publishOdom(self, pose, time):
        odom = Odometry()

        odom.header = Header(frame_id='odom', stamp=time)
        odom.child_frame_id = 'base_link'

        data = pose.get_pose_data()

        # Pose estimate
        # TODO: Fix covariance - no internal cov provided by T265 :(
        covariance = [
                COV, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, COV, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, COV, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, COV, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, COV, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, COV]
        
        # I know this is horrible, but for some reason I can't use the Point()
        # constructor to initialize these
        #
        # We also perform the transform from the optical frame to the actual camera frame here
        point = Point()
        point.x = -data.translation.z 
        point.y = -data.translation.x 
        point.z = data.translation.y 

        quat = Quaternion()
        quat.x = -data.rotation.z
        quat.y = -data.rotation.x
        quat.z = data.rotation.y
        quat.w = data.rotation.w

        odom_p = Pose()
        odom_p.position = point
        odom_p.orientation = quat

        odom.pose = PoseWithCovariance()
        odom.pose.pose = odom_p
        odom.pose.covariance = covariance

        odom.twist = TwistWithCovariance()
        odom_t = Twist()

        odom_t_vel = Vector3()
        odom_t_vel.x = -data.velocity.z #data.velocity.x
        odom_t_vel.y = -data.velocity.x #data.velocity.y
        odom_t_vel.z = data.velocity.y #data.velocity.z

        odom_t_ang = Vector3()
        odom_t_ang.x = -data.angular_velocity.z #data.angular_velocity.x
        odom_t_ang.y = -data.angular_velocity.x #data.angular_velocity.y
        odom_t_ang.z = data.angular_velocity.y #data.angular_velocity.z
       
        odom_t.linear = odom_t_vel
        odom_t.angular = odom_t_ang

        odom.twist.twist = odom_t
        odom.twist.covariance = covariance

        self.PUB_odom.publish(odom)


    def broadcastTransform(self, pose, time):
        ...

def main(args=None):
    rclpy.init()

    t265 = T265Driver()

    rclpy.spin(t265)

    t265.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
