import rclpy
from rclpy.node import Node, QoSProfile
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseWithCovariance
from interfaces.srv import FindTag

import numpy as np
import quaternion



# Publishes the true, adjusted odometry info based on inputs from the AprilTag
# Also publishes the transform from map -> odom
class Localizer(Node):

    def __init__(self):
        super().__init__('localizer')

        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        # Initialize client
        self.cli = self.create_client(FindTag, 'find_tag')

        while not self.cli.wait_for_service(timeout_sec=10):
            self.get_logger().info('Waiting for FindTag service...')
        
        self.get_logger().info('Initialized')


        self.active = True

        self.last_real_pos = None

        self.curr_translation = np.array([0.0, 0.0, 0.0]) # Pose received from AprilTag estimate, converted to odom frame
        self.curr_rotation = None
        self.odom_offset = np.array([0.0, 0.0, 0.0])
        self.map_odom_tf = None # Transform from map to odom, which we use to convert AprilTag estimates to odom

        self.PUB_odom = self.create_publisher(Odometry, '/odom_true', self.QOS)
        self.SUB_odom = self.create_subscription(Odometry, '/odom', self.onOdom, self.QOS)
        self.SUB_cmd = self.create_subscription(Int8, '/cmd_localizer', self.onCMD, self.QOS)

        self.TF_map_odom = StaticTransformBroadcaster(self)
        self.TF_odom_base_link = TransformBroadcaster(self)

        if (self.active):
            self.requestFindTag('balls')

    def onOdom(self, msg):
        self.curr_rotation = msg.pose.pose.orientation
        self.curr_translation[0] = msg.pose.pose.position.x
        self.curr_translation[1] = msg.pose.pose.position.y
        self.curr_translation[2] = msg.pose.pose.position.z


        self.publishOdom(msg)
        self.publishOdomBaseLinkTransform(msg.header.stamp)

    def publishOdom(self, msg):
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x += self.odom_offset[0]
        msg.pose.pose.position.y += self.odom_offset[1]
        msg.pose.pose.position.z += self.odom_offset[2]

        self.PUB_odom.publish(msg)

    def updateOdom(self, pose):
        quat_conj = quaternion.from_float_array([
        pose.pose.orientation.w,
        -pose.pose.orientation.x,
        -pose.pose.orientation.y,
        -pose.pose.orientation.z
        ])
        
        translate = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

        translate = quaternion.rotate_vectors(quat_conj, translate)

        self.get_logger().info(f'Received {translate}')
        self.get_logger().info(f'Base is {self.map_odom_tf.translation}')

        real_pos = np.array([
            -translate[0] - self.map_odom_tf.translation.x,
            -translate[1] - self.map_odom_tf.translation.y,
            -translate[2] - self.map_odom_tf.translation.z,
        ])

        self.get_logger().info(f'Real pose: {real_pos}')

        if self.last_real_pos is not None:
            self.get_logger().error(f'Diff: {real_pos - self.last_real_pos}')

        self.last_real_pos = real_pos
        self.get_logger().info(f'Curr translation: {self.curr_translation}')

        self.odom_offset[0] = real_pos[0] - self.curr_translation[0]
        self.odom_offset[1] = real_pos[1] - self.curr_translation[1]
        self.odom_offset[2] = real_pos[2] - self.curr_translation[2]

        self.get_logger().info(f'Corrected dead reckoning estimate: {self.odom_offset[0]}, {self.odom_offset[1]}, {self.odom_offset[2]}')


        #self.publishOdomBaseLinkTransform(pose.header.stamp)
        #self.publishOdom(pose.header.stamp)

    def publishOdomBaseLinkTransform(self, time):
        t = TransformStamped()

        t.header.stamp = time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.curr_translation[0] + self.odom_offset[0]
        t.transform.translation.y = self.curr_translation[1] + self.odom_offset[1]
        t.transform.translation.z = self.curr_translation[2] + self.odom_offset[2]

        t.transform.rotation.x = self.curr_rotation.x
        t.transform.rotation.y = self.curr_rotation.y
        t.transform.rotation.z = self.curr_rotation.z
        t.transform.rotation.w = self.curr_rotation.w

        self.TF_odom_base_link.sendTransform(t)

    
    def publishMapOdomTransform(self, pose):
        self.get_logger().info("hey")
        t = TransformStamped()

        t.header.stamp = pose.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        quat_conj = quaternion.from_float_array([
            pose.pose.orientation.w,
            -pose.pose.orientation.x,
            -pose.pose.orientation.y,
            -pose.pose.orientation.z
            ])
        
        translate = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

        translate = quaternion.rotate_vectors(quat_conj, translate)

        t.transform.translation.x = -translate[0]
        t.transform.translation.y = -translate[1]
        t.transform.translation.z = -translate[2]

        t.transform.rotation.x = -pose.pose.orientation.x
        t.transform.rotation.y = -pose.pose.orientation.y
        t.transform.rotation.z = -pose.pose.orientation.z
        t.transform.rotation.w = pose.pose.orientation.w

        self.get_logger().info('Published map -> odom')
        self.get_logger().info(f'TRANSLATION: {t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z}')
        self.get_logger().info(f'ROTATION: {t.transform.rotation.w}, {t.transform.rotation.x}, {t.transform.rotation.y}, {t.transform.rotation.z}')

        self.TF_map_odom.sendTransform(t)

        self.map_odom_tf = t.transform



    def onFindTag(self, future):
        self.get_logger().info("hai :3")
        if not self.active:
            return
        
        # Check if it was successful
        result = future.result()

        try:
            if result.success:
                if self.map_odom_tf is None:
                    self.publishMapOdomTransform(result.tag_pose)
                self.updateOdom(result.tag_pose)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


    def requestFindTag(self, frame):
        request = FindTag.Request()
        request.timeout_seconds = 10

        future = self.cli.call_async(request)
        future.add_done_callback(self.onFindTag)


    def onCMD(self, msg):
        value = int(msg.data)

        if (value == 0):
            self.active = False
            self.get_logger().info("Stopping localization")
        else:
            self.active = True
            self.get_logger().info("Starting localization")

            self.requestFindTag('balls')

def main(args=None):
    rclpy.init(args=args)
    localizer = Localizer()

    rclpy.spin(localizer)
    localizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()