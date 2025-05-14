import rclpy
import cv2
import pyapriltags
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import quaternion

from rclpy.node import Node, QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from enum import Enum
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header, ColorRGBA, String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion, PoseStamped
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as R
from threading import Condition

from interfaces.srv import FindTag

RGB_IMG_TOPIC   = '/camera/rgb/image_raw'
RGB_INFO_TOPIC  = '/camera/rgb/camera_info'

SIM = False # Whether or not we're using simulated data

# This is a total mystery to me. For some reason, I have to divide the tag size by this factor
# for the pose estimation to work. Could break for real world testing
TAG_SIZE = 0.5 / 1.79690835


IMG_TOLERANCE = 20     # How much we allow the center of the tag to be from the center of the camera
IMG_WIDTH     = 640
IMG_HEIGHT    = 480

# For debugging
DO_TRANSFORM = False
PUBLISH_POSE = True # Whether or not to publish a pose indicating the relative location and orientation of the tag


class State(Enum):
    WAITING_CAMINFO = 0
    WAITING_REQUEST = 1
    SERVICING       = 2

class ServoCMD(Enum):
    STOP  = '0'
    LEFT  = '2'
    RIGHT = '1'

'''
    This node is an additional abstraction layer for the camera head. At the bottom is the
    arduino_driver, which actually talks to the hardware. This node offers a service which
    automatically looks for an AprilTag and publishes the resulting pose estimate.

    Subscriptions:
    /camera/rgb/image_raw:   Image, RGB images from the camera on the camera head
    /camera/rgb/camera_info: CameraInfo, info about the camera which is necessary for pose calculations

    Publishes:
    /cmd_servo:     String, this is for the simulated servo
    TODO: /cmd/pan: Int8, for real servo

    Services:
    FindTag: Given a timeout, searches for AprilTag and returns pose of tag.
'''
class HeadController(Node):

    def __init__(self):
        super().__init__('head_controller')

        if not SIM:
            # For the real world tag
            TAG_SIZE = 0.43

        self.state = State.WAITING_CAMINFO

        self.bridge = CvBridge()

        self.declare_parameter('ready', False)

        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        # Camera k-matrix: contains intrinsic camera parameters, used for focal length calcs
        self.cam_mtx = np.zeros((3, 3), dtype=np.float32)
        self.cam_mtx[2][2] = 1.0

        #self.publisher_marker = self.create_publisher(Marker, '/tag_marker', self.QOS)
        #self.publisher_pose = self.create_publisher(PoseStamped, '/tag_pose', self.QOS)

        # INITIALIZING SERVICES, SUBSCRIBERS, PUBLISHERS
        # This allows the servicing thread and img subscribing thread to run in parallel
        self.cb_group = ReentrantCallbackGroup()

        self.tag_pose_found = None

        self.srv = self.create_service(FindTag, 'find_tag', self.serviceFindTag, callback_group=self.cb_group)
        self.condition = Condition()

        self.subscriber_caminfo = self.create_subscription(
            CameraInfo, RGB_INFO_TOPIC, self.onInfo, self.QOS)
        
        self.cmd_publisher = self.create_publisher(String, '/cmd_servo', 1)

        if PUBLISH_POSE:
            self.pose_publisher = self.create_publisher(PoseStamped, '/debug/tag_pose', self.QOS)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.detector = pyapriltags.Detector(families='tagStandard41h12')

        self.prev_direction = ServoCMD.RIGHT

        self.get_logger().info("Successful initialization")
        self.get_logger().info("Waiting for camera info....")

    # With the default executor (MultiThreadedExecutor), this callback runs on
    # a separate thread of execution
    def serviceFindTag(self, request, response):

        self.get_logger().info("Service received!")

        if (self.state != State.WAITING_REQUEST):
            response.success = False
            return response
        
        # Got request: now initialize loop
        self.state = State.SERVICING

        # Wait for self to find tag - or timeout
        with self.condition:
            self.condition.wait(timeout=request.timeout_seconds)

            if self.tag_pose_found is None:
                response.success = False
                self.get_logger().warn("Timeout: could not find tag")
            else:
                response.success = True
                response.tag_pose = self.tag_pose_found
                self.get_logger().info("Service success!")
                

        # Reset after service
        self.state = State.WAITING_REQUEST
        self.tag_pose_found = None

        return response

    # CMD is of type ServoCMD
    def sendServoCMD(self, cmd):
        msg = String()
        msg.data = cmd.value
        self.cmd_publisher.publish(msg)

        if (cmd != ServoCMD.STOP):
            self.prev_direction = cmd

    def onInfo(self, msg):

        self.cam_mtx[0][0] = msg.k[0]
        self.cam_mtx[0][2] = msg.k[2]
        self.cam_mtx[1][1] = msg.k[4]
        self.cam_mtx[1][2] = msg.k[5]

        IMG_HEIGHT = msg.height
        IMG_WIDTH  = msg.width
        
        self.get_logger().info("Found camera info. Now receiving images")
        # Unsubscribe from caminfo topic and begin receiving images
        self.create_subscription(Image, RGB_IMG_TOPIC, self.onImg, self.QOS, callback_group=self.cb_group)
        self.destroy_subscription(self.subscriber_caminfo)

        self.state = State.WAITING_REQUEST

        # Signal to other nodes that we're ready
        ready_param = [rclpy.Parameter(name='ready', value=True)]
        self.set_parameters(ready_param)

    def onImg(self, msg):
        # We only care about the camera info if we're servicing a request
        if (not self.state == State.SERVICING):
            self.sendServoCMD(ServoCMD.STOP)
            return
        
        detection = self.detectTag(msg)
        time = self.get_clock().now().to_msg()

        # Haven't found a detection: keep rotating until we find one
        if (detection == None):
            self.sendServoCMD(self.prev_direction)
            return
        
        # Found tag: determine where the center is on the screen and move sensor accordingly
        tag_x = round(detection.center[0])
        tag_y = round(detection.center[1]) # Might need this for linear actuator code
        x_diff = (IMG_WIDTH // 2) - tag_x

        if (x_diff > IMG_TOLERANCE):
            self.sendServoCMD(ServoCMD.RIGHT)
        elif (x_diff < -IMG_TOLERANCE):
            self.sendServoCMD(ServoCMD.LEFT)
        else:
            self.sendServoCMD(ServoCMD.STOP)
            #self.foundTag(detection, time)
        self.foundTag(detection, time)

    def detectTag(self, img):
        cv_rgb_img = self.bridge.imgmsg_to_cv2(img_msg=img, desired_encoding='passthrough')
        cv_rgb_img = cv2.cvtColor(cv_rgb_img, cv2.COLOR_BGR2GRAY)
        cv_rgb_img = cv_rgb_img.astype(np.uint8)

        detections = self.detector.detect(
            img=cv_rgb_img,
            estimate_tag_pose=True,
            camera_params=(self.cam_mtx[0][0], self.cam_mtx[0][2], self.cam_mtx[1][1], self.cam_mtx[1][2]),
            tag_size = TAG_SIZE
            )
        
        for detection in detections:
            return detection
        
        return None

    # Rviz is dumb and will only recognize our marker if it's published to the 'odom' frame
    # That's why we have to do the transformation manually instead of using tf2
    def foundTag(self, detection, stamp):

        self.get_logger().info("Found tag")

        transform = None
        if DO_TRANSFORM:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 'rgb_link_optical', rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().info(str(e))
                return
        
        self.get_logger().info("Pure rotation estimate: " + str(
            quaternion.from_rotation_matrix(detection.pose_R)
            ))
        self.get_logger().info("Pure position estimate: " + str(
        detection.pose_t
        ))
        

        # This is so lame, but the constructor doesn't work :(
        # Rotation is handled later
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = 0.0
        quat.w = 1.0
        point = Point()

        pose_t = detection.pose_t
        point.x = float(pose_t[0])
        point.y = float(pose_t[1]) 
        point.z = float(pose_t[2])
        pose = Pose()

        pose.position = point
        pose.orientation = quat

        msg = PoseStamped()
        msg.header = Header(frame_id='base_link', stamp=stamp)
        msg.pose = pose

        # Now from optical frame to base_link frame
        if DO_TRANSFORM:
            msg.pose = tf2_geometry_msgs.do_transform_pose(pose, transform)

        # Orientation of the tag is weird. We are only interested in the yaw (z-rotation)
        # component because we assume the robot and tag are basically level. Also, the
        # transformation really screws up the pose_R for some reason, so we instead
        # get the transform from the servo_link to base_link to correct for servo
        # rotation and apply that instead.
        #
        # We also rotate have to rotate it by 180 degrees about the z-axis
        
        if DO_TRANSFORM:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 'rgb_link', rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().info(str(e))
                return
            
        tag_e = (R.from_matrix(detection.pose_R)).as_euler('xyz')
        print(tag_e)
        tag_e = R.from_euler('xyz', [0,0,np.pi - tag_e[0]])
        tag_q = tag_e.as_quat()

        pose.orientation.x = tag_q[0]
        pose.orientation.y = tag_q[1]
        pose.orientation.z = tag_q[2]
        pose.orientation.w = tag_q[3]

        fixed_orientation = pose

        if DO_TRANSFORM:
            fixed_orientation = tf2_geometry_msgs.do_transform_pose(pose, transform)
        msg.pose.orientation = fixed_orientation.orientation


        if PUBLISH_POSE:
            self.pose_publisher.publish(msg)

        # Alert servicing thread we have found the tag
        with self.condition:
            self.tag_pose_found = msg
            self.state = State.WAITING_REQUEST
            self.sendServoCMD(ServoCMD.STOP)
            self.condition.notify()


def main(args=None):
    rclpy.init(args=args)

    dr = HeadController()
    executor = MultiThreadedExecutor()
    executor.add_node(dr)

    executor.spin()

    dr.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()