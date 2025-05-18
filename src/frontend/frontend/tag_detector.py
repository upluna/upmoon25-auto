import rclpy
import cv2
import pyapriltags
import numpy as np
import tf2_ros

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from rclpy.node import Node, QoSProfile
from cv_bridge import CvBridge

# Spits out tag detection data from image stream
class TagDetector(Node):

    def __init__(self):
        super().__init__('tag_detector')

        self.tag_size = 0.4

        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        # Pub subs
        self.SUB_img = None # This is initialized once caminfo is received
        self.SUB_caminfo = self.create_subscription(CameraInfo, '/camera/rgb/camera_info', self.onInfo, self.QOS)

        self.PUB_pose = self.create_publisher(PoseStamped, '/camera/rgb/tag_pose', self.QOS)

        # Image handling
        self.bridge = CvBridge()
        self.detector = pyapriltags.Detector(families='tagStandard41h12')

        # Camera k-matrix: contains intrinsic camera parameters, used for focal length calcs
        self.cam_mtx = np.zeros((3, 3), dtype=np.float32)
        self.cam_mtx[2][2] = 1.0

        self.img_width  = 640 # Default low resolution amount
        self.img_height = 480


        self.get_logger().info('Initialized')

    def onImg(self, msg):
        # Decode compressed img
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        # AprilTag detection
        detections = self.detector.detect(
            img=image,
            estimate_tag_pose=True,
            camera_params=(self.cam_mtx[0][0], self.cam_mtx[0][2], self.cam_mtx[1][1], self.cam_mtx[1][2]),
            tag_size = self.tag_size
        )

        for detection in detections:
            #self.get_logger().info(f'Tag pos: {detection.pose_t}')
            #self.get_logger().info(f'Tag rot: {detection.pose_R}')
            #self.get_logger().info(f'Tag z-dist: {detection.pose_t[2]}')

            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header = msg.header # Both come from same frame and time

            pose_msg.pose = Pose()
            pose_msg.pose.position.x = float(detection.pose_t[0])
            pose_msg.pose.position.y = float(detection.pose_t[1])
            pose_msg.pose.position.z = float(detection.pose_t[2])

            self.PUB_pose.publish(pose_msg)




    def onInfo(self, msg):
        self.cam_mtx[0][0] = msg.k[0]
        self.cam_mtx[0][2] = msg.k[2]
        self.cam_mtx[1][1] = msg.k[4]
        self.cam_mtx[1][2] = msg.k[5]

        self.img_height = msg.height
        self.img_width = msg.width

        self.get_logger().info('Found camera info, now receiving messages')

        self.destroy_subscription(self.SUB_caminfo)

        self.SUB_img = self.create_subscription(CompressedImage, '/camera/rgb/image_compressed', self.onImg, self.QOS)

def main(args=None):
    rclpy.init()
    tc = TagDetector()
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        

