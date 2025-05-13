import rclpy
import pyrealsense2 as rs
import numpy as np
import rclpy.timer
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import nav_msgs.msg as nav_msgs
import pyrealsense2 as rs

from rclpy.node import Node, QoSProfile


RGB_SN = '018322071465'

IMG_WIDTH = 640
IMG_HEIGHT = 480

class RGBDriver(Node):

    def __init__(self):
        super().__init__('rgb_driver')

        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        self.PUB_rgbinfo = self.create_publisher(sensor_msgs.CameraInfo, '/camera/rgb/camera_info', self.QOS)
        self.PUB_rgb = self.create_publisher(sensor_msgs.Image, '/camera/rgb/image_raw', self.QOS)

        self.cam_info_msg = None # Camera info doesn't change, and we store it so we can keep republishing it

        pipe = rs.pipeline()

        cfg = rs.config()
        cfg.enable_device(RGB_SN)
        cfg.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, 30)
        
        pipe.start(cfg)

        self.get_logger().info("RGB camera initialized")

        # Retrieving camera intrinsics
        time = self.get_clock().now().to_msg()
        pipe_profile = pipe.get_active_profile()
        for stream_profile in pipe_profile.get_streams():

            if (stream_profile.is_video_stream_profile()):
                vs_p = stream_profile.as_video_stream_profile()

                if vs_p.stream_type() == rs.stream.color:
                    self.publishCamInfo(vs_p.intrinsics, time, "rgb_link_optical")

        # Now publish images
        frame_count = 0
        while True:
            frames = pipe.wait_for_frames()

            color = frames.get_color_frame()

            if not color: continue

            frame_count += 1
            time=self.get_clock().now().to_msg()

            if frame_count >= 15:
                frame_count = 0
                self.publishCamInfo(intrinsics=None, time=time, frame_id=None)

            self.publishImage(color, time)

    def publishImage(self, frame, time):
            width = frame.get_width()
            height = frame.get_height()

            img = np.frombuffer(frame.get_data(), dtype=np.uint16)

            msg = sensor_msgs.Image()

            msg.header = std_msgs.Header(frame_id='rgb_link_optical', stamp=time)

            msg.height = height
            msg.width = width
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = frame.get_stride_in_bytes()
            msg.data = img.tobytes()

            self.PUB_rgb.publish(msg)

    def publishCamInfo(self, intrinsics, time, frame_id):

        if self.cam_info_msg is not None:
            self.cam_info_msg.header.stamp = time
            self.PUB_rgbinfo.publish(self.cam_info_msg)
            return
        
        msg = sensor_msgs.CameraInfo()

        msg.height = intrinsics.height
        msg.width  = intrinsics.width

        # Images are undistorted
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Intrinsic camera matrix
        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.ppx
        cy = intrinsics.ppy
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        # No rectification: identity
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Projection matrix (rectified)
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0] 

        # Leave binning and ROI as default (no binning, ROI = entire FOV)
        msg.header = std_msgs.Header(frame_id=frame_id, stamp=time)

        self.cam_info_msg = msg

        self.PUB_rgbinfo.publish(msg)




def main(args=None):
    rclpy.init(args=None)

    dp = RGBDriver()

    rclpy.spin(dp)
    dp.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()