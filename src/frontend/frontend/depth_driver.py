import rclpy
import pyrealsense2 as rs
import numpy as np
import rclpy.timer
import pyrealsense2 as rs
import sensor_msgs.msg as sensor_msgs

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


from rclpy.node import Node, QoSProfile

DEPTH_SN = '018322071045'

class DepthDriver(Node):

    def __init__(self):
        super().__init__('depth_driver')

        # This is special for Gazebo - subscriber QOS must match publisher QOS
        self.QOS = QoSProfile(
            depth=3,
            reliability=2, # Best effort
            history=1,     # Keep last
            durability=2   # Volatile
        )

        self.PUB_pc = self.create_publisher(PointCloud2, '/camera/depth/points', self.QOS)

        # Filters for pointcloud data
        dec_filter = rs.decimation_filter()
        dec_filter.set_option(rs.option.filter_magnitude, 3)

        spat_filter = rs.spatial_filter()

        temp_filter = rs.temporal_filter()

        pc = rs.pointcloud()

        pipe = rs.pipeline()

        cfg = rs.config()
        cfg.enable_device(DEPTH_SN)
        cfg.enable_stream(rs.stream.depth, rs.format.z16, 30)

        pipe.start(cfg)

        self.get_logger().info("Depth camera initialized")

        while True:

            frame = pipe.wait_for_frames()

            depth = frame.get_depth_frame()

            time = self.get_clock().now().to_msg()

            if not depth: continue

            # Apply post-processing filters
            filtered = depth
            filtered = dec_filter.process(filtered)
            filtered = spat_filter.process(filtered)
            filtered = temp_filter.process(filtered)

            points = pc.calculate(filtered)
            vertices = np.array(points.get_vertices())
            
            x = vertices['f0']
            y = vertices['f1']
            z = vertices['f2']

            vertices = np.vstack((x, y, z)).T
            
            self.publishPC(vertices, time)    

    def publishPC(self, vertices, time):
        # Credit: https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo/blob/master/pcd_demo/pcd_publisher/pcd_publisher_node.py
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = vertices.astype(dtype).tobytes()

        fields = [sensor_msgs.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]
        
        header = Header(frame_id='depth_link_optical', stamp=time)

        msg = sensor_msgs.PointCloud2(
            header=header,
            height=1, 
            width=vertices.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * vertices.shape[0]),
            data=data
        )

        self.PUB_pc.publish(msg)


def main(args=None):
    rclpy.init()
    
    dd = DepthDriver()

    rclpy.spin(dd)

    dd.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()