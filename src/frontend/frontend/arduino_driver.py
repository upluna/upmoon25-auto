import rclpy
import serial
import serial.tools.list_ports
import subprocess
import time
import numpy as np
import quaternion

from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, Int16
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseWithCovariance
from sensor_msgs.msg import JointState



'''
    This node handles all interactions with the Arduino. This includes force sensors,
    IR sensors, camera servo, camera linear servo, and bucket linear servo.

    Subscriptions:
    /cmd/pan:           Int8 - +1 for pan left, -1 for pan right, 0 for stop
    /cmd/camera_height: Int16, 0 - 100 height percentage for the camera
    /cmd/bucket_pos:    Int16, 0 - 100 extension percentage for the bucket

    Publishes:
    /sensor/ir/right - IR distance measure in centimeters
    /sensor/ir/left

    TF transforms:
    actuator_base_link -> actuator_link - describes height of camera
    actuator_link -> servo_link         - describes camera rotation
'''

DEBUG = False
PAN_MAX = 180

PAN_ANG_0 = 0
PAN_ANG_180 = 280

LIN_SERVO_SPEED = 0.012 # Meters per second
LIN_SERVO_MAX = 0.2 # Meters

class PanState(Enum):
    STOP = 0
    LEFT = 1
    RIGHT = 2

class ArduinoDriver(Node):

    def __init__(self):
        super().__init__('arduino_driver')
        self.get_logger().info('Arduino driver initialized')
        self.get_logger().info("Locating arduino port...")

        # Find and flash to Arduino first
        
        self.port = self.findArduinoPort()
        if self.port:
            self.flashHexFile(self.port, 'ServoTest.ino.hex') 
        else:
            self.get_logger().error("Could not find arduino port!")
            self.destroy_node()
            rclpy.shutdown()
            return

        self.ser = serial.Serial(self.port, 115200, timeout=1)
        time.sleep(2)

        self.get_logger().info('Found arduino')
        # We have two different callback groups here, so that one thread
        # can listen for command inputs, while the other handles the
        # serial to the Arduino without blocking
        pub_cb = MutuallyExclusiveCallbackGroup()
        sub_cb = MutuallyExclusiveCallbackGroup()

        # Flash succesful: now initialize publishers
        self.ir_pub = self.create_publisher(Float32, '/sensor/ir', 1, callback_group=pub_cb)

        self.PUB_campan = self.create_publisher(Int16, '/camera/rgb/pan', 1, callback_group=pub_cb)

        # Initialize subscribers
        self.create_subscription(Int8,  '/cmd/pan', self.onPan, 3, callback_group=sub_cb)
        self.create_subscription(Int16, '/cmd/bucket_pos', self.onBucket, 10, callback_group=sub_cb)
        self.create_subscription(Int16, '/cmd/camera_height', self.onCam, 10, callback_group=sub_cb)

        # Transform broadcasters
        self.PUB_joint = self.create_publisher(JointState, '/joint_states', 3, callback_group=sub_cb)

        self.create_timer(0.04, self.camTick, sub_cb)
        self.create_timer(0.01, self.handleSerial, pub_cb)

        self.write = True # Set to true whenever cam pan is updated, false when written to Arduino
        self.pan_state = PanState.STOP
        self.cam_pan = 0 # 0 to PAN_MAX degrees (?)

        self.cam_height = 100
        self.cam_height_tf = 0.0
        self.bucket_height = 0

        # Publish initial transforms 
        self.publishActTF(0.0)
        self.publishServoTF(0.0)

    def onBucket(self, msg):
        self.bucket_height = msg.data
        self.write = True

    def onCam(self, msg):
        self.cam_height = msg.data
        self.write = True

    def handleSerial(self):
        # Now listen for serial input from arduino
        
        #self.get_logger().info('Tick')
        
        if (self.write):
            self.write = False

            # Pad with zeroes so arduino is always getting the same size input
            cam_pan_pad = str(self.cam_pan).zfill(3)
            cam_height_pad = str(self.cam_height).zfill(3)
            bucket_height_pad = str(self.bucket_height).zfill(3)

            string = f'{cam_pan_pad}:{cam_height_pad}:{bucket_height_pad}\n'
            #self.get_logger().info(f'Writing {string}')
            self.ser.write(string.encode())
            self.ser.flush()

        if self.ser.in_waiting > 0:
            datum = self.ser.readline().decode('utf-8').rstrip('\n')
            datum = str(datum)
            if (datum[0] == '#'):
                self.ir_send(datum)
            
            if DEBUG:
                self.get_logger().info(f'ARDUINO: {datum}')

            return 
        
        return
    
    def ir_send(self, data):
        data = data[1 : ]
        tokens = data.split(':')
        right_raw = int(tokens[0])
        left_raw =  int(tokens[1])

        # Convert raw 0-1023 range to centimeters
        right = float(right_raw)*(-0.13146) + 90.769
        left = float(left_raw)*(-0.13146) + 90.769
        right = left # TODO: temporary until right side is fixed

        avg = (left + right) / 2
        
        msg = Float32()
        msg.data = avg

        # Publish a float value in centimeters
        self.ir_pub.publish(msg)

    # Updates the cam pan
    def camTick(self):
        if (self.pan_state == PanState.LEFT):
            self.cam_pan -= 1
            self.write = True
        elif (self.pan_state == PanState.RIGHT):
            self.cam_pan += 1
            self.write = True

        if (self.cam_pan < 0):
            self.cam_pan = 0
        elif(self.cam_pan > PAN_MAX):
            self.cam_pan = PAN_MAX
        
        # Publish the current camera pan
        msg = Int16()
        msg.data = self.cam_pan
        self.PUB_campan.publish(msg)

        # Logic for approximating position of linear servo
        converted_cam_height = (float(self.cam_height) / 100.0) * LIN_SERVO_MAX
        if (self.cam_height_tf < converted_cam_height):
            self.cam_height_tf += (LIN_SERVO_SPEED * 0.04) # Timer runs every 0.04 seconds
        elif (self.cam_height_tf > converted_cam_height):
            self.cam_height_tf -= (LIN_SERVO_SPEED * 0.04)

        # Stops the jittering
        if (abs(self.cam_height_tf - converted_cam_height) <= LIN_SERVO_SPEED * 0.04):
            self.cam_height_tf = converted_cam_height


        # Calculate actual transform of camera
        percentage = float(self.cam_pan) / PAN_MAX
        ang = ((PAN_ANG_180 - PAN_ANG_0) * percentage) + PAN_ANG_0

        self.publishServoTF(ang)
        self.publishActTF(self.cam_height_tf)

    def publishActTF(self, height):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = ['actuator_joint']
        msg.position = np.array([height], dtype=np.float64).tolist()
        msg.velocity = np.array([0], dtype=np.float64).tolist()
        msg.effort = np.array([0], dtype=np.float64).tolist()

        self.PUB_joint.publish(msg)


    def publishServoTF(self, ang):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        radians_ratio = np.pi / 180.0

        msg.name = ['servo_joint']
        msg.position = np.array([-ang * radians_ratio], dtype=np.float64).tolist()
        msg.velocity = np.array([0], dtype=np.float64).tolist()
        msg.effort =   np.array([0], dtype=np.float64).tolist()

        self.PUB_joint.publish(msg)
        
    def onPan(self, msg):
        if (msg.data > 0):
            self.pan_state = PanState.LEFT
        elif (msg.data < 0):
            self.pan_state = PanState.RIGHT
        else:
            self.pan_state = PanState.STOP

    def flashHexFile(self, port, hex_file):
        avrdude_cmd = [
            'avrdude',
            '-v',
            '-V',
            '-c',
            'arduino',
            '-patmega328p',
            'carduino',
            f'-P{port}',
            '-b115200',
            '-D',
            f'-Uflash:w:/home/upmoon25/ros2/upmoon25-auto/src/frontend/frontend/arduino/hex/{hex_file}:i'
        ]

        result = subprocess.run(avrdude_cmd, capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info("Flash succesful")
        else:
            self.get_logger().error("Flash failed!")
            self.get_logger().error(f"STDOUT: {result.stdout}")
            self.get_logger().error(f"STDERR: {result.stderr}")
            self.destroy_node()

    def findArduinoPort(self, arduino_vid='2341', arduino_pid='0043'):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid is not None and port.pid is not None:
                if f'{port.vid:04x}' == arduino_vid and f'{port.pid:04x}' == arduino_pid:
                    return str(port.device)
        return None




def main(args=None):
    rclpy.init(args=args)
    node = ArduinoDriver()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

