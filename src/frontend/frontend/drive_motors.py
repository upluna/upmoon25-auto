# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pysabertooth import Sabertooth
import time
from serial import SerialException
from pysabertooth import Sabertooth

'''
    This node controls the driving of the robot via two Sabertooth motor controllers.
    You may run into issues with the ports, so make sure to verify that the USB ports
    the Sabertooth controllers are connected to are correct.

    Subscriptions:
    /cmd/velocity - Twist, linear.x for forward/backward speed, angular.z for rotation speed
'''
class ReconnectableSaber:
    def __init__(self, port="/dev/ttyUSB0", baudrate=9600, address=128, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.address = address
        self.timeout = timeout
        self.saber = None
        self._connect()

    def _connect(self):
        try:
            self.saber = Sabertooth(self.port, baudrate=self.baudrate, address=self.address, timeout=self.timeout)
            print("[ReconnectableSaber] Connected to Sabertooth")
        except Exception as e:
            print(f"[ReconnectableSaber] Failed to connect: {e}")
            self.saber = None

    def drive(self, motor, speed):
        if not self.saber:
            self._connect()
            if not self.saber:
                print("[ReconnectableSaber] Still not connected. Skipping command.")
                return

        try:
            self.saber.drive(motor, speed)
        except SerialException as e:
            print(f"[ReconnectableSaber] SerialException: {e}. Reconnecting...")
            self._connect()
        except Exception as e:
            print(f"[ReconnectableSaber] Unexpected error: {e}")
    
    def stop(self):
        if not self.saber:
            self._connect()
            if not self.saber:
                print("[ReconnectableSaber] Still not connected. Skipping command.")
                return

        try:
            self.saber.stop()
        except SerialException as e:
            print(f"[ReconnectableSaber] SerialException: {e}. Reconnecting...")
            self._connect()
        except Exception as e:
            print(f"[ReconnectableSaber] Unexpected error: {e}")


saber = ReconnectableSaber('/dev/ttyACM0', baudrate=9600, address=128, timeout=0.1) #left wheels
saber2 = ReconnectableSaber('/dev/ttyACM1', baudrate=9600, address=128, timeout=0.1) #right wheels

TURN_SPEED = 50.0
DIR = 1.0          # 1 is the correct direction, set to -1 for backwards

class DriveMotors(Node):

    def __init__(self):
        super().__init__('drive_motors')
        self.subscription = self.create_subscription(
            Twist,
            'cmd/velocity',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Successful initialization!')

    def listener_callback(self, msg):
        #print(msg)
        throttle = msg.linear.x
        rotation = msg.angular.z

        if (throttle != 0.0 and rotation == 0.0):
            saber.drive(1, throttle*DIR)
            saber.drive(2, -throttle*DIR)
            saber2.drive(1, throttle*DIR)
            saber2.drive(2, -throttle*DIR)
        elif (throttle != 0.0 and rotation != 0.0):
            t1 = throttle
            t2 = throttle

            if (rotation < 0.0): # left turn
                t2 = int(t2 * 0.5)
            elif (rotation > 0.0): # right turn
                t1 = int(t2 * 0.5)

            saber.drive(1, t1*DIR)
            saber.drive(2, -t1*DIR)
            saber2.drive(1, t2*DIR)
            saber2.drive(2, -t2*DIR)
        elif (throttle == 0.0 and rotation != 0.0):
            if (rotation < 0.0): # left turn
                saber2.drive(1, -TURN_SPEED*DIR)
                saber2.drive(2, TURN_SPEED*DIR)
                saber.drive(1, TURN_SPEED*DIR)
                saber.drive(2, -TURN_SPEED*DIR)
            elif (rotation > 0.0): # right turn
                saber.drive(1, -TURN_SPEED*DIR)
                saber.drive(2, TURN_SPEED*DIR)
                saber2.drive(1, TURN_SPEED*DIR)
                saber2.drive(2, -TURN_SPEED*DIR)
        else:
            saber.stop()
            saber2.stop()


def main(args=None):
    
    rclpy.init(args=args)

    drive_motors = DriveMotors()

    rclpy.spin(drive_motors)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drive_motors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
