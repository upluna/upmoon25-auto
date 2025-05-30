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
import pygame

from rclpy.node import Node

from pygame.locals import *

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Int8


class MinimalDriver(Node):

    def __init__(self):
        super().__init__('minimal_driver')

        # How frequently pygame should be polled
        self.clk = 0.01

        self.velocity_pub = self.create_publisher(Twist, 'cmd/velocity', 10)
        self.conveyor_pub = self.create_publisher(Int16, 'cmd/conveyor', 10)
        self.bucket_vel_pub = self.create_publisher(Int16, 'cmd/bucket_vel', 10)
        self.bucket_pos_pub = self.create_publisher(Int16, 'cmd/bucket_pos', 10)
        self.tensioner_pub = self.create_publisher(Int16, 'cmd/tensioner', 10)
        self.cam_height_pub = self.create_publisher(Int16, 'cmd/camera_height', 10)
        self.pan_pub = self.create_publisher(Int8, 'cmd/pan', 3)

        # Timer for polling events from pygame
        self.timer = self.create_timer(self.clk, self.pollEvents)

        pygame.init()
        pygame.font.init()
        self.font = pygame.font.SysFont('Arial', 30)	

        self.screen = pygame.display.set_mode((300, 300))
        pygame.display.set_caption('bbot driver')

        self.throttle = 100.0
        self.cam_height = 0
        self.bucket_pos = 0

        self.FPS = 60

        self.updateThrottleText()

        self.get_logger().info("Succesful initialization")

    def updateThrottleText(self):
        text_surface = self.font.render(f'Throttle: {self.throttle}', False, (255, 255, 255))
        self.screen.fill((0,0,0))
        self.screen.blit(text_surface, (0,0)) 


    # Helper method for pollEvents
    def setKeys(self, key, val):
        velocity_msg = Twist()
        
        cam_pan = 0

        if (val == 0):
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
        else:
            if (key == 119): # w
                velocity_msg.linear.x = self.throttle # set the direction to positive
            if (key == 115): # s
                velocity_msg.linear.x = -self.throttle # set the throttle to negative
            if (key == 97): # a
                velocity_msg.angular.z = -1.0 # negative value indicates a left turn
                velocity_msg.linear.x = self.throttle # set the direction to positive
            if (key == 100): # d
                velocity_msg.angular.z = 1.0 # positive value indicates a right turn
                velocity_msg.linear.x = self.throttle # set the direction to positive
            if (key == 101):
                self.throttle += 10.0
                if (self.throttle > 100.0):
                    self.throttle = 100.0
                self.updateThrottleText()
            if (key == 113):
                self.throttle -= 10.0
                if (self.throttle < 0):
                    self.throttle = 0
                self.updateThrottleText()
            
            if (key == 103): # g
                cam_pan = 1
            if (key == 104): # h
                cam_pan = -1
            if (key == 117): # u
                self.cam_height += 1
                if (self.cam_height > 100):
                    self.cam_height = 100
            if (key == 106): # j
                self.cam_height -= 1
                if (self.cam_height < 0):
                    self.cam_height = 0
            if (key == ord('i')): # u
                self.bucket_pos += 1
                if (self.bucket_pos > 100):
                    self.bucket_pos = 100
            if (key == ord('k')): # j
                self.bucket_pos -= 1
                if (self.bucket_pos < 0):
                    self.bucket_pos = 0
            

        # Velocity data is published as a twist object. The linear x component represents throttle (+/- = forward/backward)
        # The angular z component represents turning (+/- = right/left, 0 = no turn). Turning speed is constant regardless of z magnitude
        self.velocity_pub.publish(velocity_msg)
        
        pan_msg = Int8()
        pan_msg.data = cam_pan
        self.pan_pub.publish(pan_msg)

        height_msg = Int16()
        height_msg.data = self.cam_height
        self.cam_height_pub.publish(height_msg)

        bucket_msg = Int16()
        bucket_msg.data = self.bucket_pos
        self.bucket_pos_pub.publish(bucket_msg)


    # Publishes keyboard inputs
    def pollEvents(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                self.destroy_node()
                rclpy.shutdown()

            elif event.type == KEYDOWN:
                key = event.__dict__['key']
                self.setKeys(key, 1)

            elif (event.type == KEYUP):
                key = event.__dict__['key']
                self.setKeys(key, 0)

            pygame.display.update()
            pygame.time.Clock().tick(60)


def main(args=None):
    rclpy.init(args=args)

    minimal_driver = MinimalDriver()

    rclpy.spin(minimal_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
