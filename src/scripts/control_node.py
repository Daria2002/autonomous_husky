#!/usr/bin/env python

# Python libs
import os
import sys, time
import png
import math
from sklearn.cluster import KMeans

from LineDetection import LineDetection
from PID import PID
# numpy and scipy
import numpy as np
from scipy.ndimage import filters
from PIL import Image

import matplotlib
from std_msgs.msg import Float64

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
from math import sqrt

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

VERBOSE=False

class control_manager:

    def __init__(self):
        self.angle = -1
        self.angle_pid = PID(1, 0, 0, 5, -5)
        self.vel = Twist()

        # subscribed Topic
        self.angle_sub = rospy.Subscriber("/detected_angle",
            Float64, self.callback,  queue_size = 10)

        self.velocity_pub = rospy.Publisher(
            "/husky_velocity_controller/cmd_vel",
            Twist, queue_size=1)

    def process(self):
        if self.angle == -1:
            print("Initialize self.angle")
            self.vel.angular.z = 0
            self.vel.linear.x = 0
            self.publish_velocity(self.vel)
            return

        self.vel = self.calculate_velocity(self.angle)
        self.publish_velocity(self.vel)

    def callback(self, ros_data):
        self.angle = ros_data.data

    def publish_velocity(self, vel):
        # Create published image
        self.velocity_pub.publish(vel)

    def calculate_velocity(self, angle):
        vel = Twist()
        vel.angular.z = -self.angle_pid.compute(math.pi/2, angle, 0.05)
        vel.linear.x = 2

        return vel

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node("control_node")
    cm = control_manager()
    
    while not rospy.is_shutdown():
        cm.process()
        rospy.sleep(0.05)

if __name__ == '__main__':
    main(sys.argv)