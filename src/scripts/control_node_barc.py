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

class control_manager:

    def __init__(self):
        self.angle = -1
        self.last_ok_angle = -1
        self.count = 0

        self.p = 1.15 #1
        self.i = 0
        self.d = 0

        self.angle_pid = PID(self.p, self.i, self.d, 5, -5)
        # self.angle_pid = PID(self.p, self.i, self.d, math.pi, 0)
        self.vel = Twist()

        # subscribed Topic
        self.angle_sub = rospy.Subscriber("/detected_angle",
            Float64, self.callback,  queue_size = 10)
       
        self.velocity_pub = rospy.Publisher(
            "ecu_pwm", Twist, queue_size=1)

    def process(self):
        if self.angle == -1:
            self.vel = ECU(0, 0)
            self.velocity_pub.publish(self.vel)
            return

        self.vel = self.calculate_velocity(self.angle)
        self.velocity_pub.publish(self.vel)

    def callback(self, ros_data):
        self.angle = ros_data.data

    def calculate_velocity(self, angle):
        """
        Calculating velocity depending on detected angle
        """
        # steering = angle * 180/(math.pi/2)
        # throttle = 100-abs(angle - 90) 

        help_steering = -(self.angle_pid.compute(math.pi/2, angle, 0.05))
        steering = help_steering * 180/(math.pi)

        help_throttle = abs((1.5 + vel.angular.z)*1.5)
        throttle = help_throttle * 180/(math.pi)

        ecu_cmd = ECU(throttle, steering)

        return ecu_cmd
def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node("control_node")
    cm = control_manager()
    
    while not rospy.is_shutdown():
        cm.process()
        rospy.sleep(0.05)

if __name__ == '__main__':
    main(sys.argv)
