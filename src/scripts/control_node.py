#!/usr/bin/env python

# Python libs
import os
import sys, time
import png
import math
from sklearn.cluster import KMeans
from barc.msg import ECU
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
	print("konstruktor control_manager")
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
        """
        # velocity publisher
        self.velocity_pub = rospy.Publisher(
            "/husky_velocity_controller/cmd_vel",
            Twist, queue_size=1)

        """
        self.velocity_pub = rospy.Publisher(
            "ecu_pwm", ECU, queue_size=1)

    def process(self):
        if self.angle == -1:
            self.vel = ECU(90, 90)
	    print("ne vidi se linija")
            self.velocity_pub.publish(self.vel)
            return

	print("vidi se linija")
        self.vel = self.calculate_velocity(self.angle)
        self.velocity_pub.publish(self.vel)

    def callback(self, ros_data):
        self.angle = ros_data.data

    def gaussian(self, x, mu, sig):
        return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))


    def calculate_velocity(self, angle):
        """
        Calculating velocity depending on detected angle
        """
        # steering = angle * 180/(math.pi/2)
        # throttle = 100-abs(angle - 90) 

        #help_steering = -(self.angle_pid.compute(math.pi/2, angle, 0.05))
        #steering = help_steering * 180/(math.pi)	
	
	steering = angle * 180/math.pi
	print("steering", steering)
    #throttle = 105
	throttle = self.gaussian(angle * 180/math.pi, 90, 30) * 7 + 98
    print("throttle", throttle)

        # ecu_cmd = ECU(throttle, steering)
		
	ecu_cmd = ECU(throttle, steering)	
	print("brzina je postavljena")
        return ecu_cmd

        """
        vel = Twist()

        vel.angular.z = -(self.angle_pid.compute(math.pi/2, angle, 0.05))
        #print("vel.angular.z= ", vel.angular.z);

        vel.linear.x = abs((1.5 + vel.angular.z)*1.5)
        #print("vel.linear.x= ", vel.linear.x)

        return vel
        """
def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node("control_node")
    cm = control_manager()
    
    while not rospy.is_shutdown():
        cm.process()
        rospy.sleep(0.05)

if __name__ == '__main__':
    main(sys.argv)