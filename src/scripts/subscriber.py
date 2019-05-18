#!/usr/bin/env python

# Python libs
import os
import sys, time
import png
import math

from LineDetection import LineDetection
# numpy and scipy
import numpy as np
from scipy.ndimage import filters
from PIL import Image

import matplotlib

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

class image_feature:

    def __init__(self):
        self.curr_cam = None

        # subscribed Topic
        self.subscriber = rospy.Subscriber("husky/camera1/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 10)

        self.image_pub = rospy.Publisher(
            "/output/image_raw/compressed",
            CompressedImage,
            queue_size=1)

        if VERBOSE :
            print ("subscribed to /camera/image/compressed")

    def process(self):
        if self.curr_cam is None:
            print("Initialize self.curr_cam")
            return

        if VERBOSE :
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(self.curr_cam, np.uint8)
        
        image_np = cv2.imdecode(np_arr, cv2.COLOR_BGR2GRAY) 

        h = np.size(image_np, 0)
        w = np.size(image_np, 1)

        w, h = image_np.shape[:2]
        image_np = image_np[h/4:3*h/4, 0:w].copy()

        image_orig = image_np

        greenLower = (25, 52, 72)
        greenUpper = (102, 255, 255)

        image_np = cv2.inRange(image_np, greenLower, greenUpper)

        edges = cv2.Canny(image_np, 100, 150, apertureSize=3)

        #im = Image.fromarray(image_np)
        # im.save(time.strftime("%Y%m%d-%H%M%S"), "jpeg")

        # TODO: Try different parameters
        #lines = cv2.HoughLines(edges, 1, np.pi / 180, 25, 100, 10)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 50, None, 50, 100)
        # If no lines are found punish and continue
        if lines is None:
            print("CameraProcessing.run() - no lines found")
            avg_theta = 500
        try:
            avg, img = self.draw_hough_lines(lines, image_np, image_orig)
        except:
            print("There is no detected line.")
            return

        # Create published image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()

        # Publish new image
        self.image_pub.publish(msg)

        print(avg)
        #self.subscriber.unregister()


    def callback(self, ros_data):
        self.curr_cam = ros_data.data
        

    def draw_hough_lines(self, lines, img, orig_img):
        """
       Draw Hough lines on the given image
 
       :param lines: Line array.
       :param img: Given image.
 
       :return: Image with drawn lines
       """
 
        avg_theta = 0
        for line in lines:
            # Extract line info

            rho = line[0][0]
            theta = line[0][1]
            #theta_temp = abs(theta - math.pi/2)
            #avg_theta += (theta_temp - math.pi/2)**4

            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 2000*(-b))
            y1 = int(y0 + 2000*(a))
            x2 = int(x0 - 2000*(-b))
            y2 = int(y0 - 2000*(a))

            cv2.line(orig_img, (x1, y1), (x2, y2), (0, 0, 255), 5)
            avg_theta += math.atan2(y1-y2, x1-x2)

        print(len(lines))

        avg_theta /= len(lines)

        return avg_theta * 180/math.pi, orig_img

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_feature', anonymous=True)
    ic = image_feature()
    
    while not rospy.is_shutdown():
        ic.process()
        rospy.sleep(0.05)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)