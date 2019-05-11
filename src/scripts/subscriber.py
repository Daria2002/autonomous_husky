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
        # subscribed Topic
        self.subscriber = rospy.Subscriber("husky/camera1/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 10)

        self.image_pub = rospy.Publisher(
            "/output/image_raw/compressed",
            CompressedImage,
            queue_size=1)

        if VERBOSE :
            print ("subscribed to /camera/image/compressed")

    def getAngle(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)
        edges = cv2.Canny(gray, 30, 50, apertureSize=3)
        minLineLength = 10
        maxLineGap = 10

        try:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            greenLower = (30, 86, 6)
            greenUpper = (85, 240, 230)
            mask = cv2.inRange(hsv, greenLower, greenUpper)
        except cv2.error:
            print(img)
            print("Image doesn't exists")

        lineDetection = LineDetection()
        angle_deg = lineDetection.analyse(img)
        cv2.waitKey(2)
        return angle_deg

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.COLOR_BGR2GRAY) 

        feat_det = cv2.ORB_create()
        time1 = time.time()

        # convert np image to grayscale
        featPoints = feat_det.detect(
            cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
        time2 = time.time()
        #if VERBOSE :
        #    print ('%s detector found: %s points in: %s sec.'%(method,
        #        len(featPoints),time2-time1))

        for featpoint in featPoints:
            x,y = featpoint.pt
            cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
    
        edges = cv2.Canny(image_np, 130, 200, apertureSize=3)

        #im = Image.fromarray(image_np)
        # im.save(time.strftime("%Y%m%d-%H%M%S"), "jpeg")

        # TODO: Try different parameters
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 25, 100, 10)

        # If no lines are found punish and continue
        if lines is None:
            print("CameraProcessing.run() - no lines found")
            avg_theta = 500

        avg, img = self.draw_hough_lines(lines, image_np)

        # Create published image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()

        # Publish new image
        self.image_pub.publish(msg)

        print(avg)
        #self.subscriber.unregister()

    def draw_hough_lines(self, lines, img):
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

            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
            avg_theta += math.atan2(y1-y2, x1-x2)

        avg_theta /= len(lines)
 
        return avg_theta * 180/math.pi, img

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_feature', anonymous=True)
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)