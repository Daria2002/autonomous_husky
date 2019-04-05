#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time
import copy
# numpy and scipy
import numpy as np
from scipy.ndimage import filters
import matplotlib.pyplot as plt
# OpenCV
import cv2

import scipy.misc
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
        # self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("morus/camera1/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        
        if VERBOSE :
            print "subscribed to /camera/image/compressed"


    def callback(self, ros_data):
        
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        #### Feature detectors using CV2 #### 
        # "","Grid","Pyramid" + 
        # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
        feat_det = cv2.ORB_create()
        time1 = time.time()

        # convert np image to grayscale
        featPoints = feat_det.detect(
            cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
        time2 = time.time()
        if VERBOSE :
            print '%s detector found: %s points in: %s sec.'%(method,
                len(featPoints),time2-time1)

        for featpoint in featPoints:
            x,y = featpoint.pt
            cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
        
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)

        inputImage = scipy.misc.toimage(image_np)
    
        
        plt.imshow(inputImage)

        # gray_image = cv2.cvtColor(inputImage, cv2.COLOR_BGR2GRAY)
        # plt.subplot(332)
        # plt.imshow(gray_image, cmap="gray")

        blur = cv2.medianBlur(image_np, 5)
        plt.subplot(335)
        plt.imshow(inputImage, cmap="gray")

        edges = cv2.Canny(blur, 150, 270)
        plt.subplot(333)
        plt.imshow(edges, cmap="gray")
        edges = cv2.dilate(edges, None)

        # TODO: Try different parameters
        threshold = 60
        minLineLength = 10
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 110)
        line_img, theta_deg, theta_rad = draw_hough_lines(lines, copy.deepcopy(inputImage), image_np)
        
        print(theta_deg)
        #plt.subplot(334)
        #plt.imshow(line_img)

        #plt.tight_layout()
        #plt.show()

def get_points(rho, theta):
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 2000 * (-b))
        y1 = int(y0 + 2000 * (a))
        x2 = int(x0 - 2000 * (-b))
        y2 = int(y0 - 2000 * (a))

        return x1, y1, x2, y2

def draw_hough_lines(lines, img, image_np):
        """
        Draw Hough lines on the given image
        :param lines: Line array.
        :param img: Given image.
        :return: Image with drawn lines
        """

        r_acc = 0
        theta_acc = 0
        accepted_count = 0

        for line in lines:

            # Extract line info
            r = line[0][0]
            theta = line[0][1]

            x1, y1, x2, y2 = get_points(r, theta)
            cv2.line(image_np, (x1, y1), (x2, y2), (0, 0, 255), 3)
            # Check if line is accepted as good
            # f r > 0 and temp_theta > 0 and temp_theta < 0 or r < 0 and temp_theta < 180 and temp_theta > 150:
            #print(theta * 180 / np.pi, r)
            # Check if line is accepted as good

            if -60<abs(abs(theta * 180 / np.pi) - 90) < 20:
                continue

            accepted_count += 1
            r_acc += r
            theta_acc += theta

        # Plot average accepted
        if accepted_count > 0:
            theta = theta_acc / accepted_count
            r = r_acc / accepted_count
            x1, y1, x2, y2 = get_points(r, theta)
            cv2.line(image_np, (x1, y1), (x2, y2), (255, 0, 0), 10)
            
            #print("Best theta: ", theta * 180 / np.pi)
            theta_deg = theta * 180 / np.pi
        #print("/n/n")
        #theta is in rad
        return img, theta_deg, theta

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
