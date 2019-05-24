#!/usr/bin/env python

# Python libs
import os
import sys, time
import png
import math
from sklearn.cluster import KMeans

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
from math import sqrt

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
        
        image_np = cv2.imdecode(np_arr, cv2.COLOR_BGR2RGB)

        h = np.size(image_np, 0)
        w = np.size(image_np, 1)

        w, h = image_np.shape[:2]
        image_np = image_np[h/2:h, 0:w].copy()

        image_orig = image_np

        # Do the kmeans
        image = image_np.reshape((image_np.shape[0] * image_np.shape[1], 3))
        clt = KMeans(n_clusters = 5)
        clt.fit(image[0::10])
        print("Center clusters:", clt.cluster_centers_)
        
        diff = 0
        ind = -1
        for i, c in enumerate(clt.cluster_centers_):
            d = sqrt( ( c[0] - c[1]) ** 2 + (c[1] - c[2])**2 )
            if d > diff:
                diff = d
                ind = i

        if (diff < 10):
            print("skipping...")
            self.publish_img(image_np, self.image_pub)
            return

        thresh = 20
        greenLower = (clt.cluster_centers_[ind][0] - thresh, clt.cluster_centers_[ind][1] - thresh, clt.cluster_centers_[ind][2] - thresh)
        greenUpper = (clt.cluster_centers_[ind][0] + thresh, clt.cluster_centers_[ind][1] + thresh, clt.cluster_centers_[ind][2] + thresh)
        print("greenupper: ", greenUpper)
        print("greenlower; ", greenLower)
        print("\n")

        #cv2.imshow("orig", image_np)
        #cv2.waitKey()

        image_np = cv2.inRange(image_np, greenLower, greenUpper)
        #image_np = cv2.bitwise_and(image_np, image_np, mask=greenMask)
        
        edges = cv2.Canny(image_np, 50, 150, apertureSize=3)

        #cv2.imshow("Edges", edges)
        #cv2.waitKey()

        #im = Image.fromarray(greenMask)
        #im.save(time.strftime("%Y%m%d-%H%M%S"), "jpeg")

        # TODO: Try different parameters
        #lines = cv2.HoughLines(edges, 1, np.pi / 180, 25, 100, 10)


        minLineLength = 5      # Minimum length of line. Line segments shorter than this are rejected.
        maxLineGap = 100          # Maximum allowed gap between line segments to treat them as single line.
        rho_precision = 1
        theta_precision = np.pi / 180
        vote_count = 50

        lines = cv2.HoughLines(edges, rho_precision, theta_precision, vote_count) # minLineLength, maxLineGap)
        # If no lines are found punish and continue
        if lines is None:
            print("CameraProcessing.run() - no lines found")
            avg_theta = 500
        try:
            avg, img = self.draw_hough_lines(lines, image_np, image_orig)
        except:
            print("There is no detected line.")
            self.publish_img(image_np, self.image_pub)
            return

        self.publish_img(img, self.image_pub)
        print(avg)
        #self.subscriber.unregister()


    def callback(self, ros_data):
        self.curr_cam = ros_data.data
        

    def publish_img(self, img, pub):
        # Create published image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        pub.publish(msg)

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
            avg_theta += theta

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