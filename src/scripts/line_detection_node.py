#!/usr/bin/env python

# Python libs
import os
import sys, time
import png
import math

from sklearn.cluster import KMeans
import re
from sklearn.preprocessing import Imputer
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
from sklearn.decomposition import PCA
from mpl_toolkits.mplot3d import axes3d

from sklearn.cluster import KMeans
from cv_bridge import CvBridge
from LineDetection import LineDetection
# numpy and scipy
import numpy as np
from scipy.ndimage import filters
from PIL import Image
import time

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
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

from sklearn import datasets
from sklearn.cluster import KMeans
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

VERBOSE=False
ITERATIONS = 5
DIFF_FACTOR = 10

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

        self.angle_pub = rospy.Publisher("/detected_angle", Float64, queue_size=1)

        if VERBOSE :
            print ("subscribed to /camera/image/compressed")

        self.color = [0, 0, 0]
        self.color_initilized = False
        self.kmeans_iter = 0
        self.image_printed = False
        self.camera_image_showed = False
        self.color_initilized_printed = False
            
    def k_means_plot(self, data):
        x=data[:,0]
        y=data[:,1]
        z=data[:,2]
        kmeans = KMeans(n_clusters=5)
        kmeans = kmeans.fit(data)
        labels = kmeans.predict(data)

        fig=plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ColorsA=plt.cm.viridis(np.linspace(0, 1,5),alpha=0.8) #Equally spaced color 
        for i in range(5): #Labels of the clusters 
            xL=[]
            yL=[]
            zL=[]
            for k in range(len(x)):
                if labels[k]==i: #Data points of each cluster 
                    xL.append(x[k])
                    yL.append(y[k])
                    zL.append(z[k])

            ax.scatter(xL,yL,zL,c=ColorsA[i])

    def array_to_image(self, data):
        img = Image.fromarray(data, 'BGR')
        img.save('my.png')

    def show_image(self, data):
        cv2.imshow('cv_img', data)
        cv2.waitKey(2)

    def process(self):
        if self.curr_cam is None:
            print("Initialize self.curr_cam")
            return

        if not self.color_initilized:
            print("Color not initialized, exiting detection")
            self.publish_angle(-1)
            return

        if not self.color_initilized_printed:
            print("Color initialized")
            self.color_initilized_printed = True

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(self.curr_cam, np.uint8)

        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # to show camera image uncomment next line
        # self.show_image(image_np)

        w, h = image_np.shape[:2]
        #image_np = image_np[h/2:h, 0:w].copy()
        image_orig = image_np
        
        thresh = 20
        colorLower = (self.color[0] - thresh, self.color[1] - thresh, self.color[2] - thresh)
        colorUpper = (self.color[0] + thresh, self.color[1] + thresh, self.color[2] + thresh)

        image_np = cv2.inRange(image_np, colorLower, colorUpper)
    
        color = (self.color[0], self.color[1], self.color[2])

        x_size = len(image_np[0])
        y_size = len(image_np[1])


        line_status = self.checkLine2(image_np)
        
        edges = cv2.Canny(image_np, 50, 150)



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
            #print("kut koji se pub ako se linije ne popravljaju:", avg)

            if line_status == 1:
                avg = avg + abs(avg) * 0.2

            elif line_status == 2:
                avg = avg - abs(avg) * 0.5
            
            #print("popravljeni kut", avg)

            self.publish_angle(avg)

        except Exception, e:
            print(str(e))

            self.publish_img(image_np, self.image_pub)
            self.publish_angle(-1)

            return

        self.publish_img(img, self.image_pub)
        #self.subscriber.unregister()

    def checkLine(self, imageToCheck):
        """
        This method returns 0 if line is not on border, 1 if line is on left, 2 if line is on right
        """
        # line on right and left half 
        count = 0

        first = False
        second = False
        third = False
        fourth = False

        #for a in range(0, len(imageToCheck)):
        for a in range(len(imageToCheck)/4, 3*len(imageToCheck)/4):
            for b in range(0, len(imageToCheck[0])):
                
                if imageToCheck[a][b] != 0:
                    if(0 <= b <= len(imageToCheck[0])/4 and first == False):
                        first = True
                        break

                    elif(len(imageToCheck[0])/4 < b <= len(imageToCheck[0])/2 and second == False):
                        if(third == True):
                            return 0
                        second = True
                        break

                    elif(len(imageToCheck[0])/2 < b <= 3*len(imageToCheck[0])/4 and third == False):
                        if(second == True):
                            return 0
                        third = True
                        break

                    elif(3*len(imageToCheck[0])/4 < b <= len(imageToCheck[0]) and fourth == False):
                        fourth = True
                        break

        if(first):
            return 1
        if(fourth):
            return 2
        return 0

    def checkLine2(self, imageToCheck):
        """
        This method returns 0 if line is not on border, 1 if line is on left, 2 if line is on right
        """
        # line on right and left half 
        count = 0

        first = False
        second = False
        third = False
        fourth = False

        count1 = 0
        count2 = 0

        #for a in range(0, len(imageToCheck)):
        for a in range(len(imageToCheck)/4, 3*len(imageToCheck)/4):
            for b in range(0, len(imageToCheck[0])):
                
                if imageToCheck[a][b] != 0:
                    if(0 <= b <= len(imageToCheck[0])/4 and first == False):
                        first = True
                        break

                    elif(len(imageToCheck[0])/4 < b <= len(imageToCheck[0])/2):
                        if(third == True):
                            count = count + 1
                        second = True
                        count1 = count1 + 1
                        break

                    elif(len(imageToCheck[0])/2 < b <= 3*len(imageToCheck[0])/4):
                        if(second == True):
                            count = count + 1
                        third = True
                        count2 = count2 + 1
                        break

                    elif(3*len(imageToCheck[0])/4 < b <= len(imageToCheck[0]) and fourth == False):
                        fourth = True
                        break

        if count > 120:
            return 0

        if(first):
            return 1

        if(fourth):
            return 2

        if(count1 == count2):
            return 0

        if(count1 > count2):
            return 1
        
        return 2

    def initialize_color(self):
        if self.curr_cam is None:
            print("Initialize self.curr_cam")
            return

        if self.color_initilized:
            return

        np_arr = np.fromstring(self.curr_cam, np.uint8)

        image_np = cv2.imdecode(np_arr, cv2.COLOR_BGR2RGB)

        # Do the kmeans
        image = image_np.reshape((image_np.shape[0] * image_np.shape[1], 3))
        clt = KMeans(n_clusters = 5)
        clt.fit(image)
        

        #ovo je novo
        #cols = clt.cluster_centers_
        #self.plotClusters(image, clt.labels_, clt.cluster_centers_)
        #do ovdje


        #y_kmeans = clt.predict(image)
        #plt.scatter(image[:, 0], image[:, 1], c=y_kmeans, s=50, cmap='viridis')
        #centers = clt.cluster_centers_
        #plt.scatter(centers[:, 0], centers[:, 1], c='black', s=200, alpha=0.5);
        #plt.show()

        diff = 0
        ind = -1
        # find the most different cluster
        for i, c in enumerate(clt.cluster_centers_):
            d = sqrt( ( c[0] - c[1]) ** 2 + (c[1] - c[2])**2 )
            if d > diff:
                diff = d
                ind = i

        if diff < DIFF_FACTOR:
            print("unable to find a different color")
        else:
            self.kmeans_iter += 1
            self.color[0] += clt.cluster_centers_[ind][0]
            self.color[1] += clt.cluster_centers_[ind][1]
            self.color[2] += clt.cluster_centers_[ind][2]
            #print("Found diff color:", clt.cluster_centers_[ind])
            #print("Iteration count {}", self.kmeans_iter)

        if self.kmeans_iter == ITERATIONS:
            self.color_initilized = True
            self.color[0] /= ITERATIONS
            self.color[1] /= ITERATIONS
            self.color[2] /= ITERATIONS
            #print("Using color: ", self.color)

    def rgb_to_hex(self, rgb):
        return '#%02x%02x%02x' % (int(rgb[0]), int(rgb[1]), int(rgb[2]))
    
    def plotClusters(self, image, labels, cols):
        print("uslo u metodu za plotanje")
        #plotting 
        fig = plt.figure()
        print("napravljen fig")
        ax = Axes3D(fig)        
        print("napravljen ax")
        counter = 0
        for label, pix in zip(labels, image):
            if counter % 1000 != 0:
                continue
            ax.scatter(pix[0], pix[1], pix[2], color = self.rgb_to_hex(cols[label]))
        plt.show()

    def callback(self, ros_data):
        self.curr_cam = ros_data.data
        

    def publish_img(self, img, pub):
        # Create published image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        pub.publish(msg)

    def publish_angle(self, angle):
        """
        Publishes line angle in radians
        """
        # Create message
        msg = Float64()
        msg.data = angle
        self.angle_pub.publish(msg)

    def draw_hough_lines(self, lines, img, orig_img):
        """
       Draw Hough lines on the given image
 
       :param lines: Line array.
       :param img: Given image.
 
       :return: Image with drawn lines and line angle in radians
       """

        avg_theta = 0
        for line in lines:
            # Extract line info
            rho = line[0][0]
            theta = line[0][1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 2000*(-b))
            y1 = int(y0 + 2000*(a))
            x2 = int(x0 - 2000*(-b))
            y2 = int(y0 - 2000*(a))

            cv2.line(orig_img, (x1, y1), (x2, y2), (0, 0, 255), 5)

            if theta >= math.pi:
                theta = theta - math.pi

            theta = math.pi/2-theta

            if theta < 0:
                theta = theta + math.pi

            avg_theta += theta
        avg_theta /= len(lines)
        final_angle = avg_theta
        return final_angle, orig_img

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node("line_detection_node")
    ic = image_feature()

    while not rospy.is_shutdown():
        ic.initialize_color()
        ic.process()
        rospy.sleep(0.05)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
