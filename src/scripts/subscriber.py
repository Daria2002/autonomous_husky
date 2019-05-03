#!/usr/bin/env python
import sys, time
import copy
import numpy as np
from scipy.ndimage import filters
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
import cv2
import scipy.misc
import roslib
import rospy
from sensor_msgs.msg import CompressedImage

class line_detector_and_follower:

    def __init__(self):
        """
        Constructor for line_detector_and_follower initialize
        subscriber and publisher
        """

        # make subscriber to topic with messages from camera
        self.subscriber = rospy.Subscriber("morus/camera1/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        # publisher
        self.pub = rospy.Publisher('husky_velocity_controller/cmd_vel', Twist,queue_size=1)


    def callback(self, ros_data):
        """
        Callback function for subscriber. This function will execute when
        new message come.
        """

        # converts string to array
        np_arr = np.fromstring(ros_data.data, np.uint8)
        # reads an image from a buffer in memory
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        inputImage = scipy.misc.toimage(image_np)
    
        blur = cv2.medianBlur(image_np, 5)
        edges = cv2.Canny(blur, 150, 270)
        edges = cv2.dilate(edges, None)

        # TODO: Try different parameters
        threshold = 60
        minLineLength = 10
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 110)
        line_img, theta_deg, theta_rad = draw_hough_lines(lines, copy.deepcopy(inputImage), image_np)
        
        print(theta_deg)
        
        vel = followLine(theta_deg)
        self.pub.publish(vel)

def followLine(angle_deg):
    l_x = 0.2
    l_y = 0
    l_z = 0
    
    a_x = 0
    a_y = 0
    if(angle_deg > 70 and angle_deg < 110):
        a_z = 0
    elif(angle_deg < 70):
        a_z = -0.2
    else:
        a_z = 0.2
    print(angle_deg)

    return commander(l_x, l_y, l_z, a_x, a_y, a_z)
    
def commander(l_x, l_y, l_z, a_x, a_y, a_z):
    vel = Twist()
    vel.linear.x = l_x
    vel.linear.y = l_y
    vel.linear.z = l_z
    
    vel.angular.x = a_x
    vel.angular.y = a_y
    vel.angular.z = a_z

    return vel

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
        if -60 < abs(abs(theta * 180 / np.pi) - 90) < 20:
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
        theta_deg = theta * 180 / np.pi

    else:
        theta_deg = theta * 180 / np.pi 

    #theta is in rad
    return img, theta_deg, theta

def main(args):
    """
    Main function is run when program is executed
    """

    # node initialization
    rospy.init_node('publisher_line_detection')
    detectorAndFollower = line_detector_and_follower()

    try:
        # rospy.spin() lets all the callbacks get called for subscribers
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
