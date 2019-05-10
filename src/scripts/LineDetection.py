import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
import copy


class LineDetection:
    def __init__(self):
        pass

    def get_points(self, rho, theta):
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 2000 * (-b))
        y1 = int(y0 + 2000 * (a))
        x2 = int(x0 - 2000 * (-b))
        y2 = int(y0 - 2000 * (a))

        return x1, y1, x2, y2

    def draw_hough_lines(self, lines, img):
        """
        Draw Hough lines on the given image
        :param lines: Line array.
        :param img: Given image.
        :return: Image with drawn lines
        """

        r_acc = 0
        theta_acc = 0
        accepted_count = 0

        print(lines)

        if lines is None:
            return 0

        for line in lines:

            # Extract line info
            r = line[0][0]
            theta = line[0][1]

            x1, y1, x2, y2 = self.get_points(r, theta)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
            # Check if line is accepted as good
            # f r > 0 and temp_theta > 0 and temp_theta < 0 or r < 0 and temp_theta < 180 and temp_theta > 150:
            print(theta * 180 / np.pi, r)
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
            x1, y1, x2, y2 = self.get_points(r, theta)
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 10)

            print("Best theta: ", theta * 180 / np.pi)

        print("/n/n")
        theta_deg = theta * 180 / np.pi

        return theta_deg

    def analyse(self, inputImage):
        blur = cv2.medianBlur(inputImage, 5)
        edges = cv2.Canny(blur, 150, 270)
        edges = cv2.dilate(edges, None)

        # TODO: Try different parameters
        threshold = 60
        minLineLength = 10
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 110)
        theta_deg = self.getAngleInDeg(copy.deepcopy(inputImage))
        
        return theta_deg



    def getAngleInDeg(self, img):
        blur = cv2.medianBlur(img, 5)
        edges = cv2.Canny(blur, 150, 270)
        edges = cv2.dilate(edges, None)
        lines = cv2.HoughLines(edges, 1, np.pi / 90, 200)
        new = edges.copy()

        thetas = []

        for line in lines:
            for rho, theta in line:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                if theta > np.pi / 3 and theta < np.pi * 2 / 3:
                    thetas.append(theta)
                    new = cv2.line(new, (x1, y1), (x2, y2), (255, 255, 255), 1)

        theta_mean = np.mean(thetas)

        if(len(thetas) > 0):
            theta = self.rad_to_deg(theta_mean)
        else :
            theta = 0

        print(theta)

        return theta 


    def rad_to_deg(self, theta):
        return theta*180/math.pi