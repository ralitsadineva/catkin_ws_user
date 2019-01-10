#!/usr/bin/env python
import rospy
import numpy as np
#import scipy
import cv2
#import matplotlib.pyplot as plt
import math
import sys
import collections
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

global g_m
global g_t
global image
#global cv_image
#cv_image=None

class line_detection:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/e_value", Float64, queue_size=1)
        self.pub_gradient = rospy.Publisher("/gradient", Float64, queue_size=1)


    def callback(self,data):
        print("callback")
        global cv_image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    	image1 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    	print(image1.shape)
    	image2 = image1[200:450, 0:640]
    	gray_max = 255
    	gray_min = 200
    	global image
    	ret1, image = cv2.threshold(image2, gray_min, gray_max, cv2.THRESH_BINARY)
    	#plt.imshow(image)
    	#print(image.shape)
        # x = 0
        # counter = 0
        print(image.shape)
        # row150 = image[150]
        # for index, pixel in enumerate(row150):
        #     if(pixel == 255):
        #         counter = counter + 1
        #         x += index
        # average_x = x/counter
        # print(average_x)
        #
        # center_line = 640/2
        # e = center_line - average_x
        # print("e:")
        # print(e)
        #

        e = self.find_error()
        m = self.find_gradient()
        # msg = (e, m)
        # self.pub.publish(msg)


    def find_error(self):
        global image
        # global cv_image
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # except CvBridgeError as e:
        #     print(e)
    	# image1 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    	# print(image1.shape)
    	# image2 = image1[200:450, 0:640]
    	# gray_max = 255
    	# gray_min = 200
    	# global image
    	# ret1, image = cv2.threshold(image2, gray_min, gray_max, cv2.THRESH_BINARY)
    	# #plt.imshow(image)
    	# #print(image.shape)
        x = 0
        counter = 0
        # print(image.shape)
        row150 = image[150]
        for index, pixel in enumerate(row150):
            if(pixel == 255):
                counter = counter + 1
                x += index
        average_x = x/counter
        print(average_x)

        center_line = 640/2
        e = center_line - average_x
        print("e:")
        print(e)
        # return e
        self.pub.publish(e)

    def find_gradient(self):
        global image
        x1 = 0
        x2 = 0
        counter1 = 0
        counter2 = 0
        row150 = image[150]
        for index, pixel in enumerate(row150):
            if(pixel == 255):
                counter1 = counter1 + 1
                x1 += index
        average_x1 = x1/counter1
        print(average_x1)

        row180 = image[180]
        for index, pixel in enumerate(row180):
            if(pixel == 255):
                counter2 = counter2 + 1
                x2 += index
        average_x2 = x2/counter2
        print(average_x2)

        m = (180 - 150)/float(average_x2 - average_x1)
        self.pub_gradient.publish(m)
        # return m


def main(args):
    rospy.init_node('line_detection', anonymous=True)
    ic = line_detection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
