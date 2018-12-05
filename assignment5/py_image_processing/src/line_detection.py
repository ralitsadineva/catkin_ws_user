#!/usr/bin/env python
import roslib
roslib.load_manifest('py_image_processing')
import sys
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from statistics import mean

class line_detection:
    def __init__(self):
        self.image_pub = rospy.Publisher("/image_processing/bin_img",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            #cv_image = self.bridge.imfmsog_to_cv2(data,"mono8")
        except CvBridgeError as e:
            print(e)

	crop_image = cv_image[200:450, 0:640]
        gray_max = 255
	gray_min = 245
	ret, grey_cv_image = cv2.threshold(crop_image, gray_min, gray_max, cv2.THRESH_BINARY)
	#grey_cv_image = cv2.cvtColor(crop_image, cv2.COLOR_RGB2GRAY)
        #bw_image = cv2.threshold(blurred_image, 200, 255, cv2.THRESH_BINARY)[1]
        dst = cv2.Canny(crop_image, 50, 200, None, 3)
	#cdst = cv2.cvtColor(crop_image, cv2.COLOR_GRAY2BGR)
        lines = cv2.HoughLinesP(dst, 1, np.pi / 180, 150, None, 0, 0)
        #if lines is not None:
        #    for i in range(0, len(lines)):
        #        rho = lines[i][0][0]
        #        theta = lines[i][0][1]
        #        a = math.cos(theta)
        #        b = math.sin(theta)
        #        x0 = a * rho
        #        y0 = b * rho
        #        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        #        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        #        cv2.line(crop_image, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
	right_lines = []
	right_xs = []
        right_ys = []
	right_slopes = []
	left_lines = []
	left_xs = []
        left_ys = []
	left_slopes = []
	for line in lines:
            #print(line)
	    x1,y1,x2,y2 = line[0]
	    slope = (y2-y1)/float(x2-x1)
            if slope > 0:
                right_lines.append(line)
                right_xs.append(x1)
                right_xs.append(x2)
                right_ys.append(y1)
                right_ys.append(y2)
		right_slopes.append(slope)
            elif slope < 0:
                left_lines.append(line)
                left_xs.append(x1)
                left_xs.append(x2)
                left_ys.append(y1)
                left_ys.append(y2)
		left_slopes.append(slope)

	right_m = mean(right_slopes)
        right_b = mean(right_ys) - right_m*mean(right_xs)

        left_m = mean(left_slopes)
	left_b = mean(left_ys) - left_m*mean(left_xs)

	print("\nleft m:" + str(left_m) + " left b:" + str(left_b) +  " right m:" + str(right_m) + " right b:" + str(right_b))

        y1 = 0 #crop_image.shape[0]
        y2 = 250

        right_x1 = int((y1 - right_b) / right_m)
        right_x2 = int((y2 - right_b) / right_m)
        cv2.line(crop_image, (right_x1, y1), (right_x2, y2), (0,0,255), 5, cv2.LINE_AA)

        left_x1 = int((y1 - left_b) / left_m)
        left_x2 = int((y2 - left_b) / left_m)
        cv2.line(crop_image, (left_x1, y1), (left_x2, y2), (0,0,255), 5, cv2.LINE_AA)

        try:
            #Greyscale:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(crop_image, "rgb8"))

        except CvBridgeError as e:
            print(e)


def main(args):
    ic = line_detection()
    rospy.init_node('line_detection', anonymous = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
cv2.destroyAllWindows()
if __name__ == '__main__':
     main(sys.argv)
