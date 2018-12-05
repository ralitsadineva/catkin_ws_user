#!/usr/bin/env python
import roslib
roslib.load_manifest('py_image_processing')
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
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

        grey_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        #blurred_image = cv2.GaussianBlur(grey_cv_image, (11, 11), 0)
        bw_image = cv2.threshold(grey_cv_image, 245, 255, cv2.THRESH_BINARY)[1]
        #bw_image.shape #680 rows which contain a value per pixel in that row, 1 row has 860 pixels
        #might need to do this but I don't think so :
        #bw_image = cv2.erode(thresh, None, iterations=2)
        #bw_image = cv2.dilate(thresh, None, iterations=4)

        list_coords =[]
        ret, labels = cv2.connectedComponents(bw_image)
        labels.shape #680,860

        #should be 6 labels, I found 8 where 0 and 1 were background and all pixels in image
        #therefore to be ignored in coordinate calculation
        for label in np.unique(labels):
            if label == 0:
                continue

            print(label) #0to7 -> 8
            itemindex = np.where(labels==label) #find index of items with given label
            sum_y_elements_in_label = itemindex[0].size
            #print(itemindex[0])
            #print(sum_y_elemnts_in_label)
            sum_x_elements_in_label = itemindex[1].size
            #print(itemindex[1])
            #print(sum_x_elemnts_in_label)
            #print(itemindex) #tupel first all the row indicies then all the column indecies

            y_coord = round(np.sum(itemindex[0])/sum_y_elements_in_label) #get first tupel -> y
            print("y:")
            print(y_coord)
            x_coord = round(np.sum(itemindex[1])/sum_x_elements_in_label) #get second tupel -> x
            print("x:")
            print(x_coord)
            coords = tuple([x_coord,y_coord])
            list_coords.append(coords)

        print(list_coords)
        array_coords = np.asarray(list_coords)
        print(array_coords)

        #img_points = np.array([[210.0, 120.0], [420.0, 120.0], [210.0, 240.0], [420.0, 240.0], [210.0, 360.0], [420.0, 360.0]])

        fx = 614.1699
	fy = 614.9002
	cx = 329.9491
	cy = 237.2788
	camera_mat = np.zeros((3,3,1))
	camera_mat[:,:,0] = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

	k1 = 0.1115
	k2 = -0.1089
	p1 = 0
	p2 = 0
	dist_coeffs = np.zeros((4,1))
	dist_coeffs[:,0] = np.array([[k1, k2, p1, p2]])

	# far to close, left to right (order of discovery) in cm
	obj_points = np.zeros((6,3,1))
	obj_points[:,:,0] = np.array([[00.0, 00.0, 0], [39.5, 00.0, 0], [00.0, 31.0, 0], [40.0, 30.0, 0], [00.0, 59.0, 0], [40.0, 58.5, 0]])

	retval, rvec, tvec = cv2.solvePnP(obj_points, array_coords, camera_mat, dist_coeffs)

	print("\nrotation vector:")
	print(rvec)
	print("\ntranslation vector:")
	print(tvec)

	rmat = np.zeros((3,3))
	cv2.Rodrigues(rvec, rmat, jacobian=0)

	print("\nrotation matrix:")
	print(rmat)

	rmat_inv = np.transpose(rmat)
	#print(rmat_inv)

	tmat_inv = np.zeros((4,4))
	tmat_inv[0:3, 0:3] = rmat_inv
	tmat_inv[0:3, 3] = -rmat_inv.dot(tvec[:, 0])
	tmat_inv[3, 3] = 1.0
	print("\ntransformation matrix inverse:")
	print(tmat_inv)

	yaw = np.arctan2(rmat[1,0], rmat[0,0])
	pitch = np.arctan2(-rmat[2,0], math.sqrt(rmat[2,2] * rmat[2,2] + rmat[2,1] * rmat[2,1]))
	roll = np.arctan2(rmat[2,1], rmat[2,2])
	print("\nyaw, pitch, roll:")
	print(yaw, pitch, roll)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
cv2.destroyAllWindows()
if __name__ == '__main__':
     main(sys.argv)
