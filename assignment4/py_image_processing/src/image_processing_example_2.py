#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')
#from matplotlib import pyplot as plt

# from __future__ import print_function

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #make it gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 245
    ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    clusters = np.zeros(thresh1.shape, dtype="uint8")
    coordinates = np.empty((2, 6, 2), dtype=np.intp)

    ret, labels = cv2.connectedComponents(thresh1)
    print(labels)
    #print np.amax(markers) #should be 6
    for label in np.unique(labels):
        print(label)
        #if background ignore it:
        if label == 0:
            continue
        #else get number of pixels in label and mid point of cluster
        labelCluster = np.zeros(thresh1.shape, dtype="uint8")
        labelCluster[labels == label] = 255
        numberOfPixels = cv2.countNonZero(labelCluster)
        sum_x_coord = np.sum(label[:, 0]) #get first column
        sum_y_coord = np.sum(label[:, 1]) #get second column
        mid_x = sum_x_coord/numberOfPixels
        mid_y = sum_y_coord/numberOfPixels
        #basically go to index same as label and save coordinates
        coordinates[label] = (mid_x, mid_y) #might work or use np.append
        print(mid_x + ", " + mid_y)

        #if cluster is bigger than 100 Pixels then save it
        if numberOfPixels > 100:
            cluster = cv2.add(cluster,labelCluster)
    print(coordinates)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
