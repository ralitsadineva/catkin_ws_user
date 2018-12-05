#!/usr/bin/env python
import roslib
roslib.load_manifest('py_image_processing')
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter_grey:
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
        try:
            #Greyscale:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono16"))
            #Black and White:
            #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
        except CvBridgeError as e:
            print(e)
def main(args):
    ic = image_converter_grey()
    rospy.init_node('image_converter_grey', anonymous = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
if __name__ == '__main__':
     main(sys.argv)
#To view image:
#rosrun image_view image_view image:=<image topic> [image transport type]
#rosrun image_view image_view image:=/camera/image
#2 at a time
#rosrun image_view stereo_view stereo:=<stereo namespace> image:=<image topic identifier>
#rosrun image_view stereo_view stereo:=/my_stereo_cam image:=image_rect_color
