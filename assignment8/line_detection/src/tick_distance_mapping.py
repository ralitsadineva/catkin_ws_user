#!/usr/bin/env python
import roslib
import sys
import rospy
import math
import numpy as np
from math import sqrt
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from std_msgs.msg import Int16


class tick_distance_mapping:
    global last_odom, current_scan, sum_ticks
    last_odom = None
    sum_ticks = 0

    def __init__(self):
        self.tick_sub = rospy.Subscriber("/ticks", UInt8, self.callback)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callbackOdom, queue_size=100)
        #self.waitForFirstOdom()
        self.pub_stop_start = rospy.Publisher("manual_control/stop_start", Int16, queue_size=100)
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)
        self.pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100)
        rospy.sleep(1)
        if last_odom is not None:
           self.drive(3.0, 200, 90)
        else:
            print("last_odom=None")

    def callbackOdom(self,msg):
        global last_odom
        last_odom = msg

    def waitForFirstOdom(self):
        global last_odom
        while not rospy.is_shutdown() and last_odom is None:
            rospy.loginfo(
                "%s: No initial odometry message received. Waiting for message...",
                rospy.get_caller_id())
            rospy.sleep(1.0)

    def drive(self,distance, speed, angle):
        print("drive")
        global last_odom
        epsilon = 0.05
        try:
            #self.pub_speed.publish(0)
            #self.pub_stop_start.publish(1)
            #rospy.sleep(1)
            self.pub_steering.publish(angle)
            #self.pub_stop_start.publish(0)
            #rospy.sleep(1)
            #self.pub_speed.publish(speed)
        except:
	        print("error")
        try:
            self.pub_speed.publish(speed)
        except:
            print("error1")


        start_pos = last_odom.pose.pose.position
        current_distance = 0

        while not rospy.is_shutdown() and current_distance < (distance - epsilon):
            current_pos = last_odom.pose.pose.position
            current_distance = sqrt((current_pos.x - start_pos.x)**2 + (current_pos.y - start_pos.y)**2)
            # rospy.loginfo("current distance = %f", current_distance)
            rospy.sleep(0.1)

        try:
	        self.pub_speed.publish(0)
        except:
	        print("error2")
            #is_active = False
            #current_pos = last_odom.pose.pose.position
            #current_distance = sqrt((current_pos.x - start_pos.x)
            #                       ** 2 + (current_pos.y - start_pos.y)**2)



    def callback(self,msg):
        global sum_ticks
        print("callback")
        global current_scan
        current_scan = msg.data
        print(current_scan)
        print(type(current_scan))
        distance = 0.1
        speed = 150
        angle = 90
        sum_ticks = sum_ticks + current_scan
        print("sum:" + str(sum_ticks))



def main(args):
    rospy.init_node('tick_distance_mapping')
    ic = tick_distance_mapping()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
#cv2.destroyAllWindows()
if __name__ == '__main__':
     main(sys.argv)
