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
from std_msgs.msg import UInt16
from std_msgs.msg import Int16


class PID_controller:
    global last_odom, current_scan, sum_ticks
    last_odom = None
    sum_ticks = 0

    def __init__(self):
        self.tick_sub = rospy.Subscriber("/ticks", UInt8, self.callback)
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)
        self.pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100)
        self.pub_rpm = rospy.Publisher("/rpm", UInt16, queue_size=100)
        rospy.sleep(1)
        self.pub_steering.publish(90)
        self.pub_speed.publish(self.pid_converter(0.25))

    def pid_converter(self,velocity):
        velocity = velocity * 1000
        return velocity

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
        self.pub_rpm.publish(sum_ticks)


def main(args):
    rospy.init_node('PID_controller')
    ic = PID_controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
#cv2.destroyAllWindows()
if __name__ == '__main__':
     main(sys.argv)
