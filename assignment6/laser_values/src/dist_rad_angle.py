#!/usr/bin/env python
import numpy as np
import roslib
import sys
import rospy
import collections
from math import sqrt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

class radius_detection:
    global current_scan

    def __init__(self):
        self.laser_sub = rospy.Subscriber("/scan",LaserScan,self.callback)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callbackOdom, queue_size=100)
        self.pub_stop_start = rospy.Publisher(
        "manual_control/stop_start",
        Int16,
        queue_size=100)
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)
        self.pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100)
        rospy.sleep(1)
        Point = collections.namedtuple('Point', ['x', 'y'])
        originalAngle = 179
        global msg_1
        msg_1 = current_scan
        dist_p1 = self.measure_distance(msg_1)
        self.drive(0.3, -200, originalAngle)
        global msg_2
        msg_2= current_scan
        dist_p2 = self.measure_distance(msg_2)
        self.drive(0.1, -200, originalAngle)
        global msg_3
        msg_3= current_scan
        dist_p3 = self.measure_distance(msg_3)

        results = self.find_radius_center_point(dist_p1,dist_p2,dist_p3)
        center_x = results[0].x
        center_y = results[0].y
        radius = results[1]
        lenght = 0.255
        rearRadius = dist_p1.x - center_x
        angle = np.arctan(lenght/rearRadius)
        degAngle = angle* (180.0/np.pi)
	Angles = collections.namedtuple('Angles', ['original', 'actual'])
        anglesList = []
        temp = Angles(originalAngle, actual = angle)
        anglesList.append(temp)
        self.printTable (anglesList)

    def find_radius_center_point(self,p1,p2,p3):
        Point = collections.namedtuple('Point', ['x', 'y'])
        a = p2.x * p2.x + p2.y * p2.y
        bc = (p1.x * p1.x + p1.y * p1.y - a)/2.0
        cd = (a - p3.x * p3.x - p3.y*p3.y)/2.0
        det = (p1.x - p2.x)*(p2.y - p3.y)-(p2.x - p3.x)*(p1.y - p2.y)
        if abs(det) < 1.0e-10:
            return (None, np.inf)
        center_x = (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) / det
        center_y = ((p1.x-p2.x) * cd - (p2.x - p3.x) * bc) / det
        cp = Point(center_x, y = center_y)
        radius = np.sqrt((center_x - p1.x)**2 + (center_y - p1.y)**2)
        return(cp, radius)

    def printTable(self,anglesList):
        print("Original Angle", '\t', "Actual Angle")
        print("--------------", '\t', "------------")
        for x in anglesList:
            print(x.original, '\t', x.actual)

    def measure_distance(self,msg):
        Point = collections.namedtuple('Point', ['x', 'y'])
        values = []
        for i in range(24):
            values.append(msg.ranges[i])
        j = 337
        while j<360:
            values.append(msg.ranges[j])
    	    j = j+1
        values2 = []
        t = 23
        while t < 69:
            values2.append(msg.ranges[t])
            t = t+1
        min_d1 = np.amin(np.array(values))
        min_d2 = np.amin(np.array(values2))
        print("min values1")
        print(min_d1)
        print("min values2")
        print(min_d2)
        result = Point(min_d2, y=min_d1)
        return(result)

    def callbackOdom(self,msg):
        global last_odom
        last_odom = msg

    last_odom = None

    def drive(self,distance, speed, angle):
        epsilon = 0.05
        print("1")
	try:
            self.pub_speed.publish(0)
            self.pub_stop_start.publish(1)
            rospy.sleep(1)
            self.pub_steering.publish(angle)
            self.pub_stop_start.publish(0)
            rospy.sleep(1)
            self.pub_speed.publish(speed)
	except:
	    print("error")

        start_pos = last_odom.pose.pose.position
        current_distance = 0

        print("2")
        while not rospy.is_shutdown() and current_distance < (distance - epsilon):
            current_pos = last_odom.pose.pose.position
            current_distance = sqrt((current_pos.x - start_pos.x)**2 + (current_pos.y - start_pos.y)**2)
            # rospy.loginfo("current distance = %f", current_distance)
            rospy.sleep(0.1)

        print("3")
        try:
	    self.pub_speed.publish(0)
	except:
	    print("error")
        #is_active = False
        #current_pos = last_odom.pose.pose.position
        #current_distance = sqrt((current_pos.x - start_pos.x)
        #                        ** 2 + (current_pos.y - start_pos.y)**2)


    def callback(self,msg):
	global current_scan
        current_scan = msg

def main(args):
    rospy.init_node('radius_detection')
    ic = radius_detection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
     main(sys.argv)
