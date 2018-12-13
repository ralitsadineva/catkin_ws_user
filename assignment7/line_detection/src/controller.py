#!/usr/bin/env python
import numpy as np
import roslib
import sys
import rospy
import collections
from math import sqrt
#import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class controller:
    global current_scan, last_odom
    last_odom = None
    current_scan = None

    def __init__(self):
        self.controller_sub = rospy.Subscriber("/e_value", Float64,self.callback)
    	self.timer = rospy.Timer(rospy.Duration(1.5), self.timerCallback)
    	self.controller_pub = rospy.Publisher("/steer_command", UInt8, queue_size=1)
        # self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callbackOdom, queue_size=100)
        # #self.waitForFirstOdom()
        self.pub_stop_start = rospy.Publisher("manual_control/stop_start", Int16, queue_size=100)
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)
        self.pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100)
        rospy.sleep(1)
        #Point = collections.namedtuple('Point', ['x', 'y'])
        #originalAngle = 179
        #global msg_1
        #msg_1 = current_scan
        #dist_p1 = self.measure_distance(msg_1)
        #if msg_1 != current_scan:
        #self.drive(0.3, -200, originalAngle)
        #global msg_2
        #msg_2= current_scan
        #dist_p2 = self.measure_distance(msg_2)
        #if msg_2 != msg_1:
        #self.drive(0.1, -200, originalAngle)
        #global msg_3
        #msg_3= current_scan
        #dist_p3 = self.measure_distance(msg_3)
	    #self.drive(1.0, 200, 90)

    def mappingFunction(self, u_t):
    	Angles = collections.namedtuple('Angles', ['original', 'actual'])
    	anglesList = []
    	temp1 = Angles(0.0, actual = 90.0)
    	temp2 = Angles(-160.0, actual = 179.0)
    	temp3 = Angles(-106.67, actual = 150.0)
    	temp4 = Angles(-53.335, actual = 130.0)
    	temp5 = Angles(53.335, actual = 60.0)
    	temp6 = Angles(106.67, actual = 30.0)
    	temp7 = Angles(160.0, actual = 0.0)

    	anglesList.append(temp1)
    	anglesList.append(temp2)
    	anglesList.append(temp3)
    	anglesList.append(temp4)
    	anglesList.append(temp5)
    	anglesList.append(temp6)
    	anglesList.append(temp7)

        x_vals = []
        y_vals = []
        for x in anglesList:
            x_vals.append(x.original)
            y_vals.append(x.actual)
        #print(x_vals)
        #print(y_vals)
        z=np.polyfit(x_vals, y_vals,3)
        #print(z)
        f=np.poly1d(z)
        #print(str(f))
        y = np.polyval(f,u_t)
        #print(y)
        return y

    # def callbackOdom(self,msg):
    #     global last_odom
    #     last_odom = msg
    #
    # def waitForFirstOdom(self):
    #     global last_odom
    #     while not rospy.is_shutdown() and last_odom is None:
    #         rospy.loginfo(
    #             "%s: No initial odometry message received. Waiting for message...",
    #             rospy.get_caller_id())
    #         rospy.sleep(1.0)
    #
    # def drive(self,distance, speed, angle):
    #     global last_odom
    #     epsilon = 0.05
    #     try:
    #         #self.pub_speed.publish(0)
    #         self.pub_stop_start.publish(1)
    #         rospy.sleep(1)
    #         self.pub_steering.publish(angle)
    #         self.pub_stop_start.publish(0)
    #         rospy.sleep(1)
    #         self.pub_speed.publish(speed)
    #     except:
	#         print("error")
    #
    #     start_pos = last_odom.pose.pose.position
    #     current_distance = 0
    #
    #     while not rospy.is_shutdown() and current_distance < (distance - epsilon):
    #         current_pos = last_odom.pose.pose.position
    #         current_distance = sqrt((current_pos.x - start_pos.x)**2 + (current_pos.y - start_pos.y)**2)
    #         # rospy.loginfo("current distance = %f", current_distance)
    #         rospy.sleep(0.1)
    #
    #     try:
	#         self.pub_speed.publish(0)
    #     except:
	#         print("error")
            #is_active = False
            #current_pos = last_odom.pose.pose.position
            #current_distance = sqrt((current_pos.x - start_pos.x)
            #                        ** 2 + (current_pos.y - start_pos.y)**2)


    def callback(self,msg):
        global current_scan
        current_scan = msg.data
        print(current_scan)
        print(type(current_scan))

    def timerCallback(self, event):
        global current_scan
        #if current_scan = "error":
        #    timer.shutdown()
        if current_scan is not None:
            k = 0.5
            u_t = k*(current_scan)
            print(u_t)
            drive_angle = self.mappingFunction(u_t)
            print(drive_angle)
            self.controller_pub.publish(drive_angle)
            self.pub_steering.publish(drive_angle)
            self.pub_speed.publish(180)
        else:
            print("current_scan is None")


# rospy.init_node('scan_values')
# sub = rospy.Subscriber('/scan', LaserScan, callback)
# rospy.spin()

def main(args):
    rospy.init_node('controller')
    ic = controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
     main(sys.argv)
