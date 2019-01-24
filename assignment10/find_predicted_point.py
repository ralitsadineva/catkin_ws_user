#!/usr/bin/env python
import numpy as np
import math
import tf
import sys
import rospy
import collections
from math import sqrt
from sympy import symbols, solve, Eq
from scipy.optimize import fsolve
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import String

global msg

class find_predicted_point:

    def __init__(self):
        self.sub = rospy.Subscriber("/localization/odom/8", Odometry, self.callback, queue_size=1)
        self.pub_stop_start = rospy.Publisher("manual_control/stop_start", Int16, queue_size=100)
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)
        self.pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100)
        rospy.sleep(1)

    def getClosestPoint(self,x, y, laneID, distance):
        if laneID == 1:
            radius = 1.36
            return self.find_next_point(x, y, distance, radius, laneID)

        elif laneID == 2:
            radius = 1.68
            return self.find_next_point(x, y, distance, radius, laneID)

    def find_next_point(self, x, y, distance, radius, laneID):
        center_x = 1.84
        center_y = 2.15
        if x >= center_x and x <= 4.16:
            if y < center_y:
                if laneID == 1:
                    if x + distance <= 4.16:
                        point = (x+distance, 0.79)
                    else:
                        point = self.find_next_point(4.17, 0.79, x + distance - 4.17, radius, laneID)
                elif laneID == 2:
                    if x + distance <= 4.16:
                        point = (x+distance, 0.47)
                    else:
                        point = self.find_next_point(4.17, 0.47, x + distance - 4.17, radius, laneID)
            else:
                if laneID == 1:
                    if x - distance >= 1.84:
                        point = (x-distance, 3.51)
                    else:
                        point = self.find_next_point(1.83, 3.51, 1.83 - (x - distance), radius, laneID)
                elif laneID == 2:
                    if x - distance >= 1.84:
                        point = (x-distance, 3.83)
                    else:
                        point = self.find_next_point(1.83, 3.83, 1.83 - (x - distance), radius, laneID)
        else:
            if x < center_x:
                new_x = center_x + radius * ((x-center_x)/float(sqrt(((x-center_x)**2) + ((y-center_y)**2))))
                new_y = center_y + radius * ((y-center_y)/float(sqrt(((x-center_x)**2) + ((y-center_y)**2))))
                point = self.solve_equation(new_x, new_y, center_x, center_y, distance, radius, 1)

            else:
                center_x_two = 4.16
                new_x = center_x_two + radius * ((x-center_x_two)/float(sqrt(((x-center_x_two)**2) + ((y-center_y)**2))))
                new_y = center_y + radius * ((y-center_y)/float(sqrt(((x-center_x_two)**2) + ((y-center_y)**2))))
                point = self.solve_equation(new_x, new_y, center_x_two, center_y, distance, radius, 2)
        return point

    def solve_equation(self, new_x, new_y, center_x, center_y,distance, radius, semicircle):
        x, y = symbols('x y')
        eq1 = Eq(((x-new_x)**2)+((y-new_y)**2)-(distance**2))
        eq2 = Eq(((x-center_x)**2)+((y-center_y)**2)-(radius**2))
        prediction = solve((eq1, eq2),(x, y))
        answer = 0
        distances = []
        answers = []
        if semicircle == 1:
            for point in prediction:
                if (float(str(point[0]))<center_x and float(str(point[1]))< new_y):
                    point_x = float(str(point[0]))
                    point_y = float(str(point[1]))
                    answers.append((point_x, point_y))
                    c_P = self.find_nearest_point(point_x, point_y)
                    d = self.find_distance(point_x, point_y, c_P)
                    distances.append(d)
        else:
            for point in prediction:
                if (float(str(point[0]))>center_x and float(str(point[1]))> new_y):
                    point_x = float(str(point[0]))
                    point_y = float(str(point[1]))
                    answers.append((point_x, point_y))
                    c_P = self.find_nearest_point(point_x, point_y)
                    d = self.find_distance(point_x, point_y, c_P)
                    distances.append(d)
        if len(distances) > 1:
            if distances[0]<distances[1]:
                return answers[0]
            else:
                return answers[1]
        elif len(distances) == 1:
            return answers[0]
        else:
            if semicircle == 1:
                if radius == 1.36:
                    d = self.find_distance(new_x, new_y, (1.84, 0.79))
                    return self.find_next_point(1.84, 0.79, distance - d, radius, 1)
                elif radius == 1.68:
                    d = self.find_distance(new_x, new_y, (1.84, 0.47))
                    return self.find_next_point(1.84, 0.47, distance - d, radius, 2)
            else:
                if radius == 1.36:
                    d = self.find_distance(new_x, new_y, (4.16, 3.51))
                    return self.find_next_point(4.16, 3.51, distance - d, radius, 1)
                elif radius == 1.68:
                    d = self.find_distance(new_x, new_y, (4.16, 3.83))
                    return self.find_next_point(4.16, 3.83, distance - d, radius, 2)


    def find_nearest_point(self, x, y):
        radius = 1.20
        if x >= 1.84 and x <= 4.16:
            if y < 2.15:
                point = (x, 0.95)
            else:
                point = (x, 3.35)
        else:
            if x < 1.84:
                new_x = 1.84 + radius * ((x-1.84)/float(sqrt(((x-1.84)**2) + ((y-2.15)**2))))
                new_y = 2.15 + radius * ((y-2.15)/float(sqrt(((x-1.84)**2) + ((y-2.15)**2))))
                point = (new_x, new_y)
            else:
                new_x = 4.16 + radius * ((x-4.16)/float(sqrt(((x-4.16)**2) + ((y-2.15)**2))))
                new_y = 2.15 + radius * ((y-2.15)/float(sqrt(((x-4.16)**2) + ((y-2.15)**2))))
                point = (new_x, new_y)
        return point

    def find_distance(self, x, y, point_on_track):
        distance = (sqrt(((point_on_track[0]-x)**2) + ((point_on_track[1]-y)**2)))
        return distance

    def callback(self,data):
        global msg
        msg = data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        print("x:, y:")
        print(x,y)
        laneID = 1
        point = self.getClosestPoint(x, y, laneID, 0.5)
        print("predicted point:" + str(point))
        angle = self.test(point, laneID)
        print("angle: " + str(angle))
        drive_angle = self.mappingFunction(angle)
        print(drive_angle)
        self.pub_steering.publish(drive_angle)
        self.pub_speed.publish(120)

    def test(self, point, laneID):
        global msg
    	euler = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        pitch = euler[1]
    	yaw = euler[2]
    	print (yaw)
    	x = msg.pose.pose.position.x
    	y = msg.pose.pose.position.y
    	headingVec = np.array([np.cos(yaw)*np.cos(pitch) , np.sin(yaw)*np.cos(pitch)])
    	print("headingVec")
    	print(headingVec)
    	pointingVec = (point[0] - x, point[1]-y)
    	pointingVec = np.array(pointingVec)
    	print("pointingVec")
    	print(pointingVec)
        angle_deg = self.angle_between(headingVec,pointingVec)
        if x >= 1.84 and x <= 4.16:
            if y < 2.15:
                if laneID == 1:
                    if y < 0.79:
                        angle = -angle_deg
                    else:
                        angle = angle_deg
                else:
                    if y < 0.47:
                        angle = -angle_deg
                    else:
                        angle = angle_deg
            else:
                if laneID == 1:
                    if y < 3.51:
                        angle = angle_deg
                    else:
                        angle = -angle_deg
                else:
                    if y < 3.83:
                        angle = angle_deg
                    else:
                        angle = -angle_deg
        else:
            if x < 1.84:
                if self.is_inside(x, y, laneID, 1):
                    angle = angle_deg
                else:
                    angle = -angle_deg
            else:
                if self.is_inside(x, y, laneID, 2):
                    angle = angle_deg
                else:
                    angle = -angle_deg

    	return angle

    def is_inside(self,x ,y, laneID, semicircle):
        if semicircle == 1:
            x_center = 1.84
        else:
            x_center = 4.16
        y_center = 2.15
        if laneID == 1:
            radius = 1.36
        else:
            radius = 1.68
        if self.find_distance(x, y, (x_center, y_center)) <= radius:
            return True
        else:
            return False

    def angle_between(self,v1, v2):
        v1_len = sqrt(np.dot(v1, v1))
        v2_len = sqrt(np.dot(v2, v2))
        angle_rad = np.arccos(np.dot(v1, v2)/(v1_len * v2_len))
    	return angle_rad*180/np.pi

    def mappingFunction(self, angle):
    	Angles = collections.namedtuple('Angles', ['original', 'actual'])
    	anglesList = []
    	temp1 = Angles(0.0, actual = 90.0)
    	temp2 = Angles(160.0, actual = 179.0)
    	temp3 = Angles(106.67, actual = 150.0)
    	temp4 = Angles(53.335, actual = 130.0)
    	temp5 = Angles(-53.335, actual = 60.0)
    	temp6 = Angles(-106.67, actual = 30.0)
    	temp7 = Angles(-160.0, actual = 0.0)

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
        z=np.polyfit(x_vals, y_vals,3)
        f=np.poly1d(z)
        y = np.polyval(f,angle)
        return y

def main(args):
    rospy.init_node('find_predicted_point')
    ic = find_predicted_point()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
    main(sys.argv)
