#!/usr/bin/env python
from __future__ import division
import numpy as np
import math
import tf
import sys
import rospy
import collections
from math import sqrt, atan2
from sympy import symbols, solve, Eq
from scipy.optimize import fsolve
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

global msg, laneID, scan_msg
laneID = 1
class find_predicted_point:

    def __init__(self):
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.sub = rospy.Subscriber("/localization/odom/12", Odometry, self.callback, queue_size=1)
        self.pub_stop_start = rospy.Publisher("manual_control/stop_start", Int16, queue_size=100)
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)
        self.pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100)
        rospy.sleep(1)

    def getClosestPoint(self, point, laneID):
        DISTANCE_TO_OBSTACLE = 1.5
        UPPER = 1.90
        LOWER = 4.10
        CENTER = 2.19
        UPPER_CIRCLE_CENTER = np.array([UPPER, CENTER])
        LOWER_CIRCLE_CENTER = np.array([LOWER, CENTER])

    	xp = point[0]
    	yp = point[1]
    	point = np.array(point)
    	seg = self.getSegment(xp,yp)
    	left,right,radius = self.getLaneBorders(laneID)

    	if seg == "middle_left":
    		return (xp,left)
    	elif seg == "middle_right":
    		return (xp,right)
    	elif seg == "upper":
    		dirVector = point - UPPER_CIRCLE_CENTER
    		if np.array_equal(dirVector,np.zeros(2)):
    			normDirVector = np.array([-1,0])
    		else:
    			normDirVector = dirVector/np.linalg.norm(dirVector)
    		return normDirVector*radius + UPPER_CIRCLE_CENTER
    	elif seg == "lower":
    		dirVector = point - LOWER_CIRCLE_CENTER
    		if np.array_equal(dirVector,np.zeros(2)):
    			normDirVector = np.array([1,0])
    		else:
    			normDirVector = dirVector/np.linalg.norm(dirVector)
    		return normDirVector*radius + LOWER_CIRCLE_CENTER

    def lookahead(self, point, laneID, distance):
        DISTANCE_TO_OBSTACLE = 1.5
        UPPER = 1.90
        LOWER = 4.10
        CENTER = 2.19
        UPPER_CIRCLE_CENTER = np.array([UPPER, CENTER])
        LOWER_CIRCLE_CENTER = np.array([LOWER, CENTER])

    	seg = self.getSegment(point[0],point[1])
    	restOnCurrentSeg = self.getrest(point[0],point[1],laneID,distance)

    	if seg == "middle_right":

    		if restOnCurrentSeg >= distance :
    			return self.lookaheadl(point, laneID, distance)
    		else :
    			return self.lookaheadc((UPPER,self.getLaneBorders(laneID)[1]),laneID,(distance - restOnCurrentSeg))

    	elif seg == "middle_left":

    		if restOnCurrentSeg >= distance :
    			return self.lookaheadl(point, laneID, distance)
    		else :
    			return self.lookaheadc((LOWER,self.getLaneBorders(laneID)[0]),laneID,(distance - restOnCurrentSeg))

    	elif seg == "upper":

    		if restOnCurrentSeg >= distance :
    			return self.lookaheadc(point, laneID, distance)
    		else :
    			return self.lookaheadl((UPPER,self.getLaneBorders(laneID)[0]),laneID,(distance - restOnCurrentSeg))

    	elif seg == "lower":

    		if restOnCurrentSeg >= distance :
    			return self.lookaheadc(point, laneID, distance)
    		else :
    			return self.lookaheadl((LOWER,self.getLaneBorders(laneID)[1]),laneID,(distance - restOnCurrentSeg))


    def lookaheadc(self,point, laneID, distance):
        DISTANCE_TO_OBSTACLE = 1.5
        UPPER = 1.90
        LOWER = 4.10
        CENTER = 2.19
        UPPER_CIRCLE_CENTER = np.array([UPPER, CENTER])
        LOWER_CIRCLE_CENTER = np.array([LOWER, CENTER])

    	closest =  self.getClosestPoint(point, laneID)
    	xc = closest[0]
    	yc = closest[1]
    	if  self.getSegment (point[0],point[1]) == "upper" : CIRCLE_CENTER = UPPER_CIRCLE_CENTER
    	if  self.getSegment (point[0],point[1]) == "lower" : CIRCLE_CENTER = LOWER_CIRCLE_CENTER
    	radius = self.getLaneBorders(laneID)[2]
    	angle = atan2(yc - CIRCLE_CENTER[1], xc - CIRCLE_CENTER[0])
    	angle = angle + (distance/radius)
    	xd = CIRCLE_CENTER[0] + radius * np.cos(angle);
    	yd = CIRCLE_CENTER[1] + radius * np.sin(angle);
    	return (xd,yd)

    def lookaheadl(self,point, laneID, distance):
        DISTANCE_TO_OBSTACLE = 1.5
        UPPER = 1.90
        LOWER = 4.10
        CENTER = 2.17
        UPPER_CIRCLE_CENTER = np.array([UPPER, CENTER])
        LOWER_CIRCLE_CENTER = np.array([LOWER, CENTER])

    	closest =  self.getClosestPoint(point, laneID)
    	xc = closest[0]
    	yc = closest[1]
    	if  yc < CENTER : factor = 1
    	if  yc >= CENTER  : factor = -1

    	return ((xc + (factor * distance)),yc)

    def getSegment(self,x,y):
        UPPER = 1.90
        LOWER = 4.10
        CENTER = 2.19
    	if x <= UPPER :
    		return "upper"
    	elif (x >= UPPER) and (x < LOWER):
    		if y < CENTER :
    			return "middle_left"
    		else:
    			return "middle_right"
    	else:
    		return "lower"

    def getLaneBorders(self,laneID):
    	if laneID == 1 : # inner
    		return 0.87,3.51,1.24  #left,right,radius
    	elif laneID == 2 :
    		return 0.55,3.83,1.58

    def getrest(self,x,y,laneID,distance):
        DISTANCE_TO_OBSTACLE = 1.5
        UPPER = 1.90
        LOWER = 4.10
        CENTER = 2.19
        UPPER_CIRCLE_CENTER = np.array([UPPER, CENTER])
        LOWER_CIRCLE_CENTER = np.array([LOWER, CENTER])

    	segment = self.getSegment(x,y)
    	left,right,radius = self.getLaneBorders(laneID)
    	if segment == "upper" : #upper circle
    		Vend = (UPPER - UPPER_CIRCLE_CENTER [0] ,left - UPPER_CIRCLE_CENTER[1])
    		Vstart = (x - UPPER_CIRCLE_CENTER [0] ,y - UPPER_CIRCLE_CENTER[1])
    		theta = self.angle_between_rad(Vstart,Vend)
    		rest = theta * radius
    		return rest

    	elif segment == "lower" :
    		Vend = (LOWER - LOWER_CIRCLE_CENTER [0] , right - LOWER_CIRCLE_CENTER [1])
    		Vstart = (x - LOWER_CIRCLE_CENTER [0] ,y - LOWER_CIRCLE_CENTER [1])
    		theta = self.angle_between_rad(Vstart,Vend)
    		rest = theta * radius
    		return rest

    	elif segment == "middle_left" :

    		return LOWER - x

    	elif segment == "middle_right" :

    		return x - UPPER

    def angle_between_rad(self,v1, v2):
    	v1_len = sqrt(np.dot(v1, v1))
    	v2_len = sqrt(np.dot(v2, v2))
    	angle_rad = np.arccos(np.dot(v1, v2)/(v1_len * v2_len))
    	return angle_rad

    def find_distance(self,x, y, last_point_x, last_point_y):
        distance = (sqrt(((last_point_x-x)**2) + ((last_point_y-y)**2)))
        return distance

    def scan_callback(self,msg):
        global scan_msg
        scan_msg = msg

    def poltocartGlobal(self,angles,diss,x_car,y_car,theta):
    	X = []
    	Y = []
    	cos = np.cos(theta)
    	sin = np.sin(theta)
    	tra_matrix = np.array([[cos,-1*sin,x_car],
    							[sin,cos,y_car],
    							[0,0,1]])

    	for i in range(len(angles)):
    		x = diss[i] * np.cos(angles[i])
    		y = diss[i] * np.sin(angles[i])
    		global_cor = np.matmul(tra_matrix , np.array([x,y,1]))
    		x_global , y_global = global_cor[0] , global_cor[1]
    		X.append(x_global)
    		Y.append(y_global)

    	return X,Y

    def getPointsOnLane(self,points,LaneID):
    	PointsOnLane = []
    	for point in points :
    		closest_point = self.getClosestPoint((point[0],point[1]),LaneID)
    		Distance = self.find_distance(point[0],point[1],closest_point[0],closest_point[1])
    		if Distance <= 0.15: PointsOnLane.append(point)
    	return PointsOnLane

    def getObstacle(self,x, y, points_on_lane):
    	distances = []
    	for point in points_on_lane:
    		dist = self.find_distance(point[0], point[1], x, y)
    		distances.append(dist)
    	if len(distances) == 0:
    		return None
    	nearest_dist = min(distances)
    	return nearest_dist

    def callback(self,data):
        DISTANCE_TO_OBSTACLE = 1.5
        global msg, laneID, scan_msg
        msg = data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        print("x:, y:")
        print(x,y)
    	euler = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        pitch = euler[1]
    	yaw = euler[2]
    	print (yaw)
        # laneID = 1
    	angles = []
    	mask = np.array(scan_msg.ranges<1)
        #print("mask:" + str(mask))
    	ranges = np.array(scan_msg.ranges)
        #print("ranges" + str(ranges))
    	mask = np.array(ranges<DISTANCE_TO_OBSTACLE)
        #print("mask2:" + str(mask))
    	for i in range(len(ranges)):
    		angles.append(scan_msg.angle_min + i*(scan_msg.angle_increment))
    	X,Y = np.array(self.poltocartGlobal(angles,ranges,x,y,yaw-np.pi/2)) #1.57 = 90 degree (lidar calb).
        #print("X,Y:" + str(X) + str(Y))
    	X = np.array(X[mask])
        #print("X:" + str(X))
    	Y = np.array(Y[mask])
        #print("Y:" + str(Y))
    	points = np.column_stack((X, Y))
        points = np.transpose(points)
        print(points)
    	PointsOnLane = self.getPointsOnLane(points, laneID)
        #print(PointsOnLane)
    	obstacle = self.getObstacle(x, y, PointsOnLane)
        print("obstacle:" + str(obstacle))
        if obstacle != None and obstacle <= 0.5:
            if laneID == 1:
                print("checking for obstacle on lane 2")
                PointsOnLane = self.getPointsOnLane(points, 2)
                print("points:" + str(points))
                print("points on lane" + str(PointsOnLane))
             	obstacle = self.getObstacle(x, y, PointsOnLane)
                if obstacle != None and obstacle <= 0.5:
                    print("I should stop")
                    self.pub_speed.publish(0)
                    return
                else:
                    laneID = 2
            else:
                print("checking for obstacle on lane 1")
                PointsOnLane = self.getPointsOnLane(points, 1)
                obstacle = self.getObstacle(x, y, PointsOnLane)
                if obstacle != None and obstacle <= 0.5:
                    print("I should stop")
                    self.pub_speed.publish(0)
                    return
                else:
                    laneID = 1
        position = self.lookahead((x, y), laneID, 0.2)
        #print("predicted point:" + str(position))
        angle = self.test(position, laneID, yaw, pitch)
        #print("angle: " + str(angle))
        drive_angle = self.mappingFunction(angle)
        #print(drive_angle)
        self.pub_steering.publish(drive_angle)
        self.pub_speed.publish(150)

    def test(self, point, laneID, yaw, pitch):
        global msg
    	# euler = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        # pitch = euler[1]
    	# yaw = euler[2]
    	# print (yaw)
    	x = msg.pose.pose.position.x
    	y = msg.pose.pose.position.y
    	headingVec = np.array([np.cos(yaw)*np.cos(pitch) , np.sin(yaw)*np.cos(pitch)])
    	#print("headingVec")
    	#print(headingVec)
    	pointingVec = (point[0] - x, point[1]-y)
    	pointingVec = np.array(pointingVec)
    	#print("pointingVec")
    	#print(pointingVec)
        angle_deg = self.angle_between(headingVec,pointingVec)
        if x >= 1.84 and x <= 4.16:
            if y < 2.15:
                if laneID == 1:
                    if y < 0.87:
                        angle = -angle_deg
                    else:
                        angle = angle_deg
                else:
                    if y < 0.55:
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
                # if self.is_inside(x, y, laneID, 1):
                #     angle = angle_deg
                # else:
                angle = 1.5*-angle_deg
            else:
                # if self.is_inside(x, y, laneID, 2):
                #     angle = angle_deg
                # else:
                angle = -angle_deg

    	return angle

    def is_inside(self,x ,y, laneID, semicircle):
        if semicircle == 1:
            x_center = 1.84
        else:
            x_center = 4.16
        y_center = 2.15
        if laneID == 1:
            radius = 1.26
        else:
            radius = 1.58
        if self.find_distance(x, y, x_center, y_center) <= radius:
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
    	temp1 = Angles(-2.0, actual = 90.0)
    	temp2 = Angles(70.0, actual = 179.0)
    	# temp3 = Angles(106.67, actual = 150.0)
    	# temp4 = Angles(53.335, actual = 130.0)
    	# temp5 = Angles(-53.335, actual = 60.0)
    	# temp6 = Angles(-106.67, actual = 30.0)
    	temp7 = Angles(-70.0, actual = 0.0)

    	anglesList.append(temp1)
    	anglesList.append(temp2)
    	# anglesList.append(temp3)
    	# anglesList.append(temp4)
    	# anglesList.append(temp5)
    	# anglesList.append(temp6)
    	anglesList.append(temp7)

        x_vals = []
        y_vals = []
        for x in anglesList:
            x_vals.append(x.original)
            y_vals.append(x.actual)
        z=np.polyfit(x_vals, y_vals,2)
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
