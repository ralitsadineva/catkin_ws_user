#!/usr/bin/env python
from __future__ import division
import numpy as np
import math
import tf
import sys
import rospy
import collections
from math import sqrt, atan2
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

global msg, laneID
CAL_FACTOR = 0
headfilter = np.pi/3
laneID = 1
UPPER = 1.95
LOWER = 4.10
CENTER = 2.19
UPPER_CIRCLE_CENTER = np.array([UPPER, CENTER])
LOWER_CIRCLE_CENTER = np.array([LOWER, CENTER])

def getLaneBorders(laneID):
    if laneID == 1 : # inner
        return 0.87,3.51,1.24  #left,right,radius
    elif laneID == 2 :
        return 0.55,3.83,1.58

def getClosestPoint(point, laneID):
    xp = point[0]
    yp = point[1]
    point = np.array(point)
    seg = getSegment(xp,yp)
    left,right,radius = getLaneBorders(laneID)

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

def lookahead(point, laneID, distance):
    seg = getSegment(point[0],point[1])
    restOnCurrentSeg = getrest(point[0],point[1],laneID,distance)

    if seg == "middle_right":
        if restOnCurrentSeg >= distance :
            return lookaheadl(point, laneID, distance)
        else :
            return lookaheadc((UPPER,getLaneBorders(laneID)[1]),laneID,(distance - restOnCurrentSeg))
    elif seg == "middle_left":
        if restOnCurrentSeg >= distance :
            return lookaheadl(point, laneID, distance)
        else :
            return lookaheadc((LOWER,getLaneBorders(laneID)[0]),laneID,(distance - restOnCurrentSeg))
    elif seg == "upper":
        if restOnCurrentSeg >= distance :
            return lookaheadc(point, laneID, distance)
        else :
            return lookaheadl((UPPER,getLaneBorders(laneID)[0]),laneID,(distance - restOnCurrentSeg))
    elif seg == "lower":
        if restOnCurrentSeg >= distance :
            return lookaheadc(point, laneID, distance)
        else :
            return lookaheadl((LOWER,getLaneBorders(laneID)[1]),laneID,(distance - restOnCurrentSeg))

def lookaheadc(point, laneID, distance):
    closest =  getClosestPoint(point, laneID)
    xc = closest[0]
    yc = closest[1]
    if  getSegment (point[0],point[1]) == "upper" : CIRCLE_CENTER = UPPER_CIRCLE_CENTER
    if  getSegment (point[0],point[1]) == "lower" : CIRCLE_CENTER = LOWER_CIRCLE_CENTER
    radius = getLaneBorders(laneID)[2]
    angle = atan2(yc - CIRCLE_CENTER[1], xc - CIRCLE_CENTER[0])
    angle = angle + (distance/radius)
    xd = CIRCLE_CENTER[0] + radius * np.cos(angle);
    yd = CIRCLE_CENTER[1] + radius * np.sin(angle);
    return (xd,yd)

def lookaheadl(point, laneID, distance):
    closest =  getClosestPoint(point, laneID)
    xc = closest[0]
    yc = closest[1]
    if  yc < CENTER : factor = 1
    if  yc >= CENTER  : factor = -1
    return ((xc + (factor * distance)),yc)

def getSegment(x,y):
    global LaneID
    if x <= UPPER :
        return "upper"
    elif (x >= UPPER) and (x < LOWER):
        if y < CENTER :
            return "middle_left"
        else:
            return "middle_right"
    else:
        return "lower"

def getrest(x,y,laneID,distance):
    segment = getSegment(x,y)
    left,right,radius = getLaneBorders(laneID)
    if segment == "upper" : #upper circle
        Vend = (UPPER - UPPER_CIRCLE_CENTER [0] ,left - UPPER_CIRCLE_CENTER[1])
        Vstart = (x - UPPER_CIRCLE_CENTER [0] ,y - UPPER_CIRCLE_CENTER[1])
        theta = angle_between_rad(Vstart,Vend)
        rest = theta * radius
        return rest
    elif segment == "lower" :
        Vend = (LOWER - LOWER_CIRCLE_CENTER [0] , right - LOWER_CIRCLE_CENTER [1])
        Vstart = (x - LOWER_CIRCLE_CENTER [0] ,y - LOWER_CIRCLE_CENTER [1])
        theta = angle_between_rad(Vstart,Vend)
        rest = theta * radius
        return rest
    elif segment == "middle_left" :
        return LOWER - x
    elif segment == "middle_right" :
        return x - UPPER

def angle_between_rad(v1, v2):
    v1_len = sqrt(np.dot(v1, v1))
    v2_len = sqrt(np.dot(v2, v2))
    angle_rad = np.arccos(np.dot(v1, v2)/(v1_len * v2_len))
    return angle_rad

def angle_between(v1, v2):
    v1_len = sqrt(np.dot(v1, v1))
    v2_len = sqrt(np.dot(v2, v2))
    angle_rad = np.arccos(np.dot(v1, v2)/(v1_len * v2_len))
    return angle_rad*180/np.pi

def find_distance(x, y, last_point_x, last_point_y):
    distance = (sqrt(((last_point_x-x)**2) + ((last_point_y-y)**2)))
    return distance

def poltocartGlobal(angles,diss,x_car,y_car,theta):
    X = []
    Y = []
    cos = np.cos(theta)
    sin = np.sin(theta)
    tra_matrix = np.array([[cos,-1*sin,x_car], [sin,cos,y_car], [0,0,1]])

    for i in range(len(angles)):
        x = diss[i] * np.cos(angles[i])
        y = diss[i] * np.sin(angles[i])
        global_cor = np.matmul(tra_matrix , np.array([x,y,1]))
        x_global , y_global = global_cor[0] , global_cor[1]
        X.append(x_global)
        Y.append(y_global)
    return X,Y

def getPointsOnLane(points,LaneId):
    PointsOnLane = []
    for point in points :
        closest_point = getClosestPoint((point[0],point[1]),LaneId)
        Distance = find_distance(point[0],point[1],closest_point[0],closest_point[1])
        if Distance <= 0.15:
            PointsOnLane.append(point)
    return PointsOnLane

def getObstacle(x, y, points_on_lane):
    distances = []
    for point in points_on_lane:
        dist = find_distance(point[0], point[1], x, y)
        distances.append(dist)
    if len(distances) == 0:
        return None
    return min(distances)

def calc_angle(point, laneID, yaw, pitch):
        global msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        headingVec = np.array([np.cos(yaw)*np.cos(pitch) , np.sin(yaw)*np.cos(pitch)])
        pointingVec = (point[0] - x, point[1]-y)
        pointingVec = np.array(pointingVec)
        angle_deg = angle_between(headingVec,pointingVec)
        if x >= 1.95 and x <= 4.10:
            if y < 2.17:
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
            angle = -angle_deg
        return angle

def mappingFunction(angle):
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

def scan_callback(msg):
    global scan_msg
    scan_msg = msg

def callback(data):
    global msg,counter ,scan_msg ,laneID
    DISTANCE_TO_OBSTACLE = 1.5
    DISTANCE_TO_SWITCH = 1
    msg = data
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    print(laneID)
    euler = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    pitch = euler[1]
    yaw = euler[2]

    angles = []
    ranges = np.array(scan_msg.ranges)
    mask = np.array(ranges<DISTANCE_TO_OBSTACLE)
    for i in range(len(ranges)):
        angles.append(scan_msg.angle_min + i*(scan_msg.angle_increment))
    anglesnp = np.array(angles)
    mask_angle =  np.array(np.cos(anglesnp)<-0.5)
    X,Y = np.array(poltocartGlobal(angles,ranges,x,y,yaw-np.pi)) #1.57 = 90 degree (lidar calb).
    mask1 = mask  & mask_angle
    X = np.array(X[mask1])
    Y = np.array(Y[mask1])
    points = np.column_stack((X, Y))
    PointsOnLane = getPointsOnLane(points, laneID)
    obstacle =getObstacle(x, y, PointsOnLane)
    if obstacle is not None : print("obstacle:" + str(obstacle))
    if obstacle != None and obstacle <= DISTANCE_TO_SWITCH:
        if laneID == 1:
            print("checking for obstacle on lane 2")
            PointsOnLane = getPointsOnLane(points, 2)
            print("points:" + str(points))
            print("points on lane" + str(PointsOnLane))
            obstacle = getObstacle(x, y, PointsOnLane)
            if obstacle != None and obstacle <= DISTANCE_TO_SWITCH:
                print("I should stop")
                pub_speed.publish(0)
                return
            else:
                laneID = 2
        else:
            print("checking for obstacle on lane 1")
            PointsOnLane = getPointsOnLane(points, 1)
            obstacle = getObstacle(x, y, PointsOnLane)
            if obstacle != None and obstacle <= DISTANCE_TO_SWITCH:
                print("I should stop")
                pub_speed.publish(0)
                return
            else:
                laneID = 1

    position = lookahead((x, y), laneID, 0.4)
    angle = calc_angle(position, laneID, yaw, pitch)
    drive_angle = mappingFunction(angle)
    if drive_angle < 0.0:
        drive_angle = 0.0
    pub_steering.publish(drive_angle)
    pub_speed.publish(400)


pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100)
pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100)
sub_scan = rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)
sub = rospy.Subscriber("/localization/odom/12", Odometry, callback, queue_size=1)

def main(args):
    rospy.init_node('driveeee')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
    main(sys.argv)
