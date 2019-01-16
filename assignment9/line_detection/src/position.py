#!/usr/bin/env python
import rospy
import numpy as np
import sys
from nav_msgs.msg import Odometry
from math import sqrt

class position:
    global all_distances, all_sqr_d

    def __init__(self):
        global all_distances, all_sqr_d
        self.sub = rospy.Subscriber("/localization/odom/12", Odometry, self.callback, queue_size=1)
        all_distances = []
        all_sqr_d = []


    def callback(self,msg):
        global all_distances
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        point_on_track = self.find_nearest_point(x, y)
        distance = self.find_distance(x, y, point_on_track)
        print("Distance:" + str(distance))
        all_distances.append(abs(distance))
        average_d = sum(all_distances)/len(all_distances)
        print("Average Distance:" + str(average_d))
        squared_d = self.find_squared_d(x, y, point_on_track)
        all_sqr_d.append(squared_d)
        average_sqr_d = sum(all_sqr_d)/len(all_sqr_d)
        print("Average squared distance:" + str(average_sqr_d))

    def find_distance(self, x, y, point_on_track):
        distance = sqrt(((point_on_track[0]-x)**2) + ((point_on_track[1]-y)**2))
        return distance

    def find_squared_d(self, x, y, point_on_track):
        distance = ((point_on_track[0]-x)**2) + ((point_on_track[1]-y)**2)
        return distance

    def find_nearest_point(self, x, y):
        if x >= 1.84 and x <= 4.16:
            if y < 2.15:
                point = (x, 0.95)
            else:
                point = (x, 3.35)
        else:
            if x < 1.84:
                new_x = 1.84 + 1.20 * ((x-1.84)/float(sqrt(((x-1.84)**2) + ((y-2.15)**2))))
                new_y = 2.15 + 1.20 * ((y-2.15)/float(sqrt(((x-1.84)**2) + ((y-2.15)**2))))
                point = (new_x, new_y)
            else:
                new_x = 4.16 + 1.20 * ((x-4.16)/float(sqrt(((x-4.16)**2) + ((y-2.15)**2))))
                new_y = 2.15 + 1.20 * ((y-2.15)/float(sqrt(((x-4.16)**2) + ((y-2.15)**2))))
                point = (new_x, new_y)
        return point

def main(args):
    rospy.init_node('position', anonymous=True)
    ic = position()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
