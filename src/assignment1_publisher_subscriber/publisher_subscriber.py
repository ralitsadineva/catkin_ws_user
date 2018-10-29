#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

def callback(raw_msg):
    msg = "I heard: " + str(raw_msg)
    publisher.publish(msg)

rospy.init_node("publisher_subscriber")

publisher = rospy.Publisher("/assignment1_publisher_subscriber", String)
rospy.Subscriber("/yaw", Float32, callback)

rospy.spin()
