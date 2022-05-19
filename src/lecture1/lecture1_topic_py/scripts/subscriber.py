#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def callback(msg):
    rospy.loginfo("%d", msg.data)

def subscriber():
    rospy.init_node("subscriber", anonymous=True)
    rospy.Subscriber("topic", Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()

