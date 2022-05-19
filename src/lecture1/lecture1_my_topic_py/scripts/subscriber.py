#!/usr/bin/env python3
import rospy
from lecture1_msgs.msg import TimeUInt32

def callback(msg):
    rospy.loginfo("%i.%i : %d", msg.stamp.secs, msg.stamp.nsecs, msg.data)

def subscriber():
    rospy.init_node("subscriber", anonymous=True)
    rospy.Subscriber("topic", TimeUInt32, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()

