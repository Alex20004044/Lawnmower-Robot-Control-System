#!/usr/bin/env python3
import rospy
from lecture1_msgs.msg import TimeUInt32

def publisher():
    rospy.init_node("publisher")
    p = rospy.Publisher("topic", TimeUInt32, queue_size=5)
    rate = rospy.Rate(10)
    c = 0
    while not rospy.is_shutdown():
        p.publish(rospy.Time.now(), c)
        c += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass


