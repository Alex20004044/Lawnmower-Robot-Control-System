#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def publisher():
    rospy.init_node("publisher")
    p = rospy.Publisher("topic", Int32, queue_size=5)
    rate = rospy.Rate(10)
    c = 0
    while not rospy.is_shutdown():
        p.publish(c)
        c += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass


