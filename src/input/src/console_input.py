#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def console_input():
	rospy.init_node("console_input")
	p = rospy.Publisher("command_input", String, queue_size=1)
	
	while not rospy.is_shutdown():
		inp = input('Enter command:')
		p.publish(inp)


if __name__ == '__main__':
    try:
        console_input()
    except rospy.ROSInterruptException:
        pass
