#!/usr/bin/env python3
import rospy

if __name__ == "__main__":
    rospy.init_node("node")
    rospy.loginfo(str(rospy.get_param("~param", "default_value")))

