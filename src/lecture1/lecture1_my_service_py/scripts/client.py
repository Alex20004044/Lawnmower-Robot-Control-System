#!/usr/bin/env python3
import sys
import rospy
from lecture1_srvs.srv import StrCat

def client(first, second):
    rospy.wait_for_service("service")
    try:
        sp = rospy.ServiceProxy("service", StrCat)
        resp = sp(first, second)
        print("Result: \"" + resp.result + "\"")
    except rospy.ServiceException as e:
        print("Service call failed: " + str(e))

if __name__ == "__main__":
    first = sys.argv[1] if len(sys.argv) > 1 else ""
    second = sys.argv[2] if len(sys.argv) > 2 else ""
    print("Calling service for \"" + first + "\" and \"" + second + "\"...")
    client(first, second)

