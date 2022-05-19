#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool

def client(req):
    rospy.wait_for_service("service")
    try:
        sp = rospy.ServiceProxy("service", SetBool)
        resp = sp(req)
        print("bool success: " + str(resp.success))
        print("string message: " + resp.message)
    except rospy.ServiceException as e:
        print("Service call failed: " + str(e))

if __name__ == "__main__":
    print("Calling service: bool data = True...")
    client(True)
    print("Calling service: bool data = False...")
    client(False)

