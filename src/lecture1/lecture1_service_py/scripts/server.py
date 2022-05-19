#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def callback(req):
    return SetBoolResponse(req.data, "???")

def server():
    rospy.init_node("server")
    s = rospy.Service("service", SetBool, callback)
    rospy.spin()

if __name__ == "__main__":
    server()
