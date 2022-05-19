#!/usr/bin/env python3
import rospy
from lecture1_srvs.srv import StrCat, StrCatResponse

def callback(req):
    return StrCatResponse(req.first + req.second)

def server():
    rospy.init_node("server")
    s = rospy.Service("service", StrCat, callback)
    rospy.spin()

if __name__ == "__main__":
    server()
