#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from lecture2_dr_py.cfg import DRConfig

def callback(config, level):
    rospy.loginfo("""{int_param} {double_param} {str_param} {bool_param} {enum_param}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dr_node")
    srv = Server(DRConfig, callback)
    rospy.spin()

