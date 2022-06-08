#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from SystemValues import *
from controllers_srvs.srv import SetMode, SetModeRequest, SetModeResponse,\
    SetupCommands, SetupCommandsRequest, SetupCommandsResponse     #MowCommands, MowCommandsRequest, MowCommandsResposce,\
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, PointStamped
from state_controller import *
import json
import tf2_ros
import actionlib
import cv2
import numpy as np
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
import matplotlib.pyplot as plt

class PathDrawer:

    def __init__(self):
        rospy.init_node("path_drawer")
        self.tf_buffer = tf2_ros.Buffer()


        tf2_ros.TransformListener(self.tf_buffer)

        image = np.zeros((SystemValues.map_width, SystemValues.map_height, 3), np.uint8)
        displacement = SystemValues.robotRadius / SystemValues.map_resolution
        #rospy.on_shutdown(self.save())
        while not rospy.is_shutdown():
            pos = self.get_current_position()
            if pos is not None:
                pixPosX = (pos[0] - SystemValues.map_origin[0]) / SystemValues.map_resolution
                pixPosY = (pos[1] - SystemValues.map_origin[1]) / SystemValues.map_resolution
                pixPos1 = (int(pixPosX - displacement), int(pixPosY - displacement))
                pixPos2 = (int(pixPosX + displacement), int(pixPosY + displacement))

                cv2.rectangle(image, pixPos1, pixPos2,(255,0,0),-1)
            #rospy.sleep(rospy.Duration(0.1))
        rospy.logwarn("Create image of Coverage")
        cv2.imwrite(SystemValues.dataPath + SystemValues.coverageImageName, image)

    # def save(self):
    #     rospy.logwarn("Create image of Coverage")
    #     cv2.imwrite(SystemValues.dataPath + SystemValues.coverageImageName, self.image)

    def get_current_position(self):
        try:
            pos = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time()).transform.translation
            return (pos.x, pos.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            return None

if __name__ == '__main__':
    try:
        rospy.logwarn("PathDrawer is Activated")
        PathDrawer()
    except rospy.ROSInterruptException:
        pass