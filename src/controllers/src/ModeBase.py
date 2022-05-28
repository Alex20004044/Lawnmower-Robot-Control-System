#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from SystemValues import *
from controllers_srvs.srv import SetMode, SetModeRequest, SetModeResponse,\
    SetupCommands, SetupCommandsRequest, SetupCommandsResponse     #MowCommands, MowCommandsRequest, MowCommandsResposce,\
from state_controller import *
import json
import tf2_ros
import cv2
import numpy as np
import matplotlib.pyplot as plt


class ModeBase:

    def __init__(self):
        self.isActive = False
        self.publisher_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def finish(self):
        self.isActive = False
        self.reset()

    def start(self):
        self.isActive = True
        self.reset()

    def get_mode_index(self):
        raise Exception("Mode index is not defined")

    def on_low_battery(self):
        print("Low battery!")
    def on_high_battery(self):
        print("High battery!")

    def reset(self):
        self.reset_velocity()
        SystemValues.StateControllerManager.set_blades(False)

    def reset_velocity(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0

        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.publisher_cmd_vel.publish(msg)


class ModePause(ModeBase):

    def __init__(self):
        ModeBase.__init__(self)

    def get_mode_index(self):
        return SetModeRequest.PAUSE


class ModeManual(ModeBase):

    def __init__(self):
        ModeBase.__init__(self)
        self.s = rospy.Subscriber(SystemValues.teleop_input_name, Twist, self.teleop_input_callback)

    def finish(self):
        ModeBase.finish(self)

    def teleop_input_callback(self, msg):
        if (self.isActive):
            self.publisher_cmd_vel.publish(msg)

    def get_mode_index(self):
        return SetModeRequest.MANUAL


class ModeSetup(ModeBase):

    def __init__(self):
        ModeBase.__init__(self)
        self.s = rospy.Subscriber(SystemValues.teleop_input_name, Twist, self.teleop_input_callback)

        self.isCreateZoneMode = False
        self.setup_commands_service = rospy.Service(SystemValues.setup_command_service_name, SetupCommands, self.setup_commands_callback)
        self.zoneDict = {}
        self.basePoint = (0, 0)

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)


    def teleop_input_callback(self, msg):
        if (self.isActive):
            self.publisher_cmd_vel.publish(msg)

    def start(self):
        ModeBase.start(self)
        t = None
        try:
            with open(SystemValues.dataPath + SystemValues.dataZoneName) as f:
                t = json.loads(f.read())
        except Exception as e:
            rospy.logwarn("File loading error: " + str(e))
        if(t is not None):
            self.zoneDict = t
        else:
            self.zoneDict[str(SetupCommandsRequest.ENABLE_GREEN_MODE)] = []
            self.zoneDict[str(SetupCommandsRequest.ENABLE_YELLOW_MODE)] = []
            self.zoneDict[str(SetupCommandsRequest.ENABLE_RED_MODE)] = []

        t = None
        try:
            with open(SystemValues.dataPath + SystemValues.basePointName) as f:
                t = json.loads(f.read())
        except Exception as e:
            rospy.logwarn("File loading error: " + str(e))
        if (t is not None):
            self.basePoint = t
        else:
            self.basePoint = (0, 0)

    def finish(self):
        ModeBase.finish(self)
        with open(SystemValues.dataPath + SystemValues.dataZoneName, "w") as f:
            f.write(json.dumps(self.zoneDict))
        with open(SystemValues.dataPath + SystemValues.basePointName, "w") as f:
            f.write(json.dumps(self.basePoint))
        #zoneDict save

    def get_mode_index(self):
        return SetModeRequest.SETUP

    def setup_commands_callback(self, command):
        return SetupCommandsResponse(self.setup_commands(command.command))

    def setup_commands(self, command):
        if(not self.isActive):
            return "Current mode is not SetupMode"

        if(self.isCreateZoneMode):
            if(command == SetupCommandsRequest.EXIT_MODE):
                self.isCreateZoneMode = False
                return "Exit zone mode..."
            elif(command == SetupCommandsRequest.SET_POINT):
                #call setpoint
                return self.set_point()
            elif(command == SetupCommandsRequest.CREATE_ZONE):
                #createZone
                return self.create_zone()
            else:
                return "Incorrect command in CreateZoneMode: " + str(command)
        else:
            if(command == SetupCommandsRequest.ENABLE_GREEN_MODE):
                return self.enable_zone_mode(SetupCommandsRequest.ENABLE_GREEN_MODE)
            elif(command == SetupCommandsRequest.ENABLE_YELLOW_MODE):
                return self.enable_zone_mode(SetupCommandsRequest.ENABLE_YELLOW_MODE)
            elif(command == SetupCommandsRequest.ENABLE_RED_MODE):
                return self.enable_zone_mode(SetupCommandsRequest.ENABLE_RED_MODE)
            elif (command == SetupCommandsRequest.SET_BASE_POINT):
                return self.set_base_point()
            elif (command == SetupCommandsRequest.RESET_GREEN_ZONE):
                return self.reset_zone(SetupCommandsRequest.ENABLE_GREEN_MODE)
            elif (command == SetupCommandsRequest.RESET_YELLOW_ZONE):
                return self.reset_zone(SetupCommandsRequest.ENABLE_YELLOW_MODE)
            elif (command == SetupCommandsRequest.RESET_RED_ZONE):
                return self.reset_zone(SetupCommandsRequest.ENABLE_RED_MODE)
            elif (command == SetupCommandsRequest.RESET_BASE_POINT):
                return self.reset_base_point()
            else:
                return "Incorrect command in defaultMode: " + str(command)

    def correct_log(self):
        return "OK"

    def set_point(self):
        pos = self.get_current_position()
        if(pos is None):
            return "Position is not defined"
        else:
            self.zonePoints.append(pos)
            return "Point is set in " + str(pos)

    def create_zone(self):
        self.isCreateZoneMode = False
        if(len(self.zonePoints)<=2):
            return "Zone is not created. There are incorrect points count"
        else:
            self.zoneDict[str(self.currentZoneColorIndex)].append(self.zonePoints)
            return "Zone is created" + str(self.currentZoneColorIndex) + " in points: " + str(self.zonePoints)

    def enable_zone_mode(self, zoneColorIndex):
        self.currentZoneColorIndex = zoneColorIndex
        self.zonePoints = []

        self.isCreateZoneMode = True
        return "Create zone mode activated: " + str(zoneColorIndex)

    def set_base_point(self):
        pos = self.get_current_position()
        self.basePoint = pos
        return "Base point is set in: " + str(pos)

    def reset_zone(self, zoneColorIndex):
        return "Reset zone: " + str(zoneColorIndex)

    def reset_base_point(self):
        return "Reset base point"

    def get_current_position(self):
        try:
            pos = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time()).transform.translation
            return (pos.x, pos.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            return None


class ModeMow(ModeBase):

    def __init__(self):
        ModeBase.__init__(self)
        self.zoneDict = {}
        self.width = SystemValues.map_width
        self.height = SystemValues.map_height
        self.resolution = SystemValues.map_resolution
        self.origin = SystemValues.map_origin

    def start(self):
        ModeBase.start(self)
        try:
            with open(SystemValues.dataPath + SystemValues.dataZoneName) as f:
                t = json.loads(f.read())
        except Exception as e:
            rospy.logwarn("File loading error: " + str(e))
        if(t is not None):
            self.zoneDict = t
        else:
            SystemValues.StateControllerManager.set_mode(SetModeRequest.PAUSE)
            rospy.logwarn("Target zones not found: " + str(e))
        self.create_maps()


    def finish(self):
        ModeBase.finish(self)

    def get_mode_index(self):
        return SetModeRequest.MOW

    def on_low_battery(self):
        SystemValues.StateControllerManager.set_mode(SetModeRequest.RETURN)

    def create_maps(self):
        greenZone = self.create_zone_image(SetupCommandsRequest.ENABLE_GREEN_MODE)
        cv2.imwrite(SystemValues.dataPath + SystemValues.greenZoneMapName, greenZone)


    def create_zone_image(self, zoneColorIndex):
        image = np.zeros((self.width,self.height,3), np.uint8)

        print(self.zoneDict)
        polygons = self.zoneDict[str(zoneColorIndex)]

        print(polygons)
        #polygons = (polygons - self.origin) / self.resolution

        for polygon in polygons:
            npa = np.int_((np.array(polygon) - self.origin) / self.resolution)
            print(npa)
            cv2.fillPoly(
                image,
                [npa],
                color=(255,255,255),
            )
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


    @staticmethod
    def map(value, low, high, low2, high2):
        percentage = (value - low) / (high - low)
        return low2 + (high2 - low2) * percentage

class ModeReturn(ModeBase):

    def __init__(self):
        ModeBase.__init__(self)

    def start(self):
        ModeBase.start(self)

    def finish(self):
        ModeBase.finish(self)

    def get_mode_index(self):
        return SetModeRequest.RETURN


class ModeEmergency(ModeBase):

    def __init__(self):
        ModeBase.__init__(self)

    def start(self):
        ModeBase.start(self)

    def finish(self):
        ModeBase.finish(self)

    def get_mode_index(self):
        return SystemValues.emergency_code

