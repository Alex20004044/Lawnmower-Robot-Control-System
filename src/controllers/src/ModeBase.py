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
import matplotlib.pyplot as plt



class ModeBase:

    def __init__(self):
        self.isActive = False
        self.publisher_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.zoneDict = {}
        self.width = SystemValues.map_width
        self.height = SystemValues.map_height
        self.resolution = SystemValues.map_resolution
        self.origin = SystemValues.map_origin

        self.mapPublisher = rospy.Publisher(SystemValues.roadMapTopicName, OccupancyGrid, queue_size=1, latch=True)
        self.moveBaseActionClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.logwarn("Mode Init: " + str(self.get_mode_index()))

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
        self.moveBaseActionClient.cancel_all_goals()

    def reset_velocity(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0

        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.publisher_cmd_vel.publish(msg)

    def publish_road_map(self):
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

        greenZone = self.create_zone_image(SetupCommandsRequest.ENABLE_GREEN_MODE)
        cv2.imwrite(SystemValues.dataPath + SystemValues.greenZoneMapName, greenZone)

        yellowZone = self.create_zone_image(SetupCommandsRequest.ENABLE_YELLOW_MODE)
        cv2.imwrite(SystemValues.dataPath + SystemValues.yellowZoneMapName, yellowZone)

        redZone = self.create_zone_image(SetupCommandsRequest.ENABLE_RED_MODE)
        cv2.imwrite(SystemValues.dataPath + SystemValues.redZoneMapName, redZone)

        roadZone = cv2.add(greenZone, yellowZone)
        roadZone = cv2.subtract(roadZone, redZone)
        cv2.imwrite(SystemValues.dataPath + SystemValues.roadZoneMapName, roadZone)

        roadMap = OccupancyGrid()

        roadMap.header.frame_id = SystemValues.roadMapTopicName

        roadMap.info.map_load_time = rospy.Time()
        roadMap.info.resolution = self.resolution;
        roadMap.info.width = self.width;
        roadMap.info.height = self.height;
        roadMap.info.origin.position.x = self.origin[0];
        roadMap.info.origin.position.y = self.origin[1];
        roadMap.info.origin.position.z = 0;
        roadMap.info.origin.orientation.x = 0;
        roadMap.info.origin.orientation.y = 0;
        roadMap.info.origin.orientation.z = 0;

        roadZone = (255 - roadZone) // 255 * 100
        roadMap.data = np.reshape(roadZone, -1).tolist()

        rospy.logerr(roadZone.dtype)
        self.mapPublisher.publish(roadMap)

        pass

    def publish_mow_map(self):
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

        greenZone = self.create_zone_image(SetupCommandsRequest.ENABLE_GREEN_MODE)
        cv2.imwrite(SystemValues.dataPath + SystemValues.greenZoneMapName, greenZone)

        yellowZone = self.create_zone_image(SetupCommandsRequest.ENABLE_YELLOW_MODE)
        cv2.imwrite(SystemValues.dataPath + SystemValues.yellowZoneMapName, yellowZone)

        redZone = self.create_zone_image(SetupCommandsRequest.ENABLE_RED_MODE)
        cv2.imwrite(SystemValues.dataPath + SystemValues.redZoneMapName, redZone)

        mowZone = cv2.subtract(greenZone, yellowZone)
        mowZone = cv2.subtract(mowZone, redZone)
        cv2.imwrite(SystemValues.dataPath + SystemValues.mowZoneMapName, mowZone)

        mowMap = OccupancyGrid()

        #!ROAD_MAP_COORDINATE!!!
        mowMap.header.frame_id = SystemValues.roadMapTopicName

        mowMap.info.map_load_time = rospy.Time()
        mowMap.info.resolution = self.resolution;
        mowMap.info.width = self.width;
        mowMap.info.height = self.height;
        mowMap.info.origin.position.x = self.origin[0];
        mowMap.info.origin.position.y = self.origin[1];
        mowMap.info.origin.position.z = 0;
        mowMap.info.origin.orientation.x = 0;
        mowMap.info.origin.orientation.y = 0;
        mowMap.info.origin.orientation.z = 0;

        mowZone = (255 - mowZone) // 255 * 100
        mowMap.data = np.reshape(mowZone, -1).tolist()

        self.mapPublisher.publish(mowMap)

        pass

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
        self.clickedPointsPublisher = rospy.Publisher(SystemValues.clickedPointsTopicName, PointStamped, queue_size=50)
        self.publish_mow_map()

    def start(self):
        ModeBase.start(self)

        self.publish_mow_map()
        self.publish_cpp_goal()


    def finish(self):
        ModeBase.finish(self)

    def get_mode_index(self):
        return SetModeRequest.MOW

    def on_low_battery(self):
        SystemValues.StateControllerManager.set_mode(SetModeRequest.RETURN)

    def publish_cpp_goal(self):
        xmin = SystemValues.mapxmin
        xmax = SystemValues.mapxmax
        ymin = SystemValues.mapymin
        ymax = SystemValues.mapymax

        rate = rospy.Rate(10)
        p = PointStamped()
        p.header.frame_id = SystemValues.roadMapTopicName
        p.point.x = xmin
        p.point.y = ymin

        rospy.logerr(p)
        self.clickedPointsPublisher.publish(p)
        p.point.x = xmax
        p.point.y = ymin
        rate.sleep()
        self.clickedPointsPublisher.publish(p)
        p.point.x = xmax
        p.point.y = ymax
        rate.sleep()
        self.clickedPointsPublisher.publish(p)
        p.point.x = xmin
        p.point.y = ymax
        rate.sleep()
        self.clickedPointsPublisher.publish(p)

        p.point.x = xmin
        p.point.y = ymin
        rate.sleep()
        self.clickedPointsPublisher.publish(p)
        pass


    @staticmethod
    def map(value, low, high, low2, high2):
        percentage = (value - low) / (high - low)
        return low2 + (high2 - low2) * percentage

class ModeReturn(ModeBase):

    def __init__(self):
        ModeBase.__init__(self)

    def start(self):
        ModeBase.start(self)
        try:
            with open(SystemValues.dataPath + SystemValues.basePointName) as f:
                t = json.loads(f.read())
        except Exception as e:
            rospy.logwarn("File loading error: " + str(e))
        if (t is not None):
            self.publish_road_map()
            self.basePoint = t
            goal_msg = MoveBaseGoal(
            PoseStamped(Header(0, rospy.Time(), SystemValues.roadMapTopicName), Pose(Point(self.basePoint[0], self.basePoint[1], 0), Quaternion(0, 0, 0, 1))))
            self.moveBaseActionClient.send_goal(goal_msg, done_cb=self.on_reach_base_point)
        else:
            rospy.logwarn("Base point is not set. Cancel RETURN" + str(e))
            SystemValues.StateControllerManager.set_mode(SetModeRequest.PAUSE)

    def on_reach_base_point(self, x, y):
        rospy.log("Base point is reached")
        SystemValues.StateControllerManager.set_mode(SetModeRequest.PAUSE)
        pass

    def finish(self):
        ModeBase.finish(self)
        self.moveBaseActionClient.cancel_goal()

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

