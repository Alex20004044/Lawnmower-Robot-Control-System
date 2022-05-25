#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from SystemValues import *
from controllers_srvs.srv import SetMode, SetModeRequest, SetModeResponse
from state_controller import *


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

    def start(self):
        ModeBase.start(self)

    def finish(self):
        ModeBase.finish(self)

    def get_mode_index(self):
        return SetModeRequest.SETUP


class ModeMow(ModeBase):

    def __init__(self):
        ModeBase.__init__(self)

    def start(self):
        ModeBase.start(self)

    def finish(self):
        ModeBase.finish(self)

    def get_mode_index(self):
        return SetModeRequest.MOW

    def on_low_battery(self):
        SystemValues.StateControllerManager.set_mode(SetModeRequest.RETURN)


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

