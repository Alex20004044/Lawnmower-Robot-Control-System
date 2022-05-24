#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from controllers_srvs.srv import SetMode, SetModeRequest, SetModeResponse
from std_msgs.msg import String

from geometry_msgs.msg import Twist
from SystemValues import *
from ModeBase import *


class StateController:

    def __init__(self):
        rospy.init_node("state_controller")

        self.set_lock_service = rospy.Service("set_lock", SetBool, self.set_lock_callback)
        self.set_blades_service = rospy.Service("set_blades", SetBool, self.set_blades_callback)
        self.set_battery_service = rospy.Service("set_battery", SetBool, self.set_battery_callback)
        self.set_mode_service = rospy.Service("set_mode", SetMode, self.set_mode_callback)

        self.publisher_current_state = rospy.Publisher(SystemValues.current_state, String, queue_size=5)

        self.modePause = ModePause()
        self.modeManual = ModeManual()
        self.modeSetup = ModeSetup()
        self.modeMow = ModeMow()
        self.modeReturn = ModeReturn()

        self.modeEmergency = ModeEmergency()

        self.isLocked = False
        self.isBladesActive = False
        self.isLowBattery = False
        self.set_state(self.modePause)


    def set_lock_callback(self, req):
        self.set_lock(req.data)
        return SetBoolResponse(True, "OK")

    def set_lock(self, isLock):
        self.isLocked = isLock
        if self.isLocked:
            self.set_state(self.modeEmergency)
            self.set_blades(False)
        else:
            self.set_state(self.modePause)

    def set_mode_callback(self, req):
        return SetModeResponse(self.set_mode(req.mode))

    def set_mode(self, mode):
        if(self.isLocked):
            return False

        if(mode != self.current_state.get_mode_index()):
            self.current_state.finish()
            self.set_state(self.get_mode(mode))
            self.current_state.start()
            return True
        else:
            return False

    def set_blades_callback(self, isActive):
        return SetBoolResponse(self.set_blades(isActive), "OK")

    def set_blades(self, isActive):
        if(self.isLocked):
            self.isBladesActive = False
            # publish blade state
            self._log_system_info()
            return False

        if(isActive and (self.current_state.get_mode_index() == SetModeRequest.MANUAL or self.current_state.get_mode_index() == SetModeRequest.MOW)):
            self.isBladesActive = isActive
            #publish blade state
            self._log_system_info()
            return True
        elif(not isActive):
            self.isBladesActive = False
            # publish blade state
            self._log_system_info()
            return True
        else:
            return False

    def set_battery_callback(self, isLowBattery):
        return SetBoolResponse(self.set_battery(isLowBattery), "OK")

    def set_battery(self, isLowBattery):
        self.isLowBattery = isLowBattery
        if(isLowBattery):
            self.current_state.on_low_battery()
        else:
            self.current_state.on_high_battery()
        self._log_system_info()
        return True

    def get_mode(self, mode):
        if(mode == SetModeRequest.MANUAL):
            return self.modeManual
        elif(mode == SetModeRequest.SETUP):
            return self.modeSetup
        elif(mode == SetModeRequest.MOW):
            return self.modeMow
        elif(mode == SetModeRequest.RETURN):
            return self.modeReturn
        else:
            return self.modePause

    def set_state(self, state):
        self.current_state = state
        self._log_system_info()


    def _log_system_info(self):
        info = "Current state: " + str(self.current_state.get_mode_index()) + ' | '
        info = info + "IsLocked: " + str(self.isLocked) + ' | '
        info = info + "IsBladesActive: " + str(self.isBladesActive) + ' | '
        info = info + "IsLowBattery: " + str(self.isLowBattery)
        msg = String(info)
        self.publisher_current_state.publish(msg)


if __name__ == '__main__':
    try:
        SystemValues.StateControllerManager = StateController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




        
        