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
        self.isLocked = False
        rospy.init_node("state_controller")
        instance = self;
        #Service SetMode
        #Service SetLock

        self.set_lock_service = rospy.Service("set_lock", SetBool, self.set_lock_callback)
        self.set_blades_service = rospy.Service("set_blades", SetBool, self.set_blades_callback)
        self.set_mode_service = rospy.Service("set_mode", SetMode, self.set_mode_callback)
        self.publisher_current_state = rospy.Publisher(SystemValues.current_state, String, queue_size=5)

        self.modePause = ModePause()
        self.modeManual = ModeManual()

        self.modeEmergency = ModeEmergency()

        self.isBladesActive = False
        self.set_state(self.modePause)

        #self.set_mode(SetModeRequest.PAUSE)
        rospy.spin()

       #Subscribe to action or topic of command_input
        

    #def on_new_command(self, com):
        #check is it alarm command (high priority in any state)
        #if true => run special state
        #else
        #current_state.input(com)
       # pass

    def set_lock_callback(self, req):
        self.set_lock(req.data)
        return SetBoolResponse(True, "OK")

    def set_lock(self, isLock):
        self.isLocked = isLock
        if self.isLocked:
            self.set_state(self.modeEmergency)
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
        if(isActive and (self.current_state.get_mode_index() == SetModeRequest.MANUAL or self.current_state.get_mode_index() == SetModeRequest.MOW)):
            self.isBladesActive = isActive
            self._log_system_info()
            return True
        elif(not isActive):
            self.isBladesActive = isActive
            self._log_system_info()
            return True
        else:
            return False

    def get_mode(self, mode):
        if(mode == SetModeRequest.MANUAL):
            return self.modeManual
        else:
            return self.modePause

    def set_state(self, state):
        self.current_state = state
        self._log_system_info()


    def _log_system_info(self):
        info = "Current state: " + str(self.current_state.get_mode_index()) + ' | '
        info = info + "IsLocked: " + str(self.isLocked) + ' | '
        info = info + "IsBladesActive: " + str(self.isBladesActive)
        msg = String(info)
        self.publisher_current_state.publish(msg)


if __name__ == '__main__':
    try:
        StateController()
    except rospy.ROSInterruptException:
        pass




        
        