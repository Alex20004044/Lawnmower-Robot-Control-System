#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from SystemValues import *
from controllers_srvs.srv import SetMode, SetModeRequest, SetModeResponse

class ModeBase:


	def __init__(self):
		self.isActive = False

	def finish(self):
		self.isActive = False

	def start(self):
		self.isActive = True

	def get_mode_index(self):
		raise Exeption("Mode index is not defined")


class ModeManual(ModeBase):


	def __init__(self):
		ModeBase.__init__(self)
		self.p = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		self.s = rospy.Subscriber(SystemValues.teleop_input_name, Twist, self.teleop_input_callback)

	def finish(self):
		ModeBase.finish(self)
		msg = Twist()
		msg.linear.x = 0
		msg.linear.y = 0
		msg.linear.z = 0

		msg.angular.x = 0
		msg.angular.y = 0
		msg.angular.z = 0
		self.p.publish(msg)


	def teleop_input_callback(self, msg):
		if(self.isActive):
			self.p.publish(msg)

	def get_mode_index(self):
		return SetModeRequest.MANUAL

class ModePause(ModeBase):


	def __init__(self):
		ModeBase.__init__(self)
		self.p = rospy.Publisher("cmd_vel", Twist, queue_size=1)

	def start(self):
		ModeBase.start(self)
		msg = Twist()
		msg.linear.x = 0
		msg.linear.y = 0
		msg.linear.z = 0

		msg.angular.x = 0
		msg.angular.y = 0
		msg.angular.z = 0
		self.p.publish(msg)


	def get_mode_index(self):
		return SetModeRequest.PAUSE