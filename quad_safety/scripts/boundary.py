#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, ParamValue
from sensor_msgs.msg import BatteryState
from subprocess import call

import numpy as np

class Safety:

	def __init__(self):
		#Sleep if launched from launchfile
		rospy.init_node('safety', anonymous = True)
		self.chatter = rospy.Publisher('safety_debug', String, queue_size=10)


		#initialize variables of interest
		self.refresh_rate = 10
		self.bounds = np.array(rospy.get_param('room_boundaries')) #first row - lower bounds; second row - upper bounds
		self.landing_pose = PoseStamped()
		self.position = None
		self.nodes_to_kill = set()
		self.battery_level = None


		#Initialize "safety services": landing, stopping, etc
		rospy.loginfo("Waiting for services")
		rospy.wait_for_service('mavros/cmd/land')
		rospy.wait_for_service('mavros/set_mode')
		rospy.wait_for_service('mavros/cmd/arming')
		self.armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		self.modeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
		rospy.loginfo("Services set.")	

		self.state_sub = rospy.Subscriber('mavros/local_position/pose',PoseStamped,self.agentPoseCB)
		self.battery_sub = rospy.Subscriber('mavros/battery',BatteryState,self.agentBatteryVoltage)

		while self.position is None:
			rospy.loginfo("Waiting for pose")
			rospy.sleep(0.1)
		rospy.loginfo("Pose get!")
		self.chatter.publish("Pose get!")

		#return position after boundary is trespassed
		self.landing_pose.pose.position.x = self.position[0]
		self.landing_pose.pose.position.y = self.position[1]
		self.landing_pose.pose.position.z = 0

		self.local_pos_overwrite = rospy.Publisher('mavros/setpoint_position/local', PoseStamped,queue_size=10)
		self.setpoint_killer = rospy.Subscriber('mavros/setpoint_position/local', PoseStamped, self.log_publishing_nodes)

	def agentPoseCB(self,msg):
		agentPose = msg.pose
		self.position = np.array([agentPose.position.x, agentPose.position.y, agentPose.position.z])

	def agentBatteryVoltage(self,msg):
		self.battery_level = msg.voltage

	def log_publishing_nodes(self,msg):
		msg_header = msg._connection_header
		self.nodes_to_kill.add(msg_header['callerid'])

	def setMode(self,mode):
		if (mode == "Land"):
			try:
				modeResponse = self.modeService(0, 'AUTO.MISSION')
				rospy.loginfo(modeResponse)
				return 1
			except rospy.ServiceException as e:
				rospy.loginfo("Clearance: mode switch failed: %s" %e)
				return 0
		if (mode == "Off"):
			try:
				armResponse = self.armService(False)
				rospy.loginfo(armResponse)
				return 1
			except rospy.ServiceException as e:
				rospy.loginfo("Clearance: mode switch failed: %s" %e)
				return 0
		rospy.sleep(.1) #debounce

	def kill_other_setters(self):
		l = list(self.nodes_to_kill)
		for node in l:
			call(["rosnode kill "+node], shell=True)


	def run(self):
		rate = rospy.Rate(self.refresh_rate)
		debug_text = ""
		while not rospy.is_shutdown():
			if self.battery_level < 11.0:
				rospy.loginfo("Battery Voltage too low")
				self.force_land()

			if ((self.position < self.bounds[0]).any() or (self.position > self.bounds[1]).any()):
				debug_text = np.array2string(self.position)
				rospy.loginfo(debug_text)
				self.chatter.publish(debug_text)
				self.force_land()

			rate.sleep()

	def force_land(self):
		self.kill_other_setters()
		for i in range(10):
			self.local_pos_overwrite.publish(self.landing_pose)	
		while not self.setMode("Land"):
			rospy.sleep(0.1)
		while (self.position[2] > 0.1):
			for i in range(10):
				self.local_pos_overwrite.publish(self.landing_pose)	
			rospy.loginfo(np.array2string(self.position))
			self.chatter.publish(np.array2string(self.position))
			rospy.sleep(0.1)
		while (True):
			self.local_pos_overwrite.publish(self.landing_pose)
			rospy.loginfo("Landing from here!")
			if self.setMode("Off"):
				break
			rospy.sleep(0.1)
		while (True):
			self.local_pos_overwrite.publish(self.landing_pose)
			rospy.loginfo("Quad is Landed")
			rospy.sleep(0.1)
		#exit()

if __name__ == '__main__':
	safety = Safety()
	safety.run()
