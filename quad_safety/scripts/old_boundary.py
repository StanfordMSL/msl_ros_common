#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandHome, ParamSet
from mavros_msgs.msg import HomePosition, State, ParamValue
from sensor_msgs.msg import NavSatFix

import numpy as np

#TODO: Check in force_land to see if landed, reset all variables if true, return ctrl to user (unarmed)

class Safety:

	def __init__(self):
		#Sleep if launched from launchfile
		rospy.init_node('safety', anonymous = True)
		self.chatter = rospy.Publisher('safety_debug', String, queue_size=10)


		#initialize variables of interest
		self.refresh_rate = 10
		self.isLanding = False
		self.landed_boundary = 0.1 #check for "landed" drone
		self.bounds = np.array(rospy.get_param('room_boundaries')) #first row - lower bounds; second row - upper bounds


		#Initialize "safety services": landing, stopping, etc
		rospy.wait_for_service('/quad0/mavros/cmd/land')
		rospy.wait_for_service('/quad0/mavros/set_mode')
		rospy.wait_for_service('/quad0/mavros/cmd/set_home')
		rospy.wait_for_service('/quad0/mavros/param/set')
		self.modeService = rospy.ServiceProxy('quad0/mavros/set_mode', SetMode)
		self.land_service = rospy.ServiceProxy('/quad0/mavros/cmd/land', CommandTOL)
		self.home_service = rospy.ServiceProxy('quad0/mavros/cmd/set_home', CommandHome)
		self.altitude_setter = rospy.ServiceProxy('quad0/mavros/param/set',ParamSet)

		#Set altitude for forced landing return service

		self.set_return_altitudes()

		self.arm(0)
		self.position = None
		self.lat_long = None

		self.state_sub = rospy.Subscriber('quad0/mavros/local_position/pose',PoseStamped,self.agentPoseCB)
		self.global_position = rospy.Subscriber('quad0/mavros/global_position/raw/fix', NavSatFix, self.agentLatLong)


		while self.position is None:
			rospy.loginfo("Waiting for pose")
			rospy.sleep(0.1)
		rospy.loginfo("Pose get!")
		self.chatter.publish("Pose get!")

		while self.lat_long is None:
			rospy.loginfo("Waiting for GPS")
			rospy.sleep(0.1)
		rospy.loginfo("GPS get!")
		self.chatter.publish("GPS get!")

		self.arm(1) #sets home coordinate now

	def agentPoseCB(self,msg):
		agentPose = msg.pose
		self.position = np.array([agentPose.position.x, agentPose.position.y, agentPose.position.z])

	def agentLatLong(self,msg):
		if self.lat_long is None:
			self.lat_long = np.array([msg.latitude, msg.longitude])

	def set_return_altitudes(self):
		rospy.loginfo("Setting return and descend altitude")	
		altitude_response = self.altitude_setter('RTL_RETURN_ALT',ParamValue(0,0.0))

		while True:
			altitude_response = self.altitude_setter('RTL_RETURN_ALT',ParamValue(0,0.0))
			rospy.loginfo(altitude_response)
			rospy.sleep(1)
			if altitude_response.success:
				break

		while True:
			altitude_response = self.altitude_setter('RTL_DESCEND_ALT',ParamValue(0,1.0))
			rospy.loginfo(altitude_response)
			rospy.sleep(1)
			if altitude_response.success:
				break

	def arm(self,mode):
		if (mode == 0):
			try:
				modeResponse = self.modeService(80, '')
				rospy.loginfo(modeResponse)
			except rospy.ServiceException as e:
				rospy.loginfo("Clearance: mode switch failed: %s" %e)
		elif (mode == 1):
			try:	            
				modeResponse = self.modeService(0, 'OFFBOARD')
				homeResponse = self.home_service(1,0.0,0.0,0.0)
				rospy.loginfo(modeResponse)
				rospy.loginfo(homeResponse)
			except rospy.ServiceException as e:
				rospy.loginfo("Clearance: mode switch failed: %s" %e)
		elif (mode == 2):
			try:
				modeResponse = self.modeService(0, 'AUTO.RTL')
				rospy.loginfo(modeResponse)
			except rospy.ServiceException as e:
				rospy.loginfo("Clearance: mode switch failed: %s" %e)
		rospy.sleep(.1) #debounce


	def run(self):
		rate = rospy.Rate(self.refresh_rate)
		debug_text = ""
		while not rospy.is_shutdown():
			if self.isLanding and self.position[2] < self.landed_boundary:
				exit()

			if ((self.position < self.bounds[0]).any() or (self.position > self.bounds[1]).any()):
				debug_text = np.array2string(self.position)
				rospy.loginfo(debug_text)
				self.chatter.publish(debug_text)
				if not self.isLanding:
					self.isLanding = True
					self.force_land()

			rate.sleep()

	def force_land(self):
		self.arm(2) #should probably change this to not "2"
		rospy.loginfo(np.array2string(self.lat_long))
		self.chatter.publish(np.array2string(self.lat_long))

if __name__ == '__main__':
	safety = Safety()
	safety.run()
