#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, ParamValue
from sensor_msgs.msg import BatteryState
from Emergency.srv import Emergency

#boundary node, stripped to interface with new flight controller
#Line 37: Need name 

import numpy as np

class Safety:

	def __init__(self):
		#Sleep if launched from launchfile
		rospy.init_node('safety', anonymous = True)


		#initialize variables of interest
		self.refresh_rate = 10
		self.bounds = np.array(rospy.get_param('room_boundaries')) #first row - lower bounds; second row - upper bounds
		self.landing_pose = Pose()
		self.position = None
		self.battery_level = None

		self.quad_name = rospy.get_namespace()


		#Initialize "safety services": landing, stopping, etc
		rospy.loginfo("Waiting for services")
		self.land_service = rospy.ServiceProxy('/' + self.quad_name +'/emergency', Emergency, persistent = True) #NEED SERVICE PROXY NAME AND TYPE (I assumed type = Pose)
		rospy.loginfo("Services set.")	

		self.state_sub = rospy.Subscriber('mavros/local_position/pose',PoseStamped,self.agentPoseCB)
		self.battery_sub = rospy.Subscriber('mavros/battery',BatteryState,self.agentBatteryVoltage)

		while self.position is None:
			rospy.loginfo("Waiting for pose")
			rospy.sleep(0.1)
		rospy.loginfo("Pose get!")

		#return position after boundary is trespassed
		self.landing_pose.position.x = self.position[0]
		self.landing_pose.position.y = self.position[1]
		self.landing_pose.position.z = 0

	def agentPoseCB(self,msg):
		agentPose = msg.pose
		self.position = np.array([agentPose.position.x, agentPose.position.y, agentPose.position.z])

	def agentBatteryVoltage(self,msg):
		self.battery_level = msg.voltage

	def run(self):
		rate = rospy.Rate(self.refresh_rate)
		debug_text = ""
		while not rospy.is_shutdown():
			if self.battery_level < 11.0:
				rospy.loginfo("Battery Voltage too low")
				self.land_service(True, self.landing_pose,None)

			if ((self.position < self.bounds[0]).any() or (self.position > self.bounds[1]).any()):
				debug_text = np.array2string(self.position)
				rospy.loginfo(debug_text)
				self.land_service(True,self.landing_pose,None)

			rate.sleep()


if __name__ == '__main__':
	safety = Safety()
	safety.run()
