#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, ParamValue
from sensor_msgs.msg import BatteryState
from mslquad.srv import Emergency
from mslquad.srv import EmergencyLand
#from Emergency.srv import Emergency

#boundary node, stripped to interface with new flight controller
#Line 37: Need name 

import numpy as np

class Safety:

	def __init__(self):
		#Sleep if launched from launchfile
		rospy.init_node('safety', anonymous = True)


		#initialize variables of interest
		self.refresh_rate = 10
		self.propagation_time = 0.25 #if colliding with room in 1/4 second (using linear d(t+dt) = d(t) + v*dt)
		self.bounds = np.array(rospy.get_param('room_boundaries')) #first row - lower bounds; second row - upper bounds
		self.landing_pose = Pose()
		self.position = np.zeros((3,1))
		self.velocity = np.zeros((3,1))
		self.battery_level = None

		self.quad_name = rospy.get_namespace()


		#Initialize "safety services": landing, stopping, etc
		rospy.loginfo("Waiting for services")
		self.land_service = rospy.ServiceProxy('/' + self.quad_name +'/emergency_land', EmergencyLand, persistent = True) 
		rospy.loginfo("Services set.")	

		self.state_sub = rospy.Subscriber('mavros/local_position/pose',PoseStamped,self.agentPoseCB)
		self.battery_sub = rospy.Subscriber('mavros/battery',BatteryState,self.agentBatteryVoltage)
		self.velocity_sub = rospy.Subscriber('mavros/local_position/velocity',TwistStamped,self.agentVelocity)

		while not self.position.any():
			rospy.loginfo("Waiting for pose")
			rospy.sleep(0.1)
		rospy.loginfo("Pose get!")

		while self.battery_level is None:
			rospy.loginfo("Waiting for state")
			rospy.sleep(0.1)

		#intialize position after boundary is trespassed
		self.landing_pose.position.x = self.position[0]
		self.landing_pose.position.y = self.position[1]
		self.landing_pose.position.z = 0

	def agentPoseCB(self,msg):
		agentPose = msg.pose.position
		self.position[0] = agentPose.x
		self.position[1] = agentPose.y
		self.position[2] = agentPose.z

	def agentBatteryVoltage(self,msg):
		self.battery_level = msg.voltage

	def agentVelocity(self,msg):
		vel = msg.twist.linear
		self.velocity[0] = vel.x 
		self.velocity[1] = vel.y
		self.velocity[2] = vel.z 

	def collectVelocities(self,msg):
		experimental = False
		if self.all_velocities.shape[1] <= quad_idx: #number of cols < number of quads
			num_cols = self.all_velocities.shape[1]
			self.all_velocities = np.hstack((self.all_velocities, np.zeros((3, quad_idx-num_cols+1)))) #resize-as-you-go

		vel = msg.twist.linear
		self.all_velocities[:,quad_idx] = np.array([vel.x, vel.y, vel.z])

		effective_pos = self.all_positions + self.propagation_time*self.all_velocities #Propagate position by propagation_time
		quad_norms = np.square(np.linalg.norm(effective_pos,axis=0))

	def run(self):
		rate = rospy.Rate(self.refresh_rate)
		debug_text = ""
		while not rospy.is_shutdown():
			self.landing_pose.position.x = self.position[0]
			self.landing_pose.position.y = self.position[1]

			if self.battery_level < 11.0:
				print(self.battery_level)
				rospy.loginfo("Battery Voltage too low")
				#self.land_service(True, self.landing_pose,None)
				#self.land_service(self.landing_pose)

			propagated_position = self.position + self.propagation_time*self.velocity

			lower_bounds_broken = (propagated_position < self.bounds[0])
			upper_bounds_broken = (propagated_position > self.bounds[1])

			if lower_bounds_broken.any() or upper_bounds_broken.any():
				self.state_sub.unregister()
				debug_text = np.array2string(self.position)
				rospy.loginfo(debug_text)
				move_away = 0.1 * (lower_bounds_broken - upper_bounds_broken) #scootch away from broken boundary
				self.landing_pose.position.x += move_away[0]
				self.landing_pose.position.y += move_away[1]
				#self.land_service(True,self.landing_pose,None)
				self.land_service(self.landing_pose)

			rate.sleep()


if __name__ == '__main__':
	safety = Safety()
	safety.run()
