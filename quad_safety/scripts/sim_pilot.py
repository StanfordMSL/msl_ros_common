#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, ParamValue
from sensor_msgs.msg import BatteryState
from subprocess import call

import numpy as np

#TODO: Deal with quad offsets (the 0 issue)
class Sim_Pilot:

	def __init__(self):
		#Sleep if launched from launchfile
		rospy.init_node('Sim_Pilot', anonymous = True)
		self.chatter = rospy.Publisher('Sim_Pilot_Debug', String, queue_size=10)

		self.refresh_rate = 30
		self.quad_pos = None
		self.quad_goal = None
		self.max_speed = 1
		self.velocity_goal = Twist()
		self.position_goal = PoseStamped()
		self.position_goal.pose.position.x = 0.0
		self.position_goal.pose.position.y = 0.0
		self.position_goal.pose.position.z = 1.0


		self.goal_sub = rospy.Subscriber('command/goal', PoseStamped, self.extractGoal)
		self.pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped,self.agentPoseCB)
		self.vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
		self.pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

		rospy.loginfo("Getting quad position")
		while self.quad_pos is None:
			self.pos_pub.publish(self.position_goal)
			rospy.sleep(0.05)

		rospy.loginfo("Getting quad goal")
		while self.quad_goal is None:
			self.pos_pub.publish(self.position_goal)
			rospy.sleep(0.05)


	def agentPoseCB(self,msg):
		agentPose = msg.pose
		self.quad_pos = np.array([agentPose.position.x, agentPose.position.y, agentPose.position.z])

	def extractGoal(self, msg):
		quad_goal = msg.pose
		self.quad_goal = np.array([quad_goal.position.x, quad_goal.position.y, quad_goal.position.z])

	def run(self):
		rate = rospy.Rate(self.refresh_rate)
		debug_text = ""
		while not rospy.is_shutdown():
			pos_diff = self.quad_goal - self.quad_pos
			pos_diff = (pos_diff/np.linalg.norm(pos_diff))*min(self.max_speed, np.linalg.norm(pos_diff))
			self.velocity_goal.linear.x = pos_diff[0]
			self.velocity_goal.linear.y = pos_diff[1]
			self.velocity_goal.linear.z = pos_diff[2]

			rospy.loginfo(self.velocity_goal)

			self.vel_pub.publish(self.velocity_goal)

			rate.sleep()


if __name__ == '__main__':
	sim_pilot = Sim_Pilot()
	sim_pilot.run()