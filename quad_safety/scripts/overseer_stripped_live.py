#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, ParamValue
from mslquad.srv import Emergency

import numpy as np


class Overseer:

	def __init__(self):
		#Sleep if launched from launchfile
		rospy.init_node('Overseer', anonymous = True)
		self.chatter = rospy.Publisher('Overseer_debug', String, queue_size=10)


		#initialize variables of interest
		self.propagation_time = 0.7
		self.collision_threshold = 0.5 #will need to tune this
		self.velocity_gain = 4 #How strong velocity pushback acts on pair of quads (inversely proportional to distance to each other (can be found in force_move))
		self.refresh_rate = 10
		self.position = None
		self.quad_names = [] #list of quad names airborne (NOT SORTED)
		self.position_subscribers = [] #list of subscriber nodes (SAME ORDER AS QUAD_NAMES)
		self.velocity_subscribers = []
		self.emergency_handlers = [] #list of emergency service proxies (one per quad)
		self.all_positions = np.zeros((3,1))
		self.all_velocities = np.zeros((3,1))
		self.colliding_quads_and_waypoints = [] #triples of (quad_idx_1, quad_idx_2, midpoint) (NOT NAMES! NEED TO INDEX INTO QUAD_NAMES FOR THAT)


	def agentPoseCB(self,msg):
		agentPose = msg.pose
		self.position = np.array([agentPose.position.x, agentPose.position.y, agentPose.position.z])

	def setMode(self,mode,idx):
		if (mode == "Separate"):
			try:
				modeResponse = self.mode_services[idx](0, 'AUTO.MISSION')
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



	def collectPositions(self,msg,quad_idx):
		experimental = False
		if self.all_positions.shape[1] <= quad_idx: #number of cols < number of quads
			num_cols = self.all_positions.shape[1]
			self.all_positions = np.hstack((self.all_positions, np.zeros((3, quad_idx-num_cols+1)))) #resize-as-you-go

		pose = msg.pose
		self.all_positions[:,quad_idx] = np.array([pose.position.x, pose.position.y, pose.position.z])
	def collectVelocities(self,msg,quad_idx):
		experimental = False
		if self.all_velocities.shape[1] <= quad_idx: #number of cols < number of quads
			num_cols = self.all_velocities.shape[1]
			self.all_velocities = np.hstack((self.all_velocities, np.zeros((3, quad_idx-num_cols+1)))) #resize-as-you-go

		vel = msg.twist.linear
		self.all_velocities[:,quad_idx] = np.array([vel.x, vel.y, vel.z])

		effective_pos = self.all_positions + self.propagation_time*self.all_velocities #Propagate position by propagation_time
		quad_norms = np.square(np.linalg.norm(effective_pos,axis=0))
		num_quads = quad_norms.shape[0]
		for i in range(num_quads):
			if i == quad_idx:
				continue
			if abs(quad_norms[i] - quad_norms[quad_idx]) < self.collision_threshold and abs(quad_norms[i] + quad_norms[quad_idx] - 2*np.dot(effective_pos[:,i],effective_pos[:,quad_idx])) < self.collision_threshold:
				rospy.loginfo("Quads "+ self.quad_names[i] +" and "+ self.quad_names[quad_idx] +" are in collision")
				rospy.loginfo(self.all_positions[:,i])
				rospy.loginfo(self.all_positions[:,quad_idx])
				midpoint = self.find_midpoint([i,quad_idx])
				self.colliding_quads_and_waypoints.append([i,quad_idx,midpoint])

	def find_midpoint(self,colList):
		#eventually remove
		waypoints = [] #pair of waypoints (one for each colliding quad)

		i = colList[0]
		j = colList[1]
		pos1 = self.all_positions[:,i]
		pos2 = self.all_positions[:,j]
		midpoint = ((pos1 + pos2)/2.0)
		
		return midpoint


	def run(self):
		rate = rospy.Rate(self.refresh_rate)
		debug_text = ""
		all_topics_old = set()
		i = 1
		while not rospy.is_shutdown():

			#runs once per second
			if (i % self.refresh_rate == 0):
				all_topics = rospy.get_published_topics()
				all_topics = [topic[0].split('/') for topic in all_topics]
				quad_list = [topic[1] for topic in all_topics if len(topic) > 1]
				tmp_quad_names = set([name for name in quad_list if name.find("quad") != -1]) #need list for order; THIS IS BADDDD!!!! RELIES ON NAMESPACE CONFORMITY!!!!!!
				new_quads = tmp_quad_names - set(self.quad_names)
				num_quads = len(self.quad_names) #need to know from which quad we get msg, (boost::bind in C++), passed as callback arg
				for quad in sorted(list(new_quads)):
					self.quad_names.append(quad)
					self.position_subscribers.append(rospy.Subscriber('/'+quad+'/mavros/local_position/pose', PoseStamped,self.collectPositions,(num_quads)))
					self.velocity_subscribers.append(rospy.Subscriber('/'+quad+'/mavros/local_position/velocity', TwistStamped,self.collectVelocities,(num_quads))) 
					self.emergency_handlers.append(rospy.ServiceProxy('/'+quad+'flyaway_service', FlyAway))
					num_quads+=1

				dropped_quads = set(self.quad_names) - tmp_quad_names #quads that dropped out of the experiment, remove from lists
				for drop in dropped_quads:
					quad_idx = self.quad_names.index(drop) #WILL FAIL IF NAME NOT FOUND!!! (Shouldn't happen tho)
					self.position_subscribers[quad_idx].unregister() #unregisters pub/subs/services then pops them from the list
					self.velocity_subscribers[quad_idx].unregister()
					self.emergency_handlers[quad_idx].close()
					self.position_subscribers.pop(quad_idx)
					self.velocity_subscribers.pop(quad_idx)
					self.emergency_handlers.pop(quad_idx)
					self.quad_names.pop(quad_idx)
					num_quads -= 1


			if len(self.colliding_quads_and_waypoints) > 0:
				for item in self.colliding_quads_and_waypoints:
					self.force_move(item)

			i+=1
			rate.sleep()

	def force_move(self, item):
		#IN CPP THIS NEEDS A ROS::SPIN TO HANDLE SUBSCRIBER CALLBACKS
		#THIS SHOULD ALSO BE A SERVICE THAT GETS CALLED FROM OVERSEER TO EACH QUAD
		#ideally, velocities would be additive if multiple quads collide (multiple waypoints)
		q1_idx, q2_idx, midpt = item
		vel = Twist()
		for quad in [q1_idx, q2_idx]:
			diff = self.all_positions[:,quad] - midpt
			gain = self.velocity_gain/np.linalg.norm(diff)
			vel.linear.x = gain*diff[0]
			vel.linear.y = gain*diff[1]
			vel.linear.z = gain*diff[2]
			self.emergency_handlers[quad](False, None, vel)
		

if __name__ == '__main__':
	seer = Overseer()
	seer.run()
