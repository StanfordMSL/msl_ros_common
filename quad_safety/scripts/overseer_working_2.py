#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist,TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, ParamValue
from subprocess import call

import numpy as np

#For flightroom: remove float i offset in collectPositions, and in prevent-collision, 
#For final integration: remove force-kill condition

#TODO:
#every time you get a new position, check for collision
#if collision, immediately run a service call for "prevent collision" (For simultaneous monitoring and prevention)

class Overseer:

	def __init__(self):
		#Sleep if launched from launchfile
		rospy.init_node('Overseer', anonymous = True)
		self.chatter = rospy.Publisher('Overseer_debug', String, queue_size=10)


		#initialize variables of interest
		self.propagation_time = 0.7
		self.refresh_rate = 10
		self.collision_threshold = 0.5 #will need to tune this
		self.landing_pose_tmp = PoseStamped()
		self.position = None
		self.quad_names = [] #list of quad names airborne (NOT SORTED)
		self.position_subscribers = [] #list of subscriber nodes (SAME ORDER AS QUAD_NAMES)
		self.position_publishers = [] #same as above, but for publishing override
		self.velocity_subscribers = []
		self.mode_services = [] #Holds mode services for all detected quads
		self.all_positions = np.zeros((3,1))
		self.all_velocities = np.zeros((3,1))

		#REMOVE FOR FINAL INTEGRATION
		self.nodes_to_kill = set()
		self.setpoint_killer = []

	def agentPoseCB(self,msg):
		agentPose = msg.pose
		self.position = np.array([agentPose.position.x, agentPose.position.y, agentPose.position.z])

	def agentBatteryVoltage(self,msg):
		self.battery_level = msg.voltage

	def log_publishing_nodes(self,msg):
		msg_header = msg._connection_header
		self.nodes_to_kill.add(msg_header['callerid'])

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
		#addition of float quad_idx is due to simulator default launching quads 1 x away from each other
		self.all_positions[:,quad_idx] = np.array([pose.position.x+ 2*float(quad_idx), pose.position.y, pose.position.z]) #REMOVE FLOAT OFFSET FOR FLIGHTROOM

	def collectVelocities(self,msg,quad_idx):
		experimental = False
		if self.all_velocities.shape[1] <= quad_idx: #number of cols < number of quads
			num_cols = self.all_velocities.shape[1]
			self.all_velocities = np.hstack((self.all_velocities, np.zeros((3, quad_idx-num_cols+1)))) #resize-as-you-go

		vel = msg.twist.linear
		self.all_velocities[:,quad_idx] = np.array([vel.x, vel.y, vel.z])

	def checkCollision(self):
		colList = []
		#sexy algorithm for checking collisions here




		#Not sexy algorithm here
		effective_pos = self.all_positions + self.propagation_time*self.all_velocities #Propagate position by propagation_time
		quad_norms = np.square(np.linalg.norm(effective_pos,axis=0))
		num_quads = quad_norms.shape[0]
		for i in range(num_quads-1):
			for j in range(i+1, num_quads):
				if abs(quad_norms[i] - quad_norms[j]) < self.collision_threshold and abs(quad_norms[i] + quad_norms[j] - 2*np.dot(effective_pos[:,i],effective_pos[:,j])) < self.collision_threshold:
					rospy.loginfo("Quads "+ self.quad_names[i] +" and "+ self.quad_names[j] +" are in collision")
					rospy.loginfo(self.all_positions[:,i])
					rospy.loginfo(self.all_positions[:,j])
					tmp_list = [i,j]
					colList.append(tmp_list)
		
		#(hopefully) improved not-sexy here
		# if (experimental):
		# 	continue
		return colList

	def preventCrash(self,colList):
		midpoints = []
		waypoints = []

		for col_pair in colList:
			i = col_pair[0]
			j = col_pair[1]
			pos1 = self.all_positions[:,i]
			pos2 = self.all_positions[:,j]
			midpoints.append((pos1 + pos2)/2.0)

		midpoints = np.asarray(midpoints)
		midpoint = np.mean(midpoints,axis=0)
		
		all_colliding_quads = set([item for subl in colList for item in subl])
		for quad in all_colliding_quads:
			pos = self.all_positions[:,quad]
			waypoint = pos + 2*(pos-midpoint)
			tmp_list = [quad, waypoint]
			waypoints.append(tmp_list)

		self.force_move(waypoints) #waypoints passed as [idx, pos] lists


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
				tmp_quad_names = set([name for name in quad_list if name.find("uav") != -1]) #need list for order; THIS IS BADDDD!!!! RELIES ON NAMESPACE CONFORMITY!!!!!!
				new_quads = tmp_quad_names - set(self.quad_names)
				num_quads = len(self.quad_names) #need to know from which quad we get msg, (boost::bind in C++), passed as callback arg
				for quad in sorted(list(new_quads)):
					self.quad_names.append(quad)
					self.position_subscribers.append(rospy.Subscriber('/'+quad+'/mavros/local_position/pose', PoseStamped,self.collectPositions,(num_quads)))
					self.position_publishers.append(rospy.Publisher('/'+quad+'/mavros/setpoint_position/local', PoseStamped,queue_size=10))
					self.mode_services.append(rospy.ServiceProxy('/'+quad+'/mavros/set_mode', SetMode))
					self.velocity_subscribers.append(rospy.Subscriber('/'+quad+'/mavros/local_position/velocity', TwistStamped,self.collectVelocities,(num_quads))) 
					num_quads+=1

					#REMOVE FOR FLIGHTROOM
					self.setpoint_killer.append(rospy.Subscriber('/'+quad+'/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, self.log_publishing_nodes))
					self.setpoint_killer.append(rospy.Subscriber('/'+quad+'/mavros/setpoint_position/pose', PoseStamped, self.log_publishing_nodes))

			colList = self.checkCollision()
			if len(colList) > 0:
				self.preventCrash(colList)

			i+=1
			rate.sleep()

	def kill_other_setters(self):
		l = list(self.nodes_to_kill)
		for node in l:
			call(["rosnode kill "+node], shell=True)

	def force_move(self, waypoints):
		pos = PoseStamped()
		for quad, waypoint in waypoints:
			pos.pose.position.x = waypoint[0]
			pos.pose.position.y = waypoint[1]
			pos.pose.position.z = waypoint[2]
			for i in range(10):
				self.position_publishers[quad].publish(pos)	
			while not self.setMode("Separate",quad):
				self.position_publishers[quad].publish(pos)	
				rospy.sleep(0.01)

		self.kill_other_setters() #REMOVE FOR FINAL INTEGRATION

		while(len(waypoints) > 0):
			for quad, waypoint in waypoints:
				pos.pose.position.x = waypoint[0]
				pos.pose.position.y = waypoint[1]
				pos.pose.position.z = waypoint[2]
				self.position_publishers[quad].publish(pos)	
				rospy.sleep(0.01)
				if np.max(abs(self.all_positions[:,quad] - waypoint)) < 0.2:
					#return control to user here (can't do with setpoint killer, so printing instead)
					rospy.loginfo("Quad" + str(quad) + " at waypoint")
					waypoints.remove([quad,waypoint])
					# while (self.all_positions[quad] > 0.1):
					# 	pos.pose.position.z = 0
					# 	self.position_publishers[quad].publish(pos)	
					# while not self.setMode("Off",quad):
					# 	rospy.sleep(0.01)
					# waypoints.remove([quad,waypoint])
		#exit()

		#Return control to user (ideally)

if __name__ == '__main__':
	seer = Overseer()
	seer.run()
