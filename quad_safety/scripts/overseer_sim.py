#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist,TwistStamped, Pose
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, ParamValue
from sensor_msgs.msg import BatteryState
from subprocess import call


from mslquad.srv import EmergencyLand

import numpy as np

#Beta version. More updated than plain sim. 
#To make live: remove float i offset in collectPositions
#remove sorted(list()) in first for loop of collect_quads (becuase that's only there for simulator)
#remove part that says to remove in force_move


#Quad class has emergency handler; any edits to emergency service need to edit the handler
#check battery calls the handler
#force_move also calls handler

class Quad:

	#ALWAYS DESTRUCT THIS BEFORE REMOVING!!! (Handles ending subscribers and stuff (no, python will not garbage collect this))
	def __init__(self, name="quad0", index=0):
		self.name = name
		self.index = index #Note: Index is not used for anything except simulation
		self.position = np.zeros((3,1))
		self.velocity = np.zeros((3,1))
		self.battery_voltage = 12.0

		self.position_subscriber = rospy.Subscriber('/'+self.name+'/mavros/local_position/pose', PoseStamped,self.collectPosition)
		self.velocity_subscriber = rospy.Subscriber('/'+self.name+'/mavros/local_position/velocity', TwistStamped,self.collectVelocity)
		self.battery_subscriber = rospy.Subscriber('/'+self.name+'/mavros/battery',BatteryState,self.collectVoltage)
		self.emergency_handler = rospy.ServiceProxy('/'+self.name+'/emergency_land', EmergencyLand)
		self.is_avoiding = False #makes sure that quad if quad is already on avoidance course, its emergency handler will not get overwritten
		self.avoid_position = np.zeros((3,1)) #checks when quad is close to its avoid position (used for landing) (remove once Emergency handles that internally)
		self.is_landing = False #tells quad when to land (remove once Emergency handles internally)

	def collectPosition(self, msg):
		pose = msg.pose.position
		#addition of float quad_idx is due to multi_quad_simulator launching quads 2 x units away from each other
		self.position[0] = pose.x + 2*float(self.index) #REMOVE FLOAT OFFSET FOR FLIGHTROOM! (The multi_quad_simulator.launch launches each quad 2 x units apart)
		self.position[1] = pose.y
		self.position[2] = pose.z

	def collectVelocity(self, msg):
		vel = msg.twist.linear
		self.velocity[0] = vel.x
		self.velocity[1] = vel.y
		self.velocity[2] = vel.z

	def collectVoltage(self, msg):
		self.battery_voltage = msg.voltage


	def destruct(self):
		self.position_subscriber.unregister() #unregisters pub/subs/services then pops them from the list
		self.velocity_subscriber.unregister()
		self.emergency_handler.close()
		self.battery_subscriber.unregister()


class Overseer:

	def __init__(self):
		#Sleep if launched from launchfile
		rospy.init_node('Overseer', anonymous = True)
		rospy.loginfo("Booting up.")



		#TUNEABLE VARIABLES HERE
		self.propagation_time = .75 #REDUCE
		self.collision_threshold = 1.2 #will need to tune this
		self.avoid_pos_threshold = 0.5 #once colliding quads are this close to their waypoints, have them land
		self.battery_threshold = 11.0
		#self.velocity_gain = 0.5 #How strong velocity pushback acts on pair of quads (inversely proportional to distance to each other (can be found in force_move))


		self.refresh_rate = 10
		self.quads = []
		self.colliding_quads = [] #triples of (quad1, quad2, midpoint)

		self.collect_quads() #gather initial quad readout

		rospy.sleep(0.5)

		rospy.loginfo("Now running.")

	def check_battery(self):
		for quad in self.quads:
			if quad.battery_voltage < 11.0:
				rospy.loginfo("Quad " + quad.name + " battery voltage too low! Voltage: " + str(quad.battery_voltage) + ". Now landing.")
				p = Pose()
				p.position.x = quad.position[0]
				p.position.y = quad.position[1]
				p.position.z = quad.position[2]
				quad.emergency_handler(True, p)


	def check_boundary(self):
		#Implement boundary functions here
		return True


	def check_collision(self):
		all_positions = np.squeeze(np.array([x.position for x in self.quads])).T
		all_velocities = np.squeeze(np.array([x.velocity for x in self.quads])).T

		effective_pos = all_positions + self.propagation_time*all_velocities #Propagate position by propagation_time

		quad_norms = np.square(np.linalg.norm(effective_pos,axis=0))
		num_quads = quad_norms.shape[0]
		for i in range(num_quads-1):
			for j in range(i+1, num_quads):
				if abs(quad_norms[i] - quad_norms[j]) < self.collision_threshold and abs(quad_norms[i] + quad_norms[j] - 2*np.dot(effective_pos[:,i],effective_pos[:,j])) < self.collision_threshold:
					q1 = self.quads[i]
					q2 = self.quads[j]
					if [q1, q2] not in self.colliding_quads:
						rospy.loginfo("Quads "+ q1.name +" and "+ q2.name +" are in collision")
						rospy.loginfo(np.squeeze(q1.position))
						rospy.loginfo(np.squeeze(q2.position))
						self.colliding_quads.append([q1,q2])

	def collect_quads(self):
		all_topics = rospy.get_published_topics()
		all_topics = [topic[0].split('/') for topic in all_topics]
		quad_list = [topic[1] for topic in all_topics if len(topic) > 1]
		tmp_quad_names = set([name for name in quad_list if name.find("quad") != -1]) #need list for order; THIS IS BADDDD!!!! RELIES ON NAMESPACE CONFORMITY!!!!!!
		current_quad_names = [x.name for x in self.quads]
		current_quad_set = set(current_quad_names)
		new_quads = tmp_quad_names - current_quad_set
		num_quads = len(current_quad_names) #need to know from which quad we get msg, (boost::bind in C++), passed as callback arg
		for quad in sorted(list(new_quads)):
			self.quads.append(Quad(quad, num_quads))
			num_quads+=1

		dropped_quads = current_quad_set - tmp_quad_names #quads that dropped out of the experiment, remove from lists
		for drop in dropped_quads:
			loc = self.quads.index(drop)
			self.quads[loc].destruct()
			del self.quads[loc]
			num_quads -= 1

	def run(self):
		rate = rospy.Rate(self.refresh_rate)
		debug_text = ""
		i = 1
		while not rospy.is_shutdown():

			#runs once per second
			if (i % self.refresh_rate == 0):
				self.collect_quads()

			self.check_boundary()
			self.check_battery()
			self.check_collision() #multithread this with force_move

			if len(self.colliding_quads) > 0:
				for item in self.colliding_quads:
					self.force_move(item)

			i+=1
			rate.sleep()

	def force_move(self, item):
		#IN CPP THIS NEEDS A ROS::SPIN TO HANDLE SUBSCRIBER CALLBACKS
		#ideally, velocities would be additive if multiple quads collide (multiple waypoints)
		q1, q2 = item
		midpoint = ((q1.position + q2.position)/2.0)
		p = Pose()
		
		for quad in [q1, q2]:
			# if quad.is_avoiding and not quad.is_landing:
			# 	rospy.loginfo(quad.name)
			# 	rospy.loginfo(quad.position)
			# 	rospy.loginfo(quad.avoid_position)

			if not quad.is_avoiding:
				rospy.loginfo("Quad " + quad.name + " is avoiding.")
				diff = quad.position - midpoint
				newp = quad.position + 2*diff

				rospy.loginfo(np.squeeze(quad.position))
				rospy.loginfo(np.squeeze(newp))

				newp[0] -= 2*float(quad.index) #REMOVE for LIVE
				p.position.x = newp[0]
				p.position.y = newp[1]
				p.position.z = quad.position[2]
				quad.is_avoiding = True
				quad.emergency_handler(p)
				newp[0] += 2*float(quad.index) #REMOVE FOR LIVE
				quad.avoid_position = newp
			elif not quad.is_landing and np.linalg.norm(quad.position-quad.avoid_position) < self.avoid_pos_threshold:
				rospy.loginfo("Quad " + quad.name + " is now landing.")
				p.position.x = quad.position[0] - 2*float(quad.index)
				p.position.y = quad.position[1]
				p.position.z = 0
				quad.emergency_handler(p)
				quad.is_landing = True

		if q1.is_landing and q2.is_landing:
			rospy.loginfo("Quads " + q1.name + " and " + q2.name + " are now safely landed.")
			self.colliding_quads.remove([q1, q2])

		

if __name__ == '__main__':
	seer = Overseer()
	seer.run()
