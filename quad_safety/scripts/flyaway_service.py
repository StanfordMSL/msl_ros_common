#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist,TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, ParamValue
from subprocess import call

from quad_safety.srv import * #contains location of FlyAway service type

import numpy as np

class flyaway_service:

	def __init__(self):
		self.pos = []
		self.quad_idx = 0
		self.qname = rospy.get_namespace()
		self.vel_pub = rospy.Publisher('/'+qname+'/mavros/setpoint_velocity/cmd_vel_unstamped', Twist,queue_size=10)
		self.pos_sub = rospy.Subscriber('/'+qname+'/mavros/local_position/pose', PoseStamped,collectPositions)

	#message passed as String = "[waypt],,quadidx"
	#One flyaway service PER QUAD!!!!
	def collectPositions(msg):
		p = msg.pose.position
		self.pos = np.array([p.x + 2*float(quad_idx),p.y,p.z]) #REMOVE FLOAT OFFSET FOR FLIGHTROOM

	def handle_flyaway(message):
		wpt, idx = message.split(",,")
		self.quad_idx = int(idx)
		wpt = wpt[1:-1] #remove []
		wpt = wpt.split(',')
		wpt = map(float,wpt)
		wpt = np.array(wpt)
		vel = Twist()

		while (True): 
			diff = wpt - self.pos
			vel.linear.x = diff[0]
			vel.linear.y = diff[1]
			vel.linear.z = diff[2]
			self.vel_pub.publish(vel)
			rospy.sleep(0.01)

	def flyaway_server():
		rospy.init_node('flyaway_server')
		s = rospy.Service('flyaway_service', Flyaway, self.handle_flyaway)
		rospy.spin()


if __name__ == "__main__":
	svc = flyaway_service()
	svc.flyaway_server()