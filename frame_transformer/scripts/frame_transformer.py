#!/usr/bin/env python
'''
This node is a subscriber to the VRPN client node, broadcasts to TF, and publushes to RB1/pose. 
The launch file for this node opens the node itself along with rviz, which helps vizualize the 
optitrack data. The launch file also contains a static link transform. 
Different users can create their own config.yaml files for different camera parameters, etc. If 
you create your own config file, remember to change the name in the launch file too.
'''

import rospy
import numpy as np
#import math
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class vrpnHandler():

	def __init__(self):

		rospy.init_node('quad_node', anonymous=True)

		self.rn_pose = PoseStamped()
		self.rn_pose.header.frame_id = 'world_new'

		self.br       = tf2_ros.TransformBroadcaster()
		self.br2      = tf2_ros.TransformBroadcaster()
		self.world_n  = tf2_ros.TransformStamped() # new frame specified by config.yaml
		self.rn_trans = tf2_ros.TransformStamped() 


		'''
		Load data from config files
		'''
		roll  = rospy.get_param('~roll')  # x 
		pitch = rospy.get_param('~pitch') # y 
		yaw   = rospy.get_param('~yaw')   # z
		# quaternion representation of frame from the world (defined by optitrack) to the desired world frame ("world new")
		self.q_w_wn = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx') # https://answers.ros.org/question/53688/euler-angle-convention-in-tf/

		roll  = rospy.get_param('~roll2')  # x 
		pitch = rospy.get_param('~pitch2') # y 
		yaw   = rospy.get_param('~yaw2')   # z
		# quaternion representation of frame from the world (defined by optitrack) to the robot's new local frame (it is suggested to align this with the frame in which control inputs are specified)
		self.q_w_rn = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx') # https://answers.ros.org/question/53688/euler-angle-convention-in-tf/



		'''
		SETTING 'world_n' <-- new world frame
		'''
		self.world_n.header.frame_id = 'world'
		self.world_n.child_frame_id  = 'world_new'

		self.world_n.transform.translation.x = rospy.get_param('~x')
		self.world_n.transform.translation.y = rospy.get_param('~y')
		self.world_n.transform.translation.z = rospy.get_param('~z')

		self.world_n.transform.rotation.x = self.q_w_wn[0]
		self.world_n.transform.rotation.y = self.q_w_wn[1]
		self.world_n.transform.rotation.z = self.q_w_wn[2]
		self.world_n.transform.rotation.w = self.q_w_wn[3]

		self.transVector = np.array([self.world_n.transform.translation.x,
									 self.world_n.transform.translation.y,
									 self.world_n.transform.translation.z])

		'''
		SETTING 'rn_trans' <-- new pose (of robot now in world_new frame)
		'''
		self.rn_trans.header.frame_id = 'world_new'
		self.rn_trans.child_frame_id  = 'ouijabot1_new'

		self.rn_trans.transform.translation.x = 0
		self.rn_trans.transform.translation.y = 0
		self.rn_trans.transform.translation.z = 0
		self.rn_trans.transform.rotation.x = 0
		self.rn_trans.transform.rotation.y = 0
		self.rn_trans.transform.rotation.z = 0
		self.rn_trans.transform.rotation.w = 1



		'''
		SETTING PUBLISHER AND SUBSCRIBER + LISTENER FOR TF
		'''

		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
	
		self.pubRb1 = rospy.Publisher('pose_new', PoseStamped, queue_size=10)

		rospy.Subscriber('pose', PoseStamped, self.handle_pose_CB)
	
	def controller(self):
		
		rate = rospy.Rate(120)
		
		while not rospy.is_shutdown():
			if not self.rn_pose:
				rate.sleep()
				continue
			
			self.br.sendTransform(self.world_n)
			self.br2.sendTransform(self.rn_trans)
			self.pubRb1.publish(self.rn_pose)

			rate.sleep()

	def handle_pose_CB(self, msg):
		
		self.world_n.header.stamp  = rospy.Time.now()
		self.rn_trans.header.stamp = rospy.Time.now()

		posV = np.array([msg.pose.position.x,
						 msg.pose.position.y,
						 msg.pose.position.z])

		quat =  np.array([msg.pose.orientation.x,
						  msg.pose.orientation.y,
					 	  msg.pose.orientation.z,
						  msg.pose.orientation.w])

		posF, rotF = self.applyTransform(posV, quat)

		self.rn_pose.pose.position.x = posF[0]
		self.rn_pose.pose.position.y = posF[1]
		self.rn_pose.pose.position.z = posF[2]
		self.rn_pose.pose.orientation.x = rotF[0]
		self.rn_pose.pose.orientation.y = rotF[1]
		self.rn_pose.pose.orientation.z = rotF[2]
		self.rn_pose.pose.orientation.w = rotF[3]

		self.rn_trans.transform.translation.x = posF[0]
		self.rn_trans.transform.translation.y = posF[1]
		self.rn_trans.transform.translation.z = posF[2]
		self.rn_trans.transform.rotation.x = rotF[0]
		self.rn_trans.transform.rotation.y = rotF[1]
		self.rn_trans.transform.rotation.z = rotF[2]
		self.rn_trans.transform.rotation.w = rotF[3]

		
	def applyTransform(self, posVector, quat):
		
		posT = self.qv_multi(tf.transformations.quaternion_inverse(self.q_w_wn), posVector) #+ self.transVector
			# Multiplies the Euler angles (that get converted to quaternions) from the .yaml file 
			# by the position vector of the input data. Then it adds the xyz translation from the 
			# .yaml file
		# rotT = tf.transformations.quaternion_multiply(self.q_w_wn, quat)
		rotT = tf.transformations.quaternion_multiply( tf.transformations.quaternion_inverse(self.q_w_wn), tf.transformations.quaternion_multiply(quat, self.q_w_rn) )
			# Multiplies the quaternion from the .yaml file by the quaternion of the input data 

		return(posT, rotT)

	'''
	MULTIPLIES QUATERNION BY VECTOR
	'''
	def qv_multi(self, q1, v1):

		q2 = list(v1)
		q2.append(0.0) # "real" component of quaternions in ros is w, which is the 4th element. turn vector into a pure (no real part) quaternion

		# "conjugate" vector v1 by quaternion q1 (i.e. apply the rotation to v1) - https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
		v1_rotated = tf.transformations.quaternion_multiply(
					 tf.transformations.quaternion_multiply(q1, q2),
					 tf.transformations.quaternion_inverse(q1) # should be same as tf.transformations.quaternion_conjugate(q1), assuming q1 is a unit quaternion
					)[:3]
		return v1_rotated


if __name__ == '__main__':	
	
	vrpn = vrpnHandler()
	vrpn.controller()