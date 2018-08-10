#!/usr/bin/env python
'''
This ROS node subscribes to the VRPN client node and transforms the raw pose data in an old coordinate frame "source" to a new coordinate frame "destination". The resulting pose is  broadcasted to a TF, and publushes to RB1/pose. 
The launch file for this node opens the node itself along with rviz, which helps vizualize the 
optitrack data. The launch file also contains a static link transform. 
Different users can create their own config.yaml files for different camera parameters, etc. If 
you create your own config file, remember to change the name in the launch file too.
'''


import rospy
import numpy as np

import tf
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float32

class transformHandler():

	def __init__(self):

		# Initialize ROS node
		rospy.init_node('frame_transformer', anonymous=True)

		# ROS tf listener and broadcaster
		self.tfBuffer = tf2_ros.Buffer()
		self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
		self.src2DstBroadcaster = tf2_ros.TransformBroadcaster()
		self.dst2LocalBroadcaster = tf2_ros.TransformBroadcaster()
		# ROS messages
		self.dstLocalPose = PoseStamped()
		self.dstTf = tf2_ros.TransformStamped()
		self.dstLocalTf = tf2_ros.TransformStamped() 

		'''
		Input ROS parameters
		'''
		# if !exists(rospy.getparam('/vrpn_client_node/frame_id'))
		# 	self.srcFrameID = 'world'
		self.srcFrameID = rospy.get_param('~srcFrameID','optitrack')
		# self.srcLocalFrameID = rospy.get_param('~srcFrameID','robot')
		self.dstFrameID = rospy.get_param('~dstFrameID','world')
		self.dstLocalFrameID = rospy.get_param('~dstLocalFrameID','base_link')

		self.dstLocalPose.header.frame_id = self.dstFrameID

		'''
		Load data from config files
		'''
		roll  = rospy.get_param('~src2dst_roll',0.0)  # x 
		pitch = rospy.get_param('~src2dst_pitch',0.0) # y 
		yaw   = rospy.get_param('~src2dst_yaw',0.0)   # z
		# quaternion representation of frame from the world (defined by optitrack) to the desired world frame ("world new")
		self.q_src_dst = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx') # https://answers.ros.org/question/53688/euler-angle-convention-in-tf/

		roll  = rospy.get_param('~srcLocal2dstLocal_roll',0.0)  # x 
		pitch = rospy.get_param('~srcLocal2dstLocal_pitch',0.0) # y 
		yaw   = rospy.get_param('~srcLocal2dstLocal_yaw',0.0)   # z
		# quaternion representation of frame from the world (defined by optitrack) to the robot's new local frame (it is suggested to align this with the frame in which control inputs are specified)
		self.q_src_dstLocal = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx') # https://answers.ros.org/question/53688/euler-angle-convention-in-tf/

		# Static fransform of dst frame w.r.t. src frame
		self.dstTf.header.frame_id = self.srcFrameID
		self.dstTf.child_frame_id  = self.dstFrameID
		self.dstTf.transform.translation.x = rospy.get_param('~src2dst_x')
		self.dstTf.transform.translation.y = rospy.get_param('~src2dst_y')
		self.dstTf.transform.translation.z = rospy.get_param('~src2dst_z')
		self.dstTf.transform.rotation.x = self.q_src_dst[0]
		self.dstTf.transform.rotation.y = self.q_src_dst[1]
		self.dstTf.transform.rotation.z = self.q_src_dst[2]
		self.dstTf.transform.rotation.w = self.q_src_dst[3]
		self.transVector = np.array([self.dstTf.transform.translation.x,
									 							 self.dstTf.transform.translation.y,
									 							 self.dstTf.transform.translation.z])

		# Dynamic fransform of dst_local frame w.r.t. dst frame
		self.dstLocalTf.header.frame_id = self.dstFrameID
		self.dstLocalTf.child_frame_id  = self.dstLocalFrameID
		self.dstLocalTf.transform.translation.x = 0
		self.dstLocalTf.transform.translation.y = 0
		self.dstLocalTf.transform.translation.z = 0
		self.dstLocalTf.transform.rotation.x = 0
		self.dstLocalTf.transform.rotation.y = 0
		self.dstLocalTf.transform.rotation.z = 0
		self.dstLocalTf.transform.rotation.w = 1

		# Pose subscriber and publisher
		rospy.Subscriber('src_pose', PoseStamped, self.handleCallback)
		self.dstPosStampedPub = rospy.Publisher('dst_pose', PoseStamped, queue_size=10)
	
	def controller(self):
		
		rate = rospy.Rate(120)
		
		while not rospy.is_shutdown():
			if not self.dstLocalPose:
				rate.sleep()
				continue
			
			self.src2DstBroadcaster.sendTransform(self.dstTf)
			self.dst2LocalBroadcaster.sendTransform(self.dstLocalTf)
			self.dstPosStampedPub.publish(self.dstLocalPose)

			rate.sleep()

	def handleCallback(self, msg):
		
		# Set ROS time
		# rosTime = rospy.Time.now()
		self.dstTf.header.stamp  = rospy.Time.now()
		self.dstLocalTf.header.stamp = rospy.Time.now()
		self.dstLocalPose.header.stamp = rospy.Time.now()

		# Apply coordinate transform 
		srcPos = np.array([msg.pose.position.x,
						 msg.pose.position.y,
						 msg.pose.position.z])
		srcQuat =  np.array([msg.pose.orientation.x,
						  msg.pose.orientation.y,
					 	  msg.pose.orientation.z,
						  msg.pose.orientation.w])
		dstPos, dstQuat = self.applyTransform(srcPos, srcQuat)

		# Assemble ROS messages
		self.dstLocalPose.pose.position.x = dstPos[0]
		self.dstLocalPose.pose.position.y = dstPos[1]
		self.dstLocalPose.pose.position.z = dstPos[2]
		self.dstLocalPose.pose.orientation.x = dstQuat[0]
		self.dstLocalPose.pose.orientation.y = dstQuat[1]
		self.dstLocalPose.pose.orientation.z = dstQuat[2]
		self.dstLocalPose.pose.orientation.w = dstQuat[3]
		self.dstLocalTf.transform.translation.x = dstPos[0]
		self.dstLocalTf.transform.translation.y = dstPos[1]
		self.dstLocalTf.transform.translation.z = dstPos[2]
		self.dstLocalTf.transform.rotation.x = dstQuat[0]
		self.dstLocalTf.transform.rotation.y = dstQuat[1]
		self.dstLocalTf.transform.rotation.z = dstQuat[2]
		self.dstLocalTf.transform.rotation.w = dstQuat[3]

		
	def applyTransform(self, posVector, quat):
		
		dstPos = self.qv_multi(tf.transformations.quaternion_inverse(self.q_src_dst), posVector) #+ self.transVector
			# Multiplies the Euler angles (that get converted to quaternions) from the .yaml file 
			# by the position vector of the input data. Then it adds the xyz translation from the 
			# .yaml file
		# dstQuat = tf.transformations.quaternion_multiply(self.q_src_dst, quat)
		dstQuat = tf.transformations.quaternion_multiply( tf.transformations.quaternion_inverse(self.q_src_dst), tf.transformations.quaternion_multiply(quat, self.q_src_dstLocal) )
			# Multiplies the quaternion from the .yaml file by the quaternion of the input data 

		return(dstPos, dstQuat)

	'''
	Mulitply quatertion q by vector v
	'''
	def qv_multi(self, q, v):

		v_ = list(v)
		v_.append(0.0) # "real" component of quaternions in ros is w, which is the 4th element. turn vector into a pure (no real part) quaternion

		# "conjugate" vector v1 by quaternion q1 (i.e. apply the rotation to v1) - https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
		v_ = tf.transformations.quaternion_multiply(
					 tf.transformations.quaternion_multiply(q, v_),
					 tf.transformations.quaternion_inverse(q) # should be same as tf.transformations.quaternion_conjugate(q1), assuming q1 is a unit quaternion
					)[:3]
		return v_


if __name__ == '__main__':	
	
	frame_transformer = transformHandler()
	frame_transformer.controller()