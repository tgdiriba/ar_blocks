#!/usr/bin/env python

import rospy
from geometry_msgs import Pose
from baxter_core_msgs import HeadPanCommand

class AlvarFollower():

	def __init__(self):
		rospy.init_node("baxter_ar_tracker_head")
		rospy.Subscriber("nxr_ar_tracked",Pose,self.calculate_target_position)
		self.head_pub = rospy.Publisher("/robot/head/command_head_pan",HeadPanCommand)
		self.gripper_left_pub = rospy.Publisher("/robot/end_effector/left_gripper/command")
		self.gripper_right_pub = rospy.Publisher("/robot/end_effector/right_gripper/command")
		
		self.tb = tf.TransformBroadcaster()
		self.head_target = 0
		self.gripper_target = dict()
		self.tracker_baxter()
	
	def calculate_target_position(self,pose):
		# Decide which limbs to move first and then get their transforms to calculate positions
		servprox = rospy.ServiceProxy('',)
		(transform, rotation) = servprox.lookupTransfrom('','',rospy.Time(0))
		self.head_target = 
		self.endeffector_target = 

	def tracker_baxter(self):
		self.pub_rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.head_pub.publish(self.head_target)
			self.gripper_pub.publish(self.gripper_target)
			rospy.spin()

if __name__ == '__main__':
	try:
		follower = AlvarFollower()
	except Exception, e:
		print e
	