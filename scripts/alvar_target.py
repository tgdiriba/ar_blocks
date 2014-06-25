#!/usr/bin/env python

import rospy
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Pose

class AlvarTracker():
	
	def __init__(self):
		rospy.init_node("nxr_ar_tracker")
		rospy.Subscriber("nxr_ar_rectifier",AlvarMarkers,self.ar_tracker_sub)
		self.pub = rospy.Publisher("nxr_ar_tracked",Pose)
		
		self.largest_marker = AlvarMarker()
		self.tracked = Pose()
		
		self.ar_tracker_pub()
	
	def ar_tracker_sub(self,marker_set):
		rospy.logdebug("Entered subscriber handler to nxr_ar_rectifier.")
		optimal_confidence = -1.0
		for marker in marker_set.markers:
			if marker.confidence > optimal_confidence:
				self.tracked = marker.pose.pose
				optimal_confidence = marker.confidence
		# self.tracked = max(marker_set.markers,key=lambda marker: marker.confidence).pose.pose
		rospy.logdebug("Exiting subscriber handler to nxr_ar_rectifier.")

	def ar_tracker_pub(self):
		# Track the object with the highest data confidence
		self.ar_rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			self.pub.publish(self.tracked)
			self.ar_rate.sleep() # rospy.spin()
	
if __name__ == '__main__':
	try:
		tracker = AlvarTracker()
	except Exception, e:
		print e
