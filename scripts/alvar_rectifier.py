#!/usr/bin/env python

import rospy
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from visualization_msgs.msg import MarkerArray

class AlvarRectifier():

	def __init__(self):
		self.X_OFFSET, self.Y_OFFSET, self.Z_OFFSET = float(10.5), float(10.5), float(10.5)
		self.rectified_response = AlvarMarkers()
		self.visual_marker = MarkerArray()
		rospy.init_node('nxr_ar_rectifier')
		rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.rectify_markers)
		self.visual_marker_pub = rospy.Publisher('nxr_visualization_marker', Marker)
		self.ar_pub = rospy.Publisher('nxr_ar_rectifier', AlvarMarkers)								
		self.ar_rectifier()

	def rectify_markers(self, response):
		for marker in response.markers:
			marker_pose = marker.pose.pose
			marker_pose.position.x += self.X_OFFSET
			marker_pose.position.y += self.Y_OFFSET
			marker_pose.position.z += self.Z_OFFSET
		
		# Pick the first marker and publish
		self.rectified_response = response
	
	def ar_rectifier(self):
		self.ar_rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			self.ar_pub.publish(self.rectified_response)
			self.visual_marker_pub.publish(self.visual_marker);
			self.ar_rate.sleep()

if __name__ == '__main__':
	try:
		rectifier = AlvarRectifier()
	except Exception, e:
		print e

