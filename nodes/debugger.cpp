#include <ros/ros.h>
#include <ros/console.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <map>

std::map<unsigned int,AlvarMarker> markers_dict;

void arCallback(const ar_track_alvar::AlvarMarkers::ConstPtr &markers_msg)
{
	for(int i = 0; i < markers_msg->markers.size(); i++) {
		::markers_dict[markers_msg->markers[i].id] = markers_msg->markers[i];
	}

	// Print out the entire dictionary
	std::map<unsigned int,AlvarMarker>::iterator bit = ::markers_dict.begin();
	std::map<unsigned int,AlvarMarker>::iterator eit = ::markers_dict.end();
	
	ROS_INFO_STREAM("Printing out all of the stored markers...\n\n");
		
	for( ; bit != eit; bit++) {
		marker = 
		ROS_INFO_STREAM("AlvarMarker\n");
		ROS_INFO_STREAM("Pose: "
		ROS_INFO_STREAM("		x: " << bit->pose.pose.position.x << "\n");
		ROS_INFO_STREAM("		y: " << bit->pose.pose.position.y << "\n");
		ROS_INFO_STREAM("		z: " << bit->pose.pose.position.z << "\n");
		ROS_INFO_STREAM("Orientation:\n");
		ROS_INFO_STREAM("		x: " << bit->pose.pose.position.x << "\n");
		ROS_INFO_STREAM("		y: " << bit->pose.pose.position.y << "\n");
		ROS_INFO_STREAM("		z: " << bit->pose.pose.position.z << "\n");
		ROS_INFO_STREAM("		w: " << bit->pose.pose.position.w << "\n");
	}
	
	ROS_INFO_STREAM("Done printing out the stored markers...");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv);
	ros::NodeHandle nh("~");
	
	ros::Subscriber ar_sub = nh.subscribe("ar_pose_marker", 60, arCallback);
	
	ros::spin();
	
}
