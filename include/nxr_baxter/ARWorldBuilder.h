#ifndef ARWORLDBUILDER_H
#define ARWORLDBUILDER_H

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <std_msgs/Header.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <map>
#include <deque>
#include <string>
#include <sstream>
#include <pthread.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <nxr_baxter/ARBlock.h>

namespace nxr {

// static const float g_table_dimensions[3];
// static const float g_table_position[3];

class ARWorldBuilder {
public:
	ARWorldBuilder(unsigned int cutoff = 0);
	~ARWorldBuilder();
	
	void setupCageEnvironment(std::string planning_frame = std::string("/base"));
	void arPoseMarkerCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& markers_msg);
	void updateWorld();	

	void createOrderedStack();

	static void *updateThread(void *td);
	std::deque<pthread_t> thread_ids_;
	pthread_mutex_t ar_blocks_mutex_;
	
	ros::NodeHandle nh_;
	ros::Publisher collision_object_pub_;
	ros::Subscriber ar_pose_marker_sub_;
	std::map<unsigned int,ARBlock> ar_blocks_;
	
	unsigned int cutoff_confidence_;
};

}

#endif
