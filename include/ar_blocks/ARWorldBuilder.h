#ifndef ARWORLDBUILDER_H
#define ARWORLDBUILDER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <moveit_msgs/GripperTranslation.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/grasp_filter.h>
#include <moveit_visual_tools/visual_tools.h>
#include <std_msgs/Header.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/Filter.h>
#include <ar_track_alvar/Kalman.h>
#include <ar_track_alvar/Platform.h>
#include <ar_track_alvar/AlvarException.h>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <deque>
#include <string>
#include <sstream>
#include <pthread.h>
#include <ar_blocks/ARBlock.h>

namespace nxr {

typedef ull unsigned long long;
typedef KalmanSensorPtr boost::shared_ptr<alvar::KalmanSensor>;
typedef KalmanPtr boost::shared_ptr<alvar::Kalman>;

class ARWorldBuilder {
public:
	ARWorldBuilder(unsigned int cutoff = 0);
	~ARWorldBuilder();
	
	void setupCageEnvironment();
	void arPoseMarkerCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& markers_msg);
	void updateWorld();	
	
	void createOrderedStack();
	
	void updateThread();
	boost::thread_group tg_;
	std::deque<boost::thread*> threads_;
	boost::mutex ar_blocks_mutex_;
  
  void scanEnvironment();
	
	ros::NodeHandle nh_;
	ros::Publisher collision_object_pub_;
	ros::Subscriber ar_pose_marker_sub_;
	std::map<unsigned int,ARBlock> ar_blocks_;

	// Filters
	std::map< unsigned int, boost::shared_ptr<alvar::KalmanSensor> > ar_blocks_filtered_;
	std::map< unsigned int, boost::shared_ptr<alvar::Kalman> > ar_blocks_kalman_;
	std::map<unsigned int, std::pair<ull, ull> > ar_blocks_timestamps_;
	void addBaseKalmanFilter(unsigned int);
	void filterBlocks();
	
	unsigned int cutoff_confidence_;

	// MoveIt!
	move_group_interface::MoveGroup left_arm_;
	move_group_interface::MoveGroup right_arm_;
	
	// MoveIt! Simple Grasps and Visual Tools
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
	moveit_simple_grasps::SimpleGraspsPtr left_simple_grasps_;
	moveit_simple_grasps::SimpleGraspsPtr right_simple_grasps_;
	moveit_simple_grasps::GraspFilterPtr grasp_filter_;
	moveit_simple_grasps::GraspData left_grasp_data_;
	moveit_simple_grasps::GraspData right_grasp_data_;
	moveit_visual_tools::VisualToolsPtr visual_tools_;

	// Pick and Place Routines
	
	
	// Debugging Routines
	void printInfo();

	// MoveIt! Tests
	void runAllTests();
	void primaryTest();
	void armMovementTest();	
	void endpointControlTest();
	void gripperControlTest();
	void visualizeGraspsTest();
	
};

} // namespace nxr

#endif
