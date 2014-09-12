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
#include <tf/transform_datatypes.h>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <actionlib/server/simple_action_server.h>
#include <map>
#include <cmath>
#include <deque>
#include <string>
#include <sstream>
#include <pthread.h>
#include <ar_blocks/ARBlock.h>
#include <ar_blocks/Geometry.h>
#include <ar_blocks/BuildStructureAction.h>

namespace nxr {

typedef unsigned long long ull;
typedef boost::shared_ptr<alvar::KalmanSensor> KalmanSensorPtr;
typedef boost::shared_ptr<alvar::Kalman> KalmanPtr;

class ARWorldBuilder {
public:
	ARWorldBuilder(unsigned int cutoff = 0);
	~ARWorldBuilder();
	
	void setupCageEnvironment();
	void arPoseMarkerCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& markers_msg);
	void updateWorld();	
	
	bool createOrderedStack();
	
	void updateThread();
	boost::thread_group tg_;
	std::deque<boost::thread*> threads_;
	boost::mutex ar_blocks_mutex_;
  
  void scanEnvironment();
	
	ros::NodeHandle nh_;
	ros::Publisher collision_object_pub_;
	ros::Subscriber ar_pose_marker_sub_;
	std::map<unsigned int,ARBlock> ar_blocks_;

  // Table Properties.
  Prism table_dimensions_;
  boost::mutex table_pose_mutex_;
  geometry_msgs::Pose table_pose_;  
  Rect table_freezone_;
  
  // Block Handling
  bool inFreeZone(ARBlock);
  bool clearStage();
  bool pickBlock(ARBlock &block, bool left_side);
  bool pickFreeBlock(bool left_side);
  bool isAreaClear(Rect r);
  std::vector<moveit_msgs::PlaceLocation> findFreeLocations();

  // Action Handling
  actionlib::SimpleActionServer<ar_blocks::BuildStructureAction> ar_blocks_action_server_;
  ar_blocks::BuildStructureGoal ar_blocks_goal_;
  ar_blocks::BuildStructureFeedback ar_blocks_feedback_;
  ar_blocks::BuildStructureResult ar_blocks_result_;
  void actionServerCallback(const ar_blocks::BuildStructureGoalConstPtr &goal);

	// Filters
	std::map< unsigned int, boost::shared_ptr<alvar::KalmanSensor> > ar_blocks_filtered_;
	std::map< unsigned int, boost::shared_ptr<alvar::Kalman> > ar_blocks_kalman_;
	std::map<unsigned int, std::pair<ull, ull> > ar_blocks_timestamps_;
	void addBaseKalmanFilter(unsigned int);
	void filterBlocks();
	
	unsigned int cutoff_confidence_;
  double block_size_;

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

	// Debugging Routines
	void printInfo();
	
};

} // namespace nxr

#endif
