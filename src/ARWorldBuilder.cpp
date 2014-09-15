#include <ar_blocks/ARWorldBuilder.h>

namespace nxr {

using namespace alvar;
using namespace moveit;
using namespace std;
using boost::shared_ptr;

static const float g_table_dimensions[3] = { 0.608012, 1.21602, 0.60325 };

ARWorldBuilder::ARWorldBuilder(unsigned int cutoff) : 
	nh_("~"),
  ar_blocks_action_server_(nh_, "ar_blocks_action_server", boost::bind(&ARWorldBuilder::actionServerCallback, this, _1), false),
	left_arm_("left_arm"),
	right_arm_("right_arm"),
  block_size_(0.0635),
	cutoff_confidence_(cutoff)
{
	ROS_INFO("Constructing ARWorldBuilder...");
	
	collision_object_pub_ = nh_.advertise<moveit_msgs::CollisionObject>("collision_object",30);

	left_arm_.setPlanningTime(30);
	right_arm_.setPlanningTime(30);
	
	// Setup moveit_simple_grasps
	if( !left_grasp_data_.loadRobotGraspData(nh_, "left_hand") || !right_grasp_data_.loadRobotGraspData(nh_, "right_hand"))
		ros::shutdown();

	ROS_INFO("Successfully loaded the robot's end-effector data...");
	ROS_INFO("Configuring moveit grasp generation and visualization...");

	planning_scene_monitor_.reset( new planning_scene_monitor::PlanningSceneMonitor("robot_description") );
  
  trajectory_em_.reset( new trajectory_execution_manager::TrajectoryExecutionManager( planning_scene_monitor_->getRobotModel() ) );
  plan_execution_.reset( new plan_execution::PlanExecution( planning_scene_monitor_, trajectory_em_ ) );

	visual_tools_.reset( new moveit_visual_tools::VisualTools( "base" ) );
	visual_tools_->setLifetime(10);
	visual_tools_->setMuted(false);
	visual_tools_->loadEEMarker(left_grasp_data_.ee_group_, "left_arm");
	visual_tools_->loadEEMarker(right_grasp_data_.ee_group_, "right_arm");
	visual_tools_->setFloorToBaseHeight(-0.9);

	grasp_filter_.reset( new moveit_simple_grasps::GraspFilter( 
													planning_scene_monitor_->getPlanningScene()->getCurrentState(),
													visual_tools_ ) );
	
	left_simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps( visual_tools_ ) );
	right_simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps( visual_tools_ ) );
	
	ROS_INFO("Successfully configured moveit grasp generation and visualization...");
	
	// Configure the Kalman Filter
	ROS_INFO("Setting up filters...");
	
	setupCageEnvironment();
	left_arm_.setSupportSurfaceName("table");
	right_arm_.setSupportSurfaceName("table");
	
	// Calculate the table information
	// Start with the default measured state
	table_pose_.position.x = 0.6069 + (g_table_dimensions[0]/2);	// Using the distance to the front of the table
	table_pose_.position.y = 0.5842 - (g_table_dimensions[1]/2);	// Using the distance to the center of the table on the y-axis
	table_pose_.position.z = 0.0;					// Approximation
	table_pose_.orientation.x = 0.0;
	table_pose_.orientation.y = 0.0;
	table_pose_.orientation.z = 0.0;
	table_pose_.orientation.w = 1.0;
	
	// Define the freezone to be the bottom left of the table and the position of the table tag as the top left
	table_freezone_.point.x = table_pose_.position.x - (g_table_dimensions[0]/2);
	table_freezone_.point.y = table_pose_.position.y + (g_table_dimensions[1]/2);

	left_arm_ = planning_interface::MoveGroup("left_arm");
	right_arm_ = planning_interface::MoveGroup("right_arm");

	// Initialize threads
	ROS_INFO("Spawning worker threads...");
	threads_.push_back(tg_.create_thread( boost::bind(&ARWorldBuilder::updateThread, this) ));
	
	ar_pose_marker_sub_ = nh_.subscribe("ar_pose_marker", 60, &ARWorldBuilder::arPoseMarkerCallback, this);
	
  // Perform the initial scan
  scanEnvironment();
  if(validEnvironment()) {
    ROS_INFO("Starting up the ar_blocks action server...");
    ar_blocks_action_server_.start();
  }
  else {
    ROS_ERROR("Invalid environment encountered. Expecting a table positioned withing grasp distance.");
  }
}

bool ARWorldBuilder::validEnvironment()
{
  return true;  
}

ARWorldBuilder::~ARWorldBuilder()
{
	// Clean up threads
	ROS_INFO("Destroying worker threads...");

	tg_.join_all();	
	
	ROS_INFO("All cleaned up.");
}

void ARWorldBuilder::printInfo()
{
	boost::mutex::scoped_lock l(ar_blocks_mutex_);
	
	map<unsigned int,ARBlock>::iterator it = ar_blocks_.begin();
	map<unsigned int,ARBlock>::iterator end = ar_blocks_.end();	
	for( ; it != end; it++ ) {
		it->second.printInfo();
	}	
}

bool ARWorldBuilder::scanEnvironment()
{
  // Clear the block memory
  ar_blocks_mutex_.lock();
  ar_blocks_.clear();
  ar_blocks_mutex_.unlock();
  
  vector<geometry_msgs::Pose> points;
  geometry_msgs::Pose p1;
  p1.position.x = 0.098;
  p1.position.y = 0.7;
  p1.position.z = 0.011;
  p1.orientation.x = 1.0;
  p1.orientation.y = 0.01;
  p1.orientation.z = 0.025;
  p1.orientation.w = -0.0133;

  geometry_msgs::Pose p2;
  p2.position.x = 0.565;
  p2.position.y = 0.5523;
  p2.position.z = 0.410;
  p2.orientation.x = 0.996;
  p2.orientation.y = 0.055;
  p2.orientation.z = 0.037;
  p2.orientation.w = 0.0469;
  
  geometry_msgs::Pose p3;
  p3.position.x = 0.6834;
  p3.position.y = 0.2276;
  p3.position.z = 0.627;
  p3.orientation.x = 1.0;
  p3.orientation.y = -0.0999;
  p3.orientation.z = 0.0512;
  p3.orientation.w = 0.0117;
  
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p2);
  points.push_back(p1);
  
  moveit_msgs::RobotTrajectory rt;
  double fraction = left_arm_.computeCartesianPath(points, 0.01, 0.0, rt);
  
  plan_execution_->getTrajectoryExecutionManager()->clear();
  if(plan_execution_->getTrajectoryExecutionManager()->push(rt)) {
    plan_execution_->getTrajectoryExecutionManager()->execute();
    if(plan_execution_->getTrajectoryExecutionManager()->waitForExecution() == moveit_controller_manager::ExecutionStatus::SUCCEEDED) {
      ROS_INFO("Successfully scanned the environment.");
      return true;
    }
    else {
      ROS_ERROR("Trajectory execution failed.");
      return false;
    }
  }
  else {
    ROS_ERROR("Failed to push the trajectory through the computed cartesian path.");
    return false;
  }
  
}

bool ARWorldBuilder::createOrderedStack()
{
	return false;
}

void ARWorldBuilder::updateThread()
{
	ROS_INFO("Update world thread successfully created.");
	ros::NodeHandle nh;
	ros::Rate loop_rate(2);

	while( true ) {
		// Update world by iterating over the ar_blocks_
		updateWorld();
		ROS_INFO("UPDATING THE WORLD");
		loop_rate.sleep();
		ros::spinOnce();
	}
}

bool ARWorldBuilder::inFreeZone(ARBlock block)
{
  // Make sure that the block is within some threshold of the table height
  // Current threshold is +/- 2cm
  double height_diff = (block.pose_.position.z - block.dimensions_.z) - table_dimensions_.height;
  if(abs(height_diff) < 0.02) {
    if(block.pose_.position.x >= table_freezone_.point.x &&
       block.pose_.position.x <= table_freezone_.area.length + table_freezone_.point.x) {
      if(block.pose_.position.y >= table_freezone_.point.y &&
         block.pose_.position.y <= table_freezone_.area.width + table_freezone_.point.y) {
        return true;
      }
    }
  }
  return false;
}

bool ARWorldBuilder::isAreaClear(Rect r)
{
  map<unsigned int, ARBlock>::iterator it = ar_blocks_.begin();
  map<unsigned int, ARBlock>::iterator end = ar_blocks_.end();
  
  for( ; it != end; it++ ) {
    // Compare and check if any block is contained in Rectangular area
    // Check if any point is contained in the zone or vice-versa
    // This only works for cubes for now.
    // TODO: Allow work for any rectangular prism
    
    tf::Quaternion q( it->second.pose_.orientation.x,
                      it->second.pose_.orientation.y,
                      it->second.pose_.orientation.z,
                      it->second.pose_.orientation.w );
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Assuming that yaw is the correct rotation angle. Needs to be fixed...
    int top_side = it->second.topSide();
     
    double r_angle = yaw;
    double init_angle = atan(it->second.dimensions_.y / it->second.dimensions_.x);
    Point tl = {it->second.pose_.position.x + cos(init_angle+yaw),
                it->second.pose_.position.y + sin(init_angle+yaw)};
    Point tr = {it->second.pose_.position.x - cos(init_angle+yaw),
                it->second.pose_.position.y + sin(init_angle+yaw)};
    Point bl = {it->second.pose_.position.x - cos(init_angle+yaw),
                it->second.pose_.position.y - sin(init_angle+yaw)};
    Point br = {it->second.pose_.position.x + cos(init_angle+yaw),
                it->second.pose_.position.y - sin(init_angle+yaw)};
    
    vector<Point> points;
    points.push_back(tl);
    points.push_back(tr);
    points.push_back(bl);
    points.push_back(br);
    
    // Define a Rect that is aligned with the y axis that inscribes the shape (bounding box)
    // Simplifies calculations
    Point top_left_point;
    Area r_area;
    r_area.length = (tr.x - tl.x)/2;
    r_area.width = (tr.y - bl.y)/2;
    top_left_point.x = it->second.pose_.position.x - r_area.length;
    top_left_point.y = it->second.pose_.position.y + r_area.width;
    Rect inscribed = { top_left_point, r_area };

    // Generate rectangle's points
    vector<Point> area_points;
    Point bb_tl = { r.point.x + r.area.length/2, r.point.y - r.area.width/2 };
    Point bb_tr = { r.point.x + r.area.length/2, r.point.y + r.area.width/2 };
    Point bb_bl = { r.point.x - r.area.length/2, r.point.y - r.area.width/2 };
    Point bb_br = { r.point.x - r.area.length/2, r.point.y + r.area.width/2 };
    area_points.push_back( bb_tl );
    area_points.push_back( bb_tr );
    area_points.push_back( bb_bl );
    area_points.push_back( bb_br );
    
    if(pointInRect(r, points)) return false;
    if(pointInRect(inscribed, area_points)) return false;
  }
  return true;
}

vector<moveit_msgs::PlaceLocation> ARWorldBuilder::findFreeLocations()
{
    // Create a partiion of the freezone
    double tolerance = 0.01; // Define a tolerance for the two gripper ends. 1cm for both.
    int num_partitions_x = table_freezone_.area.length/(block_size_+tolerance);
    int num_partitions_y = table_freezone_.area.width/(block_size_+tolerance);
    int num_partitions = num_partitions_x * num_partitions_y;
    
    vector<moveit_msgs::PlaceLocation> areas;
    
    // Favor the top left positions when placing a block in the free zone
    // Iterate over x and y
    for( int x = 0; x < num_partitions_x; x++ ) {
      for( int y = 0; y < num_partitions_y; y++ ) {
        
        Rect place;
        place.point.x = num_partitions*(block_size_+tolerance) + table_freezone_.point.x;
        place.point.y = num_partitions*(block_size_+tolerance) + table_freezone_.point.y;
        place.area.length = block_size_+tolerance;
        place.area.width = block_size_+tolerance;
        
        bool left_side = (place.point.x > 0) ? (true) : (false); 
        moveit_simple_grasps::GraspData &gd = (!left_side) ? (left_grasp_data_) : (right_grasp_data_);
        
        if( isAreaClear(place) ) {
          
          // Generate a placeable area
          moveit_msgs::PlaceLocation ploc;
          geometry_msgs::Pose block_pose;
          block_pose.position.x = place.point.x + (place.area.length/2.0);
          block_pose.position.y = place.point.y + (place.area.width/2.0);
          // Use the table's orientation to align the blocks
          table_pose_mutex_.lock();
          block_pose.position.z = table_pose_.position.z + (table_dimensions_.height/2);
          block_pose.orientation = table_pose_.orientation;
          table_pose_mutex_.unlock();
          
          geometry_msgs::PoseStamped goal_stamped;
          goal_stamped.header.frame_id = gd.base_link_;
          goal_stamped.header.stamp = ros::Time::now();
          goal_stamped.pose = block_pose;
          
          moveit_msgs::GripperTranslation pre_place_approach;
          pre_place_approach.direction.header.stamp = ros::Time::now();
          pre_place_approach.desired_distance = gd.approach_retreat_desired_dist_;
          pre_place_approach.min_distance = gd.approach_retreat_min_dist_;
          pre_place_approach.direction.header.frame_id = gd.base_link_;
          pre_place_approach.direction.vector.x = 0;
          pre_place_approach.direction.vector.y = 0;
          pre_place_approach.direction.vector.z = -1;
          ploc.pre_place_approach = pre_place_approach;
          
          moveit_msgs::GripperTranslation post_place_retreat;
          post_place_retreat.direction.header.stamp = ros::Time::now();
          post_place_retreat.desired_distance = gd.approach_retreat_desired_dist_;
          post_place_retreat.min_distance = gd.approach_retreat_min_dist_;
          post_place_retreat.direction.header.frame_id = gd.base_link_;
          post_place_retreat.direction.vector.x = 0;
          post_place_retreat.direction.vector.y = 0;
          post_place_retreat.direction.vector.z = 1;
          ploc.post_place_retreat = post_place_retreat;
          
          ploc.post_place_posture = gd.pre_grasp_posture_;
          areas.push_back(ploc);
          
        }
      }
    }
     
  return areas;
}

bool ARWorldBuilder::pickBlock(ARBlock &block, bool left_side = true)
{
    vector<moveit_msgs::Grasp> grasps;
    vector<trajectory_msgs::JointTrajectoryPoint> ik;
    
    moveit_simple_grasps::SimpleGraspsPtr &grasper = (!left_side) ? left_simple_grasps_ : right_simple_grasps_;
    moveit_simple_grasps::GraspData &gdata = (!left_side) ? left_grasp_data_ : right_grasp_data_;
    std::string planning_group = (!left_side) ? ("left_arm") : ("right_arm");
    move_group_interface::MoveGroup &mg = (!left_side) ? (left_arm_) : (right_arm_);
    
    ROS_INFO("Generating grasps for block %d.", block.id_);
    grasper->generateBlockGrasps( block.pose_, gdata, grasps );
    
    ROS_INFO("Filtering grasps for block %d.", block.id_);
    grasp_filter_->filterGrasps( grasps, ik, true, gdata.ee_parent_link_, planning_group );
    
    ROS_INFO("Picking up block %d.", block.id_);
    if(!mg.pick(block.getStringId(), grasps)) {
      ROS_ERROR("Problem encountered picking up block %d.", block.id_);
      return false;
    }
    return true;
}

bool ARWorldBuilder::pickFreeBlock(bool left_side = true)
{
  map<unsigned int, ARBlock>::iterator sit = ar_blocks_.begin();
  map<unsigned int, ARBlock>::iterator eit = ar_blocks_.end();
  
  for( ; sit != eit; sit++ ) {
    if( inFreeZone(sit->second) ) {
      return pickBlock(sit->second);
    }
  }
  return false;
}

bool ARWorldBuilder::clearStage()
{
  map<unsigned int, ARBlock>::iterator sit = ar_blocks_.begin();
  map<unsigned int, ARBlock>::iterator eit = ar_blocks_.end();
  
  vector<moveit_msgs::PlaceLocation> free_locations = findFreeLocations();
  int moved_blocks = 0;
  for( ; sit != eit; sit++ ) {
    if(!inFreeZone(sit->second)) {
      if(moved_blocks < free_locations.size()) {
        bool left_side = (sit->second.pose_.position.x > 0) ? (true) : (false);
		    move_group_interface::MoveGroup &mg = (!left_side) ? (left_arm_) : (right_arm_);
        
        if(pickBlock(sit->second)) {
          vector<moveit_msgs::PlaceLocation> placeable_area;
          placeable_area.push_back(free_locations[moved_blocks]);
          mg.setPlannerId("RRTConnectionkConfigDefault");
           
          if(!mg.place(sit->second.getStringId(), placeable_area)) {
            ROS_ERROR("Failed to place the block. Reset the stage and try again.");
            return false;
          }
          
          moved_blocks++;
        }
      }
      else {
        ROS_ERROR("Not enough space in the free zone to clear the stage.\nAborting...");
        return false;
      }
    }
  } 
}

void ARWorldBuilder::actionServerCallback(const ar_blocks::BuildStructureGoalConstPtr &goal)
{
  boost::mutex::scoped_lock l(ar_blocks_mutex_);

  // Reset the feedback structure
  ar_blocks_feedback_.updated_structure.header.stamp = ros::Time::now();
  ar_blocks_feedback_.updated_structure.header.frame_id = std::string("/base"); 
  ar_blocks_feedback_.updated_structure.layers.clear();
 
  // Make sure that the stage is clear to begin with
  clearStage();
 
  // Loop through the structure and place the blocks layer by layer
  // Make sure that the caller has not preempted the action at any point
  // Publish feedback after a block has been placed or misplaced
  int layer_count = 0;
  const vector<ar_blocks::Layer> &layers = goal->goal_structure.layers;
  while(!ar_blocks_action_server_.isPreemptRequested() && 
        layer_count < layers.size()) 
  {
    ar_blocks_feedback_.updated_structure.layers.push_back( ar_blocks::Layer() );
     
    // Create sorted set of blocks in the layer based on position away from Baxter
    // FIX
    // sort(layers[layer_count].blocks.begin(), layers[layer_count].blocks.end(), blockCompare);
    
    int block_count = 0;
    const ar_blocks::Layer &top_layer = layers[layer_count];
		// ar_blocks_feedback_.updated_structure.layers.push_back( top_layer );
    while(!ar_blocks_action_server_.isPreemptRequested() &&
          block_count < top_layer.blocks.size())
    {
      bool left_side = (top_layer.blocks[block_count].pose_stamped.pose.position.x > 0) ? (true) : (false);
      
      // Find and pick a block from the freezone
      int block_id = pickFreeBlock(left_side);
      if( block_id == -1 ) {
        ROS_ERROR("Could not find and pick a free block. Add more blocks to the freezone or reset the stage.");
        return;
      }
      
      // Select the corrent side's MoveGroup and GraspData
		  move_group_interface::MoveGroup &mg = (!left_side) ? (left_arm_) : (right_arm_);
      moveit_simple_grasps::GraspData &gd = (!left_side) ? (left_grasp_data_) : (right_grasp_data_);
      
      // Generate a placeable area
      vector<moveit_msgs::PlaceLocation> placeable_area;
      moveit_msgs::PlaceLocation ploc;
      
      geometry_msgs::PoseStamped goal_stamped;
      goal_stamped.header.frame_id = gd.base_link_;
      goal_stamped.header.stamp = ros::Time::now();
      goal_stamped.pose = top_layer.blocks[block_count].pose_stamped.pose;
      
      moveit_msgs::GripperTranslation pre_place_approach;
      pre_place_approach.direction.header.stamp = ros::Time::now();
      pre_place_approach.desired_distance = gd.approach_retreat_desired_dist_;
      pre_place_approach.min_distance = gd.approach_retreat_min_dist_;
      pre_place_approach.direction.header.frame_id = gd.base_link_;
      pre_place_approach.direction.vector.x = 0;
      pre_place_approach.direction.vector.y = 0;
      pre_place_approach.direction.vector.z = -1;
      ploc.pre_place_approach = pre_place_approach;
      
      moveit_msgs::GripperTranslation post_place_retreat;
      post_place_retreat.direction.header.stamp = ros::Time::now();
      post_place_retreat.desired_distance = gd.approach_retreat_desired_dist_;
      post_place_retreat.min_distance = gd.approach_retreat_min_dist_;
      post_place_retreat.direction.header.frame_id = gd.base_link_;
      post_place_retreat.direction.vector.x = 0;
      post_place_retreat.direction.vector.y = 0;
      post_place_retreat.direction.vector.z = 1;
      ploc.post_place_retreat = post_place_retreat;
      
      ploc.post_place_posture = gd.pre_grasp_posture_;
      placeable_area.push_back(ploc);
      mg.setPlannerId("RRTConnectionkConfigDefault");
      
      stringstream ss;
      ss << top_layer.blocks[block_count].id;
      if(!mg.place(ss.str(), placeable_area)) {
        ROS_ERROR("Failed to place block. Reset the stage and try again.");
        return;
      }
      
      ar_blocks_[block_id].pose_ = top_layer.blocks[block_count].pose_stamped.pose;
      ar_blocks_[block_id].time_stamp_ = top_layer.blocks[block_count].pose_stamped.header.stamp;
		  ar_blocks_feedback_.updated_structure.layers[layer_count].blocks.push_back( top_layer.blocks[block_count] );
      ar_blocks_action_server_.publishFeedback( ar_blocks_feedback_ );
      block_count++;
    }
    
    layer_count++;
  }
   
  ar_blocks_result_.final_structure = ar_blocks_feedback_.updated_structure;
  ar_blocks_result_.final_structure.header.stamp = ros::Time::now();
  ar_blocks_result_.final_structure.header.frame_id = std::string("/base"); 
  ar_blocks_action_server_.setSucceeded( ar_blocks_result_ );
}

void ARWorldBuilder::arPoseMarkerCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& markers_msg)
{
	ar_blocks_mutex_.lock();

	for(int i = 0; i < markers_msg->markers.size(); i++) {
		// Use a cutoff confidence, by default this is 0
    
    		unsigned int block_id = markers_msg->markers[i].id;
    
		if( markers_msg->markers[i].confidence >= cutoff_confidence_  && markers_msg->markers[i].header.frame_id == "/base" ) {
			// Eventually differentiate the different marker types
			if(block_id == 12) {
				// Handle the table. Assuming that the tag is placed on the bottom left of the table.
				table_pose_mutex_.lock();
				table_pose_.position.x = markers_msg->markers[i].pose.pose.position.x + (g_table_dimensions[0]/2) - 8.01111111;
				table_pose_.position.y = markers_msg->markers[i].pose.pose.position.y - (g_table_dimensions[1]/2) + 2.2;
				table_pose_.position.z = markers_msg->markers[i].pose.pose.position.z;
				table_pose_.orientation = markers_msg->markers[i].pose.pose.orientation;

        table_freezone_.point.x = table_pose_.position.x - (g_table_dimensions[0]/2);
        table_freezone_.point.y = table_pose_.position.y + (g_table_dimensions[1]/2);

				table_pose_mutex_.unlock();
			}
			else {
				if(ar_blocks_.find(block_id) == ar_blocks_.end()) {
					ar_blocks_[ block_id ].id_ = markers_msg->markers[i].id;
          
          // KF Setup
          ar_blocks_[ block_id ].filter_.reset( markers_msg->markers[i].pose.pose );
				}
        else {
          // KF Update
          ar_blocks_[ block_id ].filter_.update( markers_msg->markers[i].pose.pose );
        }
			}
		}
	}
	
	ar_blocks_mutex_.unlock();
}

void ARWorldBuilder::setupCageEnvironment()
{
	ROS_INFO("Setting up the cage environment...");

	// Setup the table
	ROS_INFO("Adding table to the cage environment scene...");
	// visual_tools_->publishCollisionTable(0.6069 + (g_table_dimensions[0]/2), 0.5842 - (g_table_dimensions[1]/2), 0.0, g_table_dimensions[1], g_table_dimensions[0], g_table_dimensions[2], "table");
	
	// Setup the walls
	
	ROS_INFO("Waiting for published collision objects to be registered...");
	// ros::Duration(2.0).sleep();

}

void ARWorldBuilder::updateWorld()
{
	boost::mutex::scoped_lock l(ar_blocks_mutex_);
	
	map<unsigned int,ARBlock>::iterator it = ar_blocks_.begin();
	map<unsigned int,ARBlock>::iterator end = ar_blocks_.end();	

	for( ; it != end; it++ ) {
    // Using KF
		visual_tools_->publishCollisionBlock( it->second.filter_.predicted_pose_, it->second.getStringId(), it->second.dimensions_.x );
	 	it->second.printInfo();
	}
}

} // namespace nxr
