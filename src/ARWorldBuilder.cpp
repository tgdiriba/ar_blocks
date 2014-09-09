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
	ar_blocks_action_server_.start();

	// Further Tests
	/*geometry_msgs::Pose tpose;
	tpose.position.x = 1.0;
	tpose.position.y = 1.0;
	tpose.position.z = 1.0;
	tpose.orientation.w = 1.0;*/
	
	geometry_msgs::Pose b6pose;
	b6pose.position.x = 0.622832;
	b6pose.position.y = 0.287662;
	b6pose.position.z = -0.264523;
	b6pose.orientation.x = 0.706081;
	b6pose.orientation.y = 0.112389;
	b6pose.orientation.z = 0.695385;
	b6pose.orientation.w = -0.0725117;
	
	geometry_msgs::Pose b14pose;
	b14pose.position.x = 0.636082;
	b14pose.position.y = 0.367458;
	b14pose.position.z = -0.276704;
	b14pose.orientation.x = 0.478385;
	b14pose.orientation.y = 0.524679;
	b14pose.orientation.z = 0.517999;
	b14pose.orientation.w = 0.477007;
	
	// visual_tools_->publishCollisionBlock( tpose, "1000", 0.0635 );
	visual_tools_->publishCollisionBlock( b6pose, "6", 0.0635 );
	visual_tools_->publishCollisionBlock( b14pose, "14", 0.0635 );
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

void ARWorldBuilder::scanEnvironment()
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
  points.push_back(p1);

  geometry_msgs::Pose p2;
  p2.position.x = 0.565;
  p2.position.y = 0.5523;
  p2.position.z = 0.410;
  p2.orientation.x = 0.996;
  p2.orientation.y = 0.055;
  p2.orientation.z = 0.037;
  p2.orientation.w = 0.0469;
  points.push_back(p2);
  
  geometry_msgs::Pose p3;
  p3.position.x = 0.6834;
  p3.position.y = 0.2276;
  p3.position.z = 0.627;
  p3.orientation.x = 1.0;
  p3.orientation.y = -0.0999;
  p3.orientation.z = 0.0512;
  p3.orientation.w = 0.0117;
  points.push_back(p3);
  
  geometry_msgs::Pose p4;
  p3.position.x = 0.6730;
  p3.position.y = -0.1514;
  p3.position.z = 0.44046;
  p3.orientation.x = 0.9361;
  p3.orientation.y = -0.3487;
  p3.orientation.z = -0.008559;
  p3.orientation.w = 0.04464;
  points.push_back(p3);
  
  geometry_msgs::Pose p5;
  p3.position.x = 0.6834;
  p3.position.y = 0.2276;
  p3.position.z = 0.627;
  p3.orientation.x = 1.0;
  p3.orientation.y = -0.0999;
  p3.orientation.z = 0.0512;
  p3.orientation.w = 0.0117;
  points.push_back(p3);

  geometry_msgs::Pose p6;
  p2.position.x = 0.565;
  p2.position.y = 0.5523;
  p2.position.z = 0.410;
  p2.orientation.x = 0.996;
  p2.orientation.y = 0.055;
  p2.orientation.z = 0.037;
  p2.orientation.w = 0.0469;
  points.push_back(p2);
  
  geometry_msgs::Pose p7;
  p1.position.x = 0.098;
  p1.position.y = 0.7;
  p1.position.z = 0.011;
  p1.orientation.x = 1.0;
  p1.orientation.y = 0.01;
  p1.orientation.z = 0.025;
  p1.orientation.w = -0.0133;
  points.push_back(p1);

  moveit_msgs::RobotTrajectory rt;
  double fraction = left_arm_.computeCartesianPath(points, 0.01, 0.0, rt);
  left_arm_.move();

}

void ARWorldBuilder::addBaseKalmanFilter(unsigned int block_id)
{
	// Setup the pose's positional filter
	ar_blocks_filtered_.insert(pair<unsigned int, KalmanSensorPtr>(block_id, KalmanSensorPtr(new KalmanSensor(6,3)) ));
	ar_blocks_kalman_.insert(pair<unsigned int, KalmanPtr>(block_id, KalmanPtr(new Kalman(6)) ));
	
	cvZero(ar_blocks_filtered_.find(block_id)->second->H);
	cvmSet(ar_blocks_filtered_.find(block_id)->second->H,0,0,1);
	cvmSet(ar_blocks_filtered_.find(block_id)->second->H,1,1,1);
	cvmSet(ar_blocks_filtered_.find(block_id)->second->H,2,2,1);
	
	cvSetIdentity(ar_blocks_filtered_.find(block_id)->second->R,cvScalar(10));
	
	cvSetIdentity(ar_blocks_kalman_.find(block_id)->second->F);
	cvmSet(ar_blocks_kalman_.find(block_id)->second->F,0,3,1);
	cvmSet(ar_blocks_kalman_.find(block_id)->second->F,1,4,1);
	cvmSet(ar_blocks_kalman_.find(block_id)->second->F,2,5,1);

	cvmSet(ar_blocks_kalman_.find(block_id)->second->Q,0,0,0.0001);
	cvmSet(ar_blocks_kalman_.find(block_id)->second->Q,1,1,0.0001);
	cvmSet(ar_blocks_kalman_.find(block_id)->second->Q,2,2,0.0001);
	cvmSet(ar_blocks_kalman_.find(block_id)->second->Q,3,3,0.00001);
	cvmSet(ar_blocks_kalman_.find(block_id)->second->Q,4,4,0.00001);
	cvmSet(ar_blocks_kalman_.find(block_id)->second->Q,5,5,0.00001);
	
	cvSetIdentity(ar_blocks_kalman_.find(block_id)->second->P,cvScalar(100));

	// Setup the pose's orientational filter
	
}

void ARWorldBuilder::filterBlocks()
{
	map<unsigned int, shared_ptr<Kalman> >::iterator it = ar_blocks_kalman_.begin();
	map<unsigned int, shared_ptr<Kalman> >::iterator end = ar_blocks_kalman_.end();	
	
	for( ; it != end; it++ ) {
		cvmSet(ar_blocks_filtered_.find(it->first)->second->z,0,0,ar_blocks_.find(it->first)->second.pose_.position.x);
		cvmSet(ar_blocks_filtered_.find(it->first)->second->z,1,0,ar_blocks_.find(it->first)->second.pose_.position.y);
		cvmSet(ar_blocks_filtered_.find(it->first)->second->z,2,0,ar_blocks_.find(it->first)->second.pose_.position.z);
		
		// Modify the time stamps and perform predict and update
		ar_blocks_kalman_.find(it->first)->second->predict_update(ar_blocks_filtered_.find(it->first)->second.get(), ar_blocks_timestamps_[it->first].second - ar_blocks_timestamps_[it->first].first);
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

bool ARWorldBuilder::pointInRectangle(Rectangle r, Point p)
{
  return (p.x >= r.point.x) && (p.y > r.point.y) &&
         (p.x <= r.point.x + r.area.length) && (p.y <= r.point.y + r.area.width);
}

bool ARWorldBuilder::pointInRectangle(Rectangle r, vector<Point> pv)
{
  for(int i = 0; i < pv.size(); i++) {
    if(pointInRectangle(r, pv[i])) return true;
  }
  return false;
}

bool ARWorldBuilder::isAreaClear(Rectangle r)
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
    
    // Define a Rectangle that is aligned with the y axis that inscribes the shape (bounding box)
    // Simplifies calculations
    Point top_left_point;
    Area r_area;
    r_area.length = (tr.x - tl.x)/2;
    r_area.width = (tr.y - bl.y)/2;
    top_left_point.x = it->second.pose_.position.x - r_area.length;
    top_left_point.y = it->second.pose_.position.y + r_area.width;
    Rectangle inscribed = { top_left_point, r_area };

    // Generate rectangle's poins
    vector<Point> area_points;
    Point bb_tl = { r.point.x + r.area.length/2, r.point.y - r.area.width/2 };
    Point bb_tr = { r.point.x + r.area.length/2, r.point.y + r.area.width/2 };
    Point bb_bl = { r.point.x - r.area.length/2, r.point.y - r.area.width/2 };
    Point bb_br = { r.point.x - r.area.length/2, r.point.y + r.area.width/2 };
    area_points.push_back( bb_tl );
    area_points.push_back( bb_tr );
    area_points.push_back( bb_bl );
    area_points.push_back( bb_br );
    
    if(pointInRectangle(r, points)) return false;
    if(pointInRectangle(inscribed, area_points)) return false;
    
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
        
        Rectangle place;
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
				table_pose_mutex_.unlock();
			}
			else {
				if(ar_blocks_.find(block_id) == ar_blocks_.end()) {
					ar_blocks_[ block_id ].id_ = markers_msg->markers[i].id;
					ar_blocks_timestamps_[ block_id ] = pair<ull, ull>(ros::Time::now().toNSec(), ros::Time::now().toNSec());
					addBaseKalmanFilter(block_id);
				}
				
	      ar_blocks_[ block_id ].time_stamp_ = ros::Time::now();
				ar_blocks_[ block_id ].pose_ = markers_msg->markers[i].pose.pose;
				ar_blocks_timestamps_[ block_id ].first = ar_blocks_timestamps_[ block_id ].second;
				ar_blocks_timestamps_[ block_id ].second = ros::Time::now().toNSec();
				// ar_blocks_[ block_id ].printInfo();
			}
		}
	}
	
	// Perform Kalman Filtering
	// filterBlocks();
	
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
	
	// map<unsigned int,Kalman>::iterator it = ar_blocks_kalman_.begin();
	// map<unsigned int,Kalman>::iterator end = ar_blocks_kalman_.end();	
	
	map<unsigned int,ARBlock>::iterator it = ar_blocks_.begin();
	map<unsigned int,ARBlock>::iterator end = ar_blocks_.end();	

	for( ; it != end; it++ ) {
		// Ignore filtering for now
		/*if(ar_blocks_kalman_.find(it->first) != ar_blocks_kalman_.end()) {
			ARBlock block(it->first, ar_blocks_kalman_[it->first]);
			// Ignore the orientation kalman filter for now
			block.pose_.orientation = ar_blocks_.find(it->first)->second.pose_.orientation;
			visual_tools_->publishCollisionBlock( block.pose_, block.getStringId(), block.dimensions_.x );
		}
		else {	
			// collision_object_pub_.publish( it->second.toCollisionObject() );
			visual_tools_->publishCollisionBlock( it->second.pose_, it->second.getStringId(), it->second.dimensions_.x );
		}*/
		// Block Info
		visual_tools_->publishCollisionBlock( it->second.pose_, it->second.getStringId(), it->second.dimensions_.x );
	 	it->second.printInfo();
	}
}

} // namespace nxr
