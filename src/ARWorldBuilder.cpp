#include <ar_blocks/ARWorldBuilder.h>

namespace nxr {

using namespace alvar;
using namespace moveit;
using namespace std;

static const float g_table_dimensions[3] = { 0.608012, 1.21602, 0.60325 };
// static const float g_table_position[3] = { 0.608012, 1.21602, 0.60325 };

ARWorldBuilder::ARWorldBuilder(unsigned int cutoff) : 
	nh_("~"),
	left_arm_("left_arm"),
	right_arm_("right_arm"),
	cutoff_confidence_(cutoff)
{
	ROS_INFO("Constructing ARWorldBuilder...");
	
	left_arm_.setPlanningTime(30);
	right_arm_.setPlanningTime(30);
	
	// Setup moveit_simple_grasps
	if( !left_grasp_data_.loadRobotGraspData(nh_, "left_hand") || ! right_grasp_data_.loadRobotGraspData(nh_, "right_hand"))
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

	// Initialize threads
	pthread_mutex_init(&ar_blocks_mutex_, NULL);
	thread_ids_.push_back( pthread_t() );
	ROS_INFO("Spawning worker threads...");
	pthread_create(&thread_ids_.back(), static_cast<pthread_attr_t*>(NULL), updateThread, static_cast<void*>(this));
	
	ar_pose_marker_sub_ = nh_.subscribe("ar_pose_marker", 60, &ARWorldBuilder::arPoseMarkerCallback, this);
}

ARWorldBuilder::~ARWorldBuilder()
{
	// Clean up threads
	ROS_INFO("Destroying worker threads...");
	deque<pthread_t>::iterator it = thread_ids_.begin();
	deque<pthread_t>::iterator end = thread_ids_.end();
	
	for( ; it != end; it++ ) {
		pthread_join(*it, NULL);
	}
	
	pthread_mutex_destroy(&ar_blocks_mutex_);
	ROS_INFO("All cleaned up.");
}

void ARWorldBuilder::printInfo()
{
	pthread_mutex_lock(&ar_blocks_mutex_);
	map<unsigned int,ARBlock>::iterator it = ar_blocks_.begin();
	map<unsigned int,ARBlock>::iterator end = ar_blocks_.end();	
	for( ; it != end; it++ ) {
		it->second.printInfo();
	}	
	pthread_mutex_unlock(&ar_blocks_mutex_);
}

void ARWorldBuilder::addBaseKalmanFilter(unsigned int block_id)
{
	// Setup the pose's positional filter
	ar_blocks_filtered_.insert(pair<unsigned int, KalmanSensor>(block_id,KalmanSensor(6,3)));
	ar_blocks_kalman_.insert(pair<unsigned int, Kalman>(block_id,Kalman(6)));
	// ar_blocks_filtered_.find(block_id)->second = KalmanSensor(6,3);
	// ar_blocks_kalman_.find(block_id)->second = Kalman(6);
	
	
	cvZero(ar_blocks_filtered_.find(block_id)->second.H);
	cvmSet(ar_blocks_filtered_.find(block_id)->second.H,0,0,1);
	cvmSet(ar_blocks_filtered_.find(block_id)->second.H,1,1,1);
	cvmSet(ar_blocks_filtered_.find(block_id)->second.H,2,2,1);
	
	cvSetIdentity(ar_blocks_filtered_.find(block_id)->second.R,cvScalar(10));
	
	cvSetIdentity(ar_blocks_kalman_.find(block_id)->second.F);
	cvmSet(ar_blocks_kalman_.find(block_id)->second.F,0,3,1);
	cvmSet(ar_blocks_kalman_.find(block_id)->second.F,1,4,1);
	cvmSet(ar_blocks_kalman_.find(block_id)->second.F,2,5,1);

	cvmSet(ar_blocks_kalman_.find(block_id)->second.Q,0,0,0.0001);
	cvmSet(ar_blocks_kalman_.find(block_id)->second.Q,1,1,0.0001);
	cvmSet(ar_blocks_kalman_.find(block_id)->second.Q,2,2,0.0001);
	cvmSet(ar_blocks_kalman_.find(block_id)->second.Q,3,3,0.00001);
	cvmSet(ar_blocks_kalman_.find(block_id)->second.Q,4,4,0.00001);
	cvmSet(ar_blocks_kalman_.find(block_id)->second.Q,5,5,0.00001);
	
	cvSetIdentity(ar_blocks_kalman_.find(block_id)->second.P,cvScalar(100));

	// cvmSet(ar_blocks_filtered_.find(block_id)->second.z,0,0,x);
	// cvmSet(ar_blocks_filtered_.find(block_id)->second.z,1,0,y);
	// cvmSet(ar_blocks_filtered_.find(block_id)->second.z,2,0,z);
	
	// Setup the pose's orientational filter
}

void ARWorldBuilder::filterBlocks()
{
	map<unsigned int,ARBlock>::iterator it = ar_blocks_.begin();
	map<unsigned int,ARBlock>::iterator end = ar_blocks_.end();	
	for( ; it != end; it++ ) {
		cvmSet(ar_blocks_filtered_.find(it->first)->second.z,0,0,it->second.pose_.position.x);
		cvmSet(ar_blocks_filtered_.find(it->first)->second.z,1,0,it->second.pose_.position.y);
		cvmSet(ar_blocks_filtered_.find(it->first)->second.z,2,0,it->second.pose_.position.z);
	
		// Modify the time stamps and perform predict and update
		unsigned long long time_now = ros::Time::now().toNSec();
		ar_blocks_kalman_.find(it->first)->second.predict_update(&(ar_blocks_filtered_.find(it->first)->second), ar_blocks_timestamps_[it->first] - time_now);
		ar_blocks_timestamps_[it->first] = time_now;
	}
}

void ARWorldBuilder::createOrderedStack()
{
	// Stack all blocks ontop of each other, largest to smallest

	
}

void *ARWorldBuilder::updateThread(void *td)
{
	ARWorldBuilder *my_world = static_cast<ARWorldBuilder*>(td);
	
	ROS_INFO("Update world thread successfully created.");
	ros::NodeHandle nh;
	ros::Rate loop_rate(30);

	while( ros::ok() ) {
		// Update world by iterating over the ar_blocks_
		pthread_mutex_lock(&my_world->ar_blocks_mutex_);
		my_world->updateWorld();
		pthread_mutex_unlock(&my_world->ar_blocks_mutex_);		

		loop_rate.sleep();
		ros::spinOnce();
	}

	pthread_exit(NULL);
}

void ARWorldBuilder::arPoseMarkerCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& markers_msg)
{

	pthread_mutex_lock(&ar_blocks_mutex_);

	for(int i = 0; i < markers_msg->markers.size(); i++) {
		// Use a cutoff confidence, by default this is 0
		if( markers_msg->markers[i].confidence >= cutoff_confidence_ ) {
			// Eventually differentiate the different marker types
			if(ar_blocks_.find(markers_msg->markers[i].id) == ar_blocks_.end())
				// addBaseKalmanFilter(markers_msg->markers[i].id);
			
			ar_blocks_[ markers_msg->markers[i].id ].pose_ = markers_msg->markers[i].pose.pose;
		}
	}
	
	// Perform Kalman Filtering
	// filterBlocks();
	ROS_INFO("CALLBACK REACHED");	
	printInfo();
	
	pthread_mutex_unlock(&ar_blocks_mutex_);
}

void ARWorldBuilder::setupCageEnvironment()
{
	ROS_INFO("Setting up the cage environment...");

	// Setup the table
	ROS_INFO("Adding table to the cage environment scene...");
	visual_tools_->publishCollisionTable(0.6069 + (g_table_dimensions[0]/2), 0.5842 - (g_table_dimensions[1]/2), 0.0, g_table_dimensions[1], g_table_dimensions[0], g_table_dimensions[2], "table");
	
	// Setup the walls
	
	ROS_INFO("Waiting for published collision objects to be registered...");
	// ros::Duration(2.0).sleep();

}

void ARWorldBuilder::updateWorld()
{
	map<unsigned int,ARBlock>::iterator it = ar_blocks_.begin();
	map<unsigned int,ARBlock>::iterator end = ar_blocks_.end();	

	for( ; it != end; it++ ) {
		// collision_object_pub_.publish( it->second.toCollisionObject() );
		std::stringstream ss;
		ss << it->second.id_;
		visual_tools_->publishCollisionBlock( it->second.pose_, ss.str(), it->second.dimensions_.x );
		// it->second.printInfo();
	}
}


void ARWorldBuilder::runAllTests()
{
	armMovementTest();
	primaryTest();
	endpointControlTest();
	gripperControlTest();
	visualizeGraspsTest();
}

void ARWorldBuilder::primaryTest()
{
	cout << endl << "Initiating the Grasp Visualization Tests..." << endl;
	cout << endl << "Begin sequence (y/n)? ";
	char state = 'n';
	cin >> state;
	
	if(state == 'y') {
		pthread_mutex_lock(&ar_blocks_mutex_);
		
		map<unsigned int, ARBlock>::iterator sit = ar_blocks_.begin();
		map<unsigned int, ARBlock>::iterator eit = ar_blocks_.end();
		vector<moveit_msgs::Grasp> grasps;
		vector<trajectory_msgs::JointTrajectoryPoint> ik;
		ARBlock final_block;
		bool left_side = true;
		for( ; sit != eit; sit++) {
			// Check which side the block is on
			left_side = (sit->second.pose_.position.x <= 0) ? (true) : (false);
			moveit_simple_grasps::SimpleGraspsPtr &grasper = (left_side) ? left_simple_grasps_ : right_simple_grasps_;
			moveit_simple_grasps::GraspData &gdata = (left_side) ? left_grasp_data_ : right_grasp_data_;
			std::string planning_group = (left_side) ? ("left_arm") : ("right_arm");
			
			grasps.clear();
			grasper->generateBlockGrasps( sit->second.pose_, gdata, grasps );
			grasp_filter_->filterGrasps( grasps, ik, true, gdata.ee_parent_link_, planning_group );
			visual_tools_->publishAnimatedGrasps( grasps, gdata.ee_parent_link_ );
			visual_tools_->publishIKSolutions( ik, planning_group, 0.25 );

			final_block = sit->second;
		}
		
		cout << endl << "Finished publishing animated grasps." << endl;;
		
		// Simply move the last block in the dictionary
		cout << endl << endl << "Initiating the Pick and Place Test on block " << final_block.getStringId() << "..." << endl;
		cout << "Attempting to transfer block from one side to the other..." << endl;
		
		cout << "Picking up block..." << endl;
		move_group_interface::MoveGroup &mg = (left_side) ? (left_arm_) : (right_arm_);
		moveit_simple_grasps::GraspData &gd = (left_side) ? (left_grasp_data_) : (right_grasp_data_);
		// mg.setSupportSurfaceName("table"); DONE IN CONSTRUCTOR
		if(mg.pick( final_block.getStringId(), grasps )) {
			cout << "Successfully picked up block." << endl;
			cout << "Continue to place operation (y/n)? ";
			cin >> state;
			if(state == 'y') {
				cout << "Placing block..." << endl;
				
				// Generate a placeable area
				vector<moveit_msgs::PlaceLocation> placeable_area;
				moveit_msgs::PlaceLocation ploc;
				
				geometry_msgs::PoseStamped goal_stamped;
				goal_stamped.header.frame_id = gd.base_link_;
				goal_stamped.header.stamp = ros::Time::now();
				geometry_msgs::Pose goal = final_block.pose_;
				goal.position.x *= -1;
				goal_stamped.pose = goal;
				
				/* USING CODE FROM DAVE COLEMAN block_pick_place.cpp IN baxter_pick_place PACKAGE */
				// Approach
				moveit_msgs::GripperTranslation pre_place_approach;
				pre_place_approach.direction.header.stamp = ros::Time::now();
				pre_place_approach.desired_distance = gd.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
				pre_place_approach.min_distance = gd.approach_retreat_min_dist_; // half of the desired? Untested.
				pre_place_approach.direction.header.frame_id = gd.base_link_;
				pre_place_approach.direction.vector.x = 0;
				pre_place_approach.direction.vector.y = 0;
				pre_place_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
				ploc.pre_place_approach = pre_place_approach;

				// Retreat
				moveit_msgs::GripperTranslation post_place_retreat;
				post_place_retreat.direction.header.stamp = ros::Time::now();
				post_place_retreat.desired_distance = gd.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
				post_place_retreat.min_distance = gd.approach_retreat_min_dist_; // half of the desired? Untested.
				post_place_retreat.direction.header.frame_id = gd.base_link_;
				post_place_retreat.direction.vector.x = 0;
				post_place_retreat.direction.vector.y = 0;
				post_place_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
				ploc.post_place_retreat = post_place_retreat;

				// Post place posture - use same as pre-grasp posture (the OPEN command)
				ploc.post_place_posture = gd.pre_grasp_posture_;
				mg.setPlannerId("RRTConnectionkConfigDefault");

				// Calcuate goal pose. Mirror the x axis to be on the other side of the table
				if(mg.place(final_block.getStringId(), placeable_area)) cout << "Successfully placed block!" << endl;
				else ROS_ERROR("Failed to place block.");
			}
		}
		else ROS_ERROR("Failed to pick up block.");
		
		pthread_mutex_unlock(&ar_blocks_mutex_);
	}
	else {
		cout << endl << "Terminating Primary Test Routine..." << endl;
	}
	
	cout << endl << "Completed the Primary Test Routine." << endl;
	cout << "Enter any key to continue...";
	cin >> state;
}

void ARWorldBuilder::visualizeGraspsTest()
{

}

void ARWorldBuilder::armMovementTest()
{
/*
	cout << endl << endl << "Initiating the Arm Movement Tests..." << endl;
	
	vector<string> left_joint_names, right_joint_names;
	left_joint_names = left_arm_.getJoints();
	right_joint_names = right_arm_.getJoints();

	cout << endl << "MoveGroup(left_arm)";
	cout << endl << "Validating left arm joints:";
	for(int i = 0; i < left_joint_names.size(); i++) {
		cout << endl << "\t" << left_joint_names[i];
	}
	
	cout << endl << "Validating left endeffector registration: ";	
	string left_endeffector = left_arm_.getEndEffectorLink();
	cout << "left endeffector " << ((left_endeffector == "") ? string("invalid") : string("valid")) << endl;
	
	cout << endl << "MoveGroup(right_arm)";
	cout << endl << "Validating right arm joints:";
	for(int i = 0; i < right_joint_names.size(); i++) {
		cout << endl << "\t" << right_joint_names[i];
	}

	cout << endl << "Validating right endeffector registration: ";	
	string right_endeffector = right_arm_.getEndEffectorLink();
	cout << "right endeffector " << ((right_endeffector == "") ? ("invalid") : ("valid")) << endl;

	char state = 'n';
	cout << endl << "Testing arm planning and execution pipeline...setting random joints.";
	cout << endl << "Continue (y/n)? ";
	cin >> state;
	vector<double> left_joint_values, right_joint_values;
	if(state == 'y') {
		left_joint_values = left_arm_.getRandomJointValues();
		right_joint_values = right_arm_.getRandomJointValues();
		cout << endl << "Moving left arm...";
		left_arm_.setJointValueTarget(left_joint_values);
		cout << endl << "Continue (y/n)? ";
		cin >> state;
		if(state == 'y') {
			cout << endl << "Moving right arm...";
			right_arm_.setJointValueTarget(right_joint_values);
		}
	}

	cout << "Finished arm movement test." << endl << endl;
*/
}

void ARWorldBuilder::endpointControlTest()
{
/*
	planning_interface::MoveGroup left_hand_("left_hand");
	planning_interface::MoveGroup right_hand_("right_hand");
	
	cout << endl << endl << "Initiating the Hand/Endeffector Movement Tests..." << endl;
	
	string left_endeffector = left_hand_.getEndEffectorLink();
	string right_endeffector = right_hand_.getEndEffectorLink();

	if(left_endeffector == "" || right_endeffector == "") {
		cout << endl << "Failed to obtain the " << ((left_endeffector == "") ? "left and " : "") << ((right_endeffector == "") ? "right" : "") << "end effeector links"; 
		return;
	}

	char state = 'n';
	cout << endl << "Testing endeffector pose control...setting random pose.";
	cout <<  "Continue (y/n)? ";
	cin >> state;
	if(state == 'y') {
		cout << endl << "Moving to left endeffector pose...";
		geometry_msgs::PoseStamped left_pose = left_hand_.getRandomPose();
		cout << endl << ((left_hand_.setPoseTarget(left_pose)) ? "Successfully " : "Could not ") << "set the pose target.";
		cout << endl << "Continue (y/n)? ";
		cin >> state;
		if(state == 'y') {
			cout << endl << "Moving to right endeffector pose...";
			geometry_msgs::PoseStamped right_pose = right_hand_.getRandomPose();
			cout << endl << ((right_hand_.setPoseTarget(right_pose)) ? "Successfully " : "Could not ") << " set the pose target";
		}
	}
	cout << "Finished endpoint control test." << endl << endl;
*/
}

void ARWorldBuilder::gripperControlTest()
{
	
}

} // namespace nxr
