#include <ar_blocks/ARWorldBuilder.h>

namespace nxr {

using namespace alvar;
using namespace moveit;
using namespace std;

static const float g_table_dimensions[3] = { 0.608012, 1.21602, 0.60325 };
// static const float g_table_position[3] = { 0.608012, 1.21602, 0.60325 };

ARWorldBuilder::ARWorldBuilder(unsigned int cutoff) : 
	nh_("~"),
	cutoff_confidence_(cutoff),
	left_arm_("left_arm"),
	right_arm_("right_arm"),
	planning_frame_("/base")
{
	ROS_INFO("Constructing ARWorldBuilder...");
	
	collision_object_pub_ = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 30);
	ar_pose_marker_sub_ = nh_.subscribe("ar_pose_marker", 60, &ARWorldBuilder::arPoseMarkerCallback, this);
	setupCageEnvironment();

	// Setup moveit_simple_grasps
	visual_tools_.reset( new moveit_visual_tools::VisualTools("base") );
	if(!grasp_data_.loadRobotGraspData(nh_, "left_hand"))
		ROS_ERROR("Failed to load Baxter's grasp data.");
	simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );

	// Configure the Kalman Filter

	// Initialize threads
	pthread_mutex_init(&ar_blocks_mutex_, NULL);
	thread_ids_.push_back( pthread_t() );
	ROS_INFO("Spawning worker threads...");
	pthread_create(&thread_ids_.back(), static_cast<pthread_attr_t*>(NULL), updateThread, static_cast<void*>(this));
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

	pthread_mutex_unlock(&ar_blocks_mutex_);

}

void ARWorldBuilder::setupCageEnvironment()
{
	ROS_INFO("Setting up the cage environment...");
	vector< moveit_msgs::CollisionObject > object_collection(2);

	// Setup the table
	ROS_INFO("Adding table to the cage environment, using planning frame %s...", planning_frame_.c_str());
	object_collection[0] = moveit_msgs::CollisionObject();
	object_collection[0].header.frame_id = planning_frame_;	
	object_collection[0].id = "table";
	
	vector < shape_msgs::SolidPrimitive > primitive_objects(1);
	primitive_objects[0].type = shape_msgs::SolidPrimitive::BOX;
	primitive_objects[0].dimensions.resize(3); 
	primitive_objects[0].dimensions[0] = g_table_dimensions[0]; //0.608012; 
	primitive_objects[0].dimensions[1] = g_table_dimensions[1]; //1.21602; 
	primitive_objects[0].dimensions[2] = g_table_dimensions[2]; //0.60325; 
	object_collection[0].primitives = primitive_objects;

	// For now this is guesswork on where the center of the table is positioned
	vector < geometry_msgs::Pose > primitive_object_poses(1);
	primitive_object_poses[0].position.x = 0.6069 + (g_table_dimensions[0]/2);
	primitive_object_poses[0].position.y = 0.5842 - (g_table_dimensions[1]/2);	
	primitive_object_poses[0].position.z = -0.889 + (g_table_dimensions[2]/2);
	primitive_object_poses[0].orientation.w = 1.0;
	object_collection[0].primitive_poses = primitive_object_poses;	
	object_collection[0].operation = moveit_msgs::CollisionObject::ADD;
	
	// Setup the walls
	

	// Setup the deskspace
	
	
	// Setup the test block
	ROS_INFO("Adding test block to the cage environment, using planning frame %s...", planning_frame_.c_str());
	object_collection[1] = moveit_msgs::CollisionObject();
	object_collection[1].header.frame_id = planning_frame_;	
	object_collection[1].id = "test_block";
	
	primitive_objects[0].type = shape_msgs::SolidPrimitive::BOX;
	primitive_objects[0].dimensions[0] = 0.063; //0.608012; 
	primitive_objects[0].dimensions[1] = 0.063; //1.21602; 
	primitive_objects[0].dimensions[2] = 0.063; //0.60325; 
	object_collection[1].primitives = primitive_objects;

	// For now this is guesswork on where the center of the table is positioned
	primitive_object_poses[0].position.x += -(object_collection[0].primitives[0].dimensions[0]/2) + (0.063/2);
	primitive_object_poses[0].position.y = 0.5842 - (g_table_dimensions[1]/2);	
	primitive_object_poses[0].position.z += (object_collection[0].primitives[0].dimensions[2]/2) + (0.063/2);
	primitive_object_poses[0].orientation.w = 1.0;
	object_collection[1].primitive_poses = primitive_object_poses;	
	object_collection[1].operation = moveit_msgs::CollisionObject::ADD;
	
	 
	for( vector< moveit_msgs::CollisionObject >::iterator object = object_collection.begin(); object != object_collection.end(); object++ ) {
		collision_object_pub_.publish(*object);
	}
	
	ROS_INFO("Waiting for published colllision objects to be registered...");
	ros::Duration(2.0).sleep();

}

void ARWorldBuilder::updateWorld()
{
	map<unsigned int,ARBlock>::iterator it = ar_blocks_.begin();
	map<unsigned int,ARBlock>::iterator end = ar_blocks_.end();	

	for( ; it != end; it++ ) {
		collision_object_pub_.publish( it->second.toCollisionObject() );
	}
}


void ARWorldBuilder::runAllTests()
{
	armMovementTest();
	endpointControlTest();
	gripperControlTest();
	visualizeGraspsTest();
}

void ARWorldBuilder::visualizeGraspsTest()
{
	cout << endl << "Initiating the Grasp Visualization Tests..." << endl;

	pthread_mutex_lock(&ar_blocks_mutex_);

	map<unsigned int,ARBlock>::iterator it = ar_blocks_.begin();
	map<unsigned int,ARBlock>::iterator end = ar_blocks_.end();	
	for( ; it != end; it++ ) {
		vector<moveit_msgs::Grasp> grasps;
		simple_grasps_->generateBlockGrasps( it->second.pose_, grasp_data_, grasps );
		visual_tools_->publishAnimatedGrasps( grasps, grasp_data_.ee_parent_link_ );
	}
	
	pthread_mutex_unlock(&ar_blocks_mutex_);

	char state = 'n';
	cout << endl << "Finished publishing animated grasps.";
	cout << endl << "Press any key to continue...";
	cin >> state;
}

void ARWorldBuilder::armMovementTest()
{
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
*/	
}

void ARWorldBuilder::gripperControlTest()
{
	
}

} // namespace nxr
