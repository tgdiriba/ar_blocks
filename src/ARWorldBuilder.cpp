#include <nxr_baxter/ARWorldBuilder.h>

namespace nxr {

using namespace moveit;
using namespace std;

static const float g_table_dimensions[3] = { 0.608012, 1.21602, 0.60325 };
// static const float g_table_position[3] = { 0.608012, 1.21602, 0.60325 };

ARWorldBuilder::ARWorldBuilder(unsigned int cutoff) : cutoff_confidence_(cutoff)
{
	ROS_INFO("Constructing ARWorldBuilder...");
	
	collision_object_pub_ = nh_.advertise<moveit_msgs::CollisionObject>("collision_object", 30);
	ar_pose_marker_sub_ = nh_.subscribe("ar_pose_marker", 60, &ARWorldBuilder::arPoseMarkerCallback, this);
	setupCageEnvironment();

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
			ar_blocks_[ markers_msg->markers[i].id ].pose_ = markers_msg->markers[i].pose.pose;
		}
	}
	pthread_mutex_unlock(&ar_blocks_mutex_);

}

void ARWorldBuilder::setupCageEnvironment(string planning_frame)
{
	ROS_INFO("Setting up the cage environment...");
	vector< moveit_msgs::CollisionObject > object_collection(1);

	planning_interface::MoveGroup left_arm("left_arm");
	planning_frame = left_arm.getPlanningFrame();

	// Setup the table
	ROS_INFO("Adding table to the cage environment, using planning frame %s...", planning_frame.c_str());
	object_collection[0] = moveit_msgs::CollisionObject();
	object_collection[0].header.frame_id = planning_frame;	
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

}
