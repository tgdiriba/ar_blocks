#include <ar_blocks/ARBlock.h>

namespace nxr {

static const float g_block_sizes[BLOCK_NUM] = { 0.063, 0.051, 0.045 };

ARBlock::ARBlock(unsigned int b_type) : block_type_(b_type)
{
	// POSE DEFAULT VALUES ARE ALL ZERO
	pose_.orientation.w = 1.0;

	// Designate the block type
	if(block_type_ < BLOCK_NUM) 
		dimensions_.x = dimensions_.y = dimensions_.z = g_block_sizes[block_type_];
}

ARBlock::ARBlock(unsigned int id, const boost::shared_ptr<alvar::Kalman> &k) 
	: block_type_(BLOCK_A),
	  id_(id)
{
	// POSE DEFAULT VALUES ARE ALL ZERO
	pose_.position.x = cvmGet(k->x,0,0);
	pose_.position.y = cvmGet(k->x,1,0);
	pose_.position.z = cvmGet(k->x,2,0);
	pose_.orientation.x = 0.0;
	pose_.orientation.y = 0.0;
	pose_.orientation.z = 0.0;
	pose_.orientation.w = 1.0;
	
}

ARBlock::ARBlock(float *dims, int id) : id_(id)
{
	dimensions_.x = dims[0];
	dimensions_.y = dims[1];
	dimensions_.z = dims[2];
}

std::string ARBlock::getStringId()
{
	std::stringstream ss;
	ss << id_;
	return ss.str();
}

void ARBlock::printInfo()
{
	ROS_INFO_STREAM("Block " << getStringId() << " Pose:");
	ROS_INFO_STREAM("\tPosition: ");
	ROS_INFO_STREAM("\t\tx: " << pose_.position.x);
	ROS_INFO_STREAM("\t\ty: " << pose_.position.y);
	ROS_INFO_STREAM("\t\tz: " << pose_.position.z);
	ROS_INFO_STREAM("\tOrientation: ");
	ROS_INFO_STREAM("\t\tx: " << pose_.orientation.x);
	ROS_INFO_STREAM("\t\ty: " << pose_.orientation.y);
	ROS_INFO_STREAM("\t\tz: " << pose_.orientation.z);
	ROS_INFO_STREAM("\t\tw: " << pose_.orientation.w);
}

alvar::Kalman toKalman()
{
	return alvar::Kalman(4);
}

moveit_msgs::CollisionObject ARBlock::toCollisionObject(std::string planning_frame)
{
    moveit_msgs::CollisionObject block;
    block.header.frame_id = planning_frame;
		std::stringstream ss;
    ss << id_;
    block.id = ss.str();

    std::vector < shape_msgs::SolidPrimitive > primitive_objects( 1 );
    std::vector < geometry_msgs::Pose > primitive_object_poses( 1 );
    primitive_objects[0].type = shape_msgs::SolidPrimitive::BOX;
    primitive_objects[0].dimensions.resize(3);
    primitive_objects[0].dimensions[0] = dims_[0];
    primitive_objects[0].dimensions[1] = dims_[1];
    primitive_objects[0].dimensions[2] = dims_[2];
    block.primitives = primitive_objects;

    block.primitive_poses = primitive_object_poses;
		block.primitive_poses[0] = pose_;
    block.operation = moveit_msgs::CollisionObject::ADD;
		
		return block;
}

}
