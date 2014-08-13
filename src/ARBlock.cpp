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

ARBlock::ARBlock(float *dims, int id) : id_(id)
{
	dimensions_.x = dims[0];
	dimensions_.y = dims[1];
	dimensions_.z = dims[2];
}

void ARBlock::printInfo()
{
	std::cout << "Block " << id_ << " Pose:" << std::endl;
	std::cout << "\tPosition: " << std::endl;
	std::cout << "\t\tx: " << pose_.position.x << std::endl;
	std::cout << "\t\ty: " << pose_.position.y << std::endl;
	std::cout << "\t\tz: " << pose_.position.z << std::endl;
	std::cout << "\tOrientation: " << std::endl;
	std::cout << "\t\tx: " << pose_.orientation.x << std::endl;
	std::cout << "\t\ty: " << pose_.orientation.y << std::endl;
	std::cout << "\t\tz: " << pose_.orientation.z << std::endl;
	std::cout << "\t\tw: " << pose_.orientation.w << std::endl;
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
