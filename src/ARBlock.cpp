#include <ar_blocks/ARBlock.h>

namespace nxr {

static const float g_block_sizes[BLOCK_NUM] = { 0.0635, 0.051, 0.045 };

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
	ROS_INFO_STREAM("\tTime Stamp: ");
	ROS_INFO_STREAM("\t\t: " << time_stamp_);
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

int ARBlock::topSide()
{
  // Calculate which side is most facing up.
  tf::Quaternion q( pose_.orientation.x,
                    pose_.orientation.y,
                    pose_.orientation.z,
                    pose_.orientation.w );
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
  // Figure out the ranges. Easier to use quaternion?
  return 0;
  if(roll <= M_PI/4 && roll > -M_PI/4 && pitch <= M_PI/4 && pitch > -M_PI/4) return 0;
  if(roll <= -M_PI/4 && roll > -3*M_PI/4 && yaw <= M_PI/4 && yaw > -M_PI/4) return 1;
  if(roll <= M_PI/4 && roll > -M_PI/4 && pitch <= M_PI/4 && pitch > -M_PI/4) return 0;
  if(roll <= M_PI/4 && roll > -M_PI/4 && pitch <= M_PI/4 && pitch > -M_PI/4) return 0;
  if(roll <= M_PI/4 && roll > -M_PI/4 && pitch <= M_PI/4 && pitch > -M_PI/4) return 0;
  if(roll <= M_PI/4 && roll > -M_PI/4 && pitch <= M_PI/4 && pitch > -M_PI/4) return 0;
}

ar_blocks::Block ARBlock::toBlockMsg()
{
  ar_blocks::Block t;
  t.length = dimensions_.x;
  t.width = dimensions_.y;
  t.height = dimensions_.z;
  t.id = id_;
  t.pose_stamped.header.stamp = time_stamp_;
  t.pose_stamped.header.frame_id = std::string("/base");
  t.pose_stamped.pose = pose_;
  return t;
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

bool blockCompare(const ar_blocks::Block &a, const ar_blocks::Block &b)
{
   return a.pose_stamped.pose.position.x > b.pose_stamped.pose.position.x;
}

} // namespace nxr
