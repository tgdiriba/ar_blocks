#ifndef ARBLOCK_H
#define ARBLOCK_H

#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <ar_track_alvar/Filter.h>
#include <ar_track_alvar/Kalman.h>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <string>
#include <ar_blocks/Block.h>

namespace nxr {

enum BLOCK_TYPE {
	BLOCK_A,
	BLOCK_B,
	BLOCK_C,
	BLOCK_NUM
};

// const float g_block_sizes[BLOCK_NUM]; // = { 0.063, 0.051, 0.045 };

struct Dimensions {
	float x;
	float y;
	float z;
};

class ARBlock {
public:
	
	ARBlock(unsigned int b_type = BLOCK_A);
	ARBlock(unsigned int id, const boost::shared_ptr<alvar::Kalman> &k);
	ARBlock(float *dims, int id = 0);	
	moveit_msgs::CollisionObject toCollisionObject(std::string planning_frame = std::string("base"));	
	alvar::Kalman toKalman();
  ar_blocks::Block toBlockMsg();
	std::string getStringId();
  
	geometry_msgs::Pose pose_;
  ros::Time time_stamp_;
	unsigned int block_type_; 
	unsigned int id_;
	
	union {
		float dims_[3];
		Dimensions dimensions_;
	};
  
  static bool blockCompare(const ar_blocks::Block &a, const ar_blocks::Block &b);

	// Debugging
	void printInfo();

};

}

#endif
