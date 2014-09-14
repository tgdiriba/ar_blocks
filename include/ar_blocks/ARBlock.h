#ifndef ARBLOCK_H
#define ARBLOCK_H

#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <ar_track_alvar/Filter.h>
#include <ar_track_alvar/Kalman.h>
#include <tf/transform_datatypes.h>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <string>
#include <cmath>
#include <ar_blocks/Block.h>
#include <ar_blocks/ARBlockFilter.h>

namespace nxr {

enum BLOCK_TYPE {
	BLOCK_A,
	BLOCK_B,
	BLOCK_C,
	BLOCK_NUM
};

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
	int topSide();
	moveit_msgs::CollisionObject toCollisionObject(std::string planning_frame = std::string("base"));	
  ar_blocks::Block toBlockMsg();
	std::string getStringId();
  
  ARBlockFilter filter_;
	geometry_msgs::Pose pose_;
  ros::Time time_stamp_;
	unsigned int block_type_; 
	unsigned int id_;
	
	union {
		float dims_[3];
		Dimensions dimensions_;
	};
  
	// Debugging
	void printInfo();

};
  
bool blockCompare(const ar_blocks::Block &a, const ar_blocks::Block &b);

}

#endif
