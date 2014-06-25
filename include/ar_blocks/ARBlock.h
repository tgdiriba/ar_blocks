#ifndef ARBLOCK_H
#define ARBLOCK_H

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <sstream>
#include <string>

namespace nxr {

enum BLOCK_TYPE {
	BLOCK_A,
	BLOCK_B,
	BLOCK_C,
	BLOCK_NUM
};

// const float g_block_sizes[BLOCK_NUM]; // = { 0.063, 0.051, 0.045 };

typedef struct _dimensions {
	float x;
	float y;
	float z;
} dimensions;

class ARBlock {
public:
	
	ARBlock(unsigned int b_type = BLOCK_A);
	ARBlock(float *dims, int id = 0);	
	moveit_msgs::CollisionObject toCollisionObject(std::string planning_frame = std::string("/base"));	

	geometry_msgs::Pose pose_;
	unsigned int block_type_; 
	unsigned int id_;
	
	union {
		float dims_[3];
		dimensions dimensions_;
	};
};

}

#endif
