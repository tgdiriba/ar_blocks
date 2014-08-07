#ifndef GRIPPERJOINTUPDATER_H
#define GRIPPERJOINTUPDATER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/EndEffectorState.h>


namespace nxr {

using namespace ros;
using namespace std;

class GripperJointUpdater
{
public:
	GripperJointUpdater();
	
protected:
	NodeHandle nh_;
	Publisher gripper_state_publisher_;
	Subscriber left_gripper_state_subscriber_, right_gripper_state_subscriber_;
	baxter_core_msgs::EndEffectorState::ConstPtr left_endeffector_state_;
	baxter_core_msgs::EndEffectorState::ConstPtr right_endeffector_state_;

	void gripperJointStatePublisher(sensor_msgs::JointState &js);
	void leftEndEffectorStateUpdater(const baxter_core_msgs::EndEffectorState::ConstPtr &js);
	void rightEndEffectorStateUpdater(const baxter_core_msgs::EndEffectorState::ConstPtr &js);
};

} // namespace nxr

#endif
