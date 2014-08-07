#include <ar_blocks/GripperJointUpdater.h>


namespace nxr {

GripperJointUpdater::GripperJointUpdater() :
	nh_()
{
	gripper_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/robot/joint_state",100);
	left_gripper_state_subscriber_ = nh_.subscribe("/robot/end_effector/left_side/state", 100, &GripperJointUpdater::leftEndEffectorStateUpdater, this);
	right_gripper_state_subscriber_ = nh_.subscribe("/robot/end_effector/right_side/state", 100, &GripperJointUpdater::rightEndEffectorStateUpdater, this);
}

void GripperJointUpdater::leftEndEffectorStateUpdater(const baxter_core_msgs::EndEffectorState::ConstPtr &js)
{
	left_endeffector_state_ = js;
}

void GripperJointUpdater::rightEndEffectorStateUpdater(const baxter_core_msgs::EndEffectorState::ConstPtr &js)
{
	right_endeffector_state_ = js;
}

void GripperJointUpdater::gripperJointStatePublisher(sensor_msgs::JointState &js)
{
	js.header.frame_id = "base";
	js.header.stamp = Time::now();
	js.name.push_back("left_gripper_l_finger_joint");
	js.name.push_back("left_gripper_r_finger_joint");
	js.name.push_back("right_gripper_l_finger_joint");
	js.name.push_back("right_gripper_r_finger_joint");

	js.velocity.push_back(0);
	js.velocity.push_back(0);
	js.velocity.push_back(0);
	js.velocity.push_back(0);

	js.effort.push_back(left_endeffector_state_->force);
	js.effort.push_back(left_endeffector_state_->force);
	js.effort.push_back(right_endeffector_state_->force);
	js.effort.push_back(right_endeffector_state_->force);

	// Making use of Dave Coleman's default uncalibrated conversion from Baxter's published endeffector state to gripper joint value
	double left_position = -0.0125 + 0.022*(left_endeffector_state_->position/100);
	double right_position = -0.0125 + 0.022*(right_endeffector_state_->position/100);
	
	js.position.push_back(left_position);
	js.position.push_back(left_position*-1);
	js.position.push_back(right_position);
	js.position.push_back(right_position*-1);

}

} // namespace nxr

