#include <ar_blocks/GripperJointUpdater.h>


int main(int argc, char **argv)
{
	ros::NodeHandle nh;
	
	ROS_INFO("Bringing up the Gripper Joint Updater.");
	nxr::GripperJointUpdater gju;
	ROS_INFO("Succesffuly spawned the Gripper Joint Updater...");
	
	ros::spin();

	return 0;
}
