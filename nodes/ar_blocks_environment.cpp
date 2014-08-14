#include <string>
#include <baxter_core_msgs/OpenCamera.h>
#include <baxter_core_msgs/CloseCamera.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/CameraSettings.h>
#include <ar_blocks/ARWorldBuilder.h>

using namespace moveit;
using namespace std;
using namespace nxr;

int main(int argc, char **argv)
{
	ros::init( argc, argv, "nxr_ar_blocks" );
	ros::NodeHandle nh("~");
	
	// Configure the hand cameras and their resolution
	ros::ServiceClient ccam = nh.serviceClient<baxter_core_msgs::CloseCamera>("/cameras/close");
	baxter_core_msgs::CloseCamera close_head_camera;
	close_head_camera.request.name = string("head_camera");
	
	ros::ServiceClient sc = nh.serviceClient<baxter_core_msgs::OpenCamera>("/cameras/open");
	baxter_core_msgs::OpenCamera open_left_camera, open_right_camera;
	open_left_camera.request.name = string("left_hand_camera");
	open_left_camera.request.settings.width = 1280;
	open_left_camera.request.settings.height= 800;
	open_right_camera.request.name = string("right_hand_camera");
	open_right_camera.request.settings.width = 1280;
	open_right_camera.request.settings.height= 800;
	
	if(!ccam.call(close_head_camera)) ROS_ERROR("Failed to close the head camera...");
	if(!sc.call(open_left_camera) || !sc.call(open_right_camera)) {
		ROS_ERROR("Failed to set the camera resolution...");
	}
		
	ARWorldBuilder block_world;
	ROS_INFO("Successfully built AR World...");
	// block_world.runAllTests();
	
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	while(true) {
		ROS_INFO("\n\nPrinting out world info..");
		block_world.printInfo();
		ros::Duration(1.0).sleep();
	}
	// ros::waitForShutdown();
	
	return 0;
}
