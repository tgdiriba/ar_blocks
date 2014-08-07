#include <string>
#include <baxter_core_msgs/OpenCamera.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/CameraSettings.h>
#include <ar_blocks/ARWorldBuilder.h>

using namespace moveit;
using namespace std;
using namespace nxr;

int main(int argc, char **argv)
{
	ros::init( argc, argv, "nxr_ar_blocks" );
	ros::NodeHandle nh;
	
	// Configure the camera resolution
	ros::ServiceClient sc = nh.serviceClient<baxter_core_msgs::OpenCamera>("/camera/open");
	baxter_core_msgs::OpenCamera open_left_camera, open_right_camera;
	open_left_camera.request.name = string("left");
	open_left_camera.request.settings.width = 1280;
	open_left_camera.request.settings.height= 800;
	open_right_camera.request.name = string("right");
	open_right_camera.request.settings.width = 1280;
	open_right_camera.request.settings.height= 800;
	
	if(!sc.call(open_left_camera) || !sc.call(open_right_camera)) {
		ROS_ERROR("Failed to set the camera resolution...");
	}
		
	ARWorldBuilder block_world;
	ROS_INFO("Successfully built AR World...");
	
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	ros::waitForShutdown();
	
	return 0;
}
