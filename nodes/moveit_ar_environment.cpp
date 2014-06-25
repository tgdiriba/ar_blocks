#include <nxr_baxter/ARWorldBuilder.h>

using namespace moveit;
using namespace std;
using namespace nxr;

int main(int argc, char **argv)
{
	ros::init( argc, argv, "nxr_ar_blocks" );
	ros::NodeHandle nh;
		
	ARWorldBuilder block_world;
	ROS_INFO("Successfully built AR World...");
	
	ros::AsyncSpinner spinner(2);
	spinner.start();

	ros::waitForShutdown();
}
