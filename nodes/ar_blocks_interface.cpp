#include <QMainWindow>
#include <baxter_core_msgs/OpenCamera.h>
#include <baxter_core_msgs/CloseCamera.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/CameraSettings.h>
#include <boost/thread.hpp>
#include <string>
#include <ar_blocks/ARBlocksInterface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ar_blocks_interface");
  ros::NodeHandle nh;
  
  QApplication app( argc, argv );
  
  nxr::ARBlocksInterface arbi;
  arbi.show();
  
	return app.exec();
}
