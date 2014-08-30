#include <ar_blocks/ARBlocksServerTest.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ar_blocks_server_test");
  ros::NodeHandle nh;
  
  nxr::ARBlocksServerTest ar_st;
  ar_st.runAllTests();
  
  return 0;
}
