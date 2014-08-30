#include <ar_blocks/ARBlocksServerTest.h>
#include <ros/ros.h>

namespace nxr {

ARBlocksServerTest::ARBlocksServerTest() :
  ar_blocks_client_("/ar_blocks_environment/ar_blocks_action_server", true)
{
  ar_blocks_client_.waitForServer();
}

void ARBlocksServerTest::arBlocksDoneCallback(const actionlib::SimpleClientGoalState &state,
                          const ar_blocks::BuildStructureResultConstPtr &result)
{
  ROS_INFO("Goal reached.");
}

void ARBlocksServerTest::arBlocksActiveCallback()
{
  ROS_INFO("AR Blocks server actively pursuing goal state.");
}

void ARBlocksServerTest::arBlocksFeedbackCallback(const ar_blocks::BuildStructureFeedbackConstPtr &feedback)
{
  // Count the number of blocks that were successfully placed and take a percentage of the total
  double valid_blocks = 0.0;
  double total_blocks = 0.0;
  for(int i = 0; i < feedback->updated_structure.layers.size(); i++) valid_blocks += feedback->updated_structure.layers[i].blocks.size();
  for(int i = 0; i < goal_structure_.goal_structure.layers.size(); i++) total_blocks += goal_structure_.goal_structure.layers[i].blocks.size();
  
  int percent = ((int)(total_blocks) != 0 && total_blocks >= 0.0) ? (int)(valid_blocks/total_blocks) : 0;
  
  ROS_INFO("Feedback message received. Reached %d%%...", percent);
}

void ARBlocksServerTest::runAllTests()
{
  sendGoalTest();
}

void ARBlocksServerTest::sendGoalTest()
{
  ar_blocks_client_.sendGoal( goal_structure_,
                              boost::bind(&ARBlocksServerTest::arBlocksDoneCallback, this, _1, _2),
                              boost::bind(&ARBlocksServerTest::arBlocksActiveCallback, this),
                              boost::bind(&ARBlocksServerTest::arBlocksFeedbackCallback, this, _1) );
  ROS_INFO("Goal state sent...waiting on response.");
}

} // namespace nxr
