#ifndef ARBLOCKSSERVERTEST_H
#define ARBLOCKSSERVERTEST_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <map>
#include <ar_blocks/BuildStructureAction.h>

namespace nxr {

class ARBlocksServerTest
{
public:
  typedef actionlib::SimpleActionClient<ar_blocks::BuildStructureAction> Client;
  
  ARBlocksServerTest();
  void runAllTests();
  void sendGoalTest();
  
private:
  // ROS
  ros::NodeHandle nh_;
  Client ar_blocks_client_;
  
private:
  void arBlocksDoneCallback(const actionlib::SimpleClientGoalState &state,
                            const ar_blocks::BuildStructureResultConstPtr &result);
  void arBlocksActiveCallback();
  void arBlocksFeedbackCallback(const ar_blocks::BuildStructureFeedbackConstPtr &feedback);

  ar_blocks::BuildStructureGoal goal_structure_;
  
};

} // namespace nxr

#endif // ARBLOCKSSERVERTEST_H
