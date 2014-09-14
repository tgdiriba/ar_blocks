#ifndef ARBLOCKFILTER_H
#define ARBLOCKFILTER_H 

#include <opencv2/video/tracking.hpp>
#include <geometry_msgs/Pose.h>

namespace nxr {
  
class ARBlockFilter {
public:
  ARBlockFilter(geometry_msgs::Pose p = geometry_msgs::Pose());
  
  cv::KalmanFilter position_filter_;
  cv::KalmanFilter orientation_filter_;
  geometry_msgs::Pose predicted_pose_;
  
  void reset(geometry_msgs::Pose p, ros::Time t = ros::Time::now());
  void update(geometry_msgs::Pose p, ros::Time latest_time = ros::Time::now());
  
  ros::Time current_time_;
};
  
} // namespace nxr

#endif // ARBLOCKFILTER_H
