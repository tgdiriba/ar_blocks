#include <ar_blocks/ARBlockFilter.h>

namespace nxr {

using namespace cv;

ARBlockFilter::ARBlockFilter(geometry_msgs::Pose p) :
  position_filter_(6, 3, 0),
  orientation_filter_(8, 4, 0),
  current_time_(ros::Time::now())
{
  // Position Filter Setup
  position_filter_.transitionMatrix = Mat_<double>(6, 6);
  setIdentity(position_filter_.transitionMatrix);
  position_filter_.measurementMatrix = *(Mat_<double>(3,6)
    <<  1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0);
  position_filter_.statePre = *(Mat_<double>(6,1)
    <<  p.position.x,
        p.position.y,
        p.position.z,
        0, 0, 0);
  setIdentity(position_filter_.measurementNoiseCov, Scalar::all(2));
  setIdentity(position_filter_.errorCovPost, Scalar::all(0.01));
  
  // Orientation Filter Setup
  setIdentity(orientation_filter_.transitionMatrix);
  orientation_filter_.measurementMatrix = *(Mat_<double>(4, 8)
    <<  1, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0);
  orientation_filter_.statePre = *(Mat_<double>(8,1)
    <<  p.orientation.x, 
        p.orientation.y, 
        p.orientation.z,
        p.orientation.w, 
        0, 0, 0, 0);
  setIdentity(orientation_filter_.measurementNoiseCov, Scalar::all(2));
  setIdentity(orientation_filter_.errorCovPost, Scalar::all(0.01));
}

void ARBlockFilter::reset(geometry_msgs::Pose p, ros::Time t)
{
  position_filter_.gain = Mat::zeros(  position_filter_.gain.rows,
                                      position_filter_.gain.cols,
                                      position_filter_.gain.type()  );
  
  position_filter_.statePre = *(Mat_<double>(6,1)
    <<  p.position.x,
        p.position.y,
        p.position.z,
        0, 0, 0);
  
  orientation_filter_.statePre = *(Mat_<double>(8,1)
    <<  p.orientation.x, 
        p.orientation.y, 
        p.orientation.z,
        p.orientation.w, 
        0, 0, 0, 0);
}

void ARBlockFilter::update(geometry_msgs::Pose p, ros::Time latest_time)
{
  double dt = (latest_time - current_time_).toSec();
  current_time_ = latest_time;
  
  position_filter_.transitionMatrix.at<double>(0, 3) = dt;
  position_filter_.transitionMatrix.at<double>(1, 4) = dt;
  position_filter_.transitionMatrix.at<double>(2, 5) = dt;
  
  orientation_filter_.transitionMatrix.at<double>(0, 4) = dt;
  orientation_filter_.transitionMatrix.at<double>(1, 5) = dt;
  orientation_filter_.transitionMatrix.at<double>(2, 6) = dt;
  orientation_filter_.transitionMatrix.at<double>(3, 7) = dt;
  
  position_filter_.predict();
  orientation_filter_.predict();
  
  Mat_<double> position_measurement(3,1), orientation_measurement(4,1);
  position_measurement(0) = p.position.x;
  position_measurement(1) = p.position.y;
  position_measurement(2) = p.position.z;
  
  orientation_measurement(0) = p.orientation.x; 
  orientation_measurement(1) = p.orientation.y; 
  orientation_measurement(2) = p.orientation.z; 
  orientation_measurement(3) = p.orientation.w; 
  
  Mat predicted_position = position_filter_.correct(position_measurement);
  Mat predicted_orientation = orientation_filter_.correct(orientation_measurement);
  
  predicted_pose_.position.x = predicted_position.at<double>(0);
  predicted_pose_.position.y = predicted_position.at<double>(1);
  predicted_pose_.position.z = predicted_position.at<double>(2);
  
  tf::Quaternion q( predicted_orientation.at<double>(0),
                    predicted_orientation.at<double>(1),
                    predicted_orientation.at<double>(2),
                    predicted_orientation.at<double>(3) );
  q.normalize();
  tf::Vector3 v = q.getAxis();
  tfScalar W = q.getW();
  
  predicted_pose_.orientation.x = v.m_floats[0];
  predicted_pose_.orientation.y = v.m_floats[1];
  predicted_pose_.orientation.z = v.m_floats[2];
  predicted_pose_.orientation.w = W;
  
  // NOTE: Re-evaluate orientation filter as quaternion estimation should be non-linear in this case. Currently using a normalized linear filter instead.
}

} // namespace nxr
