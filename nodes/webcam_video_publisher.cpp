#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

using namespace cv;
using namespace cv_bridge;
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nxr_webcam_video");

	VideoCapture vid_cap(0);
	
	if(!vid_cap.isOpened()) {
		ROS_ERROR("Cannot open webcam video stream");
	}
	else {
		
		ros::NodeHandle nh;
		image_transport::ImageTransport imt(nh);
		image_transport::Publisher webcam_publisher = imt.advertise("nxr_webcam_video", 1);

		ros::Rate looper(100);
		Mat frame;
		
		while(ros::ok()) {
			vid_cap >> frame;
			
			if(!frame.data) ROS_ERROR("Invalid image frame from webcam");
			else {
				CvImage cvi_out(std_msgs::Header(), "bgr8", frame);
				webcam_publisher.publish(cvi_out.toImageMsg());
			}
			
			looper.sleep();
		}
	}	
}
