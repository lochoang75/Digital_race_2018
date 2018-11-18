#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "lane_detect.h"
#include "sign_detect.h"

using namespace cv;
using namespace std;

ros::Publisher steer_publisher;
ros::Publisher speed_publisher;
int counter = 0;

/** Global variable */
String sign_cascade_name = "/home/fallinlove/catkin_ws/src/dytqet_node/cascade.xml";
CascadeClassifier sign;

std::string window_name = "Sign detection";
RNG rng(12345);


class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber CNN;

public:
    ImageConverter(): it_(nh_) {
        //Subcribe to input video feed and publish output video feed
        CNN  = it_.subscribe("Dytqet_image", 1, &ImageConverter::imageCb, this);
	}

	~ImageConverter()
	{
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try 
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge execption: %s", e.what());
			return;
		}
		
		waitKey(10);
		laneDetect(cv_ptr->image);
		detectAndDisplay(cv_ptr->image, sign);
	}
};

int main(int argc, char* argv[])
{
	/* Load cascade */
	if (!sign.load(sign_cascade_name)) 
	{
		printf("--(!) Error loading \n");
		return 0;
	}
	ros::init(argc, argv,"CNN");
	ImageConverter ic;
	ros::spin();
	return 0;
}
   

