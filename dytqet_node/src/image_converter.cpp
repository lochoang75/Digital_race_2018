#include <ros/ros.h>
#include <ros/package.h>
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
String sign_cascade_left_name = ros::package::getPath("dytqet_node").append("/cascade_left.xml");
String sign_cascade_right_name = ros::package::getPath("dytqet_node").append("/cascade_right_2.xml");

string name_left = "left";
string name_right = "right";
CascadeClassifier sign_left;
CascadeClassifier sign_right;

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
		detectAndDisplay(cv_ptr->image, sign_left, name_left);
		detectAndDisplay(cv_ptr->image, sign_right, name_right);
		laneDetect(cv_ptr->image);

	}
};

int main(int argc, char* argv[])
{
	/* Load cascade */
	if (!sign_left.load(sign_cascade_left_name)) 
	{
		printf("--(!) Error loading \n");
		return 0;
	}
	
	if (!sign_right.load(sign_cascade_right_name))
	{
		printf("--(!) Error loading \n");
		return 0;
	}

	ros::init(argc, argv,"CNN");
	ImageConverter ic;
	ros::spin();
	return 0;
}
   

