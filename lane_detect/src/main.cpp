#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "detectlane.h"
#include "carcontrol.h"
#include "sign_detect.h"

bool STREAM = true;

VideoCapture capture("video.avi");
DetectLane *detect;
CarControl *car;
int skipFrame = 1;

String sign_cascade_left_name = ros::package::getPath("lane_detect").append("/cascade_left.xml");
String sign_cascade_right_name = ros::package::getPath("lane_detect").append("/cascade_right_2.xml");

CascadeClassifier sign_left;
CascadeClassifier sign_right;


int sign_flag = 0;
bool turn_flag = false;

int turn_times_right = 10;
int turn_times_left = 15;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::imshow("View", cv_ptr->image);
        waitKey(1);
        detect->update(cv_ptr->image);
        //car->driverCar(detect->getLeftLane(), detect->getRightLane(), 60);
        
        if (sign_flag == 0)
        {
            if (detectSign(cv_ptr->image, sign_right))
            {
                sign_flag = 1;
                
            }
            else if (detectSign(cv_ptr->image, sign_left))
            {
                sign_flag = -1;
                
            }
            else 	
                car->driverCar(detect->getLeftLane(), detect->getRightLane(), 60);
        }    
        
        else if (sign_flag == 1) 
        {
            
            cout << turn_times_right << " " 
                << detectSign(cv_ptr->image, sign_right) << endl;
            
            if (detectSign(cv_ptr->image, sign_right) > 40)
                turn_flag = true;
            
            if (turn_flag)
            {
                if (turn_times_right != 0)
                {
                    car->drive_right();
                    turn_times_right--;
                }
                else
                { 
                    turn_times_right = 10;
                    sign_flag = 0;
                    turn_flag = false;
                }
            }
            else car->driverCar(detect->getLeftLane(), detect->getRightLane(), 60);
        }
        else 
        {
           cout << turn_times_left << " " 
                << detectSign(cv_ptr->image, sign_left) << endl;

            if (detectSign(cv_ptr->image, sign_left) > 25)
                turn_flag = true;

            if (turn_flag)
            {
                if (turn_times_left != 0)
                {
                    car->drive_left(); 
                    turn_times_left--;
                }
                else
                { 
                    turn_times_left = 15;
                    sign_flag = 0;
                    turn_flag = false;
                }
            }
            else car->driverCar(detect->getLeftLane(), detect->getRightLane(), 40);
        }
    
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}   

void videoProcess()
{
    Mat src;
    while (true)
    {
        capture >> src;
        if (src.empty()) break;
        
        imshow("View", src);
        detect->update(src);
        waitKey(30);
    }
}

int main(int argc, char **argv)
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
		  
ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    cv::namedWindow("Binary");
    cv::namedWindow("Threshold");
    cv::namedWindow("Bird View");
    cv::namedWindow("Lane Detect");
    cv::namedWindow("Lane Detect1");
    cv::namedWindow("split Layer");
    cv::namedWindow("Shadow");
    cv::namedWindow("Merge");
    //cv::namedWindow("Final");

    detect = new DetectLane();
    car = new CarControl();

    if (STREAM) {
        cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("Team1_image", 1, imageCallback);

        ros::spin();
    } 
    else {
        videoProcess();
    }
    cv::destroyAllWindows();
}
