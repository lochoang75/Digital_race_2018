#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "lane_detect.h"
#include "carcontrol.h"
#include "sign_detect.h"

bool STREAM = true;

VideoCapture capture("video.avi");
DetectLane *detect;
CarControl *car;
int skipFrame = 1;

/* XML location */
String sign_cascade_left_name = ros::package::getPath("team211").append("/cascade_left_3.xml");
String sign_cascade_right_name = ros::package::getPath("team211").append("/cascade_right_2.xml");
String rock_name = ros::package::getPath("team211").append("/cascade_rock.xml");
String stack_box_name = ros::package::getPath("team211").append("/cascade_stack_box.xml");
String single_box_name = ros::package::getPath("team211").append("/cascade_single_box.xml");

/* CascadeClassifier object create */
CascadeClassifier sign_left;
CascadeClassifier sign_right;
CascadeClassifier rock;
CascadeClassifier stack_box;
CascadeClassifier single_box;
int idx = 1;

int sign_flag = 0;
bool turn_flag = false;

int speed = 50;

bool isObstruction = false, leftObstruction = false, rightObstruction = false;
int time_obstruction = 9;

int turn_times_right = 10;
int turn_times_left = 10;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        detect->update(cv_ptr->image);
        //update and check with 5 nearest records
        //detect->update(cv_ptr->image, times);

        if (car->isTurnRight == false && car->isTurnLeft == false)
        {
            if (isObstruction == false)
            {
                if (!detectSign(cv_ptr->image, sign_right, "right") && !detectSign(cv_ptr->image, sign_left, "left"))
                    car->driverCar(detect->getLeftLane(), detect->getRightLane(), speed, 40);
                else if (detectSign(cv_ptr->image, sign_right, "right"))
                {
                    car->isTurnRight = 1;
                    car->turnRightFlag = false;
                    
                }
                else if (detectSign(cv_ptr->image, sign_left, "left")) 
                {
                    car->isTurnLeft = 1;
                    car->turnLeftFlag = false;
                }
                else
                {
                    cout << "error\n";
                }

                //Check obstruction
                vector<Point> s1 = detectRock(cv_ptr->image, rock);
                vector<Point> s2 = detectStackBox(cv_ptr->image, stack_box);
                vector<Point> s3 = detectSingleBox(cv_ptr->image, single_box);

                if (!s1.empty())
                {
                    isObstruction = true;
                    if (s1[0].x + s1[1].x > 320)
                    {
                        rightObstruction = true;
                        time_obstruction = 0;
                    }
                        
                    else
                    {
                        leftObstruction = true;
                        time_obstruction = 0;
                    }
                }
                if (!s2.empty())
                {
                    isObstruction = true;
                    if (s2[0].x + s2[1].x > 320)
                    {
                        rightObstruction = true;
                        time_obstruction = 0;
                    }
                        
                    else
                    {
                        leftObstruction = true;
                        time_obstruction = 0;
                    }

                }
                if (!s3.empty())
                {
                    isObstruction = true;
                    if (s3[0].x + s3[1].x > 320)
                    {
                        rightObstruction = true;
                        time_obstruction = 0;
                    }
                        
                    else
                    {
                        leftObstruction = true;
                        time_obstruction = 0;
                    }
                }
            }

            else
            {
                cout << "Obstruction\n";
                if (time_obstruction < 10)
                {
                    if (rightObstruction)
                    {
                        car->driverCar(detect->getLeftLane(), detect->getMidLane(), speed-10, 20);
                    }
                    if (leftObstruction)
                    {
                        car->driverCar(detect->getMidLane(), detect->getRightLane(), speed-10, 20);
                    }
                    //i++
                }
                else 
                {
                    isObstruction = false;
                    rightObstruction = false;
                    leftObstruction = false;
                }
                time_obstruction++;
            }
        }
        else if (car->isTurnRight)
        {
            //Using a function to controll drive_right() to pass the turn
            
            if (!car->turnRightFlag)
            {
                //cout <<  "Right sign size: " << detectSign(cv_ptr->image, sign_right, "right") << endl;
                if  (detect->turnRightLane())//(detectSign(cv_ptr->image, sign_right, "right") > 50) 
                {
                    car->turnRightFlag = true;
                    car->drive_right();
                   
                }
                else
                {
                    cout << "a\n";
                    car->driverCar(detect->getMidLane(), detect->getRightLane(), speed-10, 20);
                }
            }
            else 
            {
               if (detect->turnRightLane2() == 1)
                {
                    car->driverCar(detect->getLeftLane(), detect->getRightLane(), speed, 40);
                    car->isTurnRight = false;
                    car->turnRightFlag = false;
                }
                else
                    car->drive_right();
            }
        }
        
        else if (car->isTurnLeft)
        {
            //Using a function to controll drive_right() to pass the turn
            //cout <<  "Left sign size: " << detectSign(cv_ptr->image, sign_left, "left") << endl;
            
            if (!car->turnLeftFlag)
            {
                if (detect->turnLeftLane())
                {
                    car->drive_left();
                    car->turnLeftFlag = true;
                }
                else car->driverCar(detect->getLeftLane(), detect->getMidLane(), speed-10, 20);

            }

            else
            {
                if (detect->turnLeftLane2() == 1)
                {
                    car->driverCar(detect->getLeftLane(), detect->getRightLane(), speed, 40);
                    car->isTurnLeft = false;
                    car->turnLeftFlag = false;
                }
                else car->drive_left();
            }
            
        }

        else
        {
            cout << "Error2\n";

        }
        
        
        


        // if (sign_flag == 0)
        // {
        //     if (detectSign(cv_ptr->image, sign_right, "turn_right"))
        //     {
        //         sign_flag = 1;
                
        //     }
        //     else if (detectSign(cv_ptr->image, sign_left, "turn_left"))
        //     {
        //         sign_flag = -1;
                
        //     }
        //     else 
        //     {
        //         //car->get_Velocity() < 45 ?
        //         //car->driverCar(detect->getLeftLane(), detect->getRightLane(), 120) :
        //         car->driverCar(detect->getLeftLane(), detect->getRightLane(), speed);

        //         //cout << car->get_Velocity() << endl;
        //     }

        // }    
        
        // else if (sign_flag == 1) 
        // {
            
        //     cout << turn_times_right << " " 
        //         << detectSign(cv_ptr->image, sign_right, "turn_right") << endl;
            
        //     if (detectSign(cv_ptr->image, sign_right, "turn_right") > 40)
        //         turn_flag = true;
            
        //     if (turn_flag)
        //     {
        //         if (turn_times_right != 0)
        //         {
        //             car->drive_right();
        //             turn_times_right--;
        //         }
        //         else
        //         { 
        //             turn_times_right = 10;
        //             sign_flag = 0;
        //             turn_flag = false;
        //         }
        //     }
        //     else{
        //         car->driverCar(detect->getLeftLane(), detect->getRightLane(), speed);
        //     }
        // }
        // else 
        // {
        //    cout << turn_times_left << " " 
        //         << detectSign(cv_ptr->image, sign_left, "turn_left") << endl;

        //     //if (detectSign(cv_ptr->image, sign_left) > 25)
        //     if (detectSign(cv_ptr->image, sign_left, "turn_left") > 40)
        //         turn_flag = true;

        //     if (turn_flag)
        //     {
        //         if (turn_times_left != 0)
        //         {
        //             car->drive_left(); 
        //             turn_times_left--;
        //         }
        //         else
        //         { 
        //             turn_times_left = 10;
        //             sign_flag = 0;
        //             turn_flag = false;
        //         }
        //     }
        //     else{
        //          car->driverCar(detect->getLeftLane(), detect->getRightLane(), speed);
        //     }
        // }
        
        // Create point  and detect rock 
        vector<Point> points = detectRock(cv_ptr->image, rock);
        if ( !points.empty())
        {
            // do stuff with rock detected
            cout<< "Rock(" << points[0].x << ","<< points[0].y <<")("<< points[1].x<<","<<points[1].y<<")"<< endl;

            // after that empty the vector for next function
            points.clear();
        };

        // Detect stack box
        points = detectStackBox(cv_ptr->image, stack_box);
        if ( !points.empty())
        {
            // do stuff with stackbox detected
            cout<< "Stack Box(" << points[0].x << ","<< points[0].y <<")("<< points[1].x<<","<<points[1].y<<")"<< endl;

            // after that empty the vector for next function
            points.clear();
        };

        // Detect single box
        points = detectSingleBox(cv_ptr->image, single_box);
        if ( !points.empty())
        {
            // do stuff with stackbox detected
            cout<< "Single Box(" << points[0].x << ","<< points[0].y <<")("<< points[1].x<<","<<points[1].y<<")"<< endl;

            // after that empty the vector for next function
            points.clear();
        };

        cv::imshow("View", cv_ptr->image);
        waitKey(1);
        //Write image for training
        /*
        string name = to_string(idx);
        string savepath = "/home/fallinlove/catkin_ws/image/101c" + name + ".jpg";
        imwrite(savepath, cv_ptr->image);
        idx++;
        waitKey(1);
        */

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
		printf("--(!) Error loading sign left detect\n");
		return 0;
	}
	
	if (!sign_right.load(sign_cascade_right_name))
	{
		printf("--(!) Error loading sign right detect\n");
		return 0;
	}
		  
	if (!rock.load(rock_name))
	{
		printf("--(!) Error loading rock detect \n");
		return 0;
	}

	if (!stack_box.load(stack_box_name))
	{
		printf("--(!) Error loading stackbox detect \n");
		return 0;
	}

    if (!single_box.load(single_box_name))
	{
		printf("--(!) Error loading stackbox detect \n");
		return 0;
	}
ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    // cv::namedWindow("Binary");
    // //cv::namedWindow("Threshold");
    // cv::namedWindow("Bird View");
    cv::namedWindow("Lane Detect");
    // //cv::namedWindow("Lane Detect1");
    // //cv::namedWindow("Shadow");
    // cv::namedWindow("Merge");
    // //cv::namedWindow("Final");
    // cv::namedWindow("Debug");
    // cv::namedWindow("Fill road");

    detect = new DetectLane();
    car = new CarControl();

    if (STREAM) {
        cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("team211_image", 1, imageCallback);


        ros::spin();
    } 
    else {
        videoProcess();
    }
    cv::destroyAllWindows();
}
