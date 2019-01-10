#include "sign_detect.h"

/* Detect sign */
int detectSign(Mat frame, CascadeClassifier sign, String name)
{
	return	detectOject(frame, sign, name, 1.1);
}

/* Detect rock */
int detectRock(Mat frame, CascadeClassifier rock, String name)
{
	return detectOject(frame, rock, name, 1.2);
	
}
/* Detect object*/
int detectOject(Mat frame, CascadeClassifier object, String name, double scale)
{

	/* Create gray frame */
	vector<Rect> objects;
	Mat frame_gray;

	/* Convert color */
	cvtColor( frame, frame_gray, CV_BGR2GRAY);
	equalizeHist( frame_gray, frame_gray);

	int stroke = 2;

	//Init position (x,y) and size (width, height) variables
	int x, y, w, h;
	
	/* Detect sign */
	object.detectMultiScale(frame_gray, objects, scale , 2,  0|CV_HAAR_SCALE_IMAGE,Size(5, 5) );

	/* Get last object border */
	for (size_t i = 0; i < objects.size(); i++)
	{
		x = objects[i].x;
		y = objects[i].y;
		w = objects[i].width;
		h = objects[i].height;
	}

	if (objects.size() > 0)
	{
		// Sign border 
		cv::Rect object_border(x, y, w, h);

		// Draw to image
		cv::rectangle(frame,object_border, cv::Scalar(0, 255, 0));

		// Text image
		cv::putText(frame, name, cv::Point(x, w +h), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 0, 0));
	}

	return objects.size() ? w : 0;
}