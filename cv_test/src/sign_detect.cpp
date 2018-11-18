#include "sign_detect.h"
void detectAndDisplay(Mat frame, CascadeClassifier sign)
{

	vector<Rect> signs;
	Mat frame_gray;

	cvtColor( frame, frame_gray, CV_BGR2GRAY);
	equalizeHist( frame_gray, frame_gray);

	/* Detect sign */
	sign.detectMultiScale(frame_gray, signs, 1.2, 5,  0|CV_HAAR_SCALE_IMAGE,Size(30, 30) );

	for (size_t i = 0; i < signs.size(); i++)
	{
		int x  = signs[i].x;
		int y = signs[i].y;
		int w = signs[i].width;
		int h = signs[i].height;

		printf("Position: x: %d, y: %d. \n Size:, width: %d, height: %d \n", x, y, w, h);
		
		int end_x = x + w;
		int end_y = y + h;
		Point start;
		Point end;
		start.x = x;
		start.y =y;
		end.x = end_x;
		end.y = end_y;

		rectangle (frame, start, end, cv::Scalar(0,255,0) , 2); 
		imshow("Sign detection", frame);
		waitKey(10);
	}
}

