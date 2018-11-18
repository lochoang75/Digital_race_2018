#include "sign_detect.h"
void detectAndDisplay(Mat frame, CascadeClassifier sign, string name)
{
	/* Create gray frame */
	vector<Rect> signs;
	Mat frame_gray;

	/* Convert color */
	cvtColor( frame, frame_gray, CV_BGR2GRAY);
	equalizeHist( frame_gray, frame_gray);

	int stroke = 2;

	/* Detect sign */
	sign.detectMultiScale(frame_gray, signs, 1.1, 3,  0|CV_HAAR_SCALE_IMAGE,Size(10, 10) );

	for (size_t i = 0; i < signs.size(); i++)
	{
		int x  = signs[i].x;
		int y = signs[i].y;
		int w = signs[i].width;
		int h = signs[i].height;

		printf("Position: x: %d, y: %d. \n Size:, width: %d, height: %d \n", x, y, w, h);
		
		int end_x = x + w;
		int end_y = y + h;

		/* Top left corner */
		Point start;

		/* Bot right corner */
		Point end;

		/* Text position */
		Point text;
		start.x = x;
		start.y =y;
		
		end.x = end_x;
		end.y = end_y;

		text.x = start.x;
		text.y = end.y + 20;
		
		/* Draw rectangle */
		rectangle (frame, start, end, cv::Scalar(0,255,0) , 2); 

		/* Put text */
		cv::putText(frame, name, text, FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), stroke, LINE_AA); 
	}
}

