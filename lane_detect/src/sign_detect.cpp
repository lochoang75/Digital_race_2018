#include "sign_detect.h"
int detectSign(Mat frame, CascadeClassifier sign)
{

	/* Create gray frame */
	vector<Rect> signs;
	Mat frame_gray;

	/* Convert color */
	cvtColor( frame, frame_gray, CV_BGR2GRAY);
	equalizeHist( frame_gray, frame_gray);

	int stroke = 2;

	//Init position (x,y) and size (width, height) variables
	int x, y, w, h;
	
	/* Detect sign */
	sign.detectMultiScale(frame_gray, signs, 1.1, 2,  0|CV_HAAR_SCALE_IMAGE,Size(5, 5) );

	for (size_t i = 0; i < signs.size(); i++)
	{
		x = signs[i].x;
		y = signs[i].y;
		w = signs[i].width;
		h = signs[i].height;
	}
	return signs.size() ? w : 0;
	
}

