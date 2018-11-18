#ifndef SIGN_DETECT_H
#define SIGN_DETECT_H 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <stdio.h>
using namespace cv;
using namespace std;

/* Detect and show result */
void detectAndDisplay(Mat frame, CascadeClassifier sign);

#endif
