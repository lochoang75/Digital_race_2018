#ifndef LANE_DETECT_H
#define LANE_DETECT_H
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;

/* Detect lane and return coodinates */
int laneDetect(Mat inimg);
#endif
