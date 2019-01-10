#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>

#define NUMBER_ELEMENTS 31

using namespace std;
using namespace cv;

class DetectLane
{
public:
    DetectLane();
    ~DetectLane();

    void update(const Mat &src);
    
    vector<Point> getLeftLane();
    vector<Point> getRightLane();
    vector<Point> getDesireLine();
    

    static int slideThickness;

    static int BIRDVIEW_WIDTH;
    static int BIRDVIEW_HEIGHT;

    static int VERTICAL;
    static int HORIZONTAL;

    //static int NUMBER_ELEMENTS;


    
    static Point null; // 

private:
    Mat preProcess(const Mat &src);

    Mat morphological(const Mat &imgHSV);
    Mat birdViewTranform(const Mat &source);
    void fillLane(Mat &src);
    vector<Mat> splitLayer(const Mat &src, int dir = VERTICAL);
    
    vector<vector<Point> > centerRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    bool fillRoadSide(const vector<Mat> &src, vector<vector<Point> > &points_fill);
    void detectLeftRight(const vector<vector<Point> > &points);
    
    void update_desireLine(const vector<vector<Point> > &points);
    void update_last5Line(const vector<Point> &points);

    Mat laneInShadow(const Mat &src);

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};
    //int minShadowTh[3] = {90, 43, 36};
    //int maxShadowTh[3] = {120, 81, 171};
    int minShadowTh[3] = {0, 20, 36};
    int maxShadowTh[3] = {255, 97, 171};
    //int minLaneInShadow[3] = {90, 43, 97};
    //int maxLaneInShadow[3] = {120, 80, 171};
    int minLaneInShadow[3] = {0, 20, 97};
    int maxLaneInShadow[3] = {255, 97, 171};
    int binaryThreshold = 180;

    int skyLine = 85;
    int shadowParam = 40;

    vector<Point> leftLane, rightLane, desireLine;
    vector<vector<Point> > last5desireLine, last5left, last5right;

};

#endif
