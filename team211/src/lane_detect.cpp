#include "lane_detect.h"

int min(int a, int b)
{
    return a < b ? a : b;
}

int DetectLane::slideThickness = 10;
int DetectLane::BIRDVIEW_WIDTH = 240;
int DetectLane::BIRDVIEW_HEIGHT = 320;
int DetectLane::VERTICAL = 0;
int DetectLane::HORIZONTAL = 1;
Point DetectLane::null = Point();

//vector<vector<Point> > DetectLane::straightLine[0][0].x = 134;
//straightLine[0][0].y = 4;

//straightLine[0][1].x = 99;
//straightLine[0][1].y = 4;


DetectLane::DetectLane() {
    /*
    cvCreateTrackbar("LowH", "Threshold", &minThreshold[0], 179);
    cvCreateTrackbar("HighH", "Threshold", &maxThreshold[0], 179);

    cvCreateTrackbar("LowS", "Threshold", &minThreshold[1], 255);
    cvCreateTrackbar("HighS", "Threshold", &maxThreshold[1], 255);

    cvCreateTrackbar("LowV", "Threshold", &minThreshold[2], 255);
    cvCreateTrackbar("HighV", "Threshold", &maxThreshold[2], 255);

    cvCreateTrackbar("Shadow Param", "Threshold", &shadowParam, 255);
    */
    
}

DetectLane::~DetectLane(){}



vector<Point> DetectLane::getLeftLane()
{
    return leftLane;
}

vector<Point> DetectLane::getRightLane()
{
    return rightLane;
}

void DetectLane::update(const Mat &src)
{
    Mat img = preProcess(src);
    
    vector<Mat> layers1 = splitLayer(img);
    vector<vector<Point> > points1 = centerRoadSide(layers1);
    //vector<Mat> layers2 = splitLayer(img, HORIZONTAL);
    vector<vector<Point> > points2;
    //cout << points1[points1.size()-1].size() << endl;

    Mat birdView, birdView1, lane;

    //Create 2 black images
    birdView = Mat::zeros(img.size(), CV_8UC3);
    
    lane = Mat::zeros(img.size(), CV_8UC3);

    for (int i = 0; i < points1.size(); i++)
    {
        for (int j = 0; j < points1[i].size(); j++)
        {
            //Point temp_left = {134, 4 + 10*i};
            //Point temp_right = {99, 4 + 10*i};
            //cout << i << ": " << points1[i][j].x << " " << points1[i][j].y << "; ";
            circle(birdView, points1[i][j], 1, Scalar(0,0,255), 2, 8, 0 );
            //j ? circle(birdView, temp_right, 1, Scalar(0,255,0), 1, 8, 0 ) : circle(birdView, temp_left, 1, Scalar(0,255,0), 1, 8, 0 );
        }
        //cout << endl;
    }
   
    // if (fillRoadSide(layers1, points2))
    // {
    //     for (int i = 0; i < points2.size(); i++)
    //     {
    //         for (int j = 0; j < points2[i].size(); j++)
    //         {
                
    //             circle(birdView1, points2[i][j], 1, Scalar(0,255,0), 2, 8, 0 );
               
    //         }
    //     }
    //     if (!birdView1.empty())
    //     imshow("Fill road", birdView1);
    // }
    
    
    
 
    detectLeftRight(points1);

    for (int i = 1; i < leftLane.size(); i++)
    {
        if (leftLane[i] != null)
        {
            circle(lane, leftLane[i], 1, Scalar(0,0,255), 2, 8, 0 );
        }
    }

    for (int i = 1; i < rightLane.size(); i++)
    {
        if (rightLane[i] != null) {
            circle(lane, rightLane[i], 1, Scalar(255,0,0), 2, 8, 0 );
        }
    }
    //imshow("Lane Detect1", birdView); 
    imshow("Lane Detect", lane);
    
}

Mat DetectLane::preProcess(const Mat &src)
{
    Mat imgThresholded, imgHSV, dst, merge;

    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]), 
        Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), 
        imgThresholded);

     //Calling shadow func
    add(laneInShadow(imgHSV), imgThresholded, merge);

    dst = birdViewTranform(merge);

    

    imshow("Bird View", dst);

    fillLane(dst);
    imshow("fill lane", dst);
    //morphological(dst);
    //imshow("morpholoical", dst);
    imshow("Binary", imgThresholded);
    

    return dst;
}

Mat DetectLane::laneInShadow(const Mat &src)
{
    Mat shadowMask, shadow, imgHSV, shadowHSV, laneShadow;
    //imshow("srccc", src);
    cvtColor(src, imgHSV, COLOR_BGR2HSV);
   
    //imshow("shadow1", imgHSV);
    inRange(imgHSV, Scalar(minShadowTh[0], minShadowTh[1], minShadowTh[2]),
    Scalar(maxShadowTh[0], maxShadowTh[1], maxShadowTh[2]),  
    shadowMask);

    src.copyTo(shadow, shadowMask);

    cvtColor(shadow, shadowHSV, COLOR_BGR2HSV);
    

    inRange(shadowHSV, Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]), 
        Scalar(maxLaneInShadow[0], maxLaneInShadow[1], maxLaneInShadow[2]), 
        laneShadow);
        
    //imshow("Shadow", laneShadow);

    return laneShadow;
}

void DetectLane::fillLane(Mat &src)
{
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI/180, 1);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, CV_AA);
    }
}

vector<Mat> DetectLane::splitLayer(const Mat &src, int dir)
{
    int rowN = src.rows;
    int colN = src.cols;
    std::vector<Mat> res;

    if (dir == VERTICAL)
    {
        for (int i = 0; i < rowN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(0, i, colN, slideThickness);
            tmp = src(crop);
            
            res.push_back(tmp);
        }
    }
    else 
    {
        for (int i = 0; i < colN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(i, 0, slideThickness, rowN);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    return res;
}

vector<vector<Point> > DetectLane::centerRoadSide(const vector<Mat> &src, int dir)
{
    int flag = 0; //check black img
    vector<std::vector<Point> > res;
    int inputN = src.size();
    
    for (int i = 0; i < inputN; i++) {
        std::vector<std::vector<Point> > cnts;
        std::vector<Point> tmp;
        findContours(src[i], cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        int cntsN = cnts.size();
        if (cntsN == 0) {
            res.push_back(tmp);
            flag++;
            continue;
        }

        for (int j = 0; j < cntsN; j++) {
            int area = contourArea(cnts[j], false);
            if (area > 3) {
                Moments M1 = moments(cnts[j], false);
                Point2f center1 = Point2f(static_cast<float> (M1.m10 / M1.m00), static_cast<float> (M1.m01 / M1.m00));
                if (dir == VERTICAL) {
                    center1.y = center1.y + slideThickness*i;
                } 
                else
                {
                    center1.x = center1.x + slideThickness*i;
                }
        
                if (center1.x > 0 && center1.y > 0) {
                    tmp.push_back(center1);
                }
            }
        }
        
        res.push_back(tmp);
    }
    
    
    return res;
}

bool DetectLane::fillRoadSide(const vector<Mat> &src, vector<vector<Point> > &res)
{
    int flag = 0; //check black img
    vector<std::vector<Point> > points;
    int inputN = src.size();
    
    for (int i = 0; i < inputN; i++) {
        std::vector<std::vector<Point> > cnts;
        std::vector<Point> tmp;
        findContours(src[i], cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        int cntsN = cnts.size();
        if (cntsN == 0) {
            points.push_back(tmp);
            flag++;
            continue;
        }

        for (int j = 0; j < cntsN; j++) {
            int area = contourArea(cnts[j], false);
            if (area > 3) {
                Moments M1 = moments(cnts[j], false);
                Point2f center1 = Point2f(static_cast<float> (M1.m10 / M1.m00), static_cast<float> (M1.m01 / M1.m00));
                
                center1.y = center1.y + slideThickness*i;

                if (center1.x > 0 && center1.y > 0) {
                    tmp.push_back(center1);
                }
            }
        }
        
        points.push_back(tmp);   
    }
    cout << "Init:" << endl;
    for (int i = 0; i<31; i++)
        {
            for (int j = 0; j < points[i].size(); j++)
            {
                cout << i << ": " << points[i][j].x << " " << points[i][j].y << "; ";
            }
            cout << endl;
        }
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    //cout << flag << endl;
    if (flag < 31)
    {
        int d[2][31];
        for (int i = 30; i >=0; i--) {
            Point temp_left = {134, 4 + 10*i};
            Point temp_right = {99, 4 + 10*i};
            //res.insert(res.begin(), points[i]);
            
            std::vector<Point> temp;
            if (points[i].size() == 0)
            {
                res.insert(res.begin(), temp);
                continue; 
            }
            else if (points[i].size() == 1)
            {
                float coff0 = 0, coff1 = 0;

                for (int j= 30; j>i; j--)
                {
                    coff0 += d[0][j];
                    coff1 += d[1][j];
                }
                coff0 = sqrtf(coff0)/(30-i+1);
                coff1 = sqrtf(coff1)/(30-i+1);
                cout << coff0 << " " << coff1 << endl;

                if (coff0 < 5 && coff1 < 5)
                {
                    if (abs(points[i][0].x - 134) < abs(points[i][0].x - 99))
                    {
                        res[i][0] = points[i][0];
                        res[i][1].y = points[i][0].y;
                        res[i][1].x = 99 - (134 - points[i][0].x);
                    }
                    else
                    {
                        res[i][1] = points[i][0];
                        res[i][0].y = points[i][1].y;
                        res[i][0].x = 134 - (99 - points[i][1].x);
                    }   
                }
                d[0][i] = abs(points[i][0].x*points[i][0].x - 134*134);
                d[1][i] = abs(points[i][1].x*points[i][1].x - 99*99);
                
            }
            else if (points[i].size() == 2)
            {
                /*
                cout << points[i][0].x << " " << points[i][0].y << endl;
                if  (abs(points[i][0].x - temp_left.x) > abs(points[i][0].x - temp_right.x))
                {
                    temp[0].y = points[i][0].y;
                    temp[0].x = temp_left.x - (temp_right.x - points[i][0].x);
                    temp.push_back(points[i][0]);
                }
                else 
                {
                    temp.insert(temp.begin(), points[i][0]);
                    temp[1].y = points[i][0].y;
                    temp[1].x = temp_right.x - (temp_left.x - points[i][0].x);
                    
                }
                res.insert(res.begin(), temp);
                */
                d[0][i] = abs(points[i][0].x*points[i][0].x - 134*134);
                d[1][i] = abs(points[i][1].x*points[i][1].x - 99*99);
                res.insert(res.begin(), points[i]);
            }
            
            else
            {
                res.insert(res.begin(), points[i]);
            }
        }
        for (int i = 0; i<31; i++)
        {
            for (int j = 0; j < res[i].size(); j++)
            {
                cout << i << ": " << res[i][j].x << " " << res[i][j].y << "; ";
            }
            cout << endl;
        }
        return true;
    }
    return false;
    ///////////////////////////////////////////////////////////////////////////////////////////
}

void DetectLane::detectLeftRight(const vector<vector<Point> > &points)
{
    static vector<Point> lane1, lane2;
    lane1.clear();
    lane2.clear();
    
    leftLane.clear();
    rightLane.clear();
    for (int i = 0; i < BIRDVIEW_HEIGHT / slideThickness; i ++)
    {
        leftLane.push_back(null);
        rightLane.push_back(null);
    }

    int pointMap[points.size()][20];
    int prePoint[points.size()][20];
    int postPoint[points.size()][20];
    int dis = 10;
    int max = -1, max2 = -1;
    Point2i posMax, posMax2;

    memset(pointMap, 0, sizeof pointMap);

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }

    for (int i = points.size() - 2; i >= 0; i--)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            int err = 320;
            for (int m = 1; m < min(points.size() - 1 - i, 5); m++)
            {
                bool check = false;
                for (int k = 0; k < points[i + 1].size(); k ++)
                {
                    if (abs(points[i + m][k].x - points[i][j].x) < dis && 
                    abs(points[i + m][k].x - points[i][j].x) < err) {
                        err = abs(points[i + m][k].x - points[i][j].x);
                        pointMap[i][j] = pointMap[i + m][k] + 1;
                        prePoint[i][j] = k;
                        postPoint[i + m][k] = j;
                        check = true;
                    }
                }   
                break; 
            }
            
            if (pointMap[i][j] > max) 
            {
                max = pointMap[i][j];
                posMax = Point2i(i, j);
            }
        }
    }

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max2 && (i != posMax.x || j != posMax.y) && postPoint[i][j] == -1)
            {
                max2 = pointMap[i][j];
                posMax2 = Point2i(i, j);
            }
        }
    }

    if (max == -1) return;

    while (max >= 1)
    {
        lane1.push_back(points[posMax.x][posMax.y]);
        if (max == 1) break;

        posMax.y = prePoint[posMax.x][posMax.y];
        posMax.x += 1;        
        
        max--;
    }

    while (max2 >= 1)
    {
        lane2.push_back(points[posMax2.x][posMax2.y]);
        if (max2 == 1) break;

        posMax2.y = prePoint[posMax2.x][posMax2.y];
        posMax2.x += 1;        
        
        max2--;
    }
    
    vector<Point> subLane1(lane1.begin(), lane1.begin() + 5);
    vector<Point> subLane2(lane2.begin(), lane2.begin() + 5);

    Vec4f line1, line2;

    fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
    fitLine(subLane2, line2, 2, 0, 0.01, 0.01);

    int lane1X = (BIRDVIEW_WIDTH - line1[3]) * line1[0] / line1[1] + line1[2];
    int lane2X = (BIRDVIEW_WIDTH - line2[3]) * line2[0] / line2[1] + line2[2];

    if (lane1X < lane2X)
    {
        for (int i = 0; i < lane1.size(); i++)
        {
            leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
        for (int i = 0; i < lane2.size(); i++)
        {
            rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
    }
    else
    {
        for (int i = 0; i < lane2.size(); i++)
        {
            leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
        for (int i = 0; i < lane1.size(); i++)
        {
            rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
    }
}


Mat DetectLane::morphological(const Mat &img)
{
    Mat dst;

    // erode(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );
    // dilate( dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );

    dilate(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)) );
    erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)) );

    // blur(dst, dst, Size(3, 3));

    return dst;
}

void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst){
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

Mat DetectLane::birdViewTranform(const Mat &src)
{
    Point2f src_vertices[4];

    int width = src.size().width;
    int height = src.size().height;
    /*
    src_vertices[0] = Point(20, skyLine);
    src_vertices[1] = Point(width-20, skyLine);
    src_vertices[2] = Point(width, height);
    src_vertices[3] = Point(0, height);
    */
    src_vertices[0] = Point(0, skyLine);
    src_vertices[1] = Point(width, skyLine);
    src_vertices[2] = Point(width, height);
    src_vertices[3] = Point(0, height);

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(BIRDVIEW_WIDTH, 0);
    dst_vertices[2] = Point(BIRDVIEW_WIDTH - 105, BIRDVIEW_HEIGHT);
    dst_vertices[3] = Point(105, BIRDVIEW_HEIGHT);

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);

    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    return dst;
}

