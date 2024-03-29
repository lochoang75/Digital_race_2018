#include "detectlane.h"
#include "polynomialregression.h"

int min(int a, int b)
{
    return a < b ? a : b;
}

double x_regression_HORIZONTAL(const vector<double> &coeffs, const vector<Point> &points, int N)
{
    double res = 0;
    for (int i = 0; i < N; i++)
        for (int j = 0; j < NUMBER_COEFFS; j++)
            res+= coeffs[j] * pow(points[i].y, j);
    return res/N;
}

double x_regression(const vector<double> &coeffs, const double &y)
{
    double res = 0;
    for (int i = 0; i < NUMBER_COEFFS; i++)
        res+= coeffs[i] * pow(y, i);
    return res;
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
    desireLine  = vector<Point>(31);
    //leftLine    = vector<Point>(31);
    //rightLine   = vector<Point>(31);
    last_leftLine_coeffs.clear();
    last_rightLine_coeffs.clear();
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

vector<Point> DetectLane::getDesireLine()
{
    return desireLine;
}

vector<Point> DetectLane::getLeftLine()
{
    return leftLine;
}

vector<Point> DetectLane::getRightLine()
{
    return rightLine;
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
    birdView1 = Mat::zeros(img.size(), CV_8UC3);

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
    imshow("Debug", birdView);
    fillRoadSide(layers1);
    //if ()
    //{ 
        for (int i = 0; i < NUMBER_ELEMENTS; i++)
        {
            circle(birdView1, leftLine[i], 1, Scalar(0,255,0), 2, 8, 0 );
            //if (leftLine[i]!=null) cout << i << ": " << leftLine[i].x << " " << leftLine[i].y << "; ";
        }
        for (int i = 0; i < NUMBER_ELEMENTS; i++)
        {
            circle(birdView1, rightLine[i], 1, Scalar(0,255,0), 2, 8, 0 );
            //if (rightLine[i]!=null) cout << i << ": " << rightLine[i].x << " " << rightLine[i].y << "; ";
        }
        
            imshow("Fill road", birdView1);
        
    //}
    //cout << analyseLine(0, 10) << endl;
    //update_desireLine(points1);
    
    // for (int i=0; i< NUMBER_ELEMENTS; i++)
    // {
    //     if (desireLine[i]!=null) cout << i << ": " << desireLine[i].x << " " << desireLine[i].y << "; ";
    // }
    //cout << endl;
    
    // for (int i=0; i< NUMBER_ELEMENTS; i++)
    // {
    //     if (leftLine[i]!=null) cout << i << ": " << leftLine[i].x << " " << leftLine[i].y << "; ";
    // }
    // for (int i = 0; i< leftLine_x.size(); i++)
    //     cout << leftLine_x[i] << " ";
    // cout << endl;
    // for (int i = 0; i< leftLine_y.size(); i++)
    //     cout << leftLine_y[i] << " ";
    // cout << endl;
    //cout << "xy: " <<leftLine_x.size() << " " << leftLine_y.size() << endl;
    
    // update_last5Line(desireLine);
    // if (last5desireLine.size()==5) cout << "Update 5 lines" << endl;
    
    //regression
    //x = a0 + a1*y +a2*y^2 +
    // if (leftLine_x.size() && leftLine_y.size())
    // {
    //     PolynomialRegression<double> regress;
    //     regress.fitIt(leftLine_y, leftLine_x, NUMBER_COEFFS+1, coeffs);
    //     cout << "coeffs ";
    //     for (int i=0; i < NUMBER_COEFFS; i++)
    //         cout << coeffs[i] << " ";
    //     cout << endl;
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
    imshow("Merge", merge);

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

bool DetectLane::fillRoadSide(const vector<Mat> &src)
{
    leftLine.clear();
    rightLine.clear();

    for (int i = 0; i < NUMBER_ELEMENTS; i ++)
    {
        leftLine.push_back(null);
        rightLine.push_back(null);
    }
   
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
    
    // cout << "Init:" << endl;
    // for (int i = 0; i<31; i++)
    //     {
    //         for (int j = 0; j < points[i].size(); j++)
    //         {
    //             cout << i << ": " << points[i][j].x << " " << points[i][j].y << "; ";
    //         }
    //         cout << endl;
    //     }
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    //cout << flag << endl;
    //if (flag < 31)
    //{
        //Update first coeffs
        //vector<double> leftLine_coeffs, rightLine_coeffs;
        if (!last_leftLine_coeffs.size() && !last_rightLine_coeffs.size())
        {
            update_desireLine(points);
            vector<double> leftLine_x, leftLine_y, rightLine_x, rightLine_y;
            for (int i=0; i< NUMBER_ELEMENTS; i++)
            {
                if (leftLine[i]!=null)
                {
                    leftLine_x.push_back(leftLine[i].x);
                    leftLine_y.push_back(leftLine[i].y);
                }

                if (rightLine[i]!=null)
                {
                    rightLine_x.push_back(rightLine[i].x);
                    rightLine_y.push_back(rightLine[i].y);
                }
            }
            //regression
            //x = a + b*y +c*y^2
            PolynomialRegression<double> regress;
            if (leftLine_x.size() && leftLine_y.size())
            {
                regress.fitIt(leftLine_y, leftLine_x, NUMBER_COEFFS-1, last_leftLine_coeffs); 
            }
            if (rightLine_x.size() && rightLine_y.size())
            {
                regress.fitIt(rightLine_y, rightLine_x, NUMBER_COEFFS-1, last_rightLine_coeffs);
            }
            for (int i = 0; i < last_leftLine_coeffs.size(); i++)
                cout << last_leftLine_coeffs[i] << " ";
            cout << endl;
            for (int i = 0; i < last_rightLine_coeffs.size(); i++)
                cout << last_rightLine_coeffs[i] << " ";
            cout << endl;
            return false;
        }
        
        
        for (int i = 0; i < NUMBER_ELEMENTS; i++)
        {
            int N = points[i].size();
            if (N > 8 || N == 0)
            {
                leftLine[i] = null;
                rightLine[i] = null;
                //cout << i <<" failed\n";
            }
            else
            {
                //int leftXregress = x_regression_HORIZONTAL(last_leftLine_coeffs, points[i], N);
                //int rightXregress = x_regression_HORIZONTAL(last_rightLine_coeffs, points[i], N);
                
                double y_temp = 0;
                for (int j = 0; j < N; j++)
                    y_temp+= points[i][j].y;
                
                y_temp = y_temp / N;
                //cout << y_temp << endl;
                int leftXregress = x_regression(last_leftLine_coeffs, y_temp);
                int rightXregress = x_regression(last_rightLine_coeffs, y_temp);
                //cout << leftXregress << " " << rightXregress << endl;
                int minLeft=1000, minRight=1000;
                int posLeft, posRight;
                
                for (int j = 0; j < N; j++)
                {
                    int toLeftX = abs(points[i][j].x - leftXregress);
                    int toRightX = abs(points[i][j].x - rightXregress);
                    if (toLeftX < toRightX)
                    {
                        if (toLeftX < minLeft)
                        {
                            minLeft = toLeftX;
                            posLeft = j;
                        }
                    }
                    else
                    {
                        if (toRightX < minRight)
                        {
                            minRight = toRightX;
                            posRight = j;
                        }
                    }
                }
                //cout << minLeft << " " << minRight << endl;
                if (minLeft < 10)
                    leftLine[i] = points[i][posLeft];
                else 
                    leftLine[i] = null;

                if (minRight < 10)
                    rightLine[i] = points[i][posRight];
                else 
                    rightLine[i] = null;
            }
        }
        

        vector<double> leftLine_x, leftLine_y, rightLine_x, rightLine_y;
        for (int i=0; i< NUMBER_ELEMENTS; i++)
        {
            if (leftLine[i]!=null)
            {
                leftLine_x.push_back(leftLine[i].x);
                leftLine_y.push_back(leftLine[i].y);
            }

            if (rightLine[i]!=null)
            {
                rightLine_x.push_back(rightLine[i].x);
                rightLine_y.push_back(rightLine[i].y);
            }
        }
        //regression
        //x = a + b*y +c*y^2
        PolynomialRegression<double> regress;
        last_leftLine_coeffs.clear();
        last_rightLine_coeffs.clear();
        if (leftLine_x.size() && leftLine_y.size())
        {
            regress.fitIt(leftLine_y, leftLine_x, NUMBER_COEFFS-1, last_leftLine_coeffs); 
        }
        if (rightLine_x.size() && rightLine_y.size())
        {
            regress.fitIt(rightLine_y, rightLine_x, NUMBER_COEFFS-1, last_rightLine_coeffs);
        }
        
        //regression for null line[i]
        // for (int i = 0; i < NUMBER_ELEMENTS; i++)
        // {
        //     if (leftLine[i]==null) 
        //     {
        //         leftLine[i].x = x_regression(last_leftLine_coeffs, 10*i + 4);
        //         leftLine[i].y = 10*i +4;
        //     }
        //     if (rightLine[i]==null) 
        //     {
        //         rightLine[i].x = x_regression(last_rightLine_coeffs, 10*i + 4);
        //         rightLine[i].y = 10*i +4;
        //     }
               
        
        // }
        int analyse = analyseLine(0, 10);

        if ( analyse == 5 || analyse == 7) last_leftLine_coeffs.clear();
        if (analyse == 6 || analyse == 7) last_rightLine_coeffs.clear();
    //}
    return true;
    ///////////////////////////////////////////////////////////////////////////////////////////
}

//Function to analyse left and right line from the position pos to NUMBER_ELEMENTS
//Return 0 if full line 
//
//Return 2 if vacant left line
//Return 3 if vacant right line
//Return 4 if vacant left&right line
//Return 5 if null left line
//Return 6 if null right line
//Return 7 if null left&right line
int DetectLane::analyseLine(int pos, int range)
{
    int previousLeft, previousRight; //store previous null
    int countLeft = 0, countRight = 0;
    vector<int> tempLeft, tempRight; // store size of null chain
    if (leftLine[pos] == null)
    {
        previousLeft = pos;
        countLeft++;
        tempLeft.push_back(1);
        tempLeft.push_back(1);
    }
    else
    {
        previousLeft = -1;
        tempLeft.push_back(0);
    }
    if (rightLine[pos] == null)
    {
        previousRight = pos;
        countRight++;
        tempRight.push_back(1);
        tempRight.push_back(1);
    }
    else
    {
        previousRight = -1;
        tempRight.push_back(0);
    }

    for (int i = pos + 1; i < NUMBER_ELEMENTS; i++)
    {
        if (leftLine[i] == null)
        {
            if (leftLine[i-1] == null)
            { 
               tempLeft[tempLeft[0]]++;
            }
            else 
            {
                tempLeft[0]++;
                tempLeft.push_back(1);
            }
            countLeft++;
            previousLeft = i;
        }   
        

        if (rightLine[i] == null)
        {
            if (rightLine[i-1] == null)
                tempRight[tempRight[0]]++;
            else
            {
                tempRight[0]++;
                tempRight.push_back(1);
            }
            countRight++;
            previousRight = i;
        }
    }



    if (NUMBER_ELEMENTS - pos - countLeft < 5 && NUMBER_ELEMENTS - pos - countRight < 5)
        return 7;
    if (NUMBER_ELEMENTS - pos - countLeft < 5)
        return 5;
    if (NUMBER_ELEMENTS - pos - countRight < 5)
        return 6;
    
    bool isVacantLeft = false;
    bool isVacantRight = false;
    for (int i = 0; i < tempLeft[0]; i++)
    {
        if (tempLeft[i+1] > range)
            isVacantLeft = true; 
    }
    for (int i = 0; i < tempRight[0]; i++)
    {
        if (tempRight[i+1] > range)
            isVacantRight = true; 
    
    }

    if (isVacantLeft && isVacantRight)
        return 4;
    if (isVacantLeft)
        return 2;
    if (isVacantRight)
        return 3;
    
    return 0;
}

//Function to determine when turn right
//Return -1 if continue to go straight
//Return 0 if turn right
//Return 1 if end the turn right
int DetectLane::turnRight()
{
    if (turn == false)
    {
        int ana = analyseLine(0, 12); 
        if (ana == 3 || ana == 6 || ana == 7)
        {
            turn = true;
            cout << "ana: " << ana << endl;
        }
    }

    else
    {
        for (int i = NUMBER_ELEMENTS - 1; i >= 0; i--)
        {
             
            if (rightLine[i] != null && last_rightLine_coeffs[1] + last_rightLine_coeffs[2]* 2 * rightLine[i].y < 0.1)
            {
                cout << last_rightLine_coeffs[1] + last_rightLine_coeffs[2]* 2 * rightLine[i].y << endl;
                turn = false;
                return 1;
            }
        }
        
    }
    
    if (turn) return 0;
    return -1;
}


//Function to determine when turn left
//Return -1 if continue to go straight
//Return 0 if turn left
//Return 1 if end the turn left
int DetectLane::turnLeft()
{
    if (turn == false)
    {
        int ana = analyseLine(0, 12); 
        if (ana == 2 || ana == 5 || ana == 7)
        {
            turn = true;
            cout << "ana: " << ana << endl;
        }
    }

    else
    {
        for (int i = NUMBER_ELEMENTS - 1; i >= 0; i--)
        {
             
            if (leftLine[i] != null && last_leftLine_coeffs[1] + last_leftLine_coeffs[2]* 2 * leftLine[i].y < 0.1)
            {
                turn = false;
                return 1;
            }
        }
        
    }
    
    if (turn) return 0;
    return -1;
}

void DetectLane::update_desireLine(const vector<vector<Point> > &points)
{
    desireLine.clear();
    
    for (int i = 0; i < NUMBER_ELEMENTS; i++)
    {
        if (points[i].size() == 2)
        {
            //Update desire line for carcontroll
            desireLine[i].x = (points[i][0].x + points[i][0].x) / 2;
            desireLine[i].y = (points[i][1].y + points[i][1].y) / 2;

            //Update left&right line for regression
            leftLine[i] = points[i][1];
            rightLine[i] = points[i][0];
        }
    }
   
}

void DetectLane::update_last5Line(const vector<Point> &points)
{
    last5desireLine.insert(last5desireLine.begin(), points);
    if (last5desireLine.size() == 6)
        last5desireLine.erase(last5desireLine.begin()+5);
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

// int d[2][31];
//         for (int i = 30; i >=0; i--) {
//             Point temp_left = {134, 4 + 10*i};
//             Point temp_right = {99, 4 + 10*i};
//             //res.insert(res.begin(), points[i]);
            
//             std::vector<Point> temp;
//             if (points[i].size() == 0)
//             {
//                 res.insert(res.begin(), temp);
//                 continue; 
//             }
//             else if (points[i].size() == 1)
//             {
//                 float coff0 = 0, coff1 = 0;

//                 for (int j= 30; j>i; j--)
//                 {
//                     coff0 += d[0][j];
//                     coff1 += d[1][j];
//                 }
//                 coff0 = sqrtf(coff0)/(30-i+1);
//                 coff1 = sqrtf(coff1)/(30-i+1);
//                 cout << coff0 << " " << coff1 << endl;

//                 if (coff0 < 5 && coff1 < 5)
//                 {
//                     if (abs(points[i][0].x - 134) < abs(points[i][0].x - 99))
//                     {
//                         res[i][0] = points[i][0];
//                         res[i][1].y = points[i][0].y;
//                         res[i][1].x = 99 - (134 - points[i][0].x);
//                     }
//                     else
//                     {
//                         res[i][1] = points[i][0];
//                         res[i][0].y = points[i][1].y;
//                         res[i][0].x = 134 - (99 - points[i][1].x);
//                     }   
//                 }
//                 d[0][i] = abs(points[i][0].x*points[i][0].x - 134*134);
//                 d[1][i] = abs(points[i][1].x*points[i][1].x - 99*99);
                
//             }
//             else if (points[i].size() == 2)
//             {
//                 /*
//                 cout << points[i][0].x << " " << points[i][0].y << endl;
//                 if  (abs(points[i][0].x - temp_left.x) > abs(points[i][0].x - temp_right.x))
//                 {
//                     temp[0].y = points[i][0].y;
//                     temp[0].x = temp_left.x - (temp_right.x - points[i][0].x);
//                     temp.push_back(points[i][0]);
//                 }
//                 else 
//                 {
//                     temp.insert(temp.begin(), points[i][0]);
//                     temp[1].y = points[i][0].y;
//                     temp[1].x = temp_right.x - (temp_left.x - points[i][0].x);
                    
//                 }
//                 res.insert(res.begin(), temp);
//                 */
//                 d[0][i] = abs(points[i][0].x*points[i][0].x - 134*134);
//                 d[1][i] = abs(points[i][1].x*points[i][1].x - 99*99);
//                 res.insert(res.begin(), points[i]);
//             }
            
//             else
//             {
//                 res.insert(res.begin(), points[i]);
//             }
//         }
//         for (int i = 0; i<31; i++)
//         {
//             for (int j = 0; j < res[i].size(); j++)
//             {
//                 cout << i << ": " << res[i][j].x << " " << res[i][j].y << "; ";
//             }
//             cout << endl;
//         }