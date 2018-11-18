#include "lane_detect.h"

int laneDetect(Mat inimg) {
	 Mat img = inimg;
	 namedWindow("Source", CV_WINDOW_AUTOSIZE);
	 namedWindow("Gray", CV_WINDOW_AUTOSIZE);
	 namedWindow("Lane", CV_WINDOW_AUTOSIZE);
	 imshow("Source", img);
	 
	 ///////////////////////////////////
	 Point A, B, C, D, E, F, G, H, I;
	 A.x = 0;
	 A.y = 0;
	 B.x = 0;
	 B.y = img.size().height/2 - 30;
	 C.x = img.size().width / 2;
	 C.y = img.size().height/6 + 15;
	 D.x = img.size().width;
	 D.y = img.size().height / 2 - 30;
	 E.x = img.size().width;
	 E.y = 0;
	 F.x = img.size().width / 2 - 80;
	 F.y = img.size().height - 60;
	 G.x = img.size().width / 2 - 80;
	 G.y = img.size().height;
	 H.x = img.size().width / 2 + 80;
	 H.y = img.size().height;
	 I.x = img.size().width / 2 + 80;
	 I.y = img.size().height - 60;

	 vector<Point> points;
	 points.push_back(A);
	 points.push_back(B);
	 points.push_back(C);
	 
	 vector<Point> points1;
	 points1.push_back(D);
	 points1.push_back(C);
	 points1.push_back(E);

	 vector<Point> points2;
	 points2.push_back(E);
	 points2.push_back(C);
	 points2.push_back(A);

	 vector<Point> points3;
	 points3.push_back(F);
	 points3.push_back(G);
	 points3.push_back(H);
	 points3.push_back(I);

	 Mat gray;
	 cvtColor(img, gray, CV_BGR2GRAY);
	 Canny(gray, gray, 100, 450);

	 fillConvexPoly(gray, points, Scalar(0, 0, 0), CV_AA, 0);
	 fillConvexPoly(gray, points1, Scalar(0, 0, 0), CV_AA, 0);
	 fillConvexPoly(gray, points2, Scalar(0, 0, 0), CV_AA, 0);		 
	 //fillConvexPoly(gray, points3, Scalar(0, 0, 0), CV_AA, 0);
	 
	 vector<Vec4i> lines;
	 Point maxl1, maxl2, maxr1, maxr2;
	 maxl1.x = 0;
	 maxl1.y = 0;
	 maxl2.x = img.size().width;
	 maxl2.y = 0;
	 maxr1.x = img.size().width ;
	 maxr1.y = img.size().height / 4 + 20;
	 maxr2.x = img.size().width / 2;
	 maxr2.y = 0;
	 HoughLinesP(gray, lines, 1, CV_PI / 180, 30, 25, 10);
	 cvtColor(gray, gray, CV_GRAY2BGR);
	 double slope;
	 imshow("Gray1", gray);
	 for (size_t i = 0; i < lines.size(); i++)
	 {
		 Vec4i l = lines[i];
		 slope = (((double)l[3] - (double)l[1]) / ((double)l[2] - (double)l[0]));		 		 		 
			if (slope <= 0) {
				if (l[0] > img.size().width/2);				 
				else {
					if (maxl1.y < l[1]) {
						maxl1.x = l[0];
						maxl1.y = l[1];
						maxl2.x = l[2];
						maxl2.y = l[3];
					}
				}			
				line(gray, Point(0,0), Point(l[0], l[1]), Scalar(0, 255, 255), 3, CV_AA);
				line(gray, Point(img.size().width / 2, 0), Point(img.size().width / 2, img.size().height), Scalar(0, 255, 0), 1, 8, 0);
				line(gray, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 3, CV_AA);				 
			}
			else {
				if (l[2] < img.size().width / 2);
				else {					 
					if (maxr1.y < l[3]) {
						maxr2.x = l[0];
						maxr2.y = l[1];
						maxr1.x = l[2];
						maxr1.y = l[3];
					}
				}
				line(gray, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 3, CV_AA);
			}		 
	 }
	 imshow("Gray", gray);
	 Point temp1, temp2, temp3, temp4;
	 double length = sqrt(pow(maxl2.x - maxl1.x, 2.0) + pow(maxl2.y - maxl2.y, 2.0));
	 temp1.x = maxl2.x + (maxl2.x - maxl1.x) / length * 300;
	 temp1.y = maxl2.y + (maxl2.y - maxl1.y) / length * 300;
	 temp3.x = maxl1.x + (maxl1.x - maxl2.x) / length * 300;
	 temp3.y = maxl1.y + (maxl1.y - maxl2.y) / length * 300;
	 line(img, temp3, temp1, Scalar(0, 0, 255), 3, 8, 0);
	 length = sqrt(pow(maxr2.x - maxr1.x, 2.0) + pow(maxr2.y - maxr2.y, 2.0));
	 temp2.x = maxr2.x - abs(maxr1.x - maxr2.x) / length * 300;
	 temp2.y = maxr2.y - abs(maxr1.y - maxr2.y) / length * 300;
	 temp4.x = maxr1.x + abs(maxr1.x - maxr2.x) / length * 300;
	 temp4.y = maxr1.y + abs(maxr1.y - maxr2.y) / length * 300;
	 line(img, temp4, temp2, Scalar(0, 0, 255), 3, 8, 0);
	 line(img, Point(img.size().width/2,0), Point(img.size().width / 2, img.size().height), Scalar(0, 255, 0), 1, 8, 0);
	 
	 imshow("Lane", img);
	 
	 //waitKey(0);
	 return 0;
 }
