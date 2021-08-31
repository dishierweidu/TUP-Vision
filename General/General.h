//----------------------------------------------------------
//                                                          
// FileName: General.h                                                
// Author: 顾昊
// Version: 1.0.0_x64Linux
// Date: 2021.07.14
// Description: 一些常用函数的声明
// 
//----------------------------------------------------------
#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

bool makeRectSafe(cv::Rect &rect, cv::Size size);

double pointsDistance(Point2f p1, Point2f p2);

void drawRotatedRect(Mat frame, RotatedRect rRect, Scalar color, int thickness);
void ShowRotateRectDetail(RotatedRect &RRect, Mat src, Scalar color, int thickness);