//----------------------------------------------------------
//                                                          
// FileName: General.cpp                                                
// Author: 顾昊
// Version: 1.0.0_x64Linux
// Date: 2021.07.14
// Description: 一些常用函数的实现
// 
//----------------------------------------------------------

#include "General.h"

/**
 * @brief makeRectSafe 使矩形不发生越界
 * @param rect 输入的矩形， x， y为左上角的坐标
 * @param size 限制的大小，防止越界
 * @return 矩形是否为空
 */
bool makeRectSafe(cv::Rect &rect, cv::Size size)
{
    if (rect.x < 0)
        rect.x = 0;
    if (rect.x + rect.width > size.width)
        rect.width = size.width - rect.x;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.y + rect.height > size.height)
        rect.height = size.height - rect.y;
    if (rect.width <= 0 || rect.height <= 0)
        // 如果发现矩形是空的，则返回false
        return false;
    return true;
}

/**
* @brief 返回两点间距离
* @param p1 第一个点
* @param p2 第二个点
* @return 两点距离
*/
double pointsDistance(Point2f p1, Point2f p2)
{
    return pow(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2), 0.5);
}

/** @brief 绘制旋转矩形 by up
 * @param frame 直线绘制的图像
 * @param rRect 旋转矩形
 * @param color 矩形的颜色
 * @param thickness 矩形线条的线宽
*/ 
void drawRotatedRect(Mat frame, RotatedRect rRect, Scalar color, int thickness = 1)
{
    //创建存储旋转矩形顶点的数组
    Point2f vertices[4];
    //存储顶点
    rRect.points(vertices);
    //绘制旋转矩形
    for (int j = 0; j < 4; ++j)
        line(frame, vertices[j], vertices[(j + 1) % 4], color, thickness);
}

/**
 * @brief 绘制旋转矩形并显示相关信息 by 顾昊
 * @param RRect 所需矩形
 * @param src  输出图像
 * @param color 矩形的颜色与相关数据颜色
 * @param thickness 矩形线条的线宽与字体大小
*/
void ShowRotateRectDetail(RotatedRect &RRect, Mat src, Scalar color = Scalar(0, 150, 0), int thickness = 2)
{
    //储存旋转矩形顶点数组
    Point2f Apex[4];
    RRect.points(Apex);
    const string Width = "Length :" + to_string((int)(RRect.size.width > RRect.size.height ? RRect.size.width : RRect.size.height));
    const string Height = "Width :" + to_string((int)(RRect.size.width < RRect.size.height ? RRect.size.width : RRect.size.height));
    const string Area = "Area :" + to_string((int)(RRect.size.width * RRect.size.height));
    const string H_W = "H/W :" + to_string((float)((RRect.size.width > RRect.size.height ? RRect.size.width : RRect.size.height) / (RRect.size.width < RRect.size.height ? RRect.size.width : RRect.size.height)));
    for (int j = 0; j < 4; ++j)
        line(src, Apex[j], Apex[(j + 1) % 4], color, thickness);
    putText(src, Width, Apex[0], CV_FONT_HERSHEY_SIMPLEX, 0.5, color, thickness);
    putText(src, Height, Apex[0] + Point2f(0, 15), CV_FONT_HERSHEY_SIMPLEX, 0.5, color, thickness);
    putText(src, Area, Apex[0] + Point2f(0, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, color, thickness);
    putText(src, H_W, Apex[0] + Point2f(0, 45), CV_FONT_HERSHEY_SIMPLEX, 0.5, color, thickness);
}