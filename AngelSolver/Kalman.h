//----------------------------------------------------------
//
// FileName: Kalman
// Author: GuHao    GuHao0521@Gmail.com
// Version: 1.0.0 beta
// Date: 2021.4.17
// Description: Kalman类的定义
// Function List:1.Kalman::Kalman()
//               2.Kalman::~Kalman()                          
//               3.void Kalman::init()
//
//----------------------------------------------------------

#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Kalman
{
public:
    KalmanFilter KF;                //卡尔曼滤波器

    Kalman();                       //Kalman类构造函数
    ~Kalman();                      //Kalman类析构函数

private:
    int measureParams;                 //测量矩阵数据维数
    int dynamParams;                    //状态矩阵数据维数
    int controlParams;                  //控制向量矩阵维数

    void init();                    //卡尔曼参数初始化



};

