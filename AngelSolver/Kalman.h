//----------------------------------------------------------
//
// FileName: Kalman.h
// Author: 顾昊    GuHao0521@Gmail.com
// Version: 1.1.0 
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

#define KALMAN_TYPE_ENERGY 0
#define KALMAN_TYPE_ARMOR  1


class Kalman
{
public:
    KalmanFilter KF;                //卡尔曼滤波器

    Kalman(int);                       //Kalman类构造函数
    ~Kalman();                      //Kalman类析构函数
    double predict_time_;         //预测提前时间



private:
    int kalman_type_;             //设置卡尔曼滤波模式,0为大符,1为辅瞄
    int measureparams_;                 //测量矩阵数据维数
    int dynamparams_;                    //状态矩阵数据维数
    int controlparams_;                  //控制向量矩阵维数


    void init(int type);                    //卡尔曼参数初始化
};

