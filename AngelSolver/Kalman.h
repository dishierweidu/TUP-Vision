//----------------------------------------------------------
//
// FileName: Kalman
// Author: GuHao    GuHao0521@Gmail.com
// Version: 1.1.0 beta
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

    double EstimatedTimeofArrival;         //预测提前

private:
    int measureParams;                 //测量矩阵数据维数
    int dynamParams;                    //状态矩阵数据维数
    int controlParams;                  //控制向量矩阵维数



    // Mat Jacobian;                       //用于纠正数据的雅可比矩阵(暂未使用EKF)

    void init(int type);                    //卡尔曼参数初始化



};

