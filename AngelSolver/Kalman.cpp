//----------------------------------------------------------
//
// FileName: Kalman.cpp
// Author: GuHao    GuHao0521@Gmail.com
// Version: 1.0.0 beta
// Date: 2021.4.17
// Description: Kalman类的实现
// Function List:1.Kalman::Kalman()
//               2.Kalman::~Kalman()                          
//               3.void Kalman::init()
//----------------------------------------------------------



#include "Kalman.h"


//@brief    Kalman类构造函数
Kalman::Kalman()
{
    init();
}


//@brief    Kalman类析构函数
Kalman::~Kalman()
{
}

void Kalman::init()
{
    dynamParams = 2;        //状态矩阵
    measureParams = 1;      //测量矩阵为1维
    controlParams = 0;      //暂未设置控制矩阵

    KF.init(dynamParams,measureParams,controlParams);

    KF.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);//设置状态转移矩阵
    setIdentity(KF.measurementMatrix);//测量矩阵
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪声，单位阵
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪声，单位阵
    setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值

}
