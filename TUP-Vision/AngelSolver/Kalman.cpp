//----------------------------------------------------------
//
// FileName: Kalman.cpp
// Author: GuHao    GuHao0521@Gmail.com
// Version: 1.1.0 beta
// Date: 2021.4.23
// Description: Kalman类的实现
// Function List:1.Kalman::Kalman()
//               2.Kalman::~Kalman()                          
//               3.void Kalman::init()
//----------------------------------------------------------



#include "Kalman.h"


//@brief    Kalman类构造函数
Kalman::Kalman(int type)
{
    init(type);
}


//@brief    Kalman类析构函数
Kalman::~Kalman()
{
}

void Kalman::init(int type)
{
    ///////////能量机关卡尔曼///////////////
    if(type == KALMAN_TYPE_ENERGY){
    dynamParams = 3;                                     //状态矩阵
    measureParams = 2;                                   //测量矩阵为2维
    controlParams = 0;                                   //暂未设置控制矩阵

    // EstimatedTimeofArrival = 0.45;                        //设置提前时间
    EstimatedTimeofArrival = 0.40;                        //设置提前时间
    // EstimatedTimeofArrival = 0.01;                        //设置提前时间

    KF.init(dynamParams,measureParams,controlParams);   //初始化参数

    setIdentity(KF.measurementMatrix);//测量矩阵,初始化为单位阵
    KF.processNoiseCov = (Mat_<float>(3, 3) << 1e-4 , 0 , 0,
                                               0 ,7e-4 , 0  ,                      
                                               0,0 ,2e-4);//设置过程噪声Q
    KF.measurementNoiseCov = (Mat_<float>(2, 2) << 2e-3 ,0 ,
                                                    0,7e-2);//设置测量噪声R
                                                    
    setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
    setIdentity(KF.errorCovPre, Scalar::all(1));//P先验误差估计协方差矩阵，初始化为单位阵

    KF.measurementMatrix = (Mat_<float>(2, 3) <<    0,          1               ,            0               ,
                                                    0,          0               ,            1               );
                                                    
                                                    
    KF.transitionMatrix = (Mat_<float>(3, 3) << 0 , EstimatedTimeofArrival , 3 * pow(EstimatedTimeofArrival,2),
                                                0 ,         1              ,          EstimatedTimeofArrival  ,                      
                                                0,           0             ,                   1               );//设置状态转移矩阵

    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值
   }

    if(type == KALMAN_TYPE_ARMOR){
    dynamParams = 8;                                     //状态矩阵
    measureParams = 8;                                   //测量矩阵为2维
    controlParams = 0;                                   //暂未设置控制矩阵

    EstimatedTimeofArrival = 1;                        //设置提前时间

    KF.init(dynamParams,measureParams,controlParams);   //初始化参数

    setIdentity(KF.measurementMatrix);//测量矩阵,初始化为单位阵
    setIdentity(KF.processNoiseCov,Scalar::all(1e-4));//设置过程噪声Q,初始化为单位阵
    //设置测量噪声R
    KF.measurementNoiseCov = (Mat_<float>(8, 8) << 1,0,0,0,0,0,0,0,//x
                                                    0,1,0,0,0,0,0,0,//y
                                                    0,0,1,0,0,0,0,0,//width
                                                    0,0,0,1,0,0,0,0,//height
                                                    0,0,0,0,1e1,0,0,0,//Vx
                                                    0,0,0,0,0,1e1,0,0,//Vy
                                                    0,0,0,0,0,0,1e1,0,//Vwidth
                                                    0,0,0,0,0,0,0,1e1);//Vheight
                                                    
    setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
    setIdentity(KF.errorCovPre, Scalar::all(1));//P先验误差估计协方差矩阵，初始化为单位阵
    setIdentity(KF.measurementMatrix, Scalar::all(1));
    //设置状态转移矩阵           
    KF.transitionMatrix = (Mat_<float>(8, 8) << 1,0,0,0,EstimatedTimeofArrival,0,0,0,//x
                                                0,1,0,0,0,EstimatedTimeofArrival,0,0,//y
                                                0,0,1,0,0,0,EstimatedTimeofArrival,0,//width
                                                0,0,0,1,0,0,0,EstimatedTimeofArrival,//height
                                                0,0,0,0,1,0,0,0,//Vx
                                                0,0,0,0,0,1,0,0,//Vy
                                                0,0,0,0,0,0,1,0,//Vwidth
                                                0,0,0,0,0,0,0,1);//Vheight

    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值
   }


}
