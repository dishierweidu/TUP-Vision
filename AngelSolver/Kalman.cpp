//----------------------------------------------------------
//
// FileName: Kalman.cpp
// Author: 顾昊    GuHao0521@Gmail.com
// Version: 1.1.0 
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
    kalman_type_ = type;//设置卡尔曼滤波模式
    init(type);
}


//@brief    Kalman类析构函数
Kalman::~Kalman()
{
}

void Kalman::init(int type)
{
    ///////////能量机关卡尔曼滤波相关参数///////////////
    if(type == KALMAN_TYPE_ENERGY)
    {
    dynamparams_ = 3;                                     //状态矩阵
    measureparams_ = 2;                                   //测量矩阵为2维
    controlparams_ = 0;                                   //暂未设置控制矩阵
    predict_time_ = 0.40;                        //设置提前时间
    KF.init(dynamparams_,measureparams_,controlparams_);   //初始化参数
    KF.processNoiseCov = (Mat_<float>(3, 3) << 1e-4 , 0 , 0,
                                               0 ,7e-4 , 0  ,                      
                                               0,0 ,2e-4);//设置过程噪声Q
    KF.measurementNoiseCov = (Mat_<float>(2, 2) << 2e-3 ,0 ,
                                                    0,7e-2);//设置测量噪声R                                                   
    setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
    setIdentity(KF.errorCovPre, Scalar::all(1));//P先验误差估计协方差矩阵，初始化为单位阵
                            
    KF.measurementMatrix = (Mat_<float>(2, 3) <<    0,          1               ,            0               ,
                                                    0,          0               ,            1               );
    // 设置状态转移矩阵                   
    KF.transitionMatrix = (Mat_<float>(3, 3) << 0 , predict_time_ ,            3 * pow(predict_time_,2),
                                                0 ,         1              ,          predict_time_  ,                      
                                                0,           0             ,                   1               );//设置状态转移矩阵
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1)); //初始化状态为随机值
   }

    else if(type == KALMAN_TYPE_ARMOR)
    {
        dynamparams_ = 8;                                     //状态矩阵
        measureparams_ = 8;                                   //测量矩阵
        controlparams_ = 0;                                   //暂未设置控制矩阵
        predict_time_ = 1;                                   //设置提前时间

        KF.init(dynamparams_,measureparams_,controlparams_);   //初始化参数
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
        KF.transitionMatrix = (Mat_<float>(8, 8) << 1,0,0,0,predict_time_,0,0,0,//x
                                                                0,1,0,0,0,predict_time_,0,0,//y
                                                                0,0,1,0,0,0,predict_time_,0,//width
                                                                0,0,0,1,0,0,0,predict_time_,//height
                                                                0,0,0,0,1,0,0,0,//Vx
                                                                0,0,0,0,0,1,0,0,//Vy
                                                                0,0,0,0,0,0,1,0,//Vwidth
                                                                0,0,0,0,0,0,0,1);//Vheight

        randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值
   }
}
