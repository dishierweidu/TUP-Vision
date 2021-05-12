//----------------------------------------------------------
//
// FileName: ImageProcess.h
// Author: Liu Shang fyrwls@163.com
// Version: 1.0.20201115
// Date: 2020.11.15
// Description: 线程处理类
// Function List:
//
//
//----------------------------------------------------------

#pragma once


#include "../Debug.h"
#include "../Armor/Settings.hpp"


#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include "../AngelSolver/AngleSolver.hpp"
#include "../Armor/ArmorDetector.hpp"
#include "../Camera/DaHengCamera.h"
#include "../Energy/Energy.h"
#include "../Serial/serialport.h"
#include <atomic>


using namespace std;
using namespace cv;



class ImageProcess
{
public:
    ImageProcess(SerialPort &port);
    ImageProcess();
    ~ImageProcess(){};
    
    void ImageProductor();                                        // 线程生产者
    void ImageConsumer();                                         // 线程消费者

    // void ImageProductor_Single(Mat &oriFrame);                    // 线程生产者(单线程)
    // void ImageConsumer_Single(Mat &src, Energy &energy_detector); //线程消费者(单线程)
    
    // void EnergyThread();    // 能量机关识别独立线程

    void ShootingAngleCompensate(double &distance,double &angle_x,double &angle_y);//Angle Compensate based on pnp distance
    bool AdvancedPredictForArmorDetect(RotatedRect &present_armor,RotatedRect &predict_armor);//装甲板卡尔曼
private:
    SerialPort _port;
    int _sentrymode;
    int _basemode;

    bool is_target_spinning;                    //目标是否进入陀螺模式
    float spinning_coeffient;                      //旋转置信度

    RotatedRect last_switched_armor;            //最后一次切换的装甲板

    queue<RotatedRect> armor_queue;                     //记录辅瞄装甲板集合
    queue<clock_t> armor_queue_time;             //记录上面集合的保存时间

    Kalman kalmanfilter{KALMAN_TYPE_ARMOR};             //设置卡尔曼滤波器类型

    

};
