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

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "../Camera/DaHengCamera.h"
#include "../Energy/Energy.h"
#include "../Debug.h"
#include "../Serial/serialport.h"

using namespace std;
using namespace cv;

class ImageProcess
{
public:
    ImageProcess(SerialPort &port);
    ~ImageProcess(){};
    ImageProcess();
    void ImageProductor();                                        // 线程生产者
    void ImageProductor_Single(Mat &oriFrame);                    // 线程生产者(单线程)
    void ImageConsumer();                                         // 线程消费者
    void ImageConsumer_Single(Mat &src, Energy &energy_detector); //线程消费者(单线程)
    
    void ShootingAngleCompensate(double &distance,double &angle_x,double &angle_y);//Angle Compensate based on pnp distance
private:
    SerialPort _port;
    int _sentrymode;
    int _basemode;
};