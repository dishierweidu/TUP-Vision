//----------------------------------------------------------
//
// FileName: Params.h
// Author: Liu Shang fyrwls@163.com
// Version: 1.0.20200924
// Date: 2020.09.24
// Description: 一些特殊值的宏定义,进行Params中参数的定义
// Function List:
//              1. void Params::initParams()
// 
//----------------------------------------------------------

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../Debug.h"

using namespace std;
using namespace cv;



/*
ARMOR           装甲板
CENTER_R        中心R
FLOW_STRIP_FAN  流动条扇叶,也就是锤柄状的东西
*/

#define ENEMY_BLUE 0        // 敌方蓝色
#define ENEMY_RED 1         // 敌方红色

#define OUR_RED ENEMY_BLUE  // 我方红色
#define OUR_BLUE ENEMY_RED  // 我方蓝色

#define ENERGY_BIG 1        // 大能量机关模式
#define ENERGY_SMALL 0      // 小能量机关模式

#define CLOCKWISE 1         // 顺时针
#define COUNTER_CLOCKWISE 0 // 逆时针




//----------------------------------------------------------//
//                                                          //
//                     struct STM32Data                     //
//                                                          //
//----------------------------------------------------------//

// 从单片机传来的数据
struct STM32Data
{
    int enemy_color;    // 敌方颜色
    int energy_mode;    // 大小能量机关模式


    void initParams()
    {
        enemy_color = ENEMY_RED;
        energy_mode = ENERGY_BIG;
    }
};


//----------------------------------------------------------//
//                                                          //
//                          Camera                          //
//                                                          //
//----------------------------------------------------------//




//----------------------------------------------------------//
//                                                          //
//                      class Params                        //
//                                                          //
//----------------------------------------------------------//

// 能量机关各部分的统一参数类
class Params
{
public:
    Params() { initParams(); };
    ~Params() {};
    void initParams();      // 初始化参数

    STM32Data stm32Data;    // 这里默认为stm32传来的数据

    // camerainitParams
    Mat cameraMatrix;
    Mat distCoeffs;

    double buff_width;     // 能量机关装甲板宽度
    double buff_height;      // 能量机关装甲板高度
    double arm_length;          //悬臂臂长

    int max_arm_length;

    // 二值化阈值
    int DETECT_RED_GRAY_BINARY;    // 我方为红色时的阈值
    int DETECT_BLUE_GRAY_BINARY;   // 我方为蓝色时的阈值

    Mat gray_element;   // 膨胀腐蚀参数
    Mat element;        // 膨胀腐蚀参数

    float bullet_fly_time;              // 子弹实际飞行的时间
    float small_mode_predict_angle;    // 小能量机关模式打击直线偏移
    float big_mode_predict_angle;       // 大能量机关模式打击旋转偏移

    
    // ARMOR
    int ARMOR_CONTOUR_AREA_MAX;         // 装甲板轮廓面积最大值
    int ARMOR_CONTOUR_AREA_MIN;         // 装甲板轮廓面积最小值
    float ARMOR_CONTOUR_LENGTH_MAX;       // 装甲板轮廓长边最大值
    float ARMOR_CONTOUR_LENGTH_MIN;       // 装甲板轮廓长边最小值
    float ARMOR_CONTOUR_WIDTH_MAX;        // 装甲板轮廓短边最大值
    float ARMOR_CONTOUR_WIDTH_MIN;        // 装甲板轮廓短边最小值
    float ARMOR_CONTOUR_HW_RATIO_MAX;     // 装甲板长宽比最大值
    float ARMOR_CONTOUR_HW_RATIO_MIN;     // 装甲板长宽比最小值
    

    // FLOW_STRIP_FAN
    int FLOW_STRIP_FAN_CONTOUR_AREA_MAX;            // 含流动条的扇叶轮廓面积最大值
    int FLOW_STRIP_FAN_CONTOUR_AREA_MIN;            // 含流动条的扇叶轮廓面积最小值
    float FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX;       // 含流动条的扇叶轮廓长边最大值
    float FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN;       // 含流动条的扇叶轮廓长边最小值
    float FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX;        // 含流动条的扇叶轮廓短边最大值
    float FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN;        // 含流动条的扇叶轮廓短边最小值
    float FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX;     // 含流动条的扇叶长宽比最大值
    float FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN;     // 含流动条的扇叶长宽比最小值
    float FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX;   // 含流动条的扇叶轮廓的面积之比最大值
    float FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN;   // 含流动条的扇叶轮廓的面积之比最小值


    //CENTER_R
    long CENTER_R_CONTOUR_AREA_MAX;//风车中心R面积最大值
    long CENTER_R_CONTOUR_AREA_MIN;//风车中心R面积最小值
    long CENTER_R_CONTOUR_LENGTH_MIN;//风车中心R长边长度最小值
    long CENTER_R_CONTOUR_WIDTH_MIN;//风车中心R长边长度最大值
    long CENTER_R_CONTOUR_LENGTH_MAX;//风车中心R宽边长度最小值
    long CENTER_R_CONTOUR_WIDTH_MAX;//风车中心R宽边长度最大值
    float CENTER_R_CONTOUR_HW_RATIO_MAX;//风车中心R长宽比最大值
    float CENTER_R_CONTOUR_HW_RATIO_MIN;//风车中心R长宽比最小值
    float CENTER_R_CONTOUR_AREA_RATIO_MIN;          //装甲板轮廓占旋转矩形面积比最小值
    float CENTER_R_CONTOUR_INTERSETION_AREA_MIN;    //中心R占ROI区的面积最小值


};