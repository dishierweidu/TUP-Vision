//----------------------------------------------------------
//
// FileName: Params.cpp
// Author: Liu Shang fyrwls@163.com
// Version: 1.0.0_x64Linux
// Date: 2020.09.24
// Description: Params中参数的赋值
// Function List:
//              1. void Params::initParams()
// 
//----------------------------------------------------------

#include "./Params.h"


//----------------------------------------------------------//
//                                                          //
//                      class Params                        //
//                                                          //
//----------------------------------------------------------//


// @brief 初始化一些参数
void Params::initParams()
{
    stm32Data.initParams();                                     // 这里默认为stm32传来的数据

    buff_width = 23;                                            // 能量机关实际的装甲板宽度(单位:cm)
    buff_height = 12.7;
    // arm_length = 56.5;                                          // 能量机关旋臂长度(单位:cm)
    max_arm_length = 200;                                        //圆心到击打点最大像素距离
    // max_arm_length = 300;                                        //圆心到击打点最大像素距离(ZHBIT)

    // 二值化阈值
    
    // DETECT_RED_GRAY_BINARY = 100;                                  //北理珠视频DEBUG红色阈值                   
    // DETECT_RED_GRAY_BINARY = 10;                                   // 识别红色时的阈值
    DETECT_RED_GRAY_BINARY = 23;                                   // 识别红色时的阈值
    // DETECT_RED_GRAY_BINARY = 33;                                   // 识别红色时的阈值(day)
    // DETECT_BLUE_GRAY_BINARY = 100;                                  // 识别蓝色时的阈值

    gray_element = getStructuringElement(MORPH_RECT, Size(5, 5));// 膨胀腐蚀参数
    element = getStructuringElement(MORPH_RECT, Size(5, 5));    // 膨胀腐蚀参数

    // FIXME: 大小能量机关参数有变
    bullet_fly_time = 0.5;
    small_mode_predict_angle =0;                 // 小能量机关模式打击偏移 
    big_mode_predict_angle = 0;                       // 大能量机关模式打击偏移

    
    // // // // ARMOR
    ARMOR_CONTOUR_AREA_MAX = 800;        // 装甲板轮廓面积最大值
    ARMOR_CONTOUR_AREA_MIN = 200;        // 装甲板轮廓面积最小值
    ARMOR_CONTOUR_LENGTH_MAX = 80;        // 装甲板轮廓长边最大值
    ARMOR_CONTOUR_LENGTH_MIN = 20;        // 装甲板轮廓长边最小值
    ARMOR_CONTOUR_WIDTH_MAX = 40;         // 装甲板轮廓短边最大值
    ARMOR_CONTOUR_WIDTH_MIN = 8;         // 装甲板轮廓短边最小值
    ARMOR_CONTOUR_HW_RATIO_MAX = 2.6;     // 装甲板轮廓长宽比最大值
    ARMOR_CONTOUR_HW_RATIO_MIN = 1.2;     // 装甲板轮廓长宽比最小值
    
    // // //ARMOR(北理珠视频Debug)
    // ARMOR_CONTOUR_AREA_MAX = 8000;        // 装甲板轮廓面积最大值
    // ARMOR_CONTOUR_AREA_MIN = 4000;        // 装甲板轮廓面积最小值
    // ARMOR_CONTOUR_LENGTH_MAX = 120;        // 装甲板轮廓长边最大值
    // ARMOR_CONTOUR_LENGTH_MIN = 50;        // 装甲板轮廓长边最小值
    // ARMOR_CONTOUR_WIDTH_MAX = 100;         // 装甲板轮廓短边最大值
    // ARMOR_CONTOUR_WIDTH_MIN = 40;         // 装甲板轮廓短边最小值
    // ARMOR_CONTOUR_HW_RATIO_MAX = 2;     // 装甲板轮廓长宽比最大值
    // ARMOR_CONTOUR_HW_RATIO_MIN = 1;     // 装甲板轮廓长宽比最小值


    // FLOW_STRIP_FAN
    FLOW_STRIP_FAN_CONTOUR_AREA_MAX = 5000;        // 含流动条的扇叶轮廓面积最大值
    FLOW_STRIP_FAN_CONTOUR_AREA_MIN = 1000;        // 含流动条的扇叶轮廓面积最小值
    FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX = 150;       // 含流动条的扇叶轮廓长边最大值
    FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN = 80;       // 含流动条的扇叶轮廓长边最小值
    FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX = 80;         // 含流动条的扇叶轮廓短边最大值
    FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN = 20;         // 含流动条的扇叶轮廓短边最小值
    FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX = 2.5;     // 含流动条的扇叶长宽比最大值
    FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN = 1.7;       // 含流动条的扇叶长宽比最小值
    FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX = 0.65;   // 含流动条的扇叶轮廓的面积之比最大值
    FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN = 0.2;  // 含流动条的扇叶轮廓的面积之比最小值

    // // // FLOW_STRIP_FAN(北理珠视频Debug)
    // FLOW_STRIP_FAN_CONTOUR_AREA_MAX = 20000;        // 含流动条的扇叶轮廓面积最大值
    // FLOW_STRIP_FAN_CONTOUR_AREA_MIN = 8000;        // 含流动条的扇叶轮廓面积最小值
    // FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX = 300;       // 含流动条的扇叶轮廓长边最大值
    // FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN = 200;       // 含流动条的扇叶轮廓长边最小值
    // FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX = 150;         // 含流动条的扇叶轮廓短边最大值
    // FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN = 100;         // 含流动条的扇叶轮廓短边最小值
    // FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX = 2.5;     // 含流动条的扇叶长宽比最大值
    // FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN = 1.8;       // 含流动条的扇叶长宽比最小值
    // FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX = 0.8;   // 含流动条的扇叶轮廓的面积之比最大值
    // FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN = 0;  // 含流动条的扇叶轮廓的面积之比最小值

    // //CENTER_R
    CENTER_R_CONTOUR_AREA_MAX = 500;          //风车中心R面积最大值
    CENTER_R_CONTOUR_AREA_MIN = 50;           //风车中心R面积最小值
    CENTER_R_CONTOUR_LENGTH_MAX = 40;        //风车中心R宽边长度最小值
    CENTER_R_CONTOUR_LENGTH_MIN = 10;        //风车中心R长边长度最小值
    CENTER_R_CONTOUR_WIDTH_MAX = 20;         //风车中心R宽边长度最大值
    CENTER_R_CONTOUR_WIDTH_MIN = 5;         //风车中心R长边长度最大值
    CENTER_R_CONTOUR_HW_RATIO_MAX = 2;       //风车中心R长宽比最大值
    CENTER_R_CONTOUR_HW_RATIO_MIN = 0.5;      //风车中心R长宽比最小值
    CENTER_R_CONTOUR_AREA_RATIO_MIN = 0.7;          //FIXME:装甲板轮廓占旋转矩形面积比最小值
    CENTER_R_CONTOUR_INTERSETION_AREA_MIN = 100;    //中心R占ROI区的面积最小值

    //CENTER_R(北理珠视频DEBUG)
    // CENTER_R_CONTOUR_AREA_MAX = 2000;          //风车中心R面积最大值
    // CENTER_R_CONTOUR_AREA_MIN = 50;           //风车中心R面积最小值
    // CENTER_R_CONTOUR_LENGTH_MAX = 50;        //风车中心R宽边长度最小值
    // CENTER_R_CONTOUR_LENGTH_MIN = 10;        //风车中心R长边长度最小值
    // CENTER_R_CONTOUR_WIDTH_MAX = 50;         //风车中心R宽边长度最大值
    // CENTER_R_CONTOUR_WIDTH_MIN = 5;         //风车中心R长边长度最大值
    // CENTER_R_CONTOUR_HW_RATIO_MAX = 3.5;       //风车中心R长宽比最大值
    // CENTER_R_CONTOUR_HW_RATIO_MIN = 0.8;      //风车中心R长宽比最小值

    // cout << "Init params done" << endl;
}






