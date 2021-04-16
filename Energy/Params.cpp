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

    #ifdef USE_USB_CAMERA
    // USB相机1的内参矩阵
    cameraMatrix = (cv::Mat_<double>(3, 3) << 
                            1.3774427e+03, 0,             3.3584502e+02,
                            0,             1.3863849e+03, 2.6509033e+02,
                            0,             0,             1);
    // USB相机1的畸变矩阵
    distCoeffs = (cv::Mat_<double>(1, 5 ) << 0.25412988,  -1.8356142, 0, 0, 0);
    #endif // USE_USB_CAMERA


    #ifdef USE_DAHENG_CAMERA    // FIXME: MATLAB测量矩阵
    // 大恒139的内参矩阵
    // cameraMatrix = (cv::Mat_<double>(3, 3) << 
    //                         1.3774427e+03, 0,             3.3584502e+02,
    //                         0,             1.3863849e+03, 2.6509033e+02,
    //                         0,             0,             1);
    // // 大恒139的畸变矩阵
    // distCoeffs = (cv::Mat_<double>(1, 5 ) << 0.25412988,  -1.8356142, 0, 0, 0);
    //     // 大恒139的内参矩阵
    cameraMatrix = (cv::Mat_<double>(3, 3) << 
                            1, 1, 1,
                            1, 1, 1,
                            1, 1, 1);
    // 大恒139的畸变矩阵
    distCoeffs = (cv::Mat_<double>(1, 5 ) << 1,1, 1, 1, 1);
    // // static cv::Mat cameraMatrix;
    // // static cv::Mat distCoeffs;
    #endif // USE_DAHENG_CAMERA

    buff_width = 23;                                            // 能量机关实际的装甲板宽度
    buff_height = 12.7;                                         // 能量机关实际的装甲板高度

    // 二值化阈值
    
    DETECT_RED_GRAY_BINARY = 100;                                  //北理珠视频DEBUG红色阈值                   
    // DETECT_RED_GRAY_BINARY = 20;                                   // 识别红色时的阈值
    DETECT_BLUE_GRAY_BINARY = 100;                                  // 识别蓝色时的阈值

    gray_element = getStructuringElement(MORPH_RECT, Size(7, 7));// 膨胀腐蚀参数
    element = getStructuringElement(MORPH_RECT, Size(5, 5));    // 膨胀腐蚀参数

    // FIXME: 大小能量机关参数有变
    bullet_fly_time = 0.5;
    small_mode_predict_angle = CV_PI / 7.8;                 // 小能量机关模式打击偏移 
    big_mode_predict_angle = 0;                       // 大能量机关模式打击偏移

    
    // // // ARMOR
    // ARMOR_CONTOUR_AREA_MAX = 600;        // 装甲板轮廓面积最大值
    // ARMOR_CONTOUR_AREA_MIN = 200;        // 装甲板轮廓面积最小值
    // ARMOR_CONTOUR_LENGTH_MAX = 40;        // 装甲板轮廓长边最大值
    // ARMOR_CONTOUR_LENGTH_MIN = 20;        // 装甲板轮廓长边最小值
    // ARMOR_CONTOUR_WIDTH_MAX = 20;         // 装甲板轮廓短边最大值
    // ARMOR_CONTOUR_WIDTH_MIN = 10;         // 装甲板轮廓短边最小值
    // ARMOR_CONTOUR_HW_RATIO_MAX = 2.4;     // 装甲板轮廓长宽比最大值
    // ARMOR_CONTOUR_HW_RATIO_MIN = 1.5;     // 装甲板轮廓长宽比最小值
    
    // // //ARMOR(北理珠视频Debug)
    ARMOR_CONTOUR_AREA_MAX = 8000;        // 装甲板轮廓面积最大值
    ARMOR_CONTOUR_AREA_MIN = 4000;        // 装甲板轮廓面积最小值
    ARMOR_CONTOUR_LENGTH_MAX = 120;        // 装甲板轮廓长边最大值
    ARMOR_CONTOUR_LENGTH_MIN = 50;        // 装甲板轮廓长边最小值
    ARMOR_CONTOUR_WIDTH_MAX = 100;         // 装甲板轮廓短边最大值
    ARMOR_CONTOUR_WIDTH_MIN = 40;         // 装甲板轮廓短边最小值
    ARMOR_CONTOUR_HW_RATIO_MAX = 2;     // 装甲板轮廓长宽比最大值
    ARMOR_CONTOUR_HW_RATIO_MIN = 1;     // 装甲板轮廓长宽比最小值


    // FLOW_STRIP_FAN
    // FLOW_STRIP_FAN_CONTOUR_AREA_MAX = 3000;        // 含流动条的扇叶轮廓面积最大值
    // FLOW_STRIP_FAN_CONTOUR_AREA_MIN = 1000;        // 含流动条的扇叶轮廓面积最小值
    // FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX = 130;       // 含流动条的扇叶轮廓长边最大值
    // FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN = 80;       // 含流动条的扇叶轮廓长边最小值
    // FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX = 75;         // 含流动条的扇叶轮廓短边最大值
    // FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN = 40;         // 含流动条的扇叶轮廓短边最小值
    // FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX = 2.5;     // 含流动条的扇叶长宽比最大值
    // FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN = 1.8;       // 含流动条的扇叶长宽比最小值
    // FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX = 0.58;   // 含流动条的扇叶轮廓的面积之比最大值
    // FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN = 0.2;  // 含流动条的扇叶轮廓的面积之比最小值

    // // // FLOW_STRIP_FAN(北理珠视频Debug)
    FLOW_STRIP_FAN_CONTOUR_AREA_MAX = 20000;        // 含流动条的扇叶轮廓面积最大值
    FLOW_STRIP_FAN_CONTOUR_AREA_MIN = 8000;        // 含流动条的扇叶轮廓面积最小值
    FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX = 300;       // 含流动条的扇叶轮廓长边最大值
    FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN = 200;       // 含流动条的扇叶轮廓长边最小值
    FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX = 150;         // 含流动条的扇叶轮廓短边最大值
    FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN = 100;         // 含流动条的扇叶轮廓短边最小值
    FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX = 2.5;     // 含流动条的扇叶长宽比最大值
    FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN = 1.8;       // 含流动条的扇叶长宽比最小值
    FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX = 0.8;   // 含流动条的扇叶轮廓的面积之比最大值
    FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN = 0;  // 含流动条的扇叶轮廓的面积之比最小值


    // cout << "Init params done" << endl;
}






