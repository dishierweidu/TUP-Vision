//----------------------------------------------------------
//
// FileName: ArmorDetector.h
// Author: 周俊平;刘上;赵梓合;顾昊
// Version: 1.1.0
// Date: 2021.07.14
// Description: ArmorDetector类中函数的声明及一些变量的赋值
//----------------------------------------------------------
#pragma once


#include <iostream>
#include <queue>

#include "opencv2/highgui/highgui.hpp"

#include "../General/General.h"
#include "../SVM/Svm.h"
#include "../AngelSolver/AngleSolver.hpp"

enum EnemyColor
{
    RED = 0,
    BLUE = 1
};

struct ArmorParam
{
    int near_face_v;
    int light_min_height;           // 板灯最小高度值，像素单位
    int light_slope_offset;         // 允许灯柱偏离垂直线的最大偏移量，角度制
    int light_max_delta_h;          // 左右灯柱在水平位置上的最大差值，像素单位
    int light_min_delta_h;          // 左右灯柱在水平位置上的最小差值，像素单位
    int light_max_delta_v;          // 左右灯柱在垂直位置上的最大差值，像素单位
    float light_max_lr_rate;        // 左右灯柱的比例值
    float light_max_delta_angle;      // 左右灯柱在斜率最大差值，角度制
    float armor_max_wh_ratio;             // 最小长宽比
    float armor_min_wh_ratio;             // 最小长宽比
    float armor_type_wh_threshold; // 大小装甲的界限阈值
    float armor_max_angle;          //装甲板最大倾斜角度,角度制
    // 默认的构造值
    ArmorParam()
    {
        near_face_v = 600;
        light_min_height = 8;
        light_slope_offset = 30;
        light_max_delta_h = 450;
        light_min_delta_h = 12;
        // light_max_delta_v = 45;
        light_max_delta_v = 70;
        light_max_delta_angle = 10;
        // light_max_delta_angle = 30;
        // light_max_lr_rate = 2.0;
        light_max_lr_rate = 1.2;
        armor_max_wh_ratio = 4 ;
        armor_min_wh_ratio = 1.25;
        armor_type_wh_threshold = 3.6;
        armor_max_angle  = 28;
    }
};



// 匹配灯条的结构体
struct MatchedRect
{    
    float lr_rate;
    float angle_abs;
    cv::RotatedRect rrect;
    cv::Point2f apex[4];
};

class ArmorDetector
{
public:
    //@brief 参数设置
    void setPara(const ArmorParam &para)
    {
        _para = para;
    }
    //@brief 大小装甲板角度解算法
    void setPnPSlover(AngleSolver *solver_l)
    {
        l_solver = solver_l;
    }

    bool getTargetArea(const cv::Mat &src,ArmorPlate &target_armor, const int &sentry_mode, const int &base_mode);
public:
    // 无参构造函数，默认全部都为false
    ArmorDetector(const ArmorParam &para = ArmorParam())
    {
        s_solver = NULL;
        l_solver = NULL;
        _para = para;
        _res_last = cv::RotatedRect();
        _dect_rect = cv::Rect();
        _lost_cnt = 0;
        _is_lost = true;
    }

    int enemy_color;
    int sentry_mode; // 哨兵模式
    int base_mode;   // 吊射基地模式

//----------------------参数设置------------------------------//
    const int threshold_max_color_red = 30;
    const int threshold_max_color_blue = 36;
    const int threshold_gray_red = 40;
    const int threshold_gray_blue = 50;
//----------------------------------------------------------//

private:

    int _lost_cnt;              // 失去目标的次数，n帧没有识别到则全局搜索
    bool _is_lost;              //是否丢失目标

    ArmorParam _para;           // 装甲板参数
    AngleSolver *s_solver;
    AngleSolver *l_solver;

    cv::Mat _b;
    cv::Mat _r;    
    cv::Mat _src;               //原图
    cv::Mat thres_whole;        //灰度二值化后的图像
    cv::Mat _max_color;         //通道相减二值化的图像

    cv::RotatedRect _res_last;  // 最后一次的装甲板旋转矩形
    cv::Rect _dect_rect;        // ROI
    cv::Size _size;             //原图的尺寸大小

    bool setImage(const cv::Mat &src);
    bool findTargetInContours(std::vector<MatchedRect> &match_rects);
    bool chooseTarget(const std::vector<MatchedRect> &match_rects, ArmorPlate &target_armor, const cv::Mat &src);

    cv::RotatedRect boundingRRect(const cv::RotatedRect &left, const cv::RotatedRect &right);

};
