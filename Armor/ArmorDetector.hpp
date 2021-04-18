//----------------------------------------------------------
//
// FileName: ImageProcess.cpp
// Author: 周俊平;刘上;赵梓合;顾昊
// Version: 1.0.0
// Date: 2021.04.10
//
//----------------------------------------------------------
#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "../AngelSolver/AngleSolver.hpp"
#include "opencv2/dnn/dnn.hpp"

#define TRUNC_ABS(a) ((a) > 0 ? (a) : 0);
#define POINT_DIST(p1, p2) std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y))

enum EnemyColor
{
    RED = 0,
    BLUE = 1
};

// 很多成员都已经弃用了
struct ArmorParam
{
    int min_light_height;           // 板灯最小高度值
    int light_slope_offset;         // 允许灯柱偏离垂直线的最大偏移量，单位度
    int max_light_delta_h;          // 左右灯柱在水平位置上的最大差值，像素单位
    int min_light_delta_h;          // 左右灯柱在水平位置上的最小差值，像素单位
    int max_light_delta_v;          // 左右灯柱在垂直位置上的最大差值，像素单位
    int max_light_delta_angle;      // 左右灯柱在斜率最大差值，单位度
    int near_face_v;                // 贴脸的距离
    float max_lr_rate;              // 左右灯柱的比例值
    float max_wh_ratio;             // 装甲板的最大长宽比
    float min_wh_ratio;             // 最小长宽比
    float small_armor_wh_threshold; // 大小装甲的界限阈值
    int bin_cls_thres;              // 二分类防止三根灯条的分割阈值
    float target_max_angle;

    // 默认的构造值
    ArmorParam()
    {
        min_light_height = 8;
        light_slope_offset = 30;
        max_light_delta_h = 450;
        min_light_delta_h = 12;
        max_light_delta_v = 45;
        max_light_delta_angle = 30;
        near_face_v = 600;
        max_lr_rate = 2.0;
        max_wh_ratio = 6;
        min_wh_ratio = 1.25;
        small_armor_wh_threshold = 3.6;
        bin_cls_thres = 166;
        target_max_angle = 20;
    }
};

// 匹配灯条的结构体
struct matched_rect
{
    cv::RotatedRect rect;
    float lr_rate;
    float angle_abs;
};

//候选灯条的结构体
struct candidate_target
{
    int armor_height;
    float armor_angle;
    int index;
    bool is_small_armor;
    float bar_lr_rate;
    float bar_angle_abs;
};

class ArmorDetector
{
public:
    // 无参构造函数，默认全部都为false
    ArmorDetector(const ArmorParam &para = ArmorParam())
    {
        s_solver = NULL;
        l_solver = NULL;
        _para = para;
        _res_last = cv::RotatedRect();
        _dect_rect = cv::Rect();
        _is_small_armor = false;
        _lost_cnt = 0;
        _is_lost = true;
    }

    int enemy_color;
    int sentry_mode; // 哨兵模式
    int base_mode;   // 吊射基地模式
    int svm_sam_num = 0;
    cv::Mat sample_pos;

    int get_lost()
    {
        return _lost_cnt;
    }

    void setPara(const ArmorParam &para)
    {
        _para = para;
    }

    // 大小装甲板的角度解算法
    void setPnPSlover(AngleSolver *solver_l)
    {
        l_solver = solver_l;
    }

    // 所有都清零
    void reset()
    {
        _res_last = cv::RotatedRect();
        _dect_rect = cv::Rect();
        _is_small_armor = false;
        _lost_cnt = 0;
        _is_lost = true;
    }

    bool isSamllArmor()
    {
        return _is_small_armor;
    }

    cv::RotatedRect getTargetArea(const cv::Mat &src, const int &sb_mode, const int &jd_mode);

    void setLastResult(const cv::RotatedRect &rect)
    {
        _res_last = rect;
    }

    // 返回上一个结果，但是这个写法真的是骚
    const cv::RotatedRect &getLastResult() const
    {
        return _res_last;
    }

    // 调车时增加的变量
    int _find_cnt;

    int get_near_face()
    {
        return _para.near_face_v;
    }

private:
    /**
     * @brief setImage Pocess the input (set the green component and sub of blue and red component)
     * @param src
     */
    void setImage(const cv::Mat &src);

    void findTargetInContours(std::vector<matched_rect> &match_rects);
    /**
     * @brief chooseTarget Choose the most possible rectangle among all the rectangles
     * @param rects candidate rectangles
     * @return the most likely armor (RotatedRect() returned if no proper one)
     */
    cv::RotatedRect chooseTarget(const std::vector<matched_rect> &match_rects /*, const std::vector<double> & score*/, const cv::Mat &src);
    /**
     * @brief 将左右的两根灯条用一个旋转矩形拟合并返回
     * @brief boundingRRect Bounding of two ratate rectangle (minumum area that contacts two inputs)
     * @param left left RotatedRect
     * @param right right RotatedRect
     * @return minumum area that contacts two inputs
     */
    cv::RotatedRect boundingRRect(const cv::RotatedRect &left, const cv::RotatedRect &right);

    /**
     * @brief 调整旋转矩形的角度
     * @brief adjustRRect Adjust input angle
     * @param rect input
     * @return adjusted rotate rectangle
     */
    cv::RotatedRect adjustRRect(const cv::RotatedRect &rect);

    /**
     * @brief makeRectSafe 使矩形不发生越界
     * @param rect 输入的矩形， x， y为左上角的坐标
     * @param size 限制的大小，防止越界
     * @return
     */
    bool makeRectSafe(cv::Rect &rect, cv::Size size)
    {
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        if (rect.width <= 0 || rect.height <= 0)
            // 如果发现矩形是空的，则返回false
            return false;
        return true;
    }

    /**
     * @brief broadenRect 给矩形扩张，然后判断是否越界
     * @param rect        思想是左上角移动，也就是围绕中心扩张
     * @param width_added 扩张的宽 / 2
     * @param height_added 扩张的高/ 2
     * @param size
     * @return
     */
    bool broadenRect(cv::Rect &rect, int width_added, int height_added, cv::Size size)
    {
        rect.x -= width_added;
        rect.width += width_added * 2;
        rect.y -= height_added;
        rect.height += height_added * 2;
        return makeRectSafe(rect, size);
    }

private:
    AngleSolver *s_solver;
    AngleSolver *l_solver;
    bool _is_lost;
    ArmorParam _para;          // parameter of alg
    int _lost_cnt;             // 失去目标的次数，n帧没有识别到则全局搜索
    bool _is_small_armor;      // true if armor is the small one, otherwise false
    cv::RotatedRect _res_last; // last detect result
    cv::Rect _dect_rect;       // detect roi of original image

    cv::Mat _src;       // source image
    cv::Mat _g;         // green component of source image
    cv::Mat _ec;        // enemy color
    cv::Mat _max_color; // binary image of sub between blue and red component
    cv::Size _size;     // 源图的尺寸大小
    cv::Mat _b;
    cv::Mat _r;

    //Kalman滤波参数
    float v_tx_old; //线速度保留量
    float v_ty_old;
    float v_tz_old;
    float p_tx_old; //位置保留量
    float p_ty_old;
    float p_tz_old;
};
