//----------------------------------------------------------
//
// FileName: Energy.h
// Author: Liu Shang fyrwls@163.com
// Version: 1.0.20200924
// Date: 2020.09.24
// Description: Energy类的定义
// Function List:
//              1. void run(Mat &oriFrame)
//              2. void clearVectors()
//              3. void initFrames(Mat &frame)
//              4. bool getDirectionOfRotation()
//              5. bool predictTargetPoint(Mat &frame)
//              6. bool findArmors(Mat &src)
//              7. bool findTargetArmor(Mat &src_bin)
//              8. bool isValidArmor(vector<Point>&armor_contour)
//              9. bool predictRCenter(Mat &frame)
//              10. bool findFlowStripFan(Mat &src)
//              11. bool isValidFlowStripFan(Mat &src_bin, vector<Point> &flow_strip_fan_contour)
//              12. int getRectIntensity(Mat &src, Rect &roi_rect)
//
//----------------------------------------------------------


#include "./Params.h"
#include "./EnergyDebug.h"
#include "../Debug.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect.hpp>

#include <iostream>
#include <string>   // 字符串
#include <vector>   // 向量容器
#include <queue>    // 队列
// #include <thread>   // 线程库
#include <mutex>    // 互斥量
#include <time.h>


using namespace std;
using namespace cv;

//----------------------------------------------------------//
//                                                          //
//                      class Energy                        //
//                                                          //
//----------------------------------------------------------//

class Energy
{
public:
    Energy() {};    
    ~Energy() {};
    #ifdef SHOW_RECT_INFO 
    Mat src_rect_info;      //显示矩形信息
    #endif//SHOW_RECT_INFO
    
    void run(Mat &oriFrame);                                    // 识别总函数

private:
    Params energyParams;                                        // 实例化参数对象

    queue<double> armor_angle_queue;                            // 存储装甲板的角度,先存储3帧

    vector<RotatedRect> armors_rrect;                           // 初步筛选到的各种装甲板
    vector<Point2f> armor_center_points;                        // 装甲板中心点的集合,用于做拟合
    vector<RotatedRect> flow_strip_fan_rrect;                   // 筛选出来的有流动条的扇叶

    clock_t debug_cnt;                                          // DEBUG时间戳

    RotatedRect center_ROI;                                     // 中心ROI
    Point2f RCenter;                                            // 中心点坐标
    Point2f predict_point;                                      // 打击预测点
    
    RotatedRect target_armor;                                   // 最终得到的装甲板
    RotatedRect target_flow_strip_fan;                          // 最终得到的含有流动条的扇叶
    RotatedRect centerR;                                        // 最终找到的R

    int rotation;                                               // 当前旋转方向


    double angle_confidence;                                    // 角度置信度

    vector<Point2f> points2D;                                   // 用于pnp的平面二维点
    vector<Point3f> points3D;                                   // 用于pnp的空间三维点

    void getPoints2D();                                         // 获得pnp的平面二维点
    void getPoints3D();                                         // 获得pnp的空间三维点
    void solveXYZ();                                            // 使用pnp解算

    void clearVectors();                                        // 清空各vector
    void initFrame(Mat &frame);                                 // 图像预处理

    bool getDirectionOfRotation();                              // 获得当前旋转方向
    bool predictTargetPoint(Mat &frame);                        // 对打击点进行预测

    bool findArmors(Mat &src);                                  // 寻找装甲板
    bool findTargetArmor(Mat &src_bin);                         // 寻找最终的装甲板
    bool isValidArmor(vector<Point>&armor_contour);             // 检查装甲板是否符合要求

    bool predictRCenter(Mat &frame);                            // 预测字母R的中心点
    bool findCenterROI() ;                                      // 寻找中心R的ROI
    bool findCenterR(Mat &src_bin)  ;                           // 寻找中心R结构
    bool isValidCenterRContour(const vector<cv::Point> &center_R_contour);  //检查中心R是否符合要求

    inline float spd_func(float time);        // spd函数  
    inline float theta_func(float time);      // 对spd的积分函数

    bool findFlowStripFan(Mat &src);                        // 寻找含流动条的扇叶
    bool isValidFlowStripFan(Mat &src_bin, vector<Point> &flow_strip_fan_contour);// 检查含流动条的扇叶是否合格

    int getRectIntensity(Mat &src, Rect &roi_rect);         // 获取roi区域的强度
};