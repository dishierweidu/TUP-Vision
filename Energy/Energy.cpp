//----------------------------------------------------------
//
// FileName: Energy.cpp
// Author: Liu Shang fyrwls@163.com
// Version: 1.0.20200924
// Date: 2020.09.24
// Description: Energy class中函数的实现,还有3个公共函数的实现
// Function List:
//              1. void drawRotatedRect(Mat frame, RotatedRect rRect, Scalar color, int thickness = 1)
//              2. double pointsDistance(Point2f p1, Point2f p2)
//              3. bool circleLeastFit(vector<Point2f> &points, Point2f &RCenter)
//              
//              1. void Energy::run(Mat &oriFrame)
//              2. void Energy::clearVectors()
//              3. void Energy::initFrame(Mat &frame)
//              4. bool Energy::getDirectionOfRotation()
//              5. bool Energy::predictTargetPoint(Mat &frame)
//              6. bool Energy::findArmors(Mat &src)
//              7. bool Energy::findTargetArmor(Mat &src_bin)
//              8. bool Energy::isValidArmor(vector<Point>&armor_contour)
//              9. bool Energy::predictRCenter(Mat &frame)
//              10. bool Energy::findFlowStripFan(Mat &src)
//              11. bool Energy::isValidFlowStripFan(Mat &src_bin, vector<Point> &flow_strip_fan_contour)
//              12. int Energy::getRectIntensity(Mat &src, Rect &roi_rect)
// 
//----------------------------------------------------------


#include "./Energy.h"

static double absolute_run_time = static_cast<double>(getTickCount());

//----------------------------------------------------------//
//                                                          //
//                      一些公共函数                          //
//                                                          //
//----------------------------------------------------------//


// @brief 绘制旋转矩形并显示相关信息 by GuHao
// @param RRect 所需矩形
// @param src  输出图像
// @param color 矩形的颜色与相关数据颜色
// @param thickness 矩形线条的线宽与字体大小
void ShowRotateRectDetail(RotatedRect &RRect,Mat src,Scalar color=Scalar(0,150,0), int thickness = 2)
{
    //储存旋转矩形顶点数组
    Point2f Apex[4];
    RRect.points(Apex);
    const string Width = "Width :" + to_string((int)(RRect.size.width > RRect.size.height ? RRect.size.width :RRect.size.height));
    const string Height = "Height :" + to_string((int)(RRect.size.width < RRect.size.height ? RRect.size.width :RRect.size.height));
    const string Area = "Area :" + to_string((int)(RRect.size.width * RRect.size.height ));
    const string H_W = "H/W :" + to_string((float)((RRect.size.width > RRect.size.height ? RRect.size.width :RRect.size.height)/(RRect.size.width < RRect.size.height ? RRect.size.width :RRect.size.height)));
    for (int j = 0; j < 4; ++j)
        line(src, Apex[j], Apex[(j + 1) % 4], color, thickness);
    putText(src,Width,Apex[0],CV_FONT_HERSHEY_SIMPLEX,0.5,color,thickness);
    putText(src,Height,Apex[0]+Point2f(0,15),CV_FONT_HERSHEY_SIMPLEX,0.5,color,thickness);
    putText(src,Area,Apex[0]+Point2f(0,30),CV_FONT_HERSHEY_SIMPLEX,0.5,color,thickness);
    putText(src,H_W,Apex[0]+Point2f(0,45),CV_FONT_HERSHEY_SIMPLEX,0.5,color,thickness);



}


// @brief 绘制旋转矩形 by up
// @param frame 直线绘制的图像
// @param rRect 旋转矩形
// @param color 矩形的颜色
// @param thickness 矩形线条的线宽
void drawRotatedRect(Mat frame, RotatedRect rRect, Scalar color, int thickness = 1)
{
	//创建存储旋转矩形顶点的数组
	Point2f vertices[4];
	//存储顶点
	rRect.points(vertices);
	//绘制旋转矩形
	for (int j = 0; j < 4; ++j)
		line(frame, vertices[j], vertices[(j + 1) % 4], color, thickness);
}


// @brief return distance between two points
// @param p1 first point
// @param p2 second point
// @return distance between two points
double pointsDistance(Point2f p1, Point2f p2)
{
    return pow(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2), 0.5);
}


// @brief 对装甲板的旋转做拟合,寻找R字母的中心点
// @param points 装甲板中心点的集合
// @param RCenter 字母R的中心坐标
// @return 拟合成功返回true,否则返回false
bool circleLeastFit(vector<Point2f> &points, Point2f &RCenter)
{
	float center_x = 0.0f;
    float center_y = 0.0f;
    float radius = 0.0f;
    if (points.size() < 3){
        return false;
    }

    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

    int N = points.size();
    int i;
	for (i = 0; i < N; i++){
        double x = points[i].x;
        double y = points[i].y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }

    double C, D, E, G, H;
    double a, b, c;

    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

    center_x = a / (-2);
    center_y = b / (-2);
    radius = sqrt(a * a + b * b - 4 * c) / 2;
    RCenter = Point2f(center_x,center_y);

	return true;
}





//----------------------------------------------------------//
//                                                          //
//                      class Energy                        //
//                                                          //
//----------------------------------------------------------//


// @brief 识别总入口函数
// @param oriFrame 原始图像
void Energy::run(Mat &oriFrame)
{
    #ifdef DEBUG_PREDICT_INFORMATION_BUFF
    debug_cnt=clock();
    cout<<"Time:"<<(int)debug_cnt<<endl;
    #endif 


    if(oriFrame.empty())
    {
        cout << "oriFrame is empty!" << endl;
        return;
    }

    Mat frame = oriFrame.clone();

    #ifdef SHOW_RECT_INFO 
    frame.copyTo(src_rect_info);
    #endif
    clearVectors();
    initFrame(frame);

    if(!findFlowStripFan(frame))    // 寻找含流动条的装甲板
    {
        return; // 如果没找到
    }



    if(!findArmors(frame))   // 寻找装甲板
    {
        return; // 如果没找到
    }
    #ifdef SHOW_RECT_INFO 
    imshow("Rect_info",src_rect_info);
    #endif


    getPoints2D();
    getPoints3D();
    solveXYZ();
    predictTargetPoint(frame);

    #ifdef SHOW_PREDICT_POINT
    Mat PREDICT_POINT = frame.clone();
    cv::cvtColor(PREDICT_POINT, PREDICT_POINT, CV_GRAY2BGR);
    if(armor_center_points.size() == 50)
    {
        circle(PREDICT_POINT, predict_point, 5, Scalar(0, 255, 0), 2);
        line(PREDICT_POINT, predict_point, target_armor.center, Scalar(0, 0, 255), 2);
        circle(PREDICT_POINT, RCenter, pointsDistance(RCenter, target_armor.center), Scalar(0, 255, 0), 1);
    }
    imshow("SHOW_PREDICT_POINT", PREDICT_POINT);
    #ifdef DEBUG_PREDICT_INFORMATION_BUFF
    cout<<"\n\n"<<endl;
    #endif 
    #endif
}


// @brief 获得pnp平面二维坐标
void Energy::getPoints2D()
{
    Point2f points[4];  // TODO:

    points[0] = Point2f(target_armor.center.x - target_armor.size.width / 2, target_armor.center.y - target_armor.size.height / 2);
    points[1] = Point2f(target_armor.center.x + target_armor.size.width / 2, target_armor.center.y - target_armor.size.height / 2);
    points[2] = Point2f(target_armor.center.x + target_armor.size.width / 2, target_armor.center.y + target_armor.size.height / 2);
    points[3] = Point2f(target_armor.center.x - target_armor.size.width / 2, target_armor.center.y + target_armor.size.height / 2);
    
    points2D.clear();
    points2D.push_back(points[0]);
    points2D.push_back(points[1]);
    points2D.push_back(points[2]);
    points2D.push_back(points[3]);
}


// @brief 获得pnp空间三维坐标
void Energy::getPoints3D()
{
    float half_x = energyParams.buff_width / 2.0;
    float half_y = energyParams.buff_height / 2.0;

    points3D.push_back(Point3f(-half_x, -half_y, 0.0));
    points3D.push_back(Point3f(half_x, -half_y, 0.0));
    points3D.push_back(Point3f(half_x, half_y, 0.0));
    points3D.push_back(Point3f(-half_x, half_y, 0.0));
}


// @brief 使用pnp解算
void Energy::solveXYZ()
{
    // FIXME: 解算有问题
    Mat rvecs = Mat::zeros(3, 1, CV_64FC1);
    Mat tvecs = Mat::zeros(3, 1, CV_64FC1);

    // calib3d.hpp
    // solvePnP(points3D, points2D, cameraMatrix, distCoeffs, rvecs, tvecs);

    Mat rotM = Mat::eye(3, 3, CV_64F);
    Mat rotT = Mat::eye(3, 3, CV_64F);

    Rodrigues(rvecs, rotM);

    double theta_x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2));
    double theta_y = atan2(-rotM.at<double>(2, 0), sqrt(rotM.at<double>(2, 1)*rotM.at<double>(2, 1) + rotM.at<double>(2, 2)*rotM.at<double>(2, 2)));
    double theta_z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0));

    // theta_x = theta_x * (180 / CV_PI);
    // theta_y = theta_y * (180 / CV_PI);
    // theta_z = theta_z * (180 / CV_PI);

    // cout << theta_x << endl;
    // cout << theta_y << endl;
    // cout << theta_z << endl << endl;

    Mat P = (rotM.t()) * tvecs;
    double distance = -P.at<double>(2, 0);  // 相机与二维码的距离

    // cout << distance << endl;
}


// @brief 清空各vector
void Energy::clearVectors()
{
    armors_rrect.clear();
    flow_strip_fan_rrect.clear();
    points2D.clear();
    points3D.clear();
}


// @brief 图像预处理,
//      1. 通道分离 + 二值化 + 膨胀腐蚀
//      2. 转换到HSV空间与mask相减 + 二值化 + 膨胀腐蚀
// @param frame 传入的图像
void Energy::initFrame(Mat &frame)
{
    #ifdef USE_SPLIT_INIT

    // 通道分离 + 二值化 + 膨胀腐蚀
    vector<Mat> channels;
    split(frame, channels);
    #ifndef DEBUG_WITHOUT_COM
    if(energyParams.stm32Data.enemy_color == OUR_BLUE)
    {   
        frame = channels.at(0) - channels.at(2);
        threshold(frame, frame, energyParams.OUR_BLUE_GRAY_BINARY, 255, CV_THRESH_BINARY);
    }
        if(energyParams.stm32Data.enemy_color == OUR_RED)
    {   
        frame = channels.at(2) - channels.at(0);
        threshold(frame, frame, energyParams.OUR_BLUE_GRAY_BINARY, 255, CV_THRESH_BINARY);
    }

    #endif

    #ifdef DEBUG_WITHOUT_COM
    #ifdef BLUFOR
    frame = channels.at(2) - channels.at(0);
    threshold(frame, frame, energyParams.OUR_BLUE_GRAY_BINARY, 255, CV_THRESH_BINARY);
    #endif

    #ifdef REDFOR
    frame = channels.at(0) - channels.at(2);
    threshold(frame, frame, energyParams.OUR_RED_GRAY_BINARY, 255, CV_THRESH_BINARY);
    #endif
    #endif
    // GaussianBlur(frame, frame, Size(3, 3), 0);

    dilate(frame, frame, energyParams.element, Point(-1, -1), 1);
    // erode(frame, frame, element, Point(-1, -1), 0);
    #ifdef SHOW_BINARY
    imshow("SHOW_BINARY", frame);
    #endif

    #endif  // USE_SPLIT_INIT
    
    //================================================================================================

    #ifdef USE_HSV_INIT

    // 转换到hsv色彩空间
    Mat dst, hsv, mask;

    cv::cvtColor(frame, dst, CV_BGR2GRAY);
    threshold(dst, dst, 70, 255, CV_THRESH_BINARY);
    
    cv::cvtColor(frame, hsv, CV_RGB2HSV);

    inRange(hsv, Scalar(0, 10, 46), Scalar(180, 60, 255), mask);
    dilate(mask, mask, energyParams.gray_element);

    dst = dst - mask;
    dilate(dst, dst, energyParams.gray_element);
    erode(dst, dst, energyParams.element);

    frame = dst;

    #ifdef SHOW_BINARY
    imshow("SHOW_BINARY", dst);
    #endif

    #endif  // USE_HSV_INIT
}


// @brief 获取当前装甲板的旋转角度
// @return 旋转方向判断成功返回true,否则返回false
bool Energy::getDirectionOfRotation()
{
    if(armor_angle_queue.size() < 3)
    {
        armor_angle_queue.push(target_armor.angle);
        return false;
    }
    else if(armor_angle_queue.size() == 3)
    {
        armor_angle_queue.pop();
        armor_angle_queue.push(target_armor.angle);
    }

    double diff_angle = armor_angle_queue.back() - armor_angle_queue.front();

    if(fabs(diff_angle) < 10 && fabs(diff_angle) > 1e-6)
    {
        angle_confidence = 0.9 * angle_confidence + diff_angle;
    }

    if(angle_confidence > 2)
    {
        rotation = CLOCKWISE;
        return true;
    }
    else if(angle_confidence < -2)
    {
        rotation = COUNTER_CLOCKWISE;
        return true;
    }
    return false;
}


// @brief 预测装甲板的打击点
// @param frame 二值化后的图像
// @return 预测成功返回true 否则返回false
bool Energy::predictTargetPoint(Mat &frame)
{
    // 如果是小能量机关模式,需要判断旋转方向
    if(energyParams.stm32Data.energy_mode == ENERGY_SMALL)
    {

        if(!predictRCenter(frame))
        {
            cout<<"Predict R failed!"<<endl;
            return false;
        }

        if(!getDirectionOfRotation())
        {
            cout<<"Predict Rotation failed"<<endl;
            return false;
        }
        cout<<" "<<endl;
        if(rotation == CLOCKWISE)
        {
            double preAngleTemp = -energyParams.small_mode_predict_angle;

            double x = target_armor.center.x - RCenter.x;
            double y = target_armor.center.y - RCenter.y;
            predict_point.x = x * cos(preAngleTemp) + y * sin(preAngleTemp) + RCenter.x;
            predict_point.y = -x * sin(preAngleTemp) + y * cos(preAngleTemp) + RCenter.y;
            
            return true;
        }
        else if(rotation == COUNTER_CLOCKWISE)
        {
            double x = target_armor.center.x - RCenter.x;
            double y = target_armor.center.y - RCenter.y;
            predict_point.x = x * cos(energyParams.small_mode_predict_angle) + y * sin(energyParams.small_mode_predict_angle) + RCenter.x;
            predict_point.y = -x * sin(energyParams.small_mode_predict_angle) + y * cos(energyParams.small_mode_predict_angle) + RCenter.y;

            return true;
        }
    }

    // 如果为大能量机关模式,需要进行拟合
    if(energyParams.stm32Data.energy_mode == ENERGY_BIG)
    {
        // 大符旋转绝对时间
        float absolute_time = ((double)getTickCount() - absolute_run_time) / getTickFrequency();
        // cout<<absolute_time<<endl;
 
        energyParams.big_mode_predict_angle = theta_func(absolute_time + energyParams.bullet_fly_time) - theta_func(absolute_time);

        if(!predictRCenter(frame))
        {
            return false;
        }

        if(!getDirectionOfRotation())
        {
            return false;
        }
        
        if(rotation == CLOCKWISE)
        {
            double preAngleTemp = -energyParams.big_mode_predict_angle;

            double x = target_armor.center.x - RCenter.x;
            double y = target_armor.center.y - RCenter.y;
            predict_point.x = x * cos(preAngleTemp) + y * sin(preAngleTemp) + RCenter.x;
            predict_point.y = -x * sin(preAngleTemp) + y * cos(preAngleTemp) + RCenter.y;
            
            return true;
        }
        else if(rotation == COUNTER_CLOCKWISE)
        {
            double x = target_armor.center.x - RCenter.x;
            double y = target_armor.center.y - RCenter.y;
            predict_point.x = x * cos(energyParams.big_mode_predict_angle) + y * sin(energyParams.big_mode_predict_angle) + RCenter.x;
            predict_point.y = -x * sin(energyParams.big_mode_predict_angle) + y * cos(energyParams.big_mode_predict_angle) + RCenter.y;

            return true;
        }
    }
    return false;
}


// @brief 寻找装甲板
// @param src 传入的图像
// @return 寻找装甲板成功返回true,否则返回false
bool Energy::findArmors(Mat &src)
{
    if(src.empty())
    {
        cout << "empty!" << endl;
        return false;
    }


    Mat src_bin = src.clone();
    if(src.type() == CV_8UC3)
    {
        cv::cvtColor(src_bin, src_bin, CV_BGR2GRAY);
    }

    vector<vector<Point>> armor_contours;
    vector<vector<Point>> armor_contours_extenal;

    // 提取所有轮廓
    findContours(src_bin, armor_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    // 只提取外轮廓
    findContours(src_bin, armor_contours_extenal, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // 去除外轮廓
    for(size_t i = 0; i < armor_contours_extenal.size(); i++)
    {
        unsigned long contour_external_size = armor_contours_extenal[i].size();
        for(size_t j = 0; j < armor_contours.size(); j++)
        {
            unsigned long contour_size = armor_contours[j].size();
            if(contour_external_size == contour_size)
            {
                swap(armor_contours[j], armor_contours[armor_contours.size() - 1]);
                armor_contours.pop_back();
                break;
            }
        }
    }

    for(auto armor_contour : armor_contours)
    {
        if(!isValidArmor(armor_contour))
        {
            continue;
        }
        // 得到初步筛选出的装甲板
        armors_rrect.emplace_back(minAreaRect(armor_contour));
    }

    // 防止为空
    if(armors_rrect.size() == 0)
    {
        cout << "armors_rrect is empty!" << endl;
        return false;
    }

    #ifdef SHOW_ALL_ARMORS
    Mat ALL_ARMORS = src_bin.clone();
    cv::cvtColor(ALL_ARMORS, ALL_ARMORS, CV_GRAY2BGR);
    for(auto armor : armors_rrect)
    {
        drawRotatedRect(ALL_ARMORS, armor, Scalar(0, 255, 20), 2);
    }
    imshow("SHOW_ALL_ARMORS", ALL_ARMORS);
    #endif

    if(!findTargetArmor(src_bin))
    {
        cout << "can not find the target armor!" << endl;
        return false;
    }

    return true;
}

// @brief 寻找最终确定的装甲板
// @param src_bin 二值化后的图像
// @return 找到最终的装甲板返回true,否则返回false
bool Energy::findTargetArmor(Mat &src_bin)
{
    vector<Point> intersection;

    for(auto armor : armors_rrect)
    {
        rotatedRectangleIntersection(armor, target_flow_strip_fan, intersection);
        
        // 可以肯定的是,必定会有四个点
        if(intersection.size() < 4)
        {   
            continue;
        }
        else
        {
            target_armor = minAreaRect(intersection);
        }
    }

    #ifdef SHOW_TARGET_ARMOR
    Mat TARGET_ARMOR = src_bin.clone();
    cv::cvtColor(TARGET_ARMOR, TARGET_ARMOR, CV_GRAY2BGR);
    drawRotatedRect(TARGET_ARMOR, target_armor, Scalar(0, 0, 255), 2);
    imshow("SHOW_TARGET_ARMOR", TARGET_ARMOR);
    #endif

    #ifdef SHOW_TARGET_ARMOR_CENTER
    Mat TARGET_ARMOR_CENTER = src_bin.clone();
    cv::cvtColor(TARGET_ARMOR_CENTER, TARGET_ARMOR_CENTER, CV_GRAY2BGR);
    circle(TARGET_ARMOR_CENTER, target_armor.center, 3, Scalar(0, 0, 255), 2);    // 装甲板中心点
    imshow("SHOW_TARGET_ARMOR_CENTER", TARGET_ARMOR_CENTER);
    #endif
    
    return true;
}


// @brief 检查装甲板是否符合要求
// @param armor_contour 装甲板轮廓
// @return 装甲板有效返回true,否则返回false
bool Energy::isValidArmor(vector<Point>&armor_contour)
{
    // 依据面积进行筛选(若显示矩形信息则在长宽后进行判断)
    #ifndef SHOW_RECT_INFO
    double cur_contour_area = contourArea(armor_contour);
    // cout << cur_contour_area << endl;
    if(cur_contour_area > energyParams.ARMOR_CONTOUR_AREA_MAX || cur_contour_area < energyParams.ARMOR_CONTOUR_AREA_MIN)
    {
        return false;
    }
    #endif

    // 依据长和宽进行筛选
    RotatedRect cur_rrect = minAreaRect(armor_contour);
    #ifdef SHOW_RECT_INFO 
    ShowRotateRectDetail(cur_rrect,src_rect_info);
    #endif

    #ifdef SHOW_RECT_INFO
    double cur_contour_area = contourArea(armor_contour);
    // cout << cur_contour_area << endl;
    if(cur_contour_area > energyParams.ARMOR_CONTOUR_AREA_MAX || cur_contour_area < energyParams.ARMOR_CONTOUR_AREA_MIN)
    {
        return false;
    }
    #endif

    Size2f cur_size = cur_rrect.size;
    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    if(length < energyParams.ARMOR_CONTOUR_LENGTH_MIN || length > energyParams.ARMOR_CONTOUR_LENGTH_MAX 
        || width < energyParams.ARMOR_CONTOUR_WIDTH_MIN || width > energyParams.ARMOR_CONTOUR_WIDTH_MAX)
    {
        return false;
    }
    // 依据长宽比进行筛选
    float ratio = length / width;
    // cout<<ratio<<endl;
    if(ratio > energyParams.ARMOR_CONTOUR_HW_RATIO_MAX || ratio < energyParams.ARMOR_CONTOUR_HW_RATIO_MIN)
    {
        return false;
    }

    return true;
}


// @brief 预测字母R坐标
// @param frame 二值化后的图像
// @return 预测成功返回true,否则返回false
bool Energy::predictRCenter(Mat &frame)
{
    #ifdef DEBUG_PREDICT_INFORMATION_BUFF
    cout<<"sample counts:" <<armor_center_points.size()<<endl;//输出样本现有数量
    #endif
    // 只拟合一次
    if(armor_center_points.size() < 50)
    {
        // 将装甲板中心点坐标存入
        armor_center_points.emplace_back(target_armor.center);
        #ifdef DEBUG_PREDICT_INFORMATION_BUFF  
        cout<<"Insufficient sample!"<<endl;//提示样本数量不足
        #endif
        return false;
    }
    else if(armor_center_points.size() == 50)
    {
        circleLeastFit(armor_center_points, RCenter);

        #ifdef SHOW_R_CENTER
        Mat R_CENTER = frame.clone();
        cv::cvtColor(R_CENTER, R_CENTER, CV_GRAY2BGR);
        circle(R_CENTER, RCenter, 5, Scalar(0, 255, 0), 2);
        // circle(R_CENTER, RCenter.point, pointsDistance(RCenter.point, target_armor.center), Scalar(0, 255, 0), 2);
        circle(R_CENTER, target_armor.center, 5, Scalar(0, 255, 0), 1);
        for(auto armor_center : armor_center_points)
        {
            circle(R_CENTER, armor_center, 1, Scalar(0, 255, 0), 1);
        }
        imshow("SHOW_R_CENTER", R_CENTER);
        #endif
    
        return true;
    }

    return false;
}


// @brief 能量机关大符运行函数
// @param 输入的时间
// @return 返回当前的运行速度
inline float Energy::spd_func(float time)
{
    return 0.785 * sin(1.884 * time) + 1.305;
}


// @brief 能量机关大符运行函数积分
// @param 输入的时间
// @return 返回当前已经运行的角度
inline float Energy::theta_func(float time)
{
    return -((0.785 / 1.884) * cos(1.884 * time)) + 1.305 * time;
}


// @brief 寻找含流动条的扇叶
// @param src 输入的图像
// @return 寻找到返回true,否则返回false
bool Energy::findFlowStripFan(Mat &src)
{
    if(src.empty())
    {
        cout << "Flow_strip_fan src empty!" << endl;
        return false;
    }

    Mat src_bin = src.clone();

    if(src.type() == CV_8UC3)
    {
        cv::cvtColor(src_bin, src_bin, CV_BGR2GRAY);
    }

    vector<vector<Point>> flow_strip_fan_contours;
    // 寻找外轮廓
    findContours(src_bin, flow_strip_fan_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    vector<RotatedRect> candidate_flow_strip_fans;
    for(auto & flow_strip_fan : flow_strip_fan_contours)
    {
        if(!isValidFlowStripFan(src_bin, flow_strip_fan))
        {
            continue;
        }
        flow_strip_fan_rrect.emplace_back(minAreaRect(flow_strip_fan));
    }

    // 判断是否为空
    if(flow_strip_fan_rrect.empty())
    {   
        return false;
    }

    // 获得最终的含流动条的扇叶
    if(flow_strip_fan_rrect.size() == 1)
    {
        target_flow_strip_fan = flow_strip_fan_rrect.at(0);
    }

    #ifdef SHOW_FLOW_STRIP_FAN
    Mat FLOW_STRIP_FAN = src_bin.clone();
    cv::cvtColor(FLOW_STRIP_FAN, FLOW_STRIP_FAN, CV_GRAY2BGR);
    // 按理说旋转矩形只有一个
    for(int i = 0; i < flow_strip_fan_rrect.size(); i++)
    {
        circle(FLOW_STRIP_FAN, flow_strip_fan_rrect[i].center, 3, Scalar(0, 0, 255), 2);    // 中心点
        drawRotatedRect(FLOW_STRIP_FAN, flow_strip_fan_rrect.at(i), Scalar(0, 255, 0), 2);
    }
    imshow("SHOW_FLOW_STRIP_FAN", FLOW_STRIP_FAN);
    #endif  // SHOW_FLOW_STRIP_FAN

    return true;
}


// @brief 判断含流动条的扇叶是否合格
// @param src_bin 二值化后的图像
// @param flow_strip_fan_contour 需要判断的轮廓
// @return 含流动条的扇叶合格返回true,否则返回false
bool Energy::isValidFlowStripFan(Mat &src_bin, vector<Point> &flow_strip_fan_contour)
{
    #ifndef SHOW_RECT_INFO 

    // 依据面积进行筛选(若显示矩形信息则在长宽后进行判断)
    double cur_contour_area = contourArea(flow_strip_fan_contour);
    if(cur_contour_area > energyParams.FLOW_STRIP_FAN_CONTOUR_AREA_MAX || cur_contour_area < energyParams.FLOW_STRIP_FAN_CONTOUR_AREA_MIN)
    {
        return false;
    }

    #endif

    // 依据长宽进行筛选
    RotatedRect cur_rect = minAreaRect(flow_strip_fan_contour);
    #ifdef SHOW_RECT_INFO 
    ShowRotateRectDetail(cur_rect,src_rect_info);
    #endif

    #ifdef SHOW_RECT_INFO 
    double cur_contour_area = contourArea(flow_strip_fan_contour);
    cout<<cur_contour_area<<endl;
    if(cur_contour_area > energyParams.FLOW_STRIP_FAN_CONTOUR_AREA_MAX || cur_contour_area < energyParams.FLOW_STRIP_FAN_CONTOUR_AREA_MIN)
    {
        return false;
    }
    #endif
    

    Size2f cur_size = cur_rect.size;
    float height = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    if(height > energyParams.FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX || height < energyParams.FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN
        || width > energyParams.FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX || width < energyParams.FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN)
    {
        return false;
    }

    // 根据长宽比进行筛选
    float ratio = height / width;
    if(ratio > energyParams.FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX || ratio < energyParams.FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN)
    {

        return false;
    }

    // 根据面积比进行筛选
    float area_ratio = cur_contour_area / cur_size.area();
    // cout<<area_ratio<<"\n\n"<<endl;
    if(area_ratio > energyParams.FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX || area_ratio < energyParams.FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN)
    {
        return false;
    }

    // 根据两侧灯柱进行筛选
    Point2f rrect_points[4];                    // 存储四个顶点
    cur_rect.points(rrect_points);
    Point2f roi_right_center, roi_left_center;  // 两侧矩形的中心点
    Rect roi_right_rect, roi_left_rect;         // 两侧的矩形区域
    Size roi_size(12, 12);                      // 矩形的大小
    int roi_right_intensity, roi_left_intensity;// 两侧的强度值

    if(fabs(width - pointsDistance(rrect_points[0], rrect_points[1])) <= 1) // 这里的意思是0,1的距离和短边长度差不多
    {
        roi_right_center = (rrect_points[0] + rrect_points[3]) / 2;
        roi_left_center = (rrect_points[1] + rrect_points[2]) / 2;

        roi_right_rect = Rect(roi_right_center, roi_size);
        roi_left_rect = Rect(roi_left_center, roi_size);
    }
    else
    {
        roi_right_center = (rrect_points[0] + rrect_points[1]) / 2;
        roi_left_center = (rrect_points[2] + rrect_points[3]) / 2;

        roi_right_rect = Rect(roi_right_center, roi_size);
        roi_left_rect = Rect(roi_left_center, roi_size);
    }

    roi_right_intensity = getRectIntensity(src_bin, roi_right_rect);
    roi_left_intensity = getRectIntensity(src_bin, roi_left_rect);

    // 是否显示两侧roi矩形区域
    #ifdef SHOW_FLOW_STRIP_FAN_TWO_ROI
    Mat FLOW_STRIP_FAN_TWO_ROI = src_bin.clone();
    rectangle(FLOW_STRIP_FAN_TWO_ROI, roi_right_rect, Scalar(255), 1);
    rectangle(FLOW_STRIP_FAN_TWO_ROI, roi_left_rect, Scalar(255), 1);
    imshow("SHOW_FLOW_STRIP_FAN_TWO_ROI", FLOW_STRIP_FAN_TWO_ROI);
    #endif  // SHOW_FLOW_STRIP_FAN_TWO_ROI

    if(roi_right_intensity > 10 && roi_left_intensity > 10)
    {
        return false;
    }

    return true;
}


// @brief 获取roi_rect区域内的强度
// @param src 二值化图像
// @param roi_rect 矩形的roi区域
// @return 返回当前roi矩形内的强度值
int Energy::getRectIntensity(Mat &src, Rect &roi_rect)
{
    Mat roi = src(roi_rect);
    return static_cast<int>(mean(roi).val[0]);
}
