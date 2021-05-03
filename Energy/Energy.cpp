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
//              4. inline Point2f XYTransform2D(Point2f X,Point2f O1,Point2f O2)
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

// @brief 绘制旋转矩形并显示相关信息 by ROM
// @param RRect 所需矩形
// @param src  输出图像
// @param color 矩形的颜色与相关数据颜色
// @param thickness 矩形线条的线宽与字体大小
void ShowRotateRectDetail(RotatedRect &RRect, Mat src, Scalar color = Scalar(0, 150, 0), int thickness = 2)
{
    //储存旋转矩形顶点数组
    Point2f Apex[4];
    RRect.points(Apex);
    const string Width = "Length :" + to_string((int)(RRect.size.width > RRect.size.height ? RRect.size.width : RRect.size.height));
    const string Height = "Width :" + to_string((int)(RRect.size.width < RRect.size.height ? RRect.size.width : RRect.size.height));
    const string Area = "Area :" + to_string((int)(RRect.size.width * RRect.size.height));
    const string H_W = "H/W :" + to_string((float)((RRect.size.width > RRect.size.height ? RRect.size.width : RRect.size.height) / (RRect.size.width < RRect.size.height ? RRect.size.width : RRect.size.height)));
    for (int j = 0; j < 4; ++j)
        line(src, Apex[j], Apex[(j + 1) % 4], color, thickness);
    putText(src, Width, Apex[0], CV_FONT_HERSHEY_SIMPLEX, 0.5, color, thickness);
    putText(src, Height, Apex[0] + Point2f(0, 15), CV_FONT_HERSHEY_SIMPLEX, 0.5, color, thickness);
    putText(src, Area, Apex[0] + Point2f(0, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, color, thickness);
    putText(src, H_W, Apex[0] + Point2f(0, 45), CV_FONT_HERSHEY_SIMPLEX, 0.5, color, thickness);
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
//@brief 检测矩形是否安全
//@return 返回检测结果
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

// @brief return distance between two points
// @param p1 first point
// @param p2 second point
// @return distance between two points
inline double pointsDistance(Point2f p1, Point2f p2)
{
    return pow(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2), 0.5);
}

//@brief    2维坐标变换
//@param X  所需变换的坐标点
//@param O1 原坐标下原点
//@param O2 所需坐标系下原点
//@return 返回变换之后的坐标点
inline Point2f XYTransform2D(Point2f X,Point2f O1,Point2f O2)
{
    Point2f direction_vector = O2 - O1; //坐标系变化方向向量    
    X = X - direction_vector ;          //新坐标等于原坐标减去方向向量
    return X;
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
    if (points.size() < 3)
    {
        return false;
    }

    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

    int N = points.size();
    int i;
    for (i = 0; i < N; i++)
    {
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
    RCenter = Point2f(center_x, center_y);

    return true;
}

//----------------------------------------------------------//
//                                                          //
//                      class Energy                        //
//                                                          //
//----------------------------------------------------------//

//@brief 根据ROI来剪切图像
inline void Energy::CutImageByROI(Mat &frame)//检测矩形是否安全或大小为0
{
    if(!makeRectSafe(image_ROI,frame.size()))
    {
        cout<<"not safe"<<endl;
        ROI_Offset = Point2i(0,0);
        return;
    }
    frame(image_ROI).copyTo(frame);
}

//@brief 根据丢失目标帧数调整ROI
inline void Energy::ROIEnlargeByMissCnt(Mat &frame)
{
    ROI_Offset = image_ROI.tl();//记录裁剪后图像坐标系偏移量

    if(miss_cnt <= 1)
    {
        return ;
    }
    else if(miss_cnt > 1 && miss_cnt <= 5)
    {
        Point2i point_offset = Point2i(0.05 * miss_cnt * image_ROI.width ,0.05 * image_ROI.height);
        image_ROI = cv::Rect(image_ROI.tl() - point_offset , image_ROI.br() + point_offset);
    }
    else//丢失超过5帧则设置为原图大小
    {
        image_ROI = cv::Rect(Point2i(0,0),frame.size());
    }
}

Energy::Energy(SerialPort &port)  : _port(port), _sentrymode(0), _basemode(0)
{

    init();
}

Energy::Energy()
{
    init();
}


//@brief 初始化一些参数
void Energy::init()
{   
    miss_cnt = 0;
    bool is_first_predict = false;
} 



// @brief 识别总入口函数
// @param oriFrame 原始图像
bool Energy::run(Mat &oriFrame)
{
    // #ifdef DEBUG_PREDICT_INFORMATION_BUFF
    // debug_cnt = clock();    // if(i>100)
    // cout<<"T1 :"<< (double)(getTickCount() - armor_center_queue_time.back()) / getTickFrequency()<<"S\n"<<endl;
    // cout << "Time:" << (int)debug_cnt << endl;
    // #endif

    // cout<<"miss_cnt = "<<miss_cnt<<endl;
    // i++;
    if (oriFrame.empty())
    {
        cout << "oriFrame is empty!" << endl;
        return false;
    }

    Mat frame = oriFrame.clone();

    #ifdef SHOW_ORIGINAL
    imshow("SHOW_ORIGINAL", frame);
    #endif//SHOW_ORIGINAL


    #ifdef SHOW_RECT_INFO
    frame.copyTo(src_rect_info);//负责源图以供绘制相关信息
    #endif // SHOW_RECT_INFO

    #ifdef SHOW_PREDICT_ARMOR
    Mat predict_armor;
    frame.copyTo(predict_armor);//负责源图以供绘制相关信息
    #endif // SHOW_PREDICT_ARMOR

    clearVectors();

    #ifdef SHOW_ROI//是否显示ROI
    Mat show_roi = frame.clone();
    rectangle(show_roi,image_ROI,Scalar(200,100,0),1);
    circle(show_roi, RCenter + (Point2f)ROI_Offset, 5, Scalar(0, 255, 0), 2);
    imshow("ROI",show_roi);
    #endif//SHOW_ROI
    ROIEnlargeByMissCnt(frame);     //根据丢失目标帧数来调整ROI大小
    #ifdef ENABLE_ROI_CUT //是否根据ROI裁剪图像  
    CutImageByROI(frame);           //根据ROI剪切图像
    #endif//ENABLE_ROI_CUT
    initFrame(frame);               //对图像进行预处理
    // if(i>100)
    // cout<<"T2 :"<< (double)(getTickCount() - armor_center_queue_time.back()) / getTickFrequency()<<"S\n"<<endl;
    if (!findFlowStripFan(frame)) // 寻找含流动条的装甲板
    {
        #ifdef SHOW_RECT_INFO
        imshow("SHOW_CANDIDATE",src_rect_info);//未检测到流动条,return前显示图像
        #endif // SHOW_RECT_INFO
        miss_cnt++;
        waitKey(1);
        return false; // 如果没找到
    }



    if (!findArmors(frame)) {// 寻找装甲板
        #ifdef SHOW_RECT_INFO
        imshow("SHOW_CANDIDATE",src_rect_info);////未检测到装甲板,return前显示图像
        #endif // SHOW_RECT_INFO       
        miss_cnt++;//目标丢失帧数加1
        waitKey(1);
        return false; // 如果没找到
    }

    // #ifdef SHOW_RECT_INFO
    // imshow("SHOW_CANDIDATE",src_rect_info);//显示图像
    // #endif // SHOW_RECT_INFO
    

    // getPoints2D();
    // getPoints3D();
    // solveXYZ();
    predictTargetPoint(frame);

    if(pointsDistance(RCenter, target_armor.center) > energyParams.max_arm_length){
        waitKey(1);
        return false;
    }

    Final_Hit_Calc();


    #ifdef SHOW_PREDICT_POINT
    Mat PREDICT_POINT = frame.clone();
    cv::cvtColor(PREDICT_POINT, PREDICT_POINT, CV_GRAY2BGR);

    #ifdef CIRCLE_FIT
    if (armor_center_points.size() == 50)
    {
        circle(PREDICT_POINT, predict_point, 5, Scalar(0, 255, 0), 2);
        line(PREDICT_POINT, predict_point, target_armor.center, Scalar(0, 0, 255), 2);
        circle(PREDICT_POINT, RCenter, pointsDistance(RCenter, target_armor.center), Scalar(0, 255, 0), 1);
    }
    #endif//CIRCLE_FIT

    #ifdef FIND_R_STRUCT_IN_ROI
    circle(PREDICT_POINT, predict_point, 5, Scalar(0, 255, 0), 2);
    line(PREDICT_POINT, predict_point, target_armor.center, Scalar(0, 0, 255), 2);
    circle(PREDICT_POINT, RCenter, pointsDistance(RCenter, target_armor.center), Scalar(0, 255, 0), 1);
    // drawRotatedRect(PREDICT_POINT,predict_hit_armor,Scalar(255,0,0));
    #endif

    #ifdef SHOW_PREDICT_ARMOR
    drawRotatedRect(predict_armor,predict_hit_armor,Scalar(0,255,0),2);
    imshow("predict_armor",predict_armor);
    #endif // SHOW_PREDICT_ARMOR

    imshow("SHOW_PREDICT_POINT", PREDICT_POINT);
    // #ifdef DEBUG_PREDICT_INFORMATION_BUFF
    // cout << "\n\n"
    //      << endl;
    // #endif // DEBUG_PREDICT_INFORMATION_BUFF
    #endif // SHOW_PREDICT_POINT

    waitKey(1);
    return true;
}

// @brief 获得pnp平面二维坐标
void Energy::getPoints2D()
{
    Point2f points[4]; // TODO:

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
    double theta_y = atan2(-rotM.at<double>(2, 0), sqrt(rotM.at<double>(2, 1) * rotM.at<double>(2, 1) + rotM.at<double>(2, 2) * rotM.at<double>(2, 2)));
    double theta_z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0));

    // theta_x = theta_x * (180 / CV_PI);
    // theta_y = theta_y * (180 / CV_PI);
    // theta_z = theta_z * (180 / CV_PI);

    // cout << theta_x << endl;
    // cout << theta_y << endl;
    // cout << theta_z << endl << endl;

    Mat P = (rotM.t()) * tvecs;
    double distance = -P.at<double>(2, 0); // 相机与二维码的距离

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
// @param frame 传入的图像
void Energy::initFrame(Mat &frame)
{
    // 通道分离 + 二值化 + 膨胀腐蚀
    vector<Mat> channels;
    split(frame, channels);

    // if (energyParams.stm32Data.enemy_color == OUR_BLUE)
    // {
    //     frame = channels.at(0) - channels.at(2);
    //     threshold(frame, frame, energyParams.OUR_BLUE_GRAY_BINARY, 255, CV_THRESH_BINARY);
    // }
    // if (energyParams.stm32Data.enemy_color == OUR_RED)
    // {
    //     frame = channels.at(2) - channels.at(0);
    //     threshold(frame, frame, energyParams.OUR_BLUE_GRAY_BINARY, 255, CV_THRESH_BINARY);
    // }

    #ifdef E_RED // Detect Red Armor Plate
    frame = channels.at(2) - channels.at(0);
    threshold(frame, frame, energyParams.DETECT_RED_GRAY_BINARY, 255, CV_THRESH_BINARY);
    #endif // E_BLUE


    #ifdef E_BLUE // Detect Blue Armor Plate
    frame = channels.at(0) - channels.at(2);
    threshold(frame, frame, energyParams.DETECT_BLUE_GRAY_BINARY, 255, CV_THRESH_BINARY);
    #endif // E_RED

    // GaussianBlur(frame, frame, Size(3, 3), 0);   // 高斯模糊

    dilate(frame, frame, energyParams.element, Point(-1, -1), 1);   // 膨胀
    // erode(frame, frame, element, Point(-1, -1), 0);  // 腐蚀

    #ifdef SHOW_BINARY
    imshow("SHOW_BINARY", frame);
    #endif //SHOW_BINARY
}

// @brief 获取当前装甲板的旋转角度
// @return 旋转方向判断成功返回true,否则返回false
bool Energy::getDirectionOfRotation()
{
    if (armor_angle_queue.size() < 3)
    {
        armor_angle_queue.push(target_armor.angle);
        return false;
    }
    else if (armor_angle_queue.size() == 3)
    {
        armor_angle_queue.pop();
        armor_angle_queue.push(target_armor.angle);
    }

    double diff_angle = armor_angle_queue.back() - armor_angle_queue.front();

    if (fabs(diff_angle) < 10 && fabs(diff_angle) > 1e-6)
    {
        angle_confidence = 0.9 * angle_confidence + diff_angle;
    }

    if (angle_confidence > 2)
    {
        rotation = CLOCKWISE;
        return true;
    }
    else if (angle_confidence < -2)
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
    if (energyParams.stm32Data.energy_mode == ENERGY_SMALL)
    {
        if (!predictRCenter(frame))
        {
            // cout << "Predict R failed!" << endl;
            return false;
        }
        // cout << "Predict R Succeed!" << endl;

        if (!getDirectionOfRotation())
        {
            //cout << "Predict Rotation failed" << endl;
            return false;
        }

        if (rotation == CLOCKWISE)
        {
            double preAngleTemp = -energyParams.small_mode_predict_angle;

            double x = target_armor.center.x - RCenter.x;
            double y = target_armor.center.y - RCenter.y;
            predict_point.x = x * cos(preAngleTemp) + y * sin(preAngleTemp) + RCenter.x;
            predict_point.y = -x * sin(preAngleTemp) + y * cos(preAngleTemp) + RCenter.y;

            return true;
        }
        else if (rotation == COUNTER_CLOCKWISE)
        {
            double x = target_armor.center.x - RCenter.x;
            double y = target_armor.center.y - RCenter.y;
            predict_point.x = x * cos(energyParams.small_mode_predict_angle) + y * sin(energyParams.small_mode_predict_angle) + RCenter.x;
            predict_point.y = -x * sin(energyParams.small_mode_predict_angle) + y * cos(energyParams.small_mode_predict_angle) + RCenter.y;

            return true;
        }
    }

    // 如果为大能量机关模式,需要进行拟合
    if (energyParams.stm32Data.energy_mode == ENERGY_BIG)
    {
        #ifdef ABSOLUTE_TIME_INTERGRATION 

        // 大符旋转绝对时间
        float absolute_time = ((double)getTickCount() - absolute_run_time) / getTickFrequency();
        // cout<<absolute_time<<endl;

        energyParams.big_mode_predict_angle = theta_func(absolute_time + energyParams.bullet_fly_time) - theta_func(absolute_time);

        #endif//ABSOLUTE_TIME_INTERGRATION

        if (!predictRCenter(frame))
        {
            return false;
        }

        if (!getDirectionOfRotation())
        {
            return false;
        }

        #ifdef USING_KALMAN_FILTER

        //如果队列元素不足
        if(armor_center_in_centerR_cord.size() <= 1)//FIXME::可以增加一种情况以进行优化
        {
            Point2f armor_center_transform_tmp = XYTransform2D(target_armor.center,Point2f(0,0),RCenter);//将坐标点变换到圆心坐标系下
            armor_center_in_centerR_cord.push(armor_center_transform_tmp);
            armor_center_queue_time.push(getTickCount());
            return false ; 
        }
        else if(armor_center_in_centerR_cord.size() == 2)
        {
            Point2f armor_center_transform_tmp = XYTransform2D(target_armor.center,Point2f(0,0),RCenter);//将坐标点变换到圆心坐标系下
            armor_center_in_centerR_cord.pop();                             //弹出首元素
            armor_center_in_centerR_cord.push(armor_center_transform_tmp);  //压入最新装甲板坐标

            armor_center_queue_time.pop();                                  //弹出首元素
            armor_center_queue_time.push( getTickCount());                  //压入最新装甲板坐标时间

            //计算平均悬臂长度
            double average_R_length = (sqrt(pow((armor_center_in_centerR_cord.back()).x,2) + pow((armor_center_in_centerR_cord.back()).y,2)) + 
                                        sqrt(pow((armor_center_in_centerR_cord.front()).x,2) + pow((armor_center_in_centerR_cord.front()).y,2)) ) / 2 ; 

            //利用反三角函数与余弦公式计算角度增量 delta_theta = acos(1 - c^2/(2*R^2))
            double delta_theta = acos(1 - ( pow( pointsDistance(armor_center_in_centerR_cord.back(),armor_center_in_centerR_cord.front()) , 2) / 
                                                                                                            (2 * pow(average_R_length,2))));
            // cout<<"average_R_length = "<<average_R_length<<endl;
            // cout<<"delta_theta = "<< delta_theta <<" rad "<<endl;

            //计算帧间时间增量
            double delta_time = (double)(armor_center_queue_time.back() - armor_center_queue_time.front()) / getTickFrequency() ;
            // double delta_time = 1 / 30.0;


            if(delta_theta > 0.2 || delta_time > 0.1){//若delta_theta过大或delta_time过大则认为是装甲板发生切换,不进行预测
                cout<<"delta_theta or time too big!!"<<endl;
                return false;
            }
            
            //计算角速度
            double measure_omega =  delta_theta / delta_time ;



            Mat prediction = kalmanfilter.KF.predict();                     //获取卡尔曼滤波预测值
            Mat status = Mat::zeros(2,1,CV_32F);                            //设置状态矩阵


            //计算角速度增量
            if(armor_center_queue_omega.size() <= 1)        //若角速度队列不足,不进行角速度计算
            {
                armor_center_queue_omega.push(prediction.at<float>(0));
                // armor_center_queue_omega.push(measure_omega);
                return false;
            }

            else if(armor_center_in_centerR_cord.size() == 2)
            {
                armor_center_queue_omega.pop();                 //弹出首元素
                armor_center_queue_omega.push(prediction.at<float>(0));   //压入最新角速度
                // armor_center_queue_omega.push(measure_omega);
            }

            //计算角速度增量

            double delta_omega = armor_center_queue_omega.back() - armor_center_queue_omega.front(); 
            double measure_a_rad =(float)(delta_omega / delta_time) ;
            // cout<<"measured radian accelration : "<<measure_a_rad<<"rad / (s^2)"<<endl;
            if(fabs(measure_a_rad) > 3){         //若加速度过大则认为是数据出错
                cout<<"acceleration too big : "<<measure_a_rad<<endl;
                measure_a_rad = (fabs(measure_a_rad) / measure_a_rad) * 2;
                // return false;
            }

            status.at<float>(0,0) = measure_omega;
            status.at<float>(0,1) = measure_a_rad;
            // cout<<measurement<<endl;
            kalmanfilter.KF.correct(status);                           //更新测量矩阵



            energyParams.big_mode_predict_angle = prediction.at<float>(0,0) ;    //设置偏移角度
            // cout<<armor_center_in_centerR_cord.back()<<endl;
            // cout<<"delta_time :"<<delta_time<<"s"<<endl;

            // cout<<"Vp - Vm : "<<(float)(prediction.at<float>(0,1) ) - (float)(measure_omega ) <<endl;
            // cout<<"predict degree:"<<(float)(prediction.at<float>(0,0) ) <<"degree"<<endl;
            // cout<<"measure speed:"<<(float)(measure_omega ) <<"rad / s"<<endl;
            // cout<<"predict speed:"<<(float)(prediction.at<float>(0,1) ) <<"rad / s"<<endl;
            // cout<<"measure radian accelration:"<< measure_a_rad <<"rad / (s^2)"<<endl;
            // cout<<"predict radian accelration:"<<(float)(prediction.at<float>(0,2) ) <<"rad / (s^2)"<<endl;
            // cout<<"predict speed:"<<(float)(prediction.at<float>(0,1) ) * 60 / (2 * CV_PI) <<"RPM"<<endl;
            // cout<<"measure speed:" << (int)(delta_theta / delta_time) <<"rad / s"<<endl;
            // cout<<endl;





        }

        #endif//USING_KALMAN_FILTER


        if (rotation == CLOCKWISE)
        {
            double preAngleTemp = -energyParams.big_mode_predict_angle;

            double x = target_armor.center.x - RCenter.x;
            double y = target_armor.center.y - RCenter.y;
            predict_point.x = x * cos(preAngleTemp) + y * sin(preAngleTemp) + RCenter.x;
            predict_point.y = -x * sin(preAngleTemp) + y * cos(preAngleTemp) + RCenter.y;

            return true;
        }
        else if (rotation == COUNTER_CLOCKWISE)
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
    if (src.empty())
    {
        cout << "empty!" << endl;
        return false;
    }

    Mat src_bin = src.clone();
    if (src.type() == CV_8UC3)
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
    for (size_t i = 0; i < armor_contours_extenal.size(); i++)
    {
        unsigned long contour_external_size = armor_contours_extenal[i].size();
        for (size_t j = 0; j < armor_contours.size(); j++)
        {
            unsigned long contour_size = armor_contours[j].size();
            if (contour_external_size == contour_size)
            {
                swap(armor_contours[j], armor_contours[armor_contours.size() - 1]);
                armor_contours.pop_back();
                break;
            }
        }
    }

    for (auto armor_contour : armor_contours)
    {
        if (!isValidArmor(armor_contour))
        {
            continue;
        }
        // 得到初步筛选出的装甲板
        armors_rrect.emplace_back(minAreaRect(armor_contour));
    }

    // 防止为空
    if (armors_rrect.size() == 0)
    {
        cout << "armors_rrect is empty!" << endl;
        return false;
    }

    #ifdef SHOW_ALL_ARMORS
    Mat ALL_ARMORS = src_bin.clone();
    cv::cvtColor(ALL_ARMORS, ALL_ARMORS, CV_GRAY2BGR);
    for (auto armor : armors_rrect)
    {
        drawRotatedRect(ALL_ARMORS, armor, Scalar(0, 255, 20), 2);
    }
    imshow("SHOW_ALL_ARMORS", ALL_ARMORS);
    #endif // SHOW_ALL_ARMORS

    if (!findTargetArmor(src_bin))
    {
        //cout << "can not find the target armor!" << endl;
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

    for (auto armor : armors_rrect)
    {
        rotatedRectangleIntersection(armor, target_flow_strip_fan, intersection);

        // 可以肯定的是,必定会有四个点
        if (intersection.size() < 4)
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
    #endif // SHOW_TARGET_ARMOR

    #ifdef SHOW_TARGET_ARMOR_CENTER
    Mat TARGET_ARMOR_CENTER = src_bin.clone();
    cv::cvtColor(TARGET_ARMOR_CENTER, TARGET_ARMOR_CENTER, CV_GRAY2BGR);
    circle(TARGET_ARMOR_CENTER, target_armor.center, 3, Scalar(0, 0, 255), 2); // 装甲板中心点
    imshow("SHOW_TARGET_ARMOR_CENTER", TARGET_ARMOR_CENTER);
    #endif // SHOW_TARGET_ARMOR_CENTER

    return true;
}

// @brief 检查装甲板是否符合要求
// @param armor_contour 装甲板轮廓
// @return 装甲板有效返回true,否则返回false
bool Energy::isValidArmor(vector<Point> &armor_contour)
{
    // 依据面积进行筛选
    double cur_contour_area = contourArea(armor_contour);
    // cout <<"cur_contour_area : " << cur_contour_area << endl;
    if (cur_contour_area > energyParams.ARMOR_CONTOUR_AREA_MAX || cur_contour_area < energyParams.ARMOR_CONTOUR_AREA_MIN)
    {
        return false;
    }


    RotatedRect cur_rrect = minAreaRect(armor_contour);

    #ifdef SHOW_RECT_INFO_ARMOR
    ShowRotateRectDetail(cur_rrect, src_rect_info);////绘制该矩形信息
    #endif // SHOW_RECT_INFO_ARMOR



    // 依据长和宽进行筛选
    Size2f cur_size = cur_rrect.size;
    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    if (length < energyParams.ARMOR_CONTOUR_LENGTH_MIN || length > energyParams.ARMOR_CONTOUR_LENGTH_MAX || width < energyParams.ARMOR_CONTOUR_WIDTH_MIN || width > energyParams.ARMOR_CONTOUR_WIDTH_MAX)
    {
        return false;
    }

    // 依据长宽比进行筛选
    float ratio = length / width;
    // cout<<ratio<<endl;
    if (ratio > energyParams.ARMOR_CONTOUR_HW_RATIO_MAX || ratio < energyParams.ARMOR_CONTOUR_HW_RATIO_MIN)
    {
        return false;
    }
    return true;
}

//@brief 寻找中心R的ROI
bool Energy::findCenterRROI() 
{
    float length = target_armor.size.height > target_armor.size.width ?
                   target_armor.size.height : target_armor.size.width;

    Point2f p2p(target_flow_strip_fan.center.x - target_armor.center.x,
                target_flow_strip_fan.center.y - target_armor.center.y);
    p2p = p2p / pointsDistance(target_flow_strip_fan.center, target_armor.center);//单位化

    //设置圆心的ROI
    center_ROI = cv::RotatedRect(cv::Point2f(target_flow_strip_fan.center + p2p * length * 2.3),
                                 Size2f(length * 1.9, length * 1.9), -90);




    return true;
}
//@brief 寻找中心R图案
// @param frame 二值化后的图像
// @return 寻找成功返回true,否则返回false
bool Energy::findCenterR(Mat &src_bin) {
    if (src_bin.empty()) {
        return false;
    }
    std::vector<vector<Point> > center_R_contours;
    findContours(src_bin, center_R_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    for (auto &center_R_contour : center_R_contours) {
        if (!isValidCenterRContour(center_R_contour)) 
        {
            continue;
        }
        centerR = cv::minAreaRect(center_R_contour);
        // cout<<"Center R found!"<<endl;
        return true;
    }
    // cout<<"Center R missing!"<<endl;
    return false;

}

bool Energy::isValidCenterRContour(const vector<cv::Point> &center_R_contour) {
    //计算面积
    double cur_contour_area = contourArea(center_R_contour);
    if (cur_contour_area > energyParams.CENTER_R_CONTOUR_AREA_MAX ||
        cur_contour_area < energyParams.CENTER_R_CONTOUR_AREA_MIN) 
    {
        // cout<<"area fail :"<<cur_contour_area<<endl;
        return false;
        //选区面积大小不合适
    }
    // cout<<"area success :"<<cur_contour_area<<endl;

    RotatedRect cur_rect = minAreaRect(center_R_contour);
    Size2f cur_size = cur_rect.size;
    std::vector<cv::Point2f> intersection;

    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;//将矩形的长边设置为长
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;//将矩形的短边设置为宽
    if (length < energyParams.CENTER_R_CONTOUR_LENGTH_MIN || width < energyParams.CENTER_R_CONTOUR_WIDTH_MIN
        || length > energyParams.CENTER_R_CONTOUR_LENGTH_MAX ||width > energyParams.CENTER_R_CONTOUR_WIDTH_MAX) {
    // cout<<"length or width fail."<<endl;
    // cout << "length: " << length << '\t' << "width: " << width << '\t' << cur_rect.center << endl;
    return false;
    //矩形边长不合适
    }

    //计算矩形长宽比
    float length_width_ratio = length / width;
    if (length_width_ratio > energyParams.CENTER_R_CONTOUR_HW_RATIO_MAX ||
        length_width_ratio < energyParams.CENTER_R_CONTOUR_HW_RATIO_MIN){
    // cout<<"length width ratio fail."<<endl;
    // cout << "HW: " << length_width_ratio << '\t' << cur_rect.center << endl;
    return false;
    //长宽比不合适
    }

    if (cur_contour_area / cur_size.area() < energyParams.CENTER_R_CONTOUR_AREA_RATIO_MIN) {

    //cout << "area ratio fail: " << cur_contour_area / cur_size.area()  << endl;
    return false;//轮廓对矩形的面积占有率不合适
    }

    if (rotatedRectangleIntersection(cur_rect, center_ROI, intersection) == 0) {
        return false;
    } 
    else if (contourArea(intersection) < energyParams.CENTER_R_CONTOUR_INTERSETION_AREA_MIN) {
        // cout << "R intersection: " << contourArea(intersection) << '\t' << cur_rect.center << endl;
        return false;
    }
    // cout<<"Center R right!"<<endl;
    // cout << "R intersection: " << contourArea(intersection) << '\t' << cur_rect.center << endl;
    return true;
}

// @brief 预测字母R坐标
// @param frame 二值化后的图像
// @return 预测成功返回true,否则返回false
bool Energy::predictRCenter(Mat &frame)
{
    #ifdef CIRCLE_FIT
    #ifdef DEBUG_PREDICT_INFORMATION_BUFF
        cout << "sample counts:" << armor_center_points.size() << endl; //输出样本现有数量
    #endif//DEBUG_PREDICT_INFORMATION_BUFF
    // 只拟合一次
    if (armor_center_points.size() < 50)
    {
        // 将装甲板中心点坐标存入
        armor_center_points.emplace_back(target_armor.center);
        #ifdef DEBUG_PREDICT_INFORMATION_BUFF
                cout << "Insufficient sample!" << endl; //提示样本数量不足
        #endif//DEBUG_PREDICT_INFORMATION_BUFF
        return false;
    }
    else if (armor_center_points.size() == 50)
    {
        circleLeastFit(armor_center_points, RCenter);

        //FIX ME!
        //DEBUG 2021.04.16 A Temporary solution to R Center Finding Problem

        armor_center_points.erase(armor_center_points.begin());//remove the first element
        armor_center_points.emplace_back(target_armor.center);//emplace

        //DEBUG 2021.04.16 A Temporary solution to R Center Finding Problem

        #ifdef SHOW_R_CENTER
        Mat R_CENTER = frame.clone();
        cv::cvtColor(R_CENTER, R_CENTER, CV_GRAY2BGR);
        circle(R_CENTER, RCenter, 5, Scalar(0, 255, 0), 2);
        // circle(R_CENTER, RCenter.point, pointsDistance(RCenter.point, target_armor.center), Scalar(0, 255, 0), 2);
        circle(R_CENTER, target_armor.center, 5, Scalar(0, 255, 0), 1);
        for (auto armor_center : armor_center_points)
        {
            circle(R_CENTER, armor_center, 1, Scalar(0, 255, 0), 1);
        }
        imshow("SHOW_R_CENTER", R_CENTER);
        #endif // SHOW_R_CENTER

    }
    #endif//CIRCLE_FIT

    #ifdef FIND_R_STRUCT_IN_ROI
    findCenterRROI();               //寻找中心ROI区域
    if(!findCenterR(frame))              //如果未找到中心R区域
    {
        return false;
    }

    float target_length =
            target_armor.size.height > target_armor.size.width ? target_armor.size.height : target_armor.size.width;
    RCenter = centerR.center;
    // RCenter.y += target_length / 7.5;

    //设置全图的ROI
    // cout<<target_length<<endl;
    image_ROI =  cv::Rect(RCenter - Point2f(target_length * 5,target_length * 5) + (Point2f)ROI_Offset,
                                Size2f(target_length * 10, target_length * 10));

    //设置丢帧为0
    miss_cnt = 0;
    
    #ifdef SHOW_R_CENTER
    Mat R_CENTER = frame.clone();
    cv::cvtColor(R_CENTER, R_CENTER, CV_GRAY2BGR);
    drawRotatedRect(R_CENTER,centerR,Scalar(100,100,250),2);//绘制R
    drawRotatedRect(R_CENTER,center_ROI,Scalar(100,100,200),2);//绘制ROI--CenterR
    circle(R_CENTER, RCenter, 5, Scalar(0, 255, 0), 1);//绘制实际圆心



    imshow("SHOW_R_CENTER", R_CENTER);

    #endif //SHOW_R_CENTER

    #endif //FIND_R_STRUCT_IN_ROI

    return true;
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
    if (src.empty())
    {
        cout << "Flow_strip_fan src empty!" << endl;
        return false;
    }
    Mat src_bin = src.clone();

    if (src.type() == CV_8UC3)
    {
        cv::cvtColor(src_bin, src_bin, CV_BGR2GRAY);
    }

    vector<vector<Point>> flow_strip_fan_contours;
    // 寻找外轮廓
    findContours(src_bin, flow_strip_fan_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    vector<RotatedRect> candidate_flow_strip_fans;
    for (auto &flow_strip_fan : flow_strip_fan_contours)
    {
        if (!isValidFlowStripFan(src_bin, flow_strip_fan))
        {
            continue;
        }
        flow_strip_fan_rrect.emplace_back(minAreaRect(flow_strip_fan));
    }
    // 判断是否为空
    if (flow_strip_fan_rrect.empty())
    {
        return false;
    }

    // 获得最终的含流动条的扇叶
    if (flow_strip_fan_rrect.size() == 1)
    {
        target_flow_strip_fan = flow_strip_fan_rrect.at(0);
    }
    #ifdef SHOW_FLOW_STRIP_FAN
    Mat FLOW_STRIP_FAN = src_bin.clone();
    cv::cvtColor(FLOW_STRIP_FAN, FLOW_STRIP_FAN, CV_GRAY2BGR);
    // 按理说旋转矩形只有一个
    for (int i = 0; i < flow_strip_fan_rrect.size(); i++)
    {
        circle(FLOW_STRIP_FAN, flow_strip_fan_rrect[i].center, 3, Scalar(0, 0, 255), 2); // 中心点
        drawRotatedRect(FLOW_STRIP_FAN, flow_strip_fan_rrect.at(i), Scalar(0, 255, 0), 2);
    }
    imshow("SHOW_FLOW_STRIP_FAN", FLOW_STRIP_FAN);
    #endif // SHOW_FLOW_STRIP_FAN
    return true;
}

// @brief 判断含流动条的扇叶是否合格
// @param src_bin 二值化后的图像
// @param flow_strip_fan_contour 需要判断的轮廓
// @return 含流动条的扇叶合格返回true,否则返回false
bool Energy::isValidFlowStripFan(Mat &src_bin, vector<Point> &flow_strip_fan_contour)
{

    // 依据面积进行筛选
    double cur_contour_area = contourArea(flow_strip_fan_contour);
    if (cur_contour_area > energyParams.FLOW_STRIP_FAN_CONTOUR_AREA_MAX || cur_contour_area < energyParams.FLOW_STRIP_FAN_CONTOUR_AREA_MIN)
    {
    // cout<<"flow strip contour area failed :"<< cur_contour_area <<endl;
        return false;
    }
    // cout<<"flow strip contour area success :"<< cur_contour_area <<endl;


    // 依据长宽进行筛选
    RotatedRect cur_rect = minAreaRect(flow_strip_fan_contour);

    #ifdef SHOW_RECT_INFO_FLOW_STRIP
    ShowRotateRectDetail(cur_rect, src_rect_info,Scalar(100,150,0));//绘制该矩形信息
    #endif // SHOW_RECT_INFO_FLOW_STRIP


    Size2f cur_size = cur_rect.size;
    float height = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    if (height > energyParams.FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX || height < energyParams.FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN || width > energyParams.FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX || width < energyParams.FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN)
    {
        return false;
    }

    // 根据长宽比进行筛选
    float ratio = height / width;
    if (ratio > energyParams.FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX || ratio < energyParams.FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN)
    {

        return false;
    }

    // 根据面积比进行筛选
    float area_ratio = cur_contour_area / cur_size.area();
    if (area_ratio > energyParams.FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX || area_ratio < energyParams.FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN)
    {
        // cout<<"area_ratio failed !: "<<area_ratio<<"\n\n"<<endl;
        return false;
    }
    // cout<<"flow strip area_ratio success !: "<<area_ratio<<"\n\n"<<endl;

    // 根据两侧灯柱进行筛选
    Point2f rrect_points[4]; // 存储四个顶点
    cur_rect.points(rrect_points);
    Point2f roi_right_center, roi_left_center;   // 两侧矩形的中心点
    Rect roi_right_rect, roi_left_rect;          // 两侧的矩形区域
    Size roi_size(12, 12);                       // 矩形的大小
    int roi_right_intensity, roi_left_intensity; // 两侧的强度值
    if (fabs(width - pointsDistance(rrect_points[0], rrect_points[1])) <= 1) // 这里的意思是0,1的距离和短边长度差不多
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
    if( !makeRectSafe(roi_right_rect,src_bin.size()) || !makeRectSafe(roi_left_rect,src_bin.size()) )
        return false;
    roi_right_intensity = getRectIntensity(src_bin, roi_right_rect);
    roi_left_intensity = getRectIntensity(src_bin, roi_left_rect);
    // 是否显示两侧roi矩形区域
    #ifdef SHOW_FLOW_STRIP_FAN_TWO_ROI
    Mat FLOW_STRIP_FAN_TWO_ROI = src_bin.clone();
    rectangle(FLOW_STRIP_FAN_TWO_ROI, roi_right_rect, Scalar(255), 1);
    rectangle(FLOW_STRIP_FAN_TWO_ROI, roi_left_rect, Scalar(255), 1);
    imshow("SHOW_FLOW_STRIP_FAN_TWO_ROI", FLOW_STRIP_FAN_TWO_ROI);
    #endif // SHOW_FLOW_STRIP_FAN_TWO_ROI

    if (roi_right_intensity > 10 && roi_left_intensity > 10)
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

void Energy::Final_Hit_Calc()
{
    double preAngleTemp;        //DEGREE

    if (energyParams.stm32Data.energy_mode == ENERGY_BIG){


        if (rotation == CLOCKWISE){
            preAngleTemp  = -energyParams.big_mode_predict_angle * 180 / CV_PI;
        }
        else if (rotation == COUNTER_CLOCKWISE){
            preAngleTemp  =  energyParams.big_mode_predict_angle * 180 / CV_PI;
        }
    }

    if (energyParams.stm32Data.energy_mode == ENERGY_SMALL){


        if (rotation == CLOCKWISE){
            preAngleTemp  = -energyParams.small_mode_predict_angle * 180 / CV_PI;
        }
        else if (rotation == COUNTER_CLOCKWISE){
            preAngleTemp  =  energyParams.small_mode_predict_angle * 180 / CV_PI;
        }
    }
    predict_hit_point = predict_point;
    predict_hit_armor = cv::RotatedRect(predict_hit_point + (Point2f)ROI_Offset,target_armor.size,target_armor.angle - preAngleTemp);
    // cout<<"angle:"<<preAngleTemp<<endl;
}
