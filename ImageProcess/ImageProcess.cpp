//----------------------------------------------------------
//
// FileName: ImageProcess.cpp
// Author: 周俊平;刘上;赵梓合;顾昊
// Version: 1.0.0_x64Linux
// Date: 2021.03.19
// Description: 线程处理函数,包含生产者和消费者
// Function List:
//
//
//----------------------------------------------------------

#include "./ImageProcess.h"

const string source_location = "/home/rangeronmars/Desktop/video/sample.avi";
const string file_path = "./File/calib_no_4_1280.yml";


#ifdef USE_LOCAL_VIDEO
VideoCapture cap(source_location);
#endif // USE_LOCAL_VIDEO

extern atomic_int64_t vision_mode;

#ifdef USE_DAHENG_CAMERA
DaHengCamera DaHeng;
bool is_first_loop = true;
#endif // USE_DAHENG_CAMERA

#ifdef USE_USB_CAMERA
VideoCapture cap(0);
#endif // USE_USB_CAMERA

// 存储图像的结构体
typedef struct
{
    cv::Mat img;
} ImageData;

#define IMG_BUFFER 5         // 线程间相机采集最高超过处理的帧数
ImageData Image[IMG_BUFFER]; // 存储图像的缓冲区

static volatile unsigned int proIdx = 0;  // 生产ID
static volatile unsigned int consIdx = 0; // 消费ID

Mat oriFrame;  // 获取到的原始图像

// @brief ImageProcess构造函数
ImageProcess::ImageProcess(SerialPort &port) : _port(port), _sentrymode(0), _basemode(0)
{
}

// @brief ImageProcess构造函数
ImageProcess::ImageProcess()
{
}



/**
 * @brief 射击补偿函数,包括固定补偿及距离补偿,角度单位均为角度制
 * @param distance 目标距离
 * @param angle_x  Yaw轴角引用(角度制)
 * @param angle_y  Pitch轴角引用(角度制)
*/
void ImageProcess::shootingAngleCompensate(double &distance,double &angle_x,double &angle_y)
{
    double angle_x_compensate_static;//Yaw轴角度固定补偿
    double angle_y_compensate_static;//Pitch轴角度固定补偿
    double angle_x_compensate_dynamic;//Yaw轴角度动态补偿
    double angle_y_compensate_dynamic;//Pitch轴角度动态补偿
    
    angle_x_compensate_static = - 0.6;  //右侧为正方向 
    angle_y_compensate_static = 0.7;    //上方为正方向
    angle_x_compensate_dynamic = 0;
    angle_y_compensate_dynamic = -((-1.138e4) * pow(distance,-1.934) + 2.425);

    angle_y += angle_y_compensate_dynamic + angle_y_compensate_static;
    angle_x += angle_x_compensate_dynamic + angle_x_compensate_static;
}


//@brief 装甲板卡尔曼预测
//@param present_armor  检测到的装甲板
//@param predict_armor  预测的装甲板
bool ImageProcess::advancedPredictForArmorDetect(RotatedRect &present_armor,RotatedRect &predict_armor)
{
    //如果队列元素不足
    if(armor_queue.size() <= 1)//FIXME::可以增加一种情况以进行优化
    {
        armor_queue.push(present_armor);
        armor_queue_time.push(getTickCount());
        return false ; 
    }
    else if(armor_queue.size() == 2)//FIXME::可以增加一种情况以进行优化
    {
        armor_queue.pop();                             //弹出首元素
        armor_queue.push(present_armor);  //压入最新装甲板坐标

        armor_queue_time.pop();                                         //弹出时间
        armor_queue_time.push(getTickCount());                          //压入时间

        bool is_height_min = (armor_queue.back().size.height < armor_queue.back().size.width); 
        // 判断装甲板是否发生切换         
        bool is_armor_plate_switched = ((fabs(armor_queue.front().angle -armor_queue.back().angle) > 10) ||//角度

                                         fabs((armor_queue.back().size.width / armor_queue.back().size.height) - //H/W
                                         (armor_queue.front().size.width / armor_queue.front().size.height )) > 1 ||

                                         ((fabs(armor_queue.back().center.x - armor_queue.front().center.x) +
                                         fabs(armor_queue.back().center.y - armor_queue.front().center.y))) > 200//中心点 
                                         );

#ifdef USING_DEBUG_ANTISPIN
        if(is_armor_plate_switched)
        {
            last_switched_armor = present_armor;//存储上次切换装甲板
            if(spinning_coeffient == 0 ){//若置信度此时为0
                spinning_coeffient = 2;//设置初值
                predict_armor = present_armor;
                return false;
            }
            else{
                spinning_coeffient =  5 + 6 * spinning_coeffient;
            }

        }
        if(spinning_coeffient < 1)
            spinning_coeffient = 0;
        else if(spinning_coeffient > 4e3)
            spinning_coeffient = 4e3;
        else
            spinning_coeffient /= 2;
        if(spinning_coeffient > 400){
            cout<<"Spinning :"<<spinning_coeffient<<endl;
        } 
        else
            cout<<"Steady :"<<spinning_coeffient<<endl;
        #endif//USING_DEBUG_ANTISPIN


#ifndef USING_DEBUG_ANTISPIN
        if(is_armor_plate_switched)
        {
            predict_armor = present_armor;
            return false;
        }
#endif//USING_DEBUG_ANTISPIN
    
        double delta_time = (double)(armor_queue_time.back() - armor_queue_time.front()) / getTickFrequency();//处理时间
        double velocity_x = (armor_queue.back().center.x - armor_queue.front().center.x) / delta_time;//Vx
        double velocity_y = (armor_queue.back().center.y - armor_queue.front().center.y) / delta_time;  //Vy
        double velocity_height = (MIN(armor_queue.back().size.height, armor_queue.back().size.width) - 
                                    MIN(armor_queue.front().size.height, armor_queue.front().size.width)) / delta_time;
        double velocity_width = (MAX(armor_queue.back().size.height, armor_queue.back().size.width) - 
        MAX(armor_queue.front().size.height, armor_queue.front().size.width)) / delta_time;

        double predict_time = delta_time + 0.05 ;//预测时间为处理时间加响应时间
        //更新状态转移矩阵
        kalmanfilter.KF.transitionMatrix = (Mat_<float>(8, 8) << 1,0,0,0,predict_time,0,0,0,//x
                                                                0,1,0,0,0,predict_time,0,0,//y
                                                                0,0,1,0,0,0,predict_time,0,//width
                                                                0,0,0,1,0,0,0,predict_time,//height
                                                                0,0,0,0,1,0,0,0,//Vx
                                                                0,0,0,0,0,1,0,0,//Vy
                                                                0,0,0,0,0,0,1,0,//Vwidth
                                                                0,0,0,0,0,0,0,1);//Vheight
                                                                

        //设置测量矩阵
        Mat measure =(Mat_<float>(8, 1) << armor_queue.back().center.x,
                                           armor_queue.back().center.y,
                                           MAX(armor_queue.back().size.height, armor_queue.back().size.width),
                                           MIN(armor_queue.back().size.height, armor_queue.back().size.width),
                                           velocity_x,
                                           velocity_y,
                                           velocity_width,
                                           velocity_height);
        //进行卡尔曼预测
        Mat predict = kalmanfilter.KF.predict();
        //设置预测装甲板
        if(is_height_min)
            predict_armor = RotatedRect(Point2f(predict.at<float>(0,0),predict.at<float>(0,1)),
                                        Size2f( MAX(armor_queue.back().size.height, armor_queue.back().size.width),
                                                MIN(armor_queue.back().size.height, armor_queue.back().size.width)),
                                                present_armor.angle);
        else
            predict_armor = RotatedRect(Point2f(predict.at<float>(0,0),predict.at<float>(0,1)),
            Size2f(MIN(armor_queue.back().size.height, armor_queue.back().size.width),MAX(armor_queue.back().size.height, armor_queue.back().size.width)),
            present_armor.angle);
            
        kalmanfilter.KF.correct(measure);
        //若识别到中心点距离过大则不进行预测 
    if((fabs(predict_armor.center.x - present_armor.center.x) +
        fabs(predict_armor.center.y - present_armor.center.y)) > 200)
        {
            predict_armor = present_armor;
            return false;
        }
    }
    return true;

}

// @brief 线程生产者
void ImageProcess::ImageProductor()
{
#ifdef USE_DAHENG_CAMERA
    DaHeng.StartDevice(1);
    // 设置分辨率
    DaHeng.SetResolution(1,1);
    // 开始采集帧
    DaHeng.SetStreamOn();
    // 设置曝光事件
    DaHeng.SetExposureTime(500);
    // 设置
    DaHeng.SetGAIN(3, 16);
    // 是否启用自动曝光
    DaHeng.Set_BALANCE_AUTO(1);
#endif // USE_DAHENG_CAMERA


#ifdef USE_LOCAL_VIDEO
    if (!cap.isOpened())
    {
        cout << "local video is not open" << endl;
        return;
    }
#endif // USE_LOCAL_VIDEO

#ifdef SAVE_VIDEO_DAHENG
    VideoWriter videowriter;
    time_t time_now;
    struct tm *time_info;
    time(&time_now);        //Record time from 1970 Jan 1st 0800AM

    time_info = localtime(&time_now);
    string save_video_name = "/home/tup/Desktop/TUP-Vision/Video/VIDEO_DAHENG_";//define the name of video

    //Write time into video name
    save_video_name += to_string((time_info->tm_year) + 1900) + "_";//YY
    save_video_name += to_string(time_info->tm_mon) + "_";//MM
    save_video_name += to_string(time_info->tm_mday) + "_";//DD
    save_video_name += to_string(time_info->tm_hour) + "_";//Hour
    save_video_name += to_string(time_info->tm_min) + "_";//Minute
    save_video_name += to_string(time_info->tm_sec);//Second

    save_video_name += ".avi";
    cout<<"Video name :"<<save_video_name<<endl;
    videowriter.open(save_video_name,videowriter.fourcc('M','J','P','G'),60,Size(1280,1024));//initialize videowriter
#endif//SAVE_VIDEO_DAHENG


    while (true)
    {
        while(vision_mode != 1)
        {
        };
#ifdef CALC_PROCESS_TIME
        clock_t timer_productor;         //消费者处理计时变量
        timer_productor = clock();       //记录该ID任务开始时间
#endif

        // 经典的生产-消费者模型
        // 资源太多时就阻塞生产者
        while (proIdx - consIdx >= IMG_BUFFER)
            ;
        ImageData Src;

#ifdef USE_USB_CAMERA
        cap >> Src.img;
#endif // USE_USB_CAMERA


#ifdef USE_DAHENG_CAMERA
        DaHeng.GetMat(Src.img);
        if (Src.img.cols == 0)
        {
            cout << "丢帧!" << endl;
            continue;
        }
#endif // USE_DAHENG_CAMERA

#ifdef USE_LOCAL_VIDEO
        cap >> Src.img;
#endif // USE_LOCAL_VIDEO


        Image[proIdx % IMG_BUFFER] = Src;

        if (Src.img.empty())
        {
            cout << "the image is empty...";
            return;
        }
#ifdef SAVE_VIDEO_DAHENG
        videowriter << Src.img;//Save Frame into video
#endif//SAVE_VIDEO_DAHENG

#ifdef SHOW_SRC
        namedWindow("SHOW_SRC",WINDOW_NORMAL);
        imshow("SHOW_SRC", Src.img);
#endif // SHOW_SRC

        proIdx++;
        waitKey(1);
#ifdef CALC_PROCESS_TIME
        timer_productor = (clock() - timer_productor) / (CLOCKS_PER_SEC / 1000) ;    //原地计算本次任务所用时间(单位:ms) 
        cout<<endl;
        cout<<"Productor ID : "<<proIdx<<endl;                                         //输出生产者ID 
        cout<<"Process Time : "<<(int)timer_productor<<"ms"<<endl;                      //输出处理时间
        cout<<endl;
#endif//CALC_PROCESS_TIME
    }
}

// @brief 线程消费者
void ImageProcess::ImageConsumer()
{
    /*===========================相机畸变矩阵传入===========================*/
    FileStorage file(file_path, FileStorage::READ);
    Mat camera_Matrix, dis_Coeffs;
    // file["Camera_Matrix"]>>camera_Matrix;
    // file["Distortion_Coefficients"]>>dis_Coeffs;
    camera_Matrix = (cv::Mat_<double>(3, 3) << 1.451e+03,  0.000000000000,  645.373,  0.000000000000,  1.4563e+03,  491.0741,  0.000000000000,  0.000000000000,  1.000000000000);
    dis_Coeffs = (cv::Mat_<double>(1, 5) << -0.2146,  0.3219,  0,  0,  0);
    /*===========================相机畸变矩阵传入===========================*/

    /*===========================角度解算参数传入===========================*/
    // 最后不是根据这个的
    AngleSolver solver_720(camera_Matrix, dis_Coeffs, 22.5, 5.5);
    // 大小装甲板的角度解法
    AngleSolverFactory angle_slover;
    angle_slover.setTargetSize(22.5, 5.5 , AngleSolverFactory::TARGET_ARMOR);
    angle_slover.setTargetSize(13, 5.5, AngleSolverFactory::TARGET_SMALL_ARMOR);
    /*===========================角度解算参数传入===========================*/

    /*===========================创建装甲板识别类===========================*/
    ArmorParam armor;
    ArmorDetector armor_detector(armor);
    armor_detector.setPnPSlover(&solver_720);
    angle_slover.setSolver(&solver_720);
    /*===========================创建装甲板识别类===========================*/

    /*===========================创建SVM类===========================*/
    NumClassfier classfier;
    /*===========================创建SVM类===========================*/

    /*===========================函数中所使用的参数===========================*/
    Mat src;                                 //传入函数的源图
    int near_face = 0;                       //是否贴脸
    int miss_detection_cnt = 0;              //统计没有追踪到目标的帧数
    int find_cnt = 0;                        //统计追踪到目标的帧数
    float last_x = 0, last_y = 0, last_dist; //上一时刻的云台偏移角度(防止哨兵失去目标云台停顿)
    double distance = 0;                     //装甲板距离
    ArmorPlate present_armor;                 //现在的装甲板类
    RotatedRect rect;                        //getArea的函数输出,为最后筛选出来的装甲板
    ArmorPlate predict_armor;                //卡尔曼预测得到的所需击打的装甲板
    Point2f center = cv::Point2f();          //基地模式的串口发送参数
    VisionData vdata;                        //串口发送的数据结构体
    /*===========================函数中所使用的参数===========================*/

#ifdef A_RED
    int mode = 1;                              //模式选择:1(辅瞄红色),2(辅瞄蓝色)
#endif
#ifdef A_BLUE
    int mode = 2;                              //模式选择:1(辅瞄红色),2(辅瞄蓝色)
#endif

    // _port.get_Mode(mode,_sentrymode,_basemode);//从串口读取模式数据
    /*===========================函数中所使用的参数===========================*/

#ifdef CALC_PROCESS_TIME
    clock_t timer_consumer_sum = 0;         //存放消费者总处理时间(单位:ms)
#endif//CALC_PROCESS_TIME

    // ============================== 大循环 ===============================//
    while (true)
    {
        while(vision_mode != 1)
            ;        
#ifdef CALC_PROCESS_TIME
        clock_t timer_consumer;         //消费者处理计时变量
        timer_consumer = clock();       //记录该ID任务开始时间
#endif//CALC_PROCESS_TIME
        // 消费太多的时候就什么都不要做
        while (consIdx >= proIdx)
            ;
        double angle_x = 0.0, angle_y = 0.0, dist = 0;

        //模式选择
        if (mode == 1)
            armor_detector.enemy_color = RED;
        else if (mode == 2)
            armor_detector.enemy_color = BLUE;

        //等待produce
        while (proIdx - consIdx == 0)
            ;
        Image[consIdx % IMG_BUFFER].img.copyTo(src);
        ++consIdx;

        //图片数据正常
        if (src.empty())
            continue;
        if (src.channels() != 3)
            continue;
        //直接调用函数找到读进来的图中是否有目标
        if (!(armor_detector.getTargetArea(src, present_armor, 0, 0)))
        {
            vdata = {(float)0, (float)0, (float)0, 0, 0, 0, near_face};
            _port.TransformData(vdata);
            _port.send();
            continue;
        }

        // 根据长宽比判断目标是大装甲还是小装甲
        AngleSolverFactory::TargetType type = present_armor.is_small_armor ? AngleSolverFactory::TARGET_SMALL_ARMOR : AngleSolverFactory::TARGET_ARMOR;

#ifdef ENABLE_NUM_CLASSFICATION
        present_armor.serial = classfier.runSvm(src,present_armor);
#ifdef COUT_LOG
        cout<<"---------Num Classfication-------------"<<endl;
        cout<<"Armor Plate Num :"<<present_armor.serial<<endl;
#endif //COUT_LOG
#endif //ENABLE_NUM_CLASSFICATION

#ifdef USING_ADVANCED_PREDICT
        AdvancedPredictForArmorDetect(present_armor.boundingRect,predict_armor.boundingRect);//对装甲板进行卡尔曼预测与反陀螺(实验性)
        Point2f predict_vector = predict_armor.boundingRect.center - present_armor.boundingRect.center;
        for(int i = 0; i < 4 ;i++)
            predict_armor.apex[i] = present_armor.apex[i] + predict_vector; 
        // 解算出的角度，为false则说明没有识别到目标
        if (angle_slover.getAngle(predict_armor, type, angle_x, angle_y, dist) == true)
#else
        // 解算出的角度，为false则说明没有识别到目标
        if (angle_slover.getAngle(present_armor, type, angle_x, angle_y, dist) == true)
#endif//USING_KALMAN_ARMOR
        {
#ifdef SHOW_DISTANCE
            namedWindow("SHOW_DISTANCE",WINDOW_NORMAL);
            for (int i = 0; i < 4; i++)
                line(src, predict_armor.apex[i], predict_armor.apex[(i + 1) % 4], Scalar(0, 0, 255),2);
            for (int i = 0; i < 4; i++)
                line(src, present_armor.apex[i], present_armor.apex[(i + 1) % 4], Scalar(0, 100, 200),2);
            drawRotatedRect(src,present_armor.rrect,Scalar(255,0,0),1);
            String distance = "distance:";
            distance += to_string(int(dist));
            putText(src, distance, Point(20, 20), FONT_HERSHEY_SIMPLEX , 1, Scalar(0, 255, 0), 2);
            imshow("SHOW_DISTANCE", src);
#endif // SHOW_DISTANCE
            miss_detection_cnt = 0;
        }
        else
        {
            ++miss_detection_cnt;
            find_cnt = 0;
        }

        if (angle_x != 0 && angle_y != 0)
        {
            shootingAngleCompensate(dist,angle_x,angle_y);
            distance = dist;
            if (find_cnt >= 2)
                vdata = {(float)angle_x, (float)angle_y, (float)distance, 1, 1, 0, near_face};
            else
                vdata = {(float)angle_x, (float)angle_y, (float)distance, 1, 1, 0, near_face};
            last_x = angle_x;
            last_y = angle_y;
            last_dist = distance;

            _port.TransformData(vdata);
            _port.send();
        }
        else
        {
            _port.TransformData(vdata);
            _port.send();
        }
#ifdef ECHO_FINAL_INFO
        cout<<"-------------FINAL_INFO----------------"<< endl;
        cout<<"MODE : AIMING_ASSIST"<<endl;
        cout<<"yaw_angle :     "<<angle_x<< endl;
        cout<<"pitch_angle :   "<<angle_y<< endl;
        cout<<"dist :"<<distance<<endl;
        cout<<"---------------------------------------"<<endl;
#endif//ECHO_FINAL_INFO
#ifdef CALC_PROCESS_TIME
        timer_consumer = (clock() - timer_consumer)/ (CLOCKS_PER_SEC / 1000);    //原地计算本次任务所用时间(单位:ms) 
        timer_consumer_sum += timer_consumer;                                   //将处理时间存入总时间

        cout<<endl;
        cout<<"Consumer ID : "<<consIdx<<endl;                                      //输出消费者ID
        cout<<"Process Time : "<<(int)timer_consumer<<"ms"<<endl;                   //输出处理时间
        cout<<"Average Process Time : "<<timer_consumer_sum / consIdx<<"ms"<<endl;  //输出平均处理时间 
        cout<<endl;
#endif//CALC_PROCESS_TIME
    }

}