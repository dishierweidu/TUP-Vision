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


#ifdef USE_LOCAL_VIDEO
// const string source_location = "/home/rangeronmars/Desktop/video/RH.avi";
// const string source_location = "/home/rangeronmars/Desktop/video/sample.avi";
// const string source_location = "/home/rangeronmars/Desktop/video/sample1_sin_red.mp4";
// const string source_location = "/home/rangeronmars/Desktop/video/sample2_sin_red.mp4";
// const string source_location = "/home/rangeronmars/Desktop/video/sample2_sin_red_800*800.mp4";
// const string source_location = "/home/rangeronmars/Desktop/video/sample2_sin_red_800*800_conter_clockwise.mp4";
const string source_location = "/home/rangeronmars/Desktop/video/sample2_sin_red_roi_test.mp4";
// const string source_location = "/home/rangeronmars/Desktop/video/sample1_sin_blue.mp4";

VideoCapture cap(source_location);
#endif // USE_LOCAL_VIDEO

#ifdef USE_DAHENG_CAMERA
DaHengCamera DaHeng;
bool is_first_loop = true;
#endif // USE_DAHENG_CAMERA

#ifdef USE_USB_CAMERA
VideoCapture cap(0);
#endif // USE_USB_CAMERA

string file_path = "./File/calib_no_4_1280.yml";

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
Energy energy; // 创立能量机关类

// @brief ImageProcess构造函数
ImageProcess::ImageProcess(SerialPort &port) : _port(port), _sentrymode(0), _basemode(0)
{
}


// @brief ImageProcess构造函数
ImageProcess::ImageProcess()
{
}


// @brief Angle offset based on distance
// @param ditsance Distance to target armor plate.
// @param angle_x Yaw angle using in function.
// @param angle_y Pitch angle using in function.
void ImageProcess::ShootingAngleCompensate(double &distance,double &angle_x,double &angle_y)
{
    //Curve Fitted in MATLAB
    //X--Distance to armor plate(cm)
    //Y--Pitch Angle Offset
    //y=15.61 * sin (0.001314*x + 0.5077)
    //------------------------------------------
    //X--Distance to armor plate(cm)
    //Z--Yaw Angle Offset
    //z = 3.463 *x^0.1959
    // if(distance >= 450 )return;//exit the function when distance is too big.
    double angle_x_compensate;//Yaw Angle Offset
    double angle_y_compensate;//Pitch Angle Offset
    angle_y_compensate = 15.61 * sin (0.001314 * distance + 0.5077) - ((-0.01238) * distance + 3.044);
    angle_x_compensate = 3.463 * pow(distance,0.1959);
    angle_y -= angle_y_compensate;
    angle_x -=angle_x_compensate;
}

// @brief 线程生产者
void ImageProcess::ImageProductor()
{
    #ifdef USE_USB_CAMERA
    if (!cap.isOpened())
    {
        cout << "camera is not open" << endl;
        return;
    }
    #endif // USE_USB_CAMERA


    #ifdef USE_DAHENG_CAMERA
    DaHeng.StartDevice(1);
    // 设置分辨率
    DaHeng.SetResolution(2, 2);
    // 开始采集帧
    DaHeng.SetStreamOn();
    // 设置曝光事件
    DaHeng.SetExposureTime(200);
    // 设置
    DaHeng.SetGAIN(3, 200);
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


    while (true)
    {
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
        // cout<<"ImageSize : "<<Src.img.size()<<endl;
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

        #ifdef SHOW_SRC
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
    file["Camera_Matrix"]>>camera_Matrix;
    file["Distortion_Coefficients"]>>dis_Coeffs;
    // camera_Matrix = (cv::Mat_<double>(3, 3) << 1.5042e+03, 0.000000000000, 6.5358e+02, 0.000000000000, 1.5025e+03, 5.7713e+02, 0.000000000000, 0.000000000000, 1.000000000000);
    // dis_Coeffs = (cv::Mat_<double>(1, 5) << -0.2302, 0.2882, 0, 0, 0);
    /*===========================相机畸变矩阵传入===========================*/

    /*===========================角度解算参数传入===========================*/
    // 最后不是根据这个的
    AngleSolver solver_720(camera_Matrix, dis_Coeffs, 22.5, 5.5);
    // 大小装甲板的角度解法
    AngleSolverFactory angle_slover;
    angle_slover.setTargetSize(22.5, 5.5 , AngleSolverFactory::TARGET_ARMOR);
    angle_slover.setTargetSize(13, 5.5, AngleSolverFactory::TARGET_SAMLL_ATMOR);
    /*===========================角度解算参数传入===========================*/

    /*===========================创建装甲板识别类===========================*/
    ArmorParam armor;
    ArmorDetector armor_detector(armor);
    ArmorParam armor_para_720 = armor;
    armor_detector.setPnPSlover(&solver_720);
    armor_detector.setPara(armor_para_720);
    angle_slover.setSolver(&solver_720);
    /*===========================创建装甲板识别类===========================*/

    /*===========================函数中所使用的参数===========================*/
    Mat src;                                 //传入函数的源图
    int near_face = 0;                       //是否贴脸
    int miss_detection_cnt = 0;              //统计没有追踪到目标的帧数
    int find_cnt = 0;                        //统计追踪到目标的帧数
    float last_x = 0, last_y = 0, last_dist; //上一时刻的云台偏移角度(防止哨兵失去目标云台停顿)
    double distance = 0;                     //装甲板距离
    RotatedRect rect;                        //getArea的函数输出,为最后筛选出来的装甲板
    Point2f center = cv::Point2f();          //基地模式的串口发送参数
    VisionData vdata;                        //串口发送的数据结构体
    VisionData enegy_data;                   //串口发送大神符数据结构体
    /*===========================函数中所使用的参数===========================*/

    #ifdef A_RED
    int mode = 1;                              //模式选择:1(辅瞄红色),2(辅瞄蓝色),3(大符红色),4(大符蓝色)
    #endif
    #ifdef A_BLUE
    int mode = 2;                              //模式选择:1(辅瞄红色),2(辅瞄蓝色),3(大符红色),4(大符蓝色)
    #endif

    // _port.get_Mode(mode,_sentrymode,_basemode);//从串口读取模式数据
    /*===========================函数中所使用的参数===========================*/
     #ifdef COMPILE_WITH_GPU 

    int is_cuda_available;

    is_cuda_available = cuda::getCudaEnabledDeviceCount();      //检测CUDA是否可用  返回值为0说明OPENCV未编译CUDA  返回值为-1说明CUDA未成功安装或设备不支持CUDA
    //检测到OPENCV未编译CUDA
    if(is_cuda_available == 0)
    {
        cout<<"OPENCV未编译CUDA!"<<endl;
    }
    //未检测到CUDA或设备不支持CUDA
    else if(is_cuda_available == -1)
    {
        cout<<"CUDA未成功安装或设备不支持CUDA!"<<endl;
    }
    //检测到可用CUDA编译
    else if(is_cuda_available == 1)
    {
        cout<<"检测到可使用的CUDA设备!"<<endl;
        //设置所使用的CUDA设备
        cuda::setDevice(0);
    }

    #endif//COMPILE_WITH_GPU

    #ifdef CALC_PROCESS_TIME
    clock_t timer_consumer_sum = 0;         //存放消费者总处理时间(单位:ms)
    #endif//CALC_PROCESS_TIME


    // ============================== 大循环 ===============================//
    while (true)
    {        
        #ifdef CALC_PROCESS_TIME
        clock_t timer_consumer;         //消费者处理计时变量
        timer_consumer = clock();       //记录该ID任务开始时间
        #endif//CALC_PROCESS_TIME
        // 消费太多的时候就什么都不要做
        while (consIdx >= proIdx)
            ;

        if (mode == 1 || mode == 2)
        {

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
            // imshow("source img",src);

            //直接调用函数找到读进来的图中是否有目标
            rect = armor_detector.getTargetArea(src, 0, 0);
            center = rect.center;
            int len = MIN(rect.size.height, rect.size.width);

            // 根据长宽比判断目标是大装甲还是小装甲
            AngleSolverFactory::TargetType type = armor_detector.isSamllArmor() ? AngleSolverFactory::TARGET_SAMLL_ATMOR : AngleSolverFactory::TARGET_ARMOR;
            // 解算出的角度，为false则说明没有识别到目标
            if (angle_slover.getAngle(rect, type, angle_x, angle_y, dist) == true)
            {
                #ifdef SHOW_DISTANCE
                String distance = "diatance:";
                distance += to_string(int(dist));
                putText(src, distance, Point(20, 20), CV_FONT_NORMAL, 1, Scalar(0, 255, 0), 2);
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
                angle_x = angle_x + 9.0;
                // // 这是用excel拟合出来的灯条高度与实际距离的函数关系（我用单目pnp解出的距离不行）
                // distance = 10941 * pow(len, -1.066);
                ShootingAngleCompensate(dist,angle_x,angle_y);
                if (_sentrymode)
                    angle_y += 1;
                // 基地模式发送特殊的识别flag
                // if (_basemode) vdata = {(float)center.x, (float)center.y, (float)distance, 1, 8, 0, near_face};

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
                // if (_basemode){
                //     vdata = {(float)center.x, (float)center.y, (float)distance, 1, 0, 0, near_face};
                // }
                // else{
                /**
             * 对哨兵模式进行了特殊处理
             * 在掉帧超过一定帧数时才会发送未识别
             * 否则就发送之前的角度乘以一个系数
             * 这是为了防止打灭灯条后云台会停一下的问题
             */
                if (_sentrymode)
                {
                    if (miss_detection_cnt > 10)
                        vdata = {(float)angle_x, (float)angle_y, (float)distance, 0, 0, 0, near_face};
                    else
                        vdata = {last_x / 3, last_y / 3, last_dist, 0, 1, 0, near_face};
                }
                else
                {
                    if (miss_detection_cnt > 5)
                        vdata = {(float)angle_x, (float)angle_y, (float)distance, 0, 0, 0, near_face};
                    else
                        vdata = {last_x / 3, last_y / 3, last_dist, 0, 1, 0, near_face};
                }
                // }

                _port.TransformData(vdata);
                _port.send();
            }
            // cout << "yaw_angle :     " << angle_x << endl;
            // cout << "pitch_angle :   " << angle_y << endl;
        }        
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

// @brief 能量机关识别线程
void ImageProcess::EnergyThread()
{
    while(true)
    {
        // TODO: 角度偏移,串口通讯

        #ifdef USE_USB_CAMERA
        cap >> oriFrame;
        #endif // USE_USB_CAMERA


        #ifdef USE_LOCAL_VIDEO
        cap >> oriFrame;
        #endif // USE_LOCAL_VIDEO



        if(oriFrame.empty() || oriFrame.channels() != 3)
        {
            cout << "oriFrame empty" << endl;
            continue;
        }

        #ifdef SHOW_ENERGY_RUN_TIME
        double run_time = static_cast<double>(getTickCount());
        #endif // SHOW_ENERGY_RUN_TIME


        // 能量机关识别入口
        energy.run(oriFrame);


        #ifdef SHOW_ENERGY_RUN_TIME
        run_time = ((double)getTickCount() - run_time) / getTickFrequency();
        cout << "SHOW_ENERGY_RUN_TIME: " << run_time << endl;
        #endif // SHOW_ENERGY_RUN_TIME

        waitKey(1);
    }
}