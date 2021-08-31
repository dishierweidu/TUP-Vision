//----------------------------------------------------------
//
// FileName: Main.cpp
// Author: GuHao    GuHao0521@Gmail.com
// Version: 1.1.0
// Date: 2021.7.14
// Description: Energy类主函数
// Function List:
//              1.void Energy::ShootingAngleCompensate(double &distance,double &angle_x,double &angle_y)
//              2.bool Energy::EnergyThreadProductor()
//              3.bool Energy::EnergyThreadConsumer()
//----------------------------------------------------------

#include "Energy.h"

string file_path_energy = "./File/calib_no_4_1280.yml";

#ifdef USE_LOCAL_VIDEO_ENERGY

const string source_location_energy = "/home/rangeronmars/Desktop/video/sample2_sin_red_30fps.mp4";

VideoCapture cap_energy(source_location_energy);
#endif  // USE_LOCAL_VIDEO

// 存储图像的结构体
typedef struct
{
    cv::Mat img;
} ImageData;

static volatile unsigned int proIdx_Energy = 0;  // 生产ID
static volatile unsigned int consIdx_Energy = 0; // 消费ID

#define IMG_BUFFER 5         // 线程间相机采集最高超过处理的帧数

ImageData Image_Energy[IMG_BUFFER]; // 存储图像的缓冲区

#ifdef USE_LOCAL_VIDEO

#endif // USE_LOCAL_VIDEO

extern atomic_int64_t vision_mode;

/**
 * @brief 射击补偿函数,包括固定补偿及距离补偿,角度单位均为角度制
 * @param distance 目标距离
 * @param angle_x  Yaw轴角引用(角度制)
 * @param angle_y  Pitch轴角引用(角度制)
*/
void Energy::shootingAngleCompensate(double &distance,double &angle_x,double &angle_y)
{
    double angle_x_compensate_static;//Yaw轴角度固定补偿
    double angle_y_compensate_static;//Pitch轴角度固定补偿
    double angle_x_compensate_dynamic;//Yaw轴角度动态补偿
    double angle_y_compensate_dynamic;//Pitch轴角度动态补偿
    
    angle_x_compensate_static = -8.9;  //右侧为正方向 
    angle_y_compensate_static = -1.2;  //上方为正方向
    angle_x_compensate_dynamic = 0;
    angle_y_compensate_dynamic =  0;

    angle_y += angle_y_compensate_dynamic + angle_y_compensate_static;
    angle_x += angle_x_compensate_dynamic + angle_x_compensate_static;
}

bool Energy::EnergyThreadProductor()
{  

    #ifdef USE_USB_CAMERA_ENERGY
    VideoCapture cap_energy(0);
    #endif // USE_USB_CAMERA



    #ifdef SAVE_VIDEO_USB_CAMERA
    VideoWriter videowriter;
    time_t time_now;
    struct tm *time_info;
    time(&time_now);        //记录自1970.7.1,0800AM以来的时间

    time_info = localtime(&time_now);
    string save_video_name = "/home/tup/Desktop/TUP-Vision/Video/VIDEO_USB_";//定义文件名

    //将时间写入文件名
    save_video_name += to_string((time_info->tm_year) + 1900) + "_";//YY
    save_video_name += to_string(time_info->tm_mon) + "_";//MM
    save_video_name += to_string(time_info->tm_mday) + "_";//DD
    save_video_name += to_string(time_info->tm_hour) + "_";//Hour
    save_video_name += to_string(time_info->tm_min) + "_";//Minute
    save_video_name += to_string(time_info->tm_sec);//Second

    save_video_name += ".avi";
    videowriter.open(save_video_name,CV_FOURCC('M','J','P','G'),60,Size(640,480));//初始化VideoWriter
    #endif //SAVE_VIDEO_USB_CAMERA


    

    while (true)
    {
        while(vision_mode != 2 && vision_mode != 3){
        };

        // 经典的生产-消费者模型
        // 资源太多时就阻塞生产者
        while (proIdx_Energy - consIdx_Energy >= IMG_BUFFER)
            ;
        ImageData Src;

        #ifdef USE_USB_CAMERA_ENERGY
        cap_energy >> Src.img;
        #endif // USE_USB_CAMERA

        #ifdef USE_LOCAL_VIDEO_ENERGY
        cap_energy >> Src.img;
        #endif // USE_LOCAL_VIDEO


        Image_Energy[proIdx_Energy % IMG_BUFFER] = Src;

        if (Src.img.empty())
        {
            cout << "the image is empty...";
            return false;
        }


        #ifdef SAVE_VIDEO_USB_CAMERA 
        videowriter << Src.img;//Save Frame into video
        #endif//SAVE_VIDEO_DAHENG

        #ifdef SHOW_SRC
        imshow("SHOW_SRC", Src.img);
        #endif // SHOW_SRC

        proIdx_Energy++;
        waitKey(1);
    }
}

bool Energy::EnergyThreadConsumer()
{
  
  /*===========================相机畸变矩阵传入===========================*/
    FileStorage file(file_path_energy, FileStorage::READ);
    Mat camera_Matrix, dis_Coeffs;
    // file["Camera_Matrix_USB"]>>camera_Matrix;
    // file["Distortion_Coefficients_USB"]>>dis_Coeffs;
    camera_Matrix = (cv::Mat_<double>(3, 3) <<  1200.9,  0.000000000000,  134.8634,  0.000000000000,  1196.2,  366.1528,  0.000000000000,  0.000000000000,  1.000000000000); 
    dis_Coeffs = (cv::Mat_<double>(1, 5) << -0.3524,  0.2160 , 0,0 ,0);
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

    /*===========================函数中所使用的参数===========================*/
    VisionData energy_data;                   //串口发送大神符数据结构体
    /*===========================函数中所使用的参数===========================*/
    int near_face = 0;                       //是否贴脸

    double angle_x;//yaw
    double angle_y;//pitch
    double dist = 0;
    Mat oriFrame;
    while(true)
    {
        while(vision_mode != 2 && vision_mode != 3){
        };
        
        if(vision_mode == 2)
            energyParams.stm32Data.energy_mode =ENERGY_SMALL;

        if(vision_mode == 3)
            energyParams.stm32Data.energy_mode =ENERGY_BIG;

        while (consIdx_Energy >= proIdx_Energy)
            ;
        //等待produce
        while (proIdx_Energy - consIdx_Energy == 0)
            ;
        Image_Energy[consIdx_Energy % IMG_BUFFER].img.copyTo(oriFrame);
        ++consIdx_Energy;



        if(oriFrame.empty() || oriFrame.channels() != 3)
        {
            cout << "oriFrame empty" << endl;
            continue;
        }

        #ifdef SHOW_ENERGY_RUN_TIME
        double run_time = static_cast<double>(getTickCount());
        #endif // SHOW_ENERGY_RUN_TIME


        // 能量机关识别入口
        if(!run(oriFrame)){
            energy_data = {(float)0, (float)0, (float)0, 1, 1, 0, near_face};
            _port.TransformData(energy_data);
            _port.send();
            cout<<"lost"<<endl;
            continue;
        }
        if (angle_slover.getAngle(predict_hit_armor, AngleSolverFactory::TARGET_ARMOR, angle_x, angle_y, dist))
        {
            shootingAngleCompensate(dist,angle_x,angle_y);//进行角度补偿
            energy_data = {(float)angle_x, (float)angle_y, (float)dist, 1, 1, 0, near_face};
            _port.TransformData(energy_data);
            _port.send();
        }
        else
        {
            energy_data = {(float)0, (float)0, (float)0, 1, 1, 0, near_face};
            // _port.TransformData(energy_data);
            // _port.send();
        }
        #ifdef ECHO_FINAL_INFO
        cout<<"-------------FINAL_INFO----------------"<< endl;
        cout<<"MODE : ENERGY"<<endl;
        cout<<"yaw_angle :     "<<angle_x<< endl;
        cout<<"pitch_angle :   "<<angle_y<< endl;
        cout<<"---------------------------------------"<<endl;
        #endif//ECHO_FINAL_INFO
        

        #ifdef SHOW_ENERGY_RUN_TIME
        run_time = ((double)getTickCount() - run_time) / getTickFrequency();
        cout << "SHOW_ENERGY_RUN_TIME: " << run_time << endl;
        #endif // SHOW_ENERGY_RUN_TIME

        waitKey(1);
    }
}