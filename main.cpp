
//----------------------------------------------------------
//
// FileName: main.cpp
// Author: 周俊平;刘上;赵梓合;顾昊
// Version: 1.0.0
// Date: 2021.04.10
// Description: 
//
//----------------------------------------------------------


#include "./ImageProcess/ImageProcess.h"
#include <X11/Xlib.h>
#include <thread>

// @brief 主函数
int main()
{
    XInitThreads();
    char ttyUSB_path[] = "/dev/ttyUSB0";//设置串口名称
    SerialPort port(ttyUSB_path);//'创建串口类对象
    
    int _mode,_sentry,_base;
    
    port.initSerialPort();//串口初始化
    port.get_Mode(_mode,_sentry,_base);


    #ifdef MULTI_THREAD
    ImageProcess process(port);
    std::thread t1(&ImageProcess::ImageProductor, process);
    std::thread t2(&ImageProcess::ImageConsumer, process);

    t1.join();
    t2.join();
    #endif  // MULTI_THREAD



    #ifdef ENERGY_THREAD
    // TODO: 能量机关独立线程
    // DEBUG: 角度计算
    // DEBUG: 串口通讯
    ImageProcess process(port);
    std::thread t1(&ImageProcess::EnergyThread, process);

    t1.join();
    #endif // ENERGY_THREAD


    return 0;
}
