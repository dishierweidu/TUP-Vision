//----------------------------------------------------------
//                                                          
// FileName: main.cpp
// Author: Liu Shang fyrwls@163.com 
// Version: 1.0.20200924                                  
// Date: 2020.09.24
// Description: 图像读取,创建,输出帧处理时间
// Function List:
//              1. int main()
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
    SerialPort port(ttyUSB_path);//创建串口类对象
    port.initSerialPort();//串口初始化


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
