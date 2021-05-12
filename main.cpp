
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
#include <atomic>


atomic_int64_t vision_mode;




void ModeReceiver(SerialPort &port)
{
    #ifndef DEBUG_WITHOUT_COM    
    int _sentry,_base;
    int temp = 1;
    int _mode = 0;
    #endif
    while (true)
    {
        #ifndef DEBUG_WITHOUT_COM  
        port.get_Mode(_mode,_sentry,_base);
        // cout<<"MODE :"<<_mode<<endl;


        
        if(_mode != temp)
        {
            vision_mode = _mode ;   //设置mode
            temp = _mode;
            // cout << 233 <<endl;
            // cout<<"MODE :"<<_mode<<endl;
        };                      
        #endif

        #ifdef DEBUG_WITHOUT_COM    
        vision_mode = 1;                           //1为辅瞄,2为小符,3为大符
        // int input;
        // std::cin>>input;
        // vision_mode = input;
        #endif
        // cout<<"MODE :"<<vision_mode<<endl;

    }
}

// @brief 主函数
int main()
{
    char ttyUSB_path[] = "/dev/ttyUSB0";//设置串口名称
    SerialPort port(ttyUSB_path);//'创建串口类对象 
    port.initSerialPort();//串口初始化
    XInitThreads();

    // port.get_Mode(_mode,_sentry,_base);


    std::thread ModeReceiverThread(&ModeReceiver,ref(port));      //开启控制进程

    start:
    #ifdef MULTI_THREAD
    ImageProcess process(port);
    std::thread ImageProductorThread(&ImageProcess::ImageProductor, process);
    std::thread ImageConsumerThread(&ImageProcess::ImageConsumer, process);
    #endif  // MULTI_THREAD

    #ifdef ENERGY_THREAD
    // TODO: 能量机关独立线程
    // DEBUG: 角度计算
    // DEBUG: 串口通讯
    Energy energy(port);
    std::thread BuffDetectThreadProducer(&Energy::EnergyThreadProductor, energy);
    std::thread BuffDetectThreadConsumer(&Energy::EnergyThreadConsumer, energy);
    #endif // ENERGY_THREAD





    #ifdef MULTI_THREAD
    ImageProductorThread.join();
    ImageConsumerThread.join();
    #endif  // MULTI_THREAD

    #ifdef ENERGY_THREAD
    BuffDetectThreadProducer.join();
    BuffDetectThreadConsumer.join();
    #endif // ENERGY_THREAD

    goto start;

    return 0;
}

