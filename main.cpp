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


    #ifdef SINGLE_THREAD // FIXME: 单线程还需要修改

        Mat oriFrame;
        ImageProcess ImageProcessThread;
        Energy energy;
        #ifdef USE_DAHENG_CAMERA
        #endif 
    while(true)
    {   
        #ifdef USE_USB_CAMERA
        cap >> oriFrame;
        #endif

        ImageProcessThread.ImageProductor_Single(oriFrame);
        if(oriFrame.empty() || oriFrame.channels() != 3)
        {
            cout << "oriFrame empty" << endl;
            continue;
        }
        #ifdef SHOW_RUN_TIME
        double run_time = static_cast<double>(getTickCount());
        #endif

        ImageProcessThread.ImageConsumer_Single(oriFrame,energy);


        #ifdef SHOW_RUN_TIME
        run_time = ((double)getTickCount() - run_time) / getTickFrequency();
		cout << "SHOW_RUN_TIME:" << run_time << endl;
        #endif
        
        waitKey(1);
    }
    #endif // SINGLE_THREAD

    return 0;
}
