//----------------------------------------------------------
//                                                          
// FileName: Debug.h                                                
// Author: 周俊平;刘上;赵梓合;顾昊
// Version: 1.0.0_x64Linux
// Date: 2021.03.19
// Description: 各种Debug参数,用于调试
// 
//----------------------------------------------------------


//----------------------------------------------------------//
//                                                          //
//                      Terminal调试参数                     //
//                                                          //
//----------------------------------------------------------//

// #define SHOW_RUN_TIME               // 是否在终端打印每帧的处理时间
// #define SHOW_SRC_GET_TIME               // 是否在终端打印每帧的采集时间
// #define DEBUG_PREDICT_INFORMATION_BUFF       //是否在终端打印预测相关DEBUG信息

//----------------------------------------------------------//
//                                                          //
//                         线程调试参数                       //
//                                                          //
//----------------------------------------------------------//

#define MULTI_THREAD                // 是否使用多线程

#ifndef MULTI_THREAD
#define SINGLE_THREAD               // 是否使用多线程 
#endif

//----------------------------------------------------------//
//                                                          //
//                        图像调试参数                        //
//                                                          //
//----------------------------------------------------------//

// #define USE_DAHENG_CAMERA   // 是否使用大恒139相机
// #define USE_USB_CAMERA      // 是否使用USB相机
#define USE_LOCAL_VIDEO     // 是否使用本地测试视频

// #define REOPEN            //是否在丢帧后重启摄像头

// #define COMPILE_WITH_GPU    // 是否使用GPU

// #define SHOW_DEBUG_IMG
// #define SHOW_ORIGINAL      // 是否显示原始图像
#define SHOW_SRC

//----------------------------------------------------------//
//                                                          //
//                        图像处理参数                        //
//                                                          //
//----------------------------------------------------------//

#define USE_SPLIT_INIT           // 使用通道分离来进行图像初始化

#ifndef USE_SPLIT_INIT           // 如果使用USE_SPLIT_INIT就不能使用USE_HSV_INIT
#define USE_HSV_INIT             // 使用转换色彩空间来进行图像初始化
#endif
// #define SHOW_COLOR              //是否显示通道相减后图像
// #define SHOW_DISTANCE           //是否在图上显示pnp解算的距离
// #define SHOW_BINARY                   // 是否显示二值化图像

// #define SHOW_RECT_INFO              //是否显示所有矩形及相关信息
// #define SHOW_ALL_ARMORS               // 是否显示初步筛选出的装甲板
// #define SHOW_TARGET_ARMOR             // 是否能显示最终得到的装甲板
#define SHOW_TARGET_ARMOR_CENTER      // 是否显示最终得到的装甲板中心点

//  #define SHOW_FLOW_STRIP_FAN          // 是否显示含流动条的扇叶
//  #define SHOW_FLOW_STRIP_FAN_TWO_ROI  // 是否显示含流动条的扇叶两侧roi矩形区域

//  #define SHOW_R_CENTER                // 是否显示字母R中心点

//  #define SHOW_PREDICT_POINT           // 是否显示打击预测点