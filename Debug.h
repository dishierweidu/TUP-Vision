//----------------------------------------------------------
//                                                          
// FileName: Debug.h                                                
// Author: 周俊平;刘上;赵梓合;顾昊
// Version: 1.0.0_x64Linux
// Date: 2021.03.19
// Description: 各种Debug参数,用于调试
// 
//----------------------------------------------------------
//==============================================================================================================//
//                                                                                                              //
//                                              通用参数                                                         //
//                                                                                                              //
//==============================================================================================================//


//----------------------------------------------------------//
//                                                          //
//                        赛前调试参数                       //
//                                                          //
//----------------------------------------------------------//
// #define A_RED    // Detect Red Armor
#define A_BLUE   // Detect Blue Armor

#ifdef A_RED
#define E_BLUE       // Detect Blue Energy
#endif // A_RED

#ifdef A_BLUE
#define E_RED   // Detect Red Energy
#endif // A_BLUE


//----------------------------------------------------------//
//                                                          //
//                         线程调试参数                       //
//                                                          //
//----------------------------------------------------------//

#define MULTI_THREAD               // 是否使用多线程

#define ENERGY_THREAD               // 能量机关独立线程

// #define COMPILE_WITH_GPU         // FIX ME:辅瞄是否使用GPU(不适用于妙算)

// #define DEBUG_WITHOUT_COM           //无串口调试

// #define SAVE_VIDEO_DAHENG                  //Save video to local.
// #define SAVE_VIDEO_USB_CAMERA                  //Save video to local.

//----------------------------------------------------------//
//                                                          //
//                        图像调试参数                        //
//                                                          //
//----------------------------------------------------------//
//-----------------------------(辅瞄)-----------------------------------//
#define USE_DAHENG_CAMERA   // 是否使用大恒139相机
// #define USE_USB_CAMERA      // 是否使用USB相机
// #define USE_LOCAL_VIDEO     // 是否使用本地测试视频

// #define REOPEN            //是否在丢帧后重启摄像头

#define SHOW_SRC      // 是否显示原始图像
//-----------------------------(大符)-----------------------------------//
#define USE_USB_CAMERA_ENERGY      // 是否使用USB相机
// #define USE_LOCAL_VIDEO_ENERGY     // 是否使用本地测试视频

//----------------------------------------------------------//
//                                                          //
//                                                          //
//                      Terminal调试参数                     //
//                                                          //
//----------------------------------------------------------//

// #define SHOW_SRC_GET_TIME               // 是否在终端打印每帧的采集时间(仅限于大恒)

// #define SHOW_ENERGY_RUN_TIME            // 是否在终端打印能量机关每帧的处理时间

#define CALC_PROCESS_TIME    // 显示生产及消费所用时间(辅瞄)


