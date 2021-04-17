#ifndef ENERGYDEBUG_H
#define ENERGYDEBUG_H

//----------------------------------------------------------//
//                                                          //
//                        图像处理参数                        //
//                                                          //
//----------------------------------------------------------//

// $$$$$$$      这些调试参数都是有顺序的!不要随意更改!      $$$$$$$


// #define DEBUG_PREDICT_INFORMATION_BUFF       //是否在终端打印大符预测相关DEBUG信息(时间戳与预测所用装甲板位置样本数量)

#define SHOW_ORIGINAL               // 是否显示原始图像

#define SHOW_BINARY                   // 是否显示二值化图像

// #define SHOW_FLOW_STRIP_FAN_TWO_ROI  // 是否显示含流动条的扇叶两侧roi矩形区域
// #define SHOW_FLOW_STRIP_FAN          // 是否显示含流动条的扇叶

// #define SHOW_ALL_ARMORS               // 是否显示初步筛选出的装甲板
// #define SHOW_TARGET_ARMOR             // 是否能显示最终得到的装甲板
// #define SHOW_TARGET_ARMOR_CENTER      // 是否显示最终得到的装甲板中心点

 #define SHOW_R_CENTER                // 是否显示字母R中心点

#define SHOW_PREDICT_POINT           // 是否显示打击预测点



//----------------------中心R识别模式选择--------------------------//
// #define CIRCLE_FIT                  //使用装甲板中心拟合圆心(不稳定)

#ifndef CIRCLE_FIT
#define FIND_R_STRUCT_IN_ROI          //寻找ROI中R结构
#endif
//----------------------中心R识别模式选择--------------------------//


//--------------------------大符预测方案选择------------------------------//
// #define ABSOLUTE_TIME_INTERGRATION      //绝对时间积分得到预测角度(不准确)

#ifndef ABSOLUTE_TIME_INTERGRATION 
#define USING_KALMAN_FILTER                       //使用卡尔曼滤波预测大符
#endif


//--------------------------大符预测方案选择------------------------------//



//-------------For Debug Rect Only--------------------//
// #define SHOW_RECT_INFO              //是否显示所有矩形及相关5信息(因程序逻辑问题,矩形绘制在面积比较之后,使用前需先调整面积参数)

#ifdef  SHOW_RECT_INFO
#define SHOW_RECT_INFO_FLOW_STRIP   //显示待判断流动条矩形信息
#define SHOW_RECT_INFO_ARMOR        //显示待处理目标装甲版矩形信息
#endif

//-------------For Debug Rect Only--------------------//




#endif // ENERGYDEBUG_H

