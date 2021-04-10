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
#define SHOW_FLOW_STRIP_FAN          // 是否显示含流动条的扇叶

#define SHOW_ALL_ARMORS               // 是否显示初步筛选出的装甲板
// #define SHOW_TARGET_ARMOR             // 是否能显示最终得到的装甲板
// #define SHOW_TARGET_ARMOR_CENTER      // 是否显示最终得到的装甲板中心点

//  #define SHOW_R_CENTER                // 是否显示字母R中心点

// #define SHOW_PREDICT_POINT           // 是否显示打击预测点


// #define SHOW_RECT_INFO              //是否显示所有矩形及相关信息



#endif // ENERGYDEBUG_H