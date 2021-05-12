# Copyright(C),2018-2021,沈阳航空航天大学T-UP战队 All Rights Reserved
  
## Author:  周俊平;刘上;赵梓合;顾昊
  
## Version: 4.3.0

### 更新内容：
更改灯条拟合方式为椭圆拟合,进行相关修改
 *Date:  2021.05.12

卡尔曼参数调节及阈值修改,修复ROI偏移越界的bug
 *Date:  2021.05.05

更新了卡尔曼滤波相关代码；完善了和下位机的通讯机制 *Date: 2021.05.03
 *Date:  2021.05.02

更新了ArmorDetector中的阈值参数；扩大了对装甲板判定的条件范围；将能量机关识别任务独立给长焦镜头；更改编译路径为相对路径  *Date:  2021.04.10

更改了Debug.h的内容；便于调式  *Date:  2021.03.27
  
### Description: 

  沈阳航空航天大学TUP战队Robomaster 2021赛季步兵视觉程序
  
### Features：

  1.自动识别敌方车辆装甲板
  
  2.自动打击能量机关
 
  3.自动识别基地装甲板（doing）
  
  4.预测敌方车辆移动方向并预留提前量打击（waiting）
  
### Files List:
     
     1.AngelSolver 解算角度
     
     2.Armor 装甲板识别
     
     3.Build 编译文件
     
     4.Camera 驱动大恒139
     
     5.Energy 能量机关打击
     
     6.File 镜头标定参数
     
     7.ImageProcess 图像处理
     
     8.Serial 串口通讯
     
     9.CmakeLists  得到Mat图像
     
     10.Debug 调试头文件
     
     11.Readme 说明文件
     
     12.config Ubuntu 16.04/18.04 LTS 快速配置命令合集
     
     13.watchDoge 防崩溃shell
