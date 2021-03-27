#ifndef SERIALPORT_H
#define SERIALPORT_H
/**
 *@class  SerialPort
 *@brief  set serialport,recieve and send
 *@param  int fd
 */
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>
#include "CRC_Check.h"
//#include <opencv2/opencv.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//using namespace cv;
using namespace std;


#define TRUE 1
#define FALSE 0

//模式
#define CmdID0 0x00; //关闭视觉
#define CmdID1 0x01; //识别红色
#define CmdID2 0x02; //识别蓝色
#define CmdID3 0x03; //小幅
#define CmdID4 0x04; //大幅

//串口的相关参数
#define BAUDRATE 115200//波特率
#define UART_DEVICE "/dev/ttyUSB0"//默认的串口名称

//C_lflag
#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)

//字节数为4的结构体
typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

//字节数为2的uchar数据类型
typedef union
{
    int16_t d;
    unsigned char c[2];
} int16uchar;

//用于保存目标相关角度和距离信息及瞄准情况
typedef struct
{
	float2uchar yaw_angle;//偏航角
	float2uchar pitch_angle;//俯仰角
	float2uchar dis;//目标距离
    int ismiddle;//设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，目前暂不使用，默认置0
	int isFindTarget;//当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
    int isfindDafu;
    int nearFace;
} VisionData;


class SerialPort
{
private:
    int fd; //串口号
    int speed, databits, stopbits, parity;
    unsigned char rdata[255]; //raw_data
    unsigned char Tdata[30];  //transfrom data

	void set_Brate();
	int set_Bit();
public:
    SerialPort();
    SerialPort(char *);
    bool initSerialPort();
    bool get_Mode(int &mode, int &sentry_mode, int &base_mode);
	void TransformData(const VisionData &data); //主要方案
	void send();
	void closePort();
	void TransformDataFirst(int Xpos, int Ypos, int dis);//方案1
//  int set_disp_mode(int);
//  void TransformTarPos(const VisionData &data);
};

#endif //SERIALPORT_H

