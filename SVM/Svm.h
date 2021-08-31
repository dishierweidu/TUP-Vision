//----------------------------------------------------------
//
// FileName: Svm.h
// Author: 顾昊
// Version: 1.0.0_x64Linux
// Date: 2021.07.09
// Description: svm相关函数声明
// Function List:
//              1.NumClassfier::NumClassfier()
//              2.bool NumClassfier::loadSvmModel(const string &path)
//              3.bool NumClassfier::runSvm(const Mat &src,ArmorPlate &target_armor)
//              4.bool NumClassfier::initImg(const Mat &src,Mat &dst,const ArmorPlate &target_armor)
//----------------------------------------------------------
#include "../AngelSolver/AngleSolver.hpp"
#include "../General/General.h"

#include <iostream>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;
using namespace ml;

const string model_path = "/home/rangeronmars/RM/TUP-Vision/File/svm.xml";

class NumClassfier
{
public:
    bool runSvm(const Mat &src,ArmorPlate &target_armor);
    NumClassfier();
private:
    Ptr<SVM> svm;


    Point2f src_apex_cord[4];
    Point2f dst_apex_cord[4];

    int sample_size;                                     //设置样本大小
    int binary_threshold;                                //设置二值化阈值
    
    bool loadSvmModel(const string &path);               //SVM模型加载
    bool initImg(const Mat &src,Mat &dst,const ArmorPlate &target_armor);
};