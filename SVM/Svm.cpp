//----------------------------------------------------------
//
// FileName: Svm.cpp
// Author: 顾昊
// Version: 1.0.0_x64Linux
// Date: 2021.07.09
// Description: svm相关函数定义
// Function List:
//              1.NumClassfier::NumClassfier()
//              2.bool NumClassfier::loadSvmModel(const string &path)
//              3.bool NumClassfier::runSvm(const Mat &src,ArmorPlate &target_armor)
//              4.bool NumClassfier::initImg(const Mat &src,Mat &dst,const ArmorPlate &target_armor)
//----------------------------------------------------------
#include "Svm.h"
/**
 * @brief NumClassfier构造函数
*/
NumClassfier::NumClassfier()
{   
    sample_size = 40;
    binary_threshold = 20;
    loadSvmModel(model_path);
    dst_apex_cord[0] = Point2f(0,sample_size);
    dst_apex_cord[1] = Point2f(0,0);
    dst_apex_cord[2] = Point2f(sample_size,0);
    dst_apex_cord[3] = Point2f(sample_size,sample_size);
}

/**
 *@brief 载入SVM模型
 *@param path SVM模型路径
 *@return 是否成功载入 
 */
bool NumClassfier::loadSvmModel(const string &path)
{
    svm = StatModel::load<SVM>(path);
    if(svm.empty())
    {
        cout<<"svm model path failure"<<endl;
        return false;
    }

    cout<<"svm load succeed"<<endl;
    return true;
}
/**
 * @brief SVM主函数
 * @param src 原图
 * @param ArmorPlate 目标装甲板
 * @return 是否成功运行
*/
bool NumClassfier::runSvm(const Mat &src,ArmorPlate &target_armor)
{
    Mat sample_img = Mat::zeros(Size(sample_size,sample_size),CV_8UC1);
    Mat sample_img_reshaped;
    //图像初始化
    initImg(src,sample_img,target_armor);
    //样本reshape与改变格式
    sample_img_reshaped = sample_img.reshape(1, 1);
	sample_img_reshaped.convertTo(sample_img_reshaped, CV_32FC1);
    //计算结果
    target_armor.serial = (int)svm->predict(sample_img_reshaped);
    return true;
}
/**
 * @brief 图像初始化
 * @param src 原图
 * @param dst 处理后图像
 * @param target_armor 目标装甲板
 * @return 是否成功初始化
*/
bool NumClassfier::initImg(const Mat &src,Mat &dst,const ArmorPlate &target_armor)
{
    Mat warped_img = Mat::zeros(Size(sample_size,sample_size),CV_8UC3);
    Mat src_gray;
    //设置扩张高度
    int extented_height = 0.5 * std::min(target_armor.rrect.size.width,target_armor.rrect.size.height);
    //计算归一化的装甲板左右边缘方向向量
    Point2f left_edge_vector = (target_armor.apex[1] - target_armor.apex[0]) / pointsDistance(target_armor.apex[1] , target_armor.apex[0]);//由0指向1
    Point2f right_edge_vector = (target_armor.apex[2] - target_armor.apex[3]) / pointsDistance(target_armor.apex[2] , target_armor.apex[3]);//由3指向2
    src_apex_cord[0] = target_armor.apex[0] - left_edge_vector * extented_height;
    src_apex_cord[1] = target_armor.apex[1] + left_edge_vector * extented_height;
    src_apex_cord[2] = target_armor.apex[2] + right_edge_vector * extented_height;
    src_apex_cord[3] = target_armor.apex[3] - right_edge_vector * extented_height;
    //计算透视变换矩阵
    Mat warp_matrix = getPerspectiveTransform(src_apex_cord,dst_apex_cord);
    //进行图像透视变换
    warpPerspective(src, warped_img, warp_matrix, Size(sample_size,sample_size), INTER_NEAREST, BORDER_CONSTANT, Scalar(0));
    //进行图像的灰度化与二值化
    cvtColor(warped_img,src_gray,COLOR_BGR2GRAY);
    threshold(src_gray,dst,binary_threshold,255,THRESH_BINARY);
    return true;
}