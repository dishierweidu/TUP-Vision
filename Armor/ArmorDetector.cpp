//----------------------------------------------------------
//
// FileName: ArmorDetector.cpp
// Author: 周俊平;刘上;赵梓合;顾昊
// Version: 1.1.0
// Date: 2021.07.14
// Description: ArmorDetector类中函数的实现
// Function List:
//              1.bool ArmorDetector::setImage(const cv::Mat &src)
//              2.bool ArmorDetector::findTargetInContours(vector<Matched_Rect> &match_rects)
//              3.bool ArmorDetector::chooseTarget(const std::vector<Matched_Rect> &match_rects, ArmorPlate &target_armor, const cv::Mat &src)
//              4.bool ArmorDetector::getTargetArea(const cv::Mat &src,ArmorPlate &target_armor, const int &sentry_mode, const int &base_mode)
//              5.cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect &left, const cv::RotatedRect &right)              
//----------------------------------------------------------

#include "ArmorDetector.hpp"
#include "Debug.h"
#include "../Debug.h"

/**
 * @brief 图像预处理函数,包括ROI图像裁剪,通道相减去,图形学操作
 * @param src   传入的原图
 * @return 处理是否成功
 */
bool ArmorDetector::setImage(const cv::Mat &src)
{
    _size = src.size(); // 原图尺寸为1280*720
    // 注意这个_res_last是一个旋转矩形
    const cv::Point &last_result = _res_last.center;

    // 如果上一次的目标没了，原图就是输入的图
    // 并且搜索的ROI矩形（_dect_rect）就是整个图像
    if (last_result.x == 0 || last_result.y == 0)
    {
        _src = src;
        _dect_rect = Rect(0, 0, src.cols, src.rows);
    }
    else
    {
        // 如果上一次的目标没有丢失的话，用直立矩形包围上一次的旋转矩形
        Rect rect = _res_last.boundingRect();
        // _para.max_light_delta_h 是左右灯柱在水平位置上的最大差值，像素单位
        int max_half_w = _para.light_max_delta_h * 1.3;
        int max_half_h = 300;

        // 截图的区域大小。太大的话会把45度识别进去
        double scale_w = 2;
        double scale_h = 2;

        int w = int(rect.width * scale_w);
        int h = int(rect.height * scale_h);
        Point center = last_result;
        int x = std::max(center.x - w, 0);
        int y = std::max(center.y - h, 0);
        Point lu = Point(x, y);//左上角顶点
        x = std::min(center.x + w, src.cols);
        y = std::min(center.y + h, src.rows);
        Point rd = Point(x, y);//右下角顶点

        // 构造出矩形找到了搜索的ROI区域
        _dect_rect = Rect(lu, rd);
        // / 如若矩形是空的则返回false，继续搜索全局像素
        if (makeRectSafe(_dect_rect, src.size()) == false)
        {
#ifdef SHOW_ROI
            Mat roi_img;
            src.copyTo(roi_img);
            rectangle(roi_img,_dect_rect,Scalar(255,0,0));
            imshow("SHOW_ROI",roi_img);
#endif //SHOW_ROI
            _res_last = cv::RotatedRect();
            _dect_rect = Rect(0, 0, src.cols, src.rows);
            _src = src;
        }
        else
        {
#ifdef SHOW_ROI
            Mat roi_img;
            src.copyTo(roi_img);
            rectangle(roi_img,_dect_rect,Scalar(255,0,0));
            imshow("SHOW_ROI",roi_img);
#endif //SHOW_ROI
            // 如果ROI矩形合法的话则使用此ROI
            src(_dect_rect).copyTo(_src);
        }
    }

    //==========================上面已经设置好了真正处理的原图============================

    // 下面是在敌方通道上二值化进行阈值分割
    // _max_color是红蓝通道相减之后的二值图像
    ///////////////////////////// begin /////////////////////////////////////////////
    /** 
     * 预处理其实是最重要的一步，这里有HSV和RGB两种预处理的思路，其实大致都差不多
     * 只不过在特定场合可以选择特定的预处理方式
     * 例如HSV的话可以完全过滤掉日光灯的干扰，但是耗时较大
     */
    _max_color = cv::Mat(_src.size(), CV_8UC1, cv::Scalar(0));

    //设置掩模大小
    Mat mask_element_7by7 = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
    Mat mask_element_5by5 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    Mat mask_element_3by3 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

    Mat color;
    vector<Mat> splited;
    split(_src, splited);
    cvtColor(_src, thres_whole, CV_BGR2GRAY);
    // 敌方颜色为红色
    if (enemy_color == RED)
    {
        threshold(thres_whole, thres_whole, threshold_gray_red, 255, THRESH_BINARY);
        subtract(splited[2], splited[0], color);
#ifdef SHOW_THRESH_WHOLE
        imshow("THRESH_WHOLE", thres_whole);
#endif //SHOW_THRESH_WHOLE
#ifdef SHOW_COLOR

        imshow("SHOW_COLOR", color);
#endif //SHOW_COLOR
        threshold(color, color, threshold_max_color_red, 255, THRESH_BINARY);
    }
    // 敌方颜色为蓝色
    else if (enemy_color == BLUE)
    {
        threshold(thres_whole, thres_whole, threshold_gray_blue, 255, THRESH_BINARY);
        subtract(splited[0], splited[2], color);
#ifdef SHOW_THRESH_WHOLE
        imshow("THRESH_WHOLE", thres_whole);
#endif //SHOW_THRESH_WHOLE
#ifdef SHOW_COLOR
        imshow("SHOW_COLOR", color);
#endif //SHOW_COLOR
        threshold(color, color, threshold_max_color_blue, 255, THRESH_BINARY);
    }
    erode(color, color, mask_element_5by5);
    dilate(color, color, mask_element_5by5);
    dilate(color, color, mask_element_3by3);
    //对灰度二值图与通道相减二值图进行与操作,完成预处理
    _max_color = color & thres_whole;
    // _max_color = color ;
    dilate(_max_color, _max_color,mask_element_3by3);
#ifdef SHOW_INITIALIZED_IMAGE
    imshow("INIT_IMAGE", _max_color);
#endif //SHOW_INITIALIZED_IMAGE
    return true;
////////////////////////////// end /////////////////////////////////////////
}

/**
* @brief 寻找图像中可能为装甲板灯条的灯条对
* @param match_rects 候选灯条对容器
* @return 候选灯条对容器是否不为空
**/
bool ArmorDetector::findTargetInContours(vector<MatchedRect> &match_rects)
{
    vector<vector<Point2i>> contours_max;
    vector<Vec4i> hierarchy;
    Mat FirstResult;
    _src.copyTo(FirstResult);

    findContours(_max_color, contours_max, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    vector<RotatedRect> RectFirstResult;
    for (size_t i = 0; i < contours_max.size(); ++i)
    {
        if (contours_max[i].size() <= 6)//小于6点无法拟合椭圆
            continue;
        //对轮廓进行椭圆拟合
        RotatedRect rrect = fitEllipse(contours_max[i]);
        double max_rrect_len = MAX(rrect.size.width, rrect.size.height);
        double min_rrect_len = MIN(rrect.size.width, rrect.size.height);

        /////////////////////////////// 单根灯条的条件 //////////////////////////////////////
        // 角度要根据实际进行略微改动
        bool if1 = (fabs(rrect.angle) < 45.0 ); // 左倾
        bool if2 = (fabs(rrect.angle) > 135.0 ); // 右倾
        bool if3 = max_rrect_len > _para.light_min_height;                             // 灯条的最小长度
        bool if4 = (max_rrect_len / min_rrect_len >= 1.1) && (max_rrect_len / min_rrect_len < 15); // 灯条的长宽比
        if ((if1 || if2) && if3 && if4)
        {
#ifdef COUT_LOG
            cout<<"------------Fitting led----------------"<<endl;
            cout << if1 << " " << if2 << " " << if3 << " " << if4 << endl;
#endif //COUT_LOG
            RectFirstResult.push_back(rrect);
//绘制灯条旋转矩形轮廓
#ifdef SHOW_TARGET_SHOT_ARMOR
            Point2f vertice[4];
            rrect.points(vertice);
            for (int i = 0; i < 4; i++) // 绿色
                line(FirstResult, vertice[i], vertice[(i + 1) % 4], Scalar(0, 255, 0), 3);
            imshow("SHOW_TARGET_SHOT_ARMOR", FirstResult);
#endif //SHOW_TARGET_SHOT_ARMOR
        }
    }

    // 少于两根灯条就认为没有匹配到
    if (RectFirstResult.size() < 2)
    {
        return false;
    }
    // 将所有候选旋转矩形从左到右排序
    sort(RectFirstResult.begin(), RectFirstResult.end(),
         [](RotatedRect &a1, RotatedRect &a2) { return a1.center.x < a2.center.x; });

    Point2f _pt[4], pt[4];
    auto ptangle = [](const Point2f &p1, const Point2f &p2) {
        return fabs(atan2(p2.y - p1.y, p2.x - p1.x) * 180.0 / CV_PI);
    };

    ///////////////////////////////////// 匹配灯条的条件 //////////////////////////////////////////////////////
    // 两两比较，有符合条件的就组成一个目标旋转矩形
    for (size_t i = 0; i < RectFirstResult.size() - 1; ++i)
    {
        const RotatedRect &rect_i = RectFirstResult[i];
        const Point &center_i = rect_i.center;
        float xi = center_i.x;
        float yi = center_i.y;
        float leni = MAX(rect_i.size.width, rect_i.size.height);
        float anglei = fabs(rect_i.angle);
        rect_i.points(_pt);
        /*
              * 0 2
              * 1 3
              * */
        // 往右斜的长灯条
        // rRect.points有顺序的，y最小的点是0,顺时针1 2 3
        if (anglei > 45.0)
        {
            pt[0] = _pt[3];
            pt[1] = _pt[0];
        }
        // 往左斜的
        else
        {
            pt[0] = _pt[2];
            pt[1] = _pt[3];
        }

        for (size_t j = i + 1; j < RectFirstResult.size(); j++)
        {
            const RotatedRect &rect_j = RectFirstResult[j];
            const Point &center_j = rect_j.center;
            float xj = center_j.x;
            float yj = center_j.y;
            float lenj = MAX(rect_j.size.width, rect_j.size.height);
            float anglej = fabs(rect_j.angle);

            float delta_h = xj - xi;
            float lr_rate = leni > lenj ? leni / lenj : lenj / leni; //较大的与较小的相比
            float angleabs;

            rect_j.points(_pt);
            if (anglej > 45.0)
            {
                pt[2] = _pt[2];
                pt[3] = _pt[1];
            }
            else
            {
                pt[2] = _pt[1];
                pt[3] = _pt[0];
            }

            double maxangle = MAX(ptangle(pt[0], pt[2]), ptangle(pt[1], pt[3]));
            // 灯条呈"八字"情况 ---> " //   1   \\"
            if (anglei < 45.0 && anglej > 135.0)
            {  
                angleabs = 180 -anglej + anglei;
            }
            // 灯条呈"倒八字"情况 ---> " \\   1   //"
            else if (anglei >= 135.0 && anglej <= 45.0)
            {
                angleabs = 180 - anglei + anglej;
            }
            // 灯条倾角同侧情况 ---> " \\   1   \\"
            else
            {
                if (anglei > anglej)
                    angleabs = anglei - anglej;
                else
                    angleabs = anglej - anglei;
            }
            //Condition 1 : 灯条高度差值
            bool condition1 = delta_h > _para.light_min_delta_h && delta_h < _para.light_max_delta_h;
            //Condition 2 : 灯条水平差值
            bool condition2 = MAX(leni, lenj) >= 113 ? 
                                abs(yi - yj) < 166 && abs(yi - yj) < 1.66 * MAX(leni, lenj) :
                                abs(yi - yj) < _para.light_max_delta_v && abs(yi - yj) 
                                < 1.2 * MAX(leni, lenj);
            //Condition3 : 左右灯条最长边的比值
            bool condition3 = lr_rate < _para.light_max_lr_rate;
            //Condition4 : 角度差的绝对值
            bool condition4 = abs(angleabs) < _para.light_max_delta_angle;

#ifdef COUT_LOG
            cout <<"-------Fitting matched led pair--------"<<endl;
            cout << "len i:" <<  leni << endl;
            cout << "len j:" <<  lenj << endl;
            cout << "delta_h:  " << abs(yi - yj) << endl;
            cout << "lr rate:  " << lr_rate << endl;
            cout << "length:   " << MAX(leni, lenj) << endl;
            cout << condition1 << " " << condition2 << " " << condition3 << " " << condition4 << endl;
#endif //COUT_LOG

            if (condition1 && condition2 && condition3 && condition4)
            {
                RotatedRect obj_rect = boundingRRect(rect_i, rect_j);//灯条拟合旋转矩形
                Point2f apex_left_LED[4];
                Point2f apex_right_LED[4];  
                Point2f apex[4];           //存储候选矩形区域点
                rect_i.points(apex_left_LED);//存储左灯条顶点
                rect_j.points(apex_right_LED);//存储右灯条顶点
                /*
                根据椭圆拟合的角度向容器中压入灯条对八个顶点中的内侧四点,
                排列顺序为以左下的内侧点为起点,按顺时针排列,如下图所示:
                
                | 1=========================2 |
                | |                         | |
                | |                         | |
                | |                         | |
                | 0=========================3 |
                */
                if(rect_i.angle < 90)
                {
                    apex[0] = apex_left_LED[3];     //左侧灯条的右下顶点
                    apex[1] = apex_left_LED[2];     //左侧灯条的右上顶点
                }
                else if(rect_i.angle >= 90)
                {
                    apex[0] = apex_left_LED[1];     //左侧灯条的右下顶点
                    apex[1] = apex_left_LED[0];     //左侧灯条的右上顶点

                }

                if(rect_j.angle < 90)
                {            
                    apex[2] = apex_right_LED[1];    //右侧灯条的左上顶点
                    apex[3] = apex_right_LED[0];    //右侧灯条的左下顶点

                }
                else if(rect_j.angle >= 90)
                {            
                    apex[2] = apex_right_LED[3];    //右侧灯条的左上顶点
                    apex[3] = apex_right_LED[2];    //右侧灯条的左下顶点

                }
                double w = obj_rect.size.width;
                double h = obj_rect.size.height;
                double wh_ratio = w / h;
#ifdef COUT_LOG
                cout <<"----------Checking wh_ratio------------"<< endl;
                cout << "wh_ratio:  " << wh_ratio << endl;
#endif //COUT_LOG

                if (wh_ratio > _para.armor_max_wh_ratio || wh_ratio < _para.armor_min_wh_ratio)
                    continue;
                Point2f ROI_Offset;
                ROI_Offset = Point2f(_dect_rect.x,_dect_rect.y);
                // 将初步匹配到的结构体信息push进入vector向量
                //此处顶点坐标皆为裁剪后的图像中的装甲板坐标,需在坐标上加上ROI坐标偏移向量才能得到原图坐标系下的装甲板坐标
                match_rects.push_back(MatchedRect{lr_rate,angleabs,obj_rect,apex[0] + ROI_Offset,apex[1] + ROI_Offset,apex[2] + ROI_Offset,apex[3] + ROI_Offset});
                // for debug use
#ifdef SHOW_TARGET_SHOT_ARMOR
                Point2f vertice[4];
                obj_rect.points(vertice);
                for (int i = 0; i < 4; i++)
                    line(FirstResult, vertice[i], vertice[(i + 1) % 4], Scalar(255, 0, 0), 2);
                imshow("SHOW_TARGET_SHOT_ARMOR", FirstResult);
#endif //SHOW_TARGET_SHOT_ARMOR
                return true;
            }
        }
    }
#ifdef SHOW_FIRST_RESULT
    imshow("SHOW_FIRST_RESULT", FirstResult);
#endif //SHOW_FIRST_RESULT
    return false;
}
/**
 * @brief 在候选装甲板容器中选出目标装甲板
 * @param match_rects 候选装甲板容器
 * @param target_armor 目标装甲板
 * @return 是否成功选取目标
 */
bool ArmorDetector::chooseTarget(const std::vector<MatchedRect> &match_rects, ArmorPlate &target_armor, const cv::Mat &src)
{
    // 如果没有两条矩形围成一个目标装甲板就返回一个空的旋转矩形
    if (match_rects.size() < 1)
    {
        _is_lost = true;
        return false;
    }

    // 初始化参数
    int ret_idx = -1;
    bool is_small = false;
    double weight = 0;
    vector<ArmorPlate> candidate;
    vector<Mat> channels;

    ///////////////////////// 匹配灯条 ////////////////////////////////////////////////
    //======================= 开始循环 ====================================

    for (size_t i = 0; i < match_rects.size(); ++i)
    {
        const RotatedRect &rect = match_rects[i].rrect;

        // 长宽比不符,担心角度出问题，可以侧着车身验证一下（上面一个函数好像写过这个条件了）
        double w = rect.size.width;
        double h = rect.size.height;
        double wh_ratio = w / h;

        // 设置角度解法，其实不要这个也可以，只是为了根据这个算出距离来筛选太远太近的
        AngleSolver *slover = NULL;
        if (_size.height == 720)
            slover = l_solver;
        // 用均值和方差去除中间太亮的图片（例如窗外的灯光等）
        RotatedRect screen_rect = RotatedRect(rect.center, Size2f(rect.size.width * 0.88, rect.size.height), rect.angle);
        Point p1, p2;
        int x = screen_rect.center.x - screen_rect.size.width / 2 + _dect_rect.x;
        int y = screen_rect.center.y - screen_rect.size.height / 2 + _dect_rect.y;
        p1 = Point(x, y);
        x = screen_rect.center.x + screen_rect.size.width / 2 + _dect_rect.x;
        y = screen_rect.center.y + screen_rect.size.height / 2 + _dect_rect.y;
        p2 = Point(x, y);
        Rect roi_rect = Rect(p1, p2);
        Mat roi;
        Mat roi_thresh;

        if (makeRectSafe(roi_rect, src.size()))
        {
            roi = src(roi_rect).clone();
            Mat mean, stdDev;
            double avg, stddev;
            meanStdDev(roi, mean, stdDev);
            avg = mean.ptr<double>(0)[0];
            stddev = stdDev.ptr<double>(0)[0];
            // 阈值可通过实际测量修改
            if (avg > 100.66)
                continue;
        }

        // 现在这个旋转矩形的角度
        float cur_angle = match_rects[i].rrect.size.width > match_rects[i].rrect.size.height ? abs(match_rects[i].rrect.angle) : 90 - abs(match_rects[i].rrect.angle);
        // 现在这个旋转矩形的高度（用来判断远近）
        int cur_height = MIN(w, h);
        // 最终目标如果太倾斜的话就筛除
        if (cur_angle > _para.armor_max_angle)
            continue;
        ArmorPlate cur_armor;
        // 如果矩形的w和h之比小于阈值的话就是小装甲，否则是大装甲
        if (wh_ratio < _para.armor_type_wh_threshold)
            cur_armor.is_small_armor = true;
        else
            cur_armor.is_small_armor = false;
        cur_armor.rrect = match_rects[i].rrect;
        copy(begin(match_rects[i].apex),end(match_rects[i].apex),begin(cur_armor.apex));//拷贝顶点元素至装甲板
        candidate.push_back(cur_armor);
        ret_idx = 0;
    }
    //================================ 结束循环 =======================================
    int final_index = 0;
    if (candidate.size() > 1)
    {
        // 将候选矩形按照高度大小排序，选出最大的（距离最近）
        sort(candidate.begin(), candidate.end(),
             [](ArmorPlate &target1, ArmorPlate &target2) {
                 return std::min(target1.rrect.size.width,target1.rrect.size.height) > 
                        std::min(target2.rrect.size.width,target2.rrect.size.height);
             });
        // 做比较，在最近的几个矩形中（包括45度）选出斜率最小的目标

        float temp_angle = candidate[0].rrect.size.width > candidate[0].rrect.size.height ?
                         abs(candidate[0].rrect.angle) : 90 - abs(candidate[0].rrect.angle);
        float temp_height = std::min(candidate[0].rrect.size.width,candidate[0].rrect.size.height);

        for (int i = 1; i < candidate.size(); i++)
        {
            if (temp_height / std::min(candidate[i].rrect.size.width,candidate[i].rrect.size.height) < 1.1)
            {
                float cur_angle = candidate[i].rrect.size.width > candidate[i].rrect.size.height ?
                                    abs(candidate[i].rrect.angle) : 90 - abs(candidate[i].rrect.angle);
                //若i倾斜角度小于0号
                if (cur_angle < temp_angle)
                {
                    temp_angle = cur_angle;
                    ret_idx = i;
                }
            }
        }
    }
    // ret_idx为-1,说明未寻找到目标
    if (ret_idx == -1)
    {
        _is_lost = true;
        return false;
    }
    target_armor = candidate[ret_idx];
    return true;
}

/**
* @brief 检测目标区域中是否存在装甲板
* @param src 原图像
* @param target_armor 目标装甲板
* @param sentry_mode  是否在哨兵模式
* @param base_mode    是否在基地模式
* @return 图像中是否存在装甲板
*/
bool ArmorDetector::getTargetArea(const cv::Mat &src,ArmorPlate &target_armor, const int &sentry_mode, const int &base_mode)
{
    ArmorPlate candidate_armor;
    vector<MatchedRect> match_rects;
    //图像预处理
    setImage(src);
    //若未找到目标
    if(!findTargetInContours(match_rects))
    {
#ifdef COUT_LOG
        cout<<"Target Not Found! "<<endl;
#endif //COUT_LOG
        ++_lost_cnt;
        // 逐次加大搜索范围（根据相机帧率调整参数）
        if (_lost_cnt < 8)
            _res_last.size = Size2f(_res_last.size.width, _res_last.size.height);
        else if (_lost_cnt == 9)
            _res_last.size = Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if (_lost_cnt == 12)
            _res_last.size = Size2f(_res_last.size.width * 2, _res_last.size.height * 2);
        else if (_lost_cnt == 15)
            _res_last.size = Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if (_lost_cnt == 18)
            _res_last.size = Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if (_lost_cnt > 33)
            _res_last = RotatedRect();
        return false;
    }
    //若无可供选择的目标
    if(!chooseTarget(match_rects,target_armor,src))
    {
        return false;
    };
    //设置下一帧ROI与原图之间的坐标偏移向量
    target_armor.rrect.center.x += _dect_rect.x;
    target_armor.rrect.center.y += _dect_rect.y;
    _res_last = target_armor.rrect;
    _lost_cnt = 0;

    return true;
}
/**
 * @brief 将左右的两根灯条用一个旋转矩形拟合并返回
 * @param left 左侧旋转矩形
 * @param right 右侧旋转矩形
 * @return 包含两个旋转矩形的最小旋转举行
 */
cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect &left, const cv::RotatedRect &right)
{
    // 此函数用来将左右边的灯条拟合成一个目标旋转矩形，没有考虑角度
    const Point &pl = left.center, &pr = right.center;
    Point2f center = (pl + pr) / 2.0;
    // 这里的目标矩形的height是之前灯柱的width
    double width_l = MIN(left.size.width, left.size.height);
    double width_r = MIN(right.size.width, right.size.height);
    double height_l = MAX(left.size.width, left.size.height);
    double height_r = MAX(right.size.width, right.size.height);
    float width = pointsDistance(pl,pr) - (width_l + width_r)/ 2.0;
    float height = std::max(height_l, height_r);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}
