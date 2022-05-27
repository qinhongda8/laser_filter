

#ifndef EXAMPLE_ROS_CLASS_H_
#define EXAMPLE_ROS_CLASS_H_

#include <vector>
#include <numeric>
#include <iostream>
#include <algorithm>
#include <string>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <math.h> 
#include <string.h>
/* -------------------------------------说明文档----------------------------------*/

/*  1: 功能说明
    本代码主要功能是平滑车辆左前右前两侧距离值和判定前方壁体凸起。
        用于对输入左前五个相邻的激光雷达距离数据和右前五个相邻的激光雷达距离数据进行处理，
    通过观察距离的突变和比较距离的变化量，来判定车辆什么时候经过匝道。驶过匝道时，过滤掉
    匝道的距离值，用经过匝道之前的距离值来覆盖。驶出匝道时，重新使用真实的距离值，并判定
    是否存在墙壁凸出。

    2：输入输出参数说明
    -> 输入为两个vector容器，分别装有左前五个相邻的激光雷达距离数据和右前五个相邻的激光
    雷达距离数据。
        具体格式为： vector<double> left_list, vector<double> right_list；
    -> 输出为大小等于5数组指针；各个位置数据分别为：
        output_value[0]: 调整后的左前距离平均值
        output_value[1]: 调整后的右前距离平均值
        output_value[2]: 左前壁体凸出警示
        output_value[3]: 右前壁体凸出警示
    
    3：示例
        // 获取调整后的左前距离平均值
        Process_LR_front* Process_LR = new Process_LR_front;
        double *output = Process_LR->Process_LR_data(left_list, right_list);
        std::cout << "left_front_value : " << output[0]  << "\n";

    4：调参注意事项说明
        根据当前的雷达信号角度进行 * 阈值 * 变量的调整；当前的阈值为13, 13.5, 14, 14.5,
        15度和-13, -13.5, -14, -14.5, -15度的参数示例；
*/

/* -----------------------------------------------------------------------------*/

/* -------------------------------修改日志---------------------------------------*/
/* 2022.5.25
    1：增加记录20帧的平均值 left_ave_01[20]；改%方式为index循环；过滤方式修改；
        原先的警告输出无效，后续按需修改

*/



class Process_LR_front{
    public:
        Process_LR_front(){};
        ~Process_LR_front(){};
        int Index(int index,int sub,int window_val){ 
            if (index < sub) {
                out = window_val + (index - sub);
            }
            else{
                out = index -sub;
            }
            return out;
        };
        double Average(std::vector<double> &left_list);     //移除等于0的数据,并返回均值
        double* Process_LR_data(std::vector<double> left_list, std::vector<double> right_list);
    
    private:
        int index = 0;
        double const threshold_distance_bias = 0.5;         // * 阈值 *, 用于与距离突变差值比较来判断进入匝道
        int const L_threshold_width = 2;                    // * 阈值 *, 用于判断左侧出匝道变宽时，距离值与之前距离距离差值要在阈值之内
        int const R_threshold_width = 2;                    // * 阈值 *, 用于判断右侧出匝道变宽时，距离值与之前距离距离差值要在阈值之内
        int L_obsracle_predic_flag = 0;                     // 左侧出匝道阻碍预测标识; (0：正在出匝道口，进行下一步判定; 1：未在出匝道口，不进行下一步判定)
        int R_obsracle_predic_flag = 0;                     // 右侧出匝道阻碍预测标识; (0：正在出匝道口，进行下一步判定; 1：未在出匝道口，不进行下一步判定)
        double left_ave_01[20] = {0};                        // 用于记录上一帧和当前帧的距离均值
        double right_ave_01[20] = {0};                       // 用于记录上一帧和当前帧的距离均值
        int window_value = 20;
        
        double left_ave_02[50] = {0};                        // 用于记录上一帧和当前帧的距离均值
        double right_ave_02[50] = {0};                       // 用于记录上一帧和当前帧的距离均值
        int window_value_large = 50;

        int cave_l  = 0;                                    // 表示车辆左侧是否驶过匝道口; (0：未在匝道口; 1：在匝道口)
        int cave_r  = 0;                                    // 表示车辆右侧是否驶过匝道口; (0：未在匝道口; 1：在匝道口)
        double ave_l_use[2] = {0};                          // 用于保存进入匝道前左侧两帧正常的两个距离均值
        double ave_l_real_use = 0;                          // 用于保存进入匝道前左侧正常的距离均值
        double ave_r_use[2] = {0};                          // 用于保存进入匝道前右侧两帧正常的两个距离均值
        double ave_r_real_use = 0;                          // 用于保存进入匝道前右侧正常的距离均值
        int left_Waring = 0;                                // 左侧碰撞提醒; (0：不提示警告; 1：提示警告)
        int right_Waring = 0;                               // 右侧碰撞提醒; (0：不提示警告; 1：提示警告)

        int out_flag_index_l = 0;
        int out_flag_index_r = 0;
        double width_change_l = 0;
        double width_change_r = 0;
        int out;

        //  Left的碰撞提醒变量
        double left_line_list[2] = {0};                      // 用于保存近两个左侧非匝道的距离均值
        double L_change_rate_list[20] = {0};                 // 记录左侧非匝道的距离变化率
        double const L_threshold_line_chg = 0.5;             // * 阈值 * ,左侧驶出匝道时，20帧内距离变化的总大小，超过判断为有阻碍
        
        //  Right的碰撞提醒变量
        double right_line_list[2] = {0};                     // 用于保存近两个右侧非匝道的距离均值
        double R_change_rate_list[20] = {0};                 // 记录右侧非匝道的距离变化率
        double const R_threshold_line_chg = 0.5;             // * 阈值 *, 右侧驶出匝道时，20帧内距离变化的总大小，超过判断为有阻碍
        

        double output_value[4] = {0};                        // 总输出
        
};


#endif