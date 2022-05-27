
#include <creating_a_ros_library/Process_LR_front.hpp>



double Process_LR_front::Average(std::vector<double> &left_list){
    left_list.erase(remove(left_list.begin(),left_list.end(),0),left_list.end()); // 移除等于0的数据
    double mean = std::accumulate(left_list.begin(), left_list.end(), 0.00) / left_list.size(); // 求均值
    return mean;
}

double* Process_LR_front::Process_LR_data(std::vector<double> left_list, std::vector<double> right_list){
    left_Waring = 0;
    right_Waring = 0;
    index++;
    if (index == window_value){
        index = 0;
    }
    double left_list_ave = Average(left_list); // 获得左前平均值
    if (left_list_ave > 6){
        left_ave_01[index] = left_list_ave;
    }else{
        left_ave_01[index] =left_ave_01[Index(index,1)];
    }

    double right_list_ave = Average(right_list);   // 获得右前平均值
    if ( right_list_ave > 6){
        right_ave_01[index] = right_list_ave;
    }else{
        right_ave_01[index] = right_ave_01[Index(index,1)];
    }
    if ((left_ave_01[Index(index,1)] != 0) && (right_ave_01[Index(index,1)] != 0)){

    
        // 不是在经过匝道中，直接使用真实距离，不做调整
        if (cave_l != 1){
                ave_l_real_use = left_list_ave;
        }
        if (cave_r != 1){
                ave_r_real_use = right_list_ave;
        }
        // 刷新一下初始标识变量

        // 判断经过匝道中，是否准备驶出匝道
        if (cave_l == 1){
            out_flag_index_l = out_flag_index_l + 1;
            if  (left_list_ave < ave_l_real_use ){  
                cave_l = 0;
                L_obsracle_predic_flag = 1;
                out_flag_index_l = 0;
            }   // 判断出匝道后是否更窄,窄的话判定驶出
            if ((( left_ave_01[index] - left_ave_01[Index(index,15)] ) < 0) && ((out_flag_index_l) > 150)) {  
                if (fabs( left_ave_01[index] - ave_l_real_use ) < L_threshold_width){
                    cave_l = 0;
                    out_flag_index_l = 0;
                } // 判断出匝道后是否更宽, 需要 两个条件，一个是处于变宽趋势，二是前距离值差距要在阈值之内
            }
        }
        if (cave_r == 1){
            out_flag_index_r = out_flag_index_r + 1;
            if  (right_list_ave < ave_r_real_use ){  
                cave_r = 0;
                R_obsracle_predic_flag = 1;
                out_flag_index_r = 0;
            }   // 判断出匝道后是否更窄,窄的话判定驶出
            if ((( right_ave_01[index] - right_ave_01[Index(index,15)] ) < 0) && ((out_flag_index_r) > 150)) {  
                if (fabs( right_ave_01[index] - ave_r_real_use ) < R_threshold_width){
                    cave_r = 0;
                    out_flag_index_r = 0;
                } // 判断出匝道后是否更宽, 需要 两个条件，一个是处于变宽趋势，二是前距离值差距要在阈值之内
            }
        }

        /* ---- 判断这一帧中的五个角度距离和上一帧平均值的差距有没有超过阈值 -----*/
        if (cave_l != 1){
            width_change_l = 0;
            for (int i = 0; i < window_value; ++i){
                width_change_l = (left_ave_01[Index(index,i)] - left_ave_01[Index(index,i + 1)]) + width_change_l;
            }
            if (width_change_l > threshold_distance_bias){
                cave_l = 1;
                ave_l_use[index%2] = left_ave_01[Index(index,1)];
                ave_l_real_use = left_ave_01[Index(index,1)];
                L_obsracle_predic_flag = 0;
            }
            
        }
        if (cave_r != 1){
            width_change_r = 0;
            for (int i = 0; i < window_value; ++i){
                width_change_r = (right_ave_01[Index(index,i)] - right_ave_01[Index(index,i + 1)]) + width_change_r;
            }
            if (width_change_r > threshold_distance_bias){
                cave_r = 1;
                ave_r_use[index%2] = right_ave_01[Index(index,1)];
                ave_r_real_use = right_ave_01[Index(index,1)];
                R_obsracle_predic_flag = 0;
            }
        }

        /* ---- 计算在出匝道后是否存在墙壁凸出  -----*/
        

        /* 如果现在在匝道口，使用原距离; 如果不在匝道口，使用实时距离 */
        // std::cout<< "ave_l_real_use3 : "<< ave_l_real_use << "\n"; 
        if (cave_l != 1){   
            // std::cout<< "left_list_ave : "<< left_ave_01[index] << "\n"; 
            output_value[0] = left_list_ave;  // 输出的左前平均值
        }
        else{
            output_value[0]  = ave_l_real_use; // 输出调整后的左前平均值
        }
        if (cave_r != 1){    
            output_value[1] = right_list_ave;  // 输出的右前平均值
        }
        else{
            output_value[1]  = ave_r_real_use; // 输出调整后的右前平均值
        }
        output_value[2] = left_list_ave;
        output_value[3] = right_list_ave; // 
        // output_value[4] = left_list_ave;
        // output_value[5] = right_list_ave; // 仅调试代码可视化时使用
            
        return output_value;
    }
    output_value[0] = left_list_ave;
    output_value[1] = right_list_ave;
    output_value[2] = left_list_ave;
    output_value[3] = right_list_ave;
    // output_value[4] = left_list_ave;
    // output_value[5] = right_list_ave;  // 仅调试代码可视化时使用
    
return output_value;
}
