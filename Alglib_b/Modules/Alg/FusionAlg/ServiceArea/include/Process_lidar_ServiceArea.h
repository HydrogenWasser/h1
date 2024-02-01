/*******************************************************
 文件名：process_lidar_ServiceArea.h
 作者：HuaHuan
 描述：雷达检测数据和轨迹关联匹配
 版本：v1.0
 日期：2024-01-11
 *******************************************************/
#ifndef PROCESS_LIDAR_SERVICEAREA_H
#define PROCESS_LIDAR_SERVICEAREA_H

#include "TPcResult_ServiceArea.h"
#include "BaseAssociate_ServiceArea.h"
#include "FUKalmanBoxTracker_ServiceArea.h"
#include "IouAssociate_ServiceArea.h"
#include "DistanceAssociate_ServiceArea.h"
#include "json_ServiceArea.hpp"

#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.1415926


class Process_lidar_ServiceArea
{
private:
    /* data */
    // TPcResult m_dets;  // 雷达检测结果
    // int pc_timestamp; // 雷达时间戳
    xt::xarray<float> lidar_box;            // 数据预处理后的检测信息：shape:(n, )  x, y, w, l, theta(角度), z,    h,       label, speed, id, score, x_min,y_min,x_max,y_max, data_source, data_channel
    xt::xarray<float> track_box;            // 数据预处理后的目标信息：shape:(m, )  x, y, z, w，   l,             h, theta(角度), speed
    xt::xarray<float> younger_track_box;    // 数据预处理后的目标信息（新跟踪目标）：shape:(m, )  x, y, z, w，   l,             h, theta(角度), speed
    xt::xarray<float> elder_track_box;      // 数据预处理后的目标信息（旧跟踪目标）：shape:(m, )  x, y, z, w，   l,             h, theta(角度), speed

    // // 目标向量（需要外面传进来）
    // std::vector<FUKalmanBoxTracker> m_trackers;
    nlohmann::json *m_fusion_parameter = nullptr;
    std::unordered_map<int, std::pair<int, float>> *m_chedao = nullptr;
    fusion_match_out_ServiceArea *m_associate_out = nullptr;
    cal_dis_result_ServiceArea *m_cal_dis_result = nullptr;

    int m_height;
    int m_width;

    std::vector<bool> m_service_area_BaseStation_select = {true, false, false}; //三个基站

public:
    Process_lidar_ServiceArea(nlohmann::json *parameter, 
                              std::unordered_map<int, 
                              std::pair<int, float>>  *chedao, 
                              fusion_match_out_ServiceArea *match_out, 
                              cal_dis_result_ServiceArea *cal_dis_result, 
                              float x_limit[], 
                              float y_limit[]);

    ~Process_lidar_ServiceArea();

    // 雷达检测数据预处理
    void process_lidar_data(xt::xarray<float> *p_pPcResult);

    // 执行关联匹配
    void excute_match_lidar_stage(std::vector<FUKalmanBoxTracker_ServiceArea> &_trackers, 
                                  IouAssociate_ServiceArea *iou_associate, 
                                  DistanceAssociate_ServiceArea *dis_associate);

    void excute_match_lidar_stage_3rd_match(std::vector<FUKalmanBoxTracker_ServiceArea>     &trackers, 
                                            IouAssociate_ServiceArea                        *iou_associate, 
                                            DistanceAssociate_ServiceArea                   *dis_associate);


    // 更新类别
    void update_type(std::vector<FUKalmanBoxTracker_ServiceArea> &_trackers, 
                     unsigned long pc_timestamp);
};



#endif // PROCESS_LIDAR_SERVICEAREA_H










