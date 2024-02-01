/*******************************************************
 文件名：process_lidar.h
 作者：
 描述：雷达检测数据和轨迹关联匹配
 版本：v1.0
 日期：2023-4-4
 *******************************************************/

// #ifndef JSON_H
// #define JSON_H
// #include "json.hpp"
// using json = nlohmann::json;
// #endif


#ifndef PROCESS_LIDAR_H
#define PROCESS_LIDAR_H


#include "TPcResult.h"

#include "BaseAssociate.h"
#include "FUKalmanBoxTracker.h"
#include "IouAssociate.h"
#include "DistanceAssociate.h"

#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "json.hpp"

#define PI 3.1415926

class Process_lidar
{
private:
    /* data */
    // TPcResult m_dets;  // 雷达检测结果
    // int pc_timestamp; // 雷达时间戳
    xt::xarray<float> lidar_box; // 数据预处理后的检测信息：shape:(n, )  x, y, w, l, thetatheta(角度), z,    h,       label, speed, id, score, x_min,y_min,x_max,y_max, data_source, data_channel
    xt::xarray<float> track_box; // 数据预处理后的目标信息：shape:(m, )  x, y, z, w，   l,             h, theta(角度), speed
    xt::xarray<float> younger_track_box; // 数据预处理后的目标信息（新跟踪目标）：shape:(m, )  x, y, z, w，   l,             h, theta(角度), speed
    xt::xarray<float> elder_track_box; // 数据预处理后的目标信息（旧跟踪目标）：shape:(m, )  x, y, z, w，   l,             h, theta(角度), speed

    // // 目标向量（需要外面传进来）
    // std::vector<FUKalmanBoxTracker> m_trackers;
    nlohmann::json *m_fusion_parameter = nullptr;
    std::unordered_map<int, std::pair<int, float>> *m_chedao = nullptr;
    fusion_match_out *m_associate_out = nullptr;
    cal_dis_result *m_cal_dis_result = nullptr;

    int m_height;
    int m_width;

public:
    Process_lidar(nlohmann::json *parameter, std::unordered_map<int, std::pair<int, float>>  *chedao, fusion_match_out *match_out, cal_dis_result *cal_dis_result, float x_limit[], float y_limit[]);
    ~Process_lidar();

    // 雷达检测数据预处理
    void process_lidar_data(xt::xarray<float> *p_pPcResult);
    // void process_lidar_data(TPcResult p_pPcResult, xt::xarray<float> lidar_box, xt::xarray<float> track_box, std::vector<FUKalmanBoxTracker> _trackers);

    // 执行关联匹配
    void excute_match_lidar_stage(std::vector<FUKalmanBoxTracker> &_trackers, IouAssociate *iou_associate, DistanceAssociate *dis_associate);
    void excute_match_lidar_stage_3rd_match(std::vector<FUKalmanBoxTracker> &_trackers, IouAssociate *iou_associate, DistanceAssociate *dis_associate);
    // match_out excute_match_stage(xt::xarray<float> dets, xt::xarray<float> trks);

    // 更新类别
    void update_type(std::vector<FUKalmanBoxTracker> &_trackers, unsigned long pc_timestamp);
};
#endif