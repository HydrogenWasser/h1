/*******************************************************
 文件名：BaseAssociate_ServiceArea.h
 作者：HuaHuan
 描述：关联匹配算法接口（基于此实现IOU关联匹配和距离关联匹配）
 版本：v1.0
 日期：2024-01-11
  *******************************************************/

#ifndef BASEASSOCIATE_SERVICEAREA_H
#define BASEASSOCIATE_SERVICEAREA_H

#include "hungarian_fusion_ServiceArea.h"
#include "FUKalmanBoxTracker_ServiceArea.h"
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include "json_ServiceArea.hpp"


struct fusion_match_out_ServiceArea
{
    xt::xarray<int> _matched_indices; // 匈牙利匹配结果
    xt::xarray<float> _cost_matrix; // 损失矩阵
    
    std::vector<int> _unmatched_detections_indices; // 未匹配上的检测索引
    std::vector<int> _unmatched_trackers_indices;  // 未匹配上的目标索引

    std::vector<int> _matched_detections_indices;  // 匹配上的检测索引
    std::vector<int> _matched_trackers_indices;  // 匹配上的目标索引

    xt::xarray<float> _unmatched_detections; // 未匹配上的检测数据（前处理过的）
    xt::xarray<float> _unmatched_trackers;  // 未匹配到的轨迹数据

    std::vector<int> _matched_all_trks_id; // s相机跟踪模块匹配到的轨迹id
};


struct cal_dis_result_ServiceArea
{
    xt::xarray<float> square_dis = xt::empty<float>({0, 0});   // 欧式距离
    xt::xarray<float> head_dis = xt::empty<float>({0, 0});  // 纵向距离
    xt::xarray<float> ano_dis = xt::empty<float>({0, 0});  // 横向距离
    xt::xarray<float> head_dis_thres_matrix = xt::empty<float>({0, 0});  // 纵向距离阈值矩阵
    xt::xarray<float> ano_dis_thres_matrix = xt::empty<float>({0, 0});   // 横向距离阈值矩阵

};


class BaseAssociate_ServiceArea
{
private:
    xt::xarray<float> _dets; // 检测数据
    xt::xarray<float> _trks; // 目标数据

public:
    BaseAssociate_ServiceArea(){};
    ~BaseAssociate_ServiceArea(){};

    // 关联之前数据预处理
    virtual void pre_process_associate_data(xt::xarray<float> *_dets,
                                            xt::xarray<float> *_trks,
                                            int flag,
                                            nlohmann::json *parameter) = 0;
    
    // 计算代价矩阵
    virtual void cal_cost_matrix(xt::xarray<float> &cost_matrix) = 0;
    
    // 匈牙利匹配
    virtual void execute_match(xt::xarray<float> _cost_matrix,
                               fusion_match_out_ServiceArea *associate_out,
                               nlohmann::json *parameter) = 0;

    // 后处理
    virtual void post_process_associate_data(fusion_match_out_ServiceArea *associate_out,
                                             xt::xarray<float> *_dets,
                                             int flag,
                                             nlohmann::json *parameter) = 0;

};
















#endif // BASEASSOCIATE_SERVICEAREA_H