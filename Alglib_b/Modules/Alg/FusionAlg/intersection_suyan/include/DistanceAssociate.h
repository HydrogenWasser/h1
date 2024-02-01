#ifndef DISTANCEASSOCIATE_H
#define DISTANCEASSOCIATE_H

#include "json.hpp"

#include "BaseAssociate.h"

// #include <xtensor/xarray.hpp>
// #include <xtensor/xadapt.hpp>
// #include <xtensor/xnpy.hpp>
// #include <xtensor/xtensor.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>

class DistanceAssociate : public BaseAssociate
{
private:
    /* data */
    // m_detections: x,y,w,l,theta(角度),z,h,label,speed, id, score, x_min,y_min,x_max,y_max, data_source, data_channel  雷达距离匹配时
    // m_trackers: x, y,z,w,l,h,theta(角度),id                                                                        雷达距离匹配时
    xt::xarray<float> m_detections; // 关联匹配之前处理后的检测数据 
    xt::xarray<float> m_trackers;   // 关联匹配之前处理后的轨迹数据  

    cal_dis_result* m_cal_dis_result = nullptr;

    int m_flag;  // 是雷达距离匹配，还是相机距离匹配

    xt::xarray<int> quadrant; //dets相对于trks的象限

    std::vector<int> motor_vehicle_labels;

    void parse_json(nlohmann::json *parameter);

public:
    DistanceAssociate(cal_dis_result* _cal_dis_result, nlohmann::json *parameter);
    ~DistanceAssociate();

    virtual void pre_process_associate_data(xt::xarray<float> *_dets, xt::xarray<float> *_trks, int flag, nlohmann::json *parameter) override;

    // 计算代价矩阵
    virtual void cal_cost_matrix(xt::xarray<float> &cost_matrix) override;

    // 匈牙利匹配
    virtual void execute_match(xt::xarray<float> _cost_matrix, fusion_match_out *associate_out, nlohmann::json *parameter) override;

    // 后处理
    virtual void post_process_associate_data(fusion_match_out *associate_out, xt::xarray<float> *_dets, int flag, nlohmann::json *parameter) override;

    virtual void post_process_associate_data_third(fusion_match_out *associate_out, xt::xarray<float> *_dets, int flag, nlohmann::json *parameter, std::vector<int> *vaild_elder, std::vector<int> *vaild_younger);
};
#endif