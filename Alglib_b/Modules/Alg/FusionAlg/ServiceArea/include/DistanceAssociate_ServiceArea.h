#ifndef DISTANCEASSOCIATE_SERVICEAREA_H
#define DISTANCEASSOCIATE_SERVICEAREA_H

#include "json_ServiceArea.hpp"
#include "BaseAssociate_ServiceArea.h"


class DistanceAssociate_ServiceArea : public BaseAssociate_ServiceArea
{
private:
    xt::xarray<float> m_detections; // 关联匹配之前处理后的检测数据 
    xt::xarray<float> m_trackers;   // 关联匹配之前处理后的轨迹数据  

    cal_dis_result_ServiceArea* m_cal_dis_result = nullptr;

    int m_flag;  // 是雷达距离匹配，还是相机距离匹配

    xt::xarray<int> quadrant; //dets相对于trks的象限

    std::vector<int> motor_vehicle_labels;
    
    void parse_json(nlohmann::json *parameter);

public:
    DistanceAssociate_ServiceArea(cal_dis_result_ServiceArea *_cal_dis_result);
    ~DistanceAssociate_ServiceArea();

    virtual void pre_process_associate_data(xt::xarray<float> *_dets, 
                                            xt::xarray<float> *_trks, 
                                            int flag, 
                                            nlohmann::json *parameter) 
                                            override;

    // 计算代价矩阵
    virtual void cal_cost_matrix(xt::xarray<float> &cost_matrix) override;

    // 匈牙利匹配
    virtual void execute_match(xt::xarray<float> _cost_matrix, 
                               fusion_match_out_ServiceArea *associate_out, 
                               nlohmann::json *parameter) 
                               override;

    // 后处理
    virtual void post_process_associate_data(fusion_match_out_ServiceArea *associate_out, 
                                             xt::xarray<float> *_dets, 
                                             int flag, 
                                             nlohmann::json *parameter) 
                                             override;

    virtual void post_process_associate_data_third(fusion_match_out_ServiceArea *associate_out, 
                                                   xt::xarray<float>            *_dets, 
                                                   int                          flag, 
                                                   nlohmann::json               *parameter, 
                                                   std::vector<int>             *vaild_elder, 
                                                   std::vector<int>             *vaild_younger);


private:
    // std::vector<bool> m_service_area_BaseStation_select;                     // 基站号service_area_classification = {true, flase, false}
    std::map<std::string, std::vector<float>> m_blind_area;                     // 各基站的盲区
};














#endif // DISTANCEASSOCIATE_SERVICEAREA_H