/*******************************************************
 文件名：IouAssociate_ServiceArea.h
 作者：HuaHuan
 描述：Iou匹配
 版本：v1.0
 日期：2023-4-6
 *******************************************************/
#ifndef IOUASSOCIATE_SERVICEAREA_H
#define IOUASSOCIATE_SERVICEAREA_H

#include "BaseAssociate_ServiceArea.h"


class IouAssociate_ServiceArea : public BaseAssociate_ServiceArea
{
private:
    xt::xarray<float> m_detections;
    xt::xarray<float> m_trackers;
    int m_flag;
public:
    IouAssociate_ServiceArea();
    ~IouAssociate_ServiceArea();

    virtual void pre_process_associate_data(xt::xarray<float> *_dets,
                                            xt::xarray<float> *_trks, 
                                            int flag, nlohmann::json *parameter) 
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

    

private:
    // 角度转弧度，以及转换航向角坐标系
    void adjust_boxes_iou_bev_input(xt::xarray<float> &origin_box);

};


#endif // 