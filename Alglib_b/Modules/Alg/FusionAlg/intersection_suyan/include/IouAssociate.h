#ifndef IOUASSOCIATE_H
#define IOUASSOCIATE_H


#include "BaseAssociate.h"

// #include <xtensor/xarray.hpp>
// #include <xtensor/xadapt.hpp>
// #include <xtensor/xnpy.hpp>
// #include <xtensor/xtensor.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>

class IouAssociate : public BaseAssociate
{
private:
    /* data */
    xt::xarray<float> m_detections; // 关联匹配之前处理后的检测数据  // x, y, w, l, theta(角度)； x, y, theta(角度) ---距离
    xt::xarray<float> m_trackers;   // 关联匹配之前处理后的目标数据  // x, y, w, l, theta(角度)； x, y, theta(角度) ---距离
    int m_flag;

public:
    IouAssociate(/* args */);
    ~IouAssociate();

    virtual void pre_process_associate_data(xt::xarray<float> *_dets, xt::xarray<float> *_trks, int flag, nlohmann::json *parameter) override;

    // 计算代价矩阵
    virtual void cal_cost_matrix(xt::xarray<float> &cost_matrix) override;

    // 计算代价矩阵
    void cal_cost_matrix_new(xt::xarray<float> &cost_matrix, nlohmann::json *parameter);

    // 匈牙利匹配
    virtual void execute_match(xt::xarray<float> _cost_matrix, fusion_match_out *associate_out, nlohmann::json *parameter) override;

    // 后处理
    virtual void post_process_associate_data(fusion_match_out *associate_out, xt::xarray<float> *_dets, int flag, nlohmann::json *parameter) override;

private:
    // 角度转弧度，以及转换航向角坐标系
    void adjust_boxes_iou_bev_input(xt::xarray<float> &origin_box);


};
#endif