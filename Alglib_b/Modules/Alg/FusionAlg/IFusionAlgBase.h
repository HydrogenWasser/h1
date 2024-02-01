#pragma once

#include <iostream>
#include "xtensor/xarray.hpp"

struct Fusion_Res {
    xt::xarray<float> camlidar;
    xt::xarray<float> indicss;
    xt::xarray<float> infor_list;
    std::vector<std::vector<int>> output_index;
    xt::xarray<float> lidtovid_inf;
    xt::xarray<float> output_infor;   // 融合结果
    xt::xarray<int> PeopleCount;
    xt::xarray<float> pc_res;   // 融合结果 xt::xarray<float>
    std::vector<xt::xarray<float>> video_res;   // 多路相机检测结果，  std::vector<xt::xarray<float>>  x0, y0, x1, y1, score, label, 
    std::vector<xt::xarray<float>> lidar_box_vector;  //  雷达检测框三维映射到二维角点坐标，映射到多路相机下 std::vector<xt::xarray<float>>

};

struct IFusionAlgBase
{
    IFusionAlgBase() {};
    virtual ~IFusionAlgBase() {};
    virtual Fusion_Res RUN(std::vector<xt::xarray<float>> &video, xt::xarray<float> &data, int cam_num) = 0;

    unsigned long m_pc_timestamp;
    std::vector<unsigned long> m_camera_timestamp;
    // virtual Fusion_Res RUN(TPcResult *p_pPcResult, TVideoResult *p_pVideoResult) = 0;
};
