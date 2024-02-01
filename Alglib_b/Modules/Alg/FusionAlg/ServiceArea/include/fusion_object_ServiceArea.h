/********
 文件名：fusion_object_ServiceArea.h
 作者：Huahuan
 描述：更新目标以及平滑后处理
 版本：v1.0
 日期：2024.01.10
 *******/

#ifndef FUSION_OBJECT_SERVICEAREA_H
#define FUSION_OBJECT_SERVICEAREA_H

#include <map>
#include <xtensor/xarray.hpp>


class TypeFusion_ServiceArea{
public:
    TypeFusion_ServiceArea(xt::xarray<float> bbox);
    TypeFusion_ServiceArea(){};
    virtual ~TypeFusion_ServiceArea(){};
    
    void updateType(xt::xarray<float> bbox,
                    int src = 2);
    
    std::map<std::string, int> _framework;
    xt::xarray<float> mass_A;
    xt::xarray<float> mass_B;
    int fusion_label;
    float fusion_score;
    xt::xarray<float> _bbox;
    float _dis_thresh;
    float _intersection_center[2]; // 服务区没有这玩意儿但是先不删了
};
#endif // FUSION_OBJECT_SERVICEAREA_H