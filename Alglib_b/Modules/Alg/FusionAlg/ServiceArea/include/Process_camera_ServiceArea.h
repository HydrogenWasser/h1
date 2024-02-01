/*******************************************************
 文件名：process_camera_ServiceArea.h
 作者：HuaHuan
 描述：图像检测数据和轨迹关联匹配
 版本：v1.0
 日期：2024-01-11
 *******************************************************/


#ifndef PROCESS_CEMERA_SERVICEAREA_H
#define PROCESS_CEMERA_SERVICEAREA_H

#include "TVideoResult.h"
#include "BaseAssociate_ServiceArea.h"
#include "FUKalmanBoxTracker_ServiceArea.h"
#include "FunctionHub_ServiceArea.h"
#include "IouAssociate_ServiceArea.h"
#include "DistanceAssociate_ServiceArea.h"

#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include "json_ServiceArea.hpp"


class Process_camera_ServiceArea
{
private:
    xt::xarray<float> camera_box; // 数据预处理后的检测信息，shape: (n, )
    // x, y, w, l, theta, z, h, label, speed, id, score,
    // x_min,y_min,x_max,y_max, data_source, data_channel
    xt::xarray<float> track_box; // 数据预处理后的目标信息：shape:(m, )
    // x, y, z, w, l, h, theta, id

    nlohmann::json *m_fusion_parameter;
    std::unordered_map<int, std::pair<int, float>> *m_chedao;
    fusion_match_out_ServiceArea *m_associate_out = nullptr;
    cal_dis_result_ServiceArea *m_cal_dis_result = nullptr;

    xt::xarray<int> m_camera_reflect_limit;

    int m_height;
    int m_width;

public:
    Process_camera_ServiceArea(nlohmann::json *parameter,
                   std::unordered_map<int, std::pair<int, float>> *chedao,
                   fusion_match_out_ServiceArea *match_out,
                   cal_dis_result_ServiceArea *cal_dis_result,
                   float x_limit[],
                   float y_limit[]); // TSelfFusionAlgParam这里里面要包含每一路相机的内外参

    ~Process_camera_ServiceArea();

    // 多路相机检测数据预处理
    void process_camera_data(std::vector<xt::xarray<float>> *camera_result);

    // 执行关联匹配
    void excute_match_camera_stage(std::vector<FUKalmanBoxTracker_ServiceArea> &_trakcers,
                                   IouAssociate_ServiceArea *iou_associate,
                                   DistanceAssociate_ServiceArea *dis_associate);

    // 更新类别
    void update_type(std::vector<FUKalmanBoxTracker_ServiceArea> &_trackers, 
                     unsigned long timestamp);

    // 得到相机跟踪后的图像检测信息，用以显示
    void getCameraBoxInfo(std::vector<xt::xarray<float>> *_p_pCameraResult);

private:
    // 加载csv文件
    void load_csv(std::string _file_path);
};




#endif // PROCESS_CEMERA_SERVICEAREA_H