/*******************************************************
 文件名：process_camera.h
 作者：
 描述：图像检测数据和轨迹关联匹配
 版本：v1.0
 日期：2023-4-6
 *******************************************************/




#ifndef PROCESS_CAMERA_H
#define PROCESS_CAMERA_H


#include "TVideoResult.h"

#include "BaseAssociate.h"
#include "FUKalmanBoxTracker.h"
// #include "coordinate_map.h"
#include "FunctionHub.h"

#include "IouAssociate.h"
#include "DistanceAssociate.h"

#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include "json.hpp"

class Process_camera
{
private:
    /* data */

    // int pc_timestamp // 雷达时间戳
    xt::xarray<float> camera_box; // 数据预处理后的检测信息：shape:(n, )  数据预处理后的检测信息：shape:(n, )  x, y, w, l, theta(角度), z,    h,       label, speed, id, score, x_min,y_min,x_max,y_max, data_source, data_channel
    xt::xarray<float> track_box; // 数据预处理后的目标信息：shape:(m, )   // x, y, z, w，   l,  h,   theta, id
    xt::xarray<float> camera_box_all_channel;
    // // 目标向量（需要外面传进来）
    // std::vector<FUKalmanBoxTracker> _trackers;

    nlohmann::json *m_fusion_parameter;
    std::unordered_map<int, std::pair<int, float>> *m_chedao;

    fusion_match_out *m_associate_out = nullptr;
    cal_dis_result *m_cal_dis_result = nullptr;

    xt::xarray<int> m_camera_reflect_limit;
    
    int m_height;
    int m_width;

public:
    Process_camera(nlohmann::json *parameter, std::unordered_map<int, std::pair<int, float>> *chedao, fusion_match_out *match_out, cal_dis_result *cal_dis_result, float x_limit[], float y_limit[]); // TSelfFusionAlgParam这里里面要包含每一路相机的内外参
    ~Process_camera();

    // 多路相机检测数据预处理，
    void process_camera_data(xt::xarray<float> *CameraResult);

    // 执行关联匹配
    void excute_match_camera_stage(std::vector<FUKalmanBoxTracker> &_trackers, IouAssociate *iou_associate, DistanceAssociate *dis_associate);

    // 更新类别
    void update_type(std::vector<FUKalmanBoxTracker> &_trackers, unsigned long timestamp);

    // 得到相机跟踪后的图像检测信息,用以显示。
    void getCameraBoxInfo(std::vector<xt::xarray<float>> *_p_pCameraResult);
    

private:

    // 加载csv文件，
    void load_csv(std::string _file_path); 


};
#endif