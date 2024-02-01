/*******************************************************
 文件名：FunctionHub_ServiceArea.h
 作者：HuaHuan
 描述：基本的通用函数库
 版本：v1.0
 日期：2024-01-11
 *******************************************************/
#ifndef FUNCTIONHUB_SERVICEAREA_H
#define FUNCTIONHUB_SERVICEAREA_H

#include <xtensor/xnpy.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xrepeat.hpp>
#include <xtensor/xstrides.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <set>
#include <numeric>

#include "FUKalmanBoxTracker_ServiceArea.h"
#include "json_ServiceArea.hpp"

#define None -10000.1
#define PI 3.1415926

namespace FunctionHub_ServiceArea
{
    xt::xarray<float> corners_nd(xt::xarray<float> &dims, 
                                 float origin = 0.5);

    xt::xarray<float> rotation_2d(xt::xarray<float> &corners, 
                                  const xt::xarray<float> &angles);

    xt::xarray<float> center_to_corner_box2d(xt::xarray<float> &centers, 
                                             xt::xarray<float> &dims,
                                             xt::xarray<float> &angles, 
                                             float origin = 0.5);

    xt::xarray<float> center_to_corner_box3d(xt::xarray<float> &centers, 
                                             xt::xarray<float> &dims,
                                             float &angles, 
                                             float origin = 0.5, 
                                             int axis = 2);

    void rotation_3d_in_axis(xt::xarray<float> &points, 
                             float &angles, 
                             int axis = 0);

    xt::xarray<float> corner_to_standup_nd(xt::xarray<float> &boxes_corner);

    std::pair<xt::xarray<float>, xt::xarray<float>> rotate_nms_cc(xt::xarray<float> &dets,
                                                                  xt::xarray<float> &trackers);

    float cal_angle(std::vector<xt::xarray<float>> &state_list, 
                    float &thresh);

    std::pair<float, bool> cal_angle_new(FUKalmanBoxTracker_ServiceArea *tracker,
                                         nlohmann::json *m_fusion_parameter);

    std::pair<xt::xarray<float>, xt::xarray<float>> iou_jit_new(xt::xarray<float> &boxes,
                                                                xt::xarray<float> &query_boxes,
                                                                float eps = 0.0);

    float IOU_calc(xt::xarray<float> box1, 
                   xt::xarray<float> box2, 
                   bool wh = false);

    int pcClass2label(const std::string &p_strClass, 
                      nlohmann::json *m_fusion_parameter);

    int videoClass2label(const std::string &p_strClass, 
                         nlohmann::json *m_fusion_parameter);

    // 距离计算方法，传入引用，在被调用之前，先定三个距离矩阵
    void cal_distance(xt::xarray<float> &detections, 
                      xt::xarray<float> &trackers, 
                      xt::xarray<float> &head_dis_new, 
                      xt::xarray<float> &ano_dis_new, 
                      xt::xarray<float> &square_dis, 
                      int flag);
    
    // nms
    void result_nms(xt::xarray<float> &result, 
                    std::vector<int> &delete_index, 
                    float nms_min_threshold, 
                    float nms_max_threshold, 
                    bool flag = true);

    // 计算目标对应在车道图上的索引
    std::pair<int, int> get_pixel_location_xc(float x, 
                                              float y, 
                                              int width, 
                                              int height, 
                                              float x_limit[], 
                                              float y_limit[]);
    // 获得车道信息，包括车道号和车道角
    std::pair<int, float> get_lane_info(float x, 
                                        float y, 
                                        int width, 
                                        int height, 
                                        float x_limit[], 
                                        float y_limit[], 
                                        std::unordered_map<int, std::pair<int, float>>  *chedao);

    //  轨迹航向角平滑
    void smooth_angle(FUKalmanBoxTracker_ServiceArea *tracker, 
                      std::vector<int> *motor_vehicle_labels);

    // 检测位置平滑
    // void smooth_xy(FUKalmanBoxTracker *tracker, float dt, std::vector<int> *motor_vehicle_labels_without_car, std::vector<int> *motor_vehicle_labels);
    
    std::map<int, float> neighbor_search(xt::xarray<float>  &trackers, 
                                         int                idx, 
                                         float              search_radius, 
                                         nlohmann::json     *m_fusion_parameter);

    template <typename T>
    std::pair<float, float> cal_mean_sin_cos_with_weight(T his_angle)
    {
        float m_sin = 0,
              m_cos = 0;
        
        for (int i=0; i < his_angle.size(); i ++)
        {
            float rad = his_angle[i] * PI / 180;
            m_sin += sin(rad) * (i + 1);
            m_cos += cos(rad) * (i + 1);
        }

        float weight = (his_angle.size() + 1) * his_angle.size() / 2;
        std::pair<float, float> res(m_sin / weight, m_cos / weight);

        return res;
    }

    template <typename T>
    float cal_mean_angle_with_weight(T his_angle)
    {
        auto m_res = cal_mean_sin_cos_with_weight<T>(his_angle);

        float mean_angle = atan(m_res.first / m_res.second) * (180 / PI);
        
        if (m_res.second < 0)
            mean_angle = mean_angle + 180;
        
        if (mean_angle < 0)
            mean_angle += 360;

        return mean_angle;
    }
    
    

    template <typename T>
    auto cal_mean_container(T container)
    {
        return accumulate(container.begin(), container.end(), 0.0) / container.size();
    }

}


#endif // FUNCTIONHUB_SERVICEAREA_H