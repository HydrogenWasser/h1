/*******************************************************
 文件名：update_tracks_ServiceArea.h
 作者：HuaHuan
 描述：匹配关联之后，需要对轨迹进行位置更新和类别更新
 版本：v1.0
 日期：2024-01-11
 *******************************************************/
#ifndef UPDATE_TRACKS_SERVICEAREA_H
#define UPDATE_TRACKS_SERVICEAREA_H

#include <numeric>

#include "BaseAssociate_ServiceArea.h"
#include "FunctionHub_ServiceArea.h"
#include "json_ServiceArea.hpp"
#include "FusionTrackAlg_ServiceArea.h"


// struct FusionTrackResult_ServiceArea;

class Update_tracks_ServiceArea
{
private:
    /* data */
    nlohmann::json *m_fusion_parameter;
    std::unordered_map<int, std::pair<int, float>>  *m_chedao;
    // xt::xarray<float> *m_fusion_track_trsult;

    std::vector<int> camera_create_trackers;

    std::vector<int> motor_vehicle_labels;

    int m_height;   // 车道角覆盖范围准换到像素大小
    int m_width;

    float *m_x_limit;
    float *m_y_limit;

public:
    Update_tracks_ServiceArea(nlohmann::json                                    *parameter, 
                              std::unordered_map<int, std::pair<int, float>>    *chedao, 
                              float                                             x_limit[], 
                              float                                             y_limit[]);

    ~Update_tracks_ServiceArea(){};

    // // 更新类别
    // void update_type(match_out* match_out, std::vector<FUKalmanBoxTracker>* _m_trackers);

    // 生成新的轨迹
    void create_tracks(fusion_match_out_ServiceArea                 *match_out, 
                       std::vector<FUKalmanBoxTracker_ServiceArea>  &_m_trackers, 
                       unsigned long                                timestamp, 
                       int                                          flag);

    // 轨迹更新（位置平滑和更新，航向角平滑，轨迹删除）
    void update(std::vector<FUKalmanBoxTracker_ServiceArea> &_m_trackers, 
                xt::xarray<float>                           *fusion_track_result, 
                unsigned long                               timestamp, 
                float                                       dt,
                std::map<int, int>                         parking_space_number);


private:
    void parse_json();



private:
    // ******************************** 融合跟踪文件参数 ******************************** //
    int m_mean_xy_num;                                      // 判断运动状态时需要的历史数据量大小
    int m_driving_frame_num;                                // 这个和上一个应该是一样
    float m_smooth_whipping_xy;                             // 用于判断抖动的最小位移距离
    float m_parking_speed_limit;                            // 用于判断运动的最小速度
    int m_smooth_parking_xy;                                // 用于锁定的数据量参数
    float m_high_speed_for_changing_angle;                  // 速度大于该值时认为计算角是一定正确的
    float m_parking_angle_limit;                            // 误差角限制
    float m_limit_l_dis;                                    // l 限制参数
    // std::vector<bool> m_service_area_BaseStation_select;        // service_area_classification = {true, flase, false}
    std::map<std::string, std::vector<float>> m_blind_area;
    bool m_flag_use_chewei_angle;                           // 是否使用车位角
    std::vector<int> m_chewei_left_right_num;               // chewei_left_right_num数组
    std::vector<int> m_static_coefficient;
    // ********************************************************************************** //

    // ******************************** 融合跟踪对象参数 ******************************** //
    std::map<int, int> m_parking_space_number;
    std::vector<std::vector<float>> m_chewei_angle;  // chewei_angle = np.load(chewei_angle_file)
    std::set<int> m_special_chewei_num;                 // 特殊车位，待修改
    float angle_change_parameter[3] = {0.0, 0.0, 0.0};
    // ********************************************************************************** //

public:
    // ******************************** 服务区融合跟踪函数 ******************************** //
    FusionTrackResult_ServiceArea deal_lock_smooth(FUKalmanBoxTracker_ServiceArea& trk,         // 平滑后处理主函数
                                                   float                          dt = 0.1);                                
    
    bool judge_motion_status(FUKalmanBoxTracker_ServiceArea& trk,
                             const float&                   temp_x,                             // 返回目标的运动状态
                             const float&                   temp_y,
                             const float&                   temp_angle);

    void z_lock(FUKalmanBoxTracker_ServiceArea& trk,
                float&                         temp_z);                                         // 锁定z轴坐标

    bool deal_angle_lock(FUKalmanBoxTracker_ServiceArea& trk,
                         float&                         temp_angle, 
                         const bool                     isMoving);

    void deal_wlh_lock(FUKalmanBoxTracker_ServiceArea& trk,
                       float&                         temp_w,                                   // 尺寸锁定
                       float&                         temp_l, 
                       float&                         temp_h);

    void deal_blind(FUKalmanBoxTracker_ServiceArea& trk,
                    float                          dt,                                          // 盲区处理，盲区参数default_blind_area
                    float&                         temp_x,
                    float&                         temp_y,
                    const float                    temp_speed,
                    float&                         temp_angle,
                    std::vector<float>&            blind_area_3);       

    float modify_chewei_angle(FUKalmanBoxTracker_ServiceArea& trk,
                              float                          temp_angle, 
                              const bool                     isMoving,
                              const float&                   temp_x,
                              const float&                   temp_y);

    // ********************************************************************************** //



};
#endif // UPDATE_TRACKS_SERVICEAREA_H