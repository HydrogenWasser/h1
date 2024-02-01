/********
 文件名：FUKalmanBoxTracker_ServiceArea.h
 作者：Huahuan
 描述：更新目标以及平滑后处理
 版本：v1.0
 日期：2024.01.10
 *******/

#include <iostream>
#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include <chrono>
#include <xtensor/xcsv.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xcontainer.hpp>
#include <xtensor/xsort.hpp>
#include <xtensor/xbuilder.hpp>

#include "json_ServiceArea.hpp"
#include "FUKalmanFilter_ServiceArea.h"
#include "fusion_object_ServiceArea.h"

// using namespace std; // ???
// using namespace chrono; // ？？？

#ifndef FUKALMANBOXTRACKER_SERVICEAREA_H
#define FUKALMANBOXTRACKER_SERVICEAREA_H



struct fustate_output_ServiceArea{
    xt::xarray<float> x;
    xt::xarray<float> bbox;
    float speed;
    std::vector<xt::xarray<float>> angle_box;
    float lefttopx;
    float lefttopy;
    float rightbottomx;
    float rightbottomy;
    float camid;
    float VideoBoxid;
};


struct state_output_ServiceArea{
    xt::xarray<float> output_x;
    xt::xarray<float> bbox;
    float speed;
};


// struct smooth_data
// {
//     /* data */
// };


class FUKalmanBoxTracker_ServiceArea{
public:
    FUKalmanBoxTracker_ServiceArea(xt::xarray<float> bbox,
                                   unsigned long timestamp,
                                   nlohmann::json *fusion_parameter,
                                   bool from_lidar = false,
                                   bool from_camera = false,
                                   bool from_radar = false);

    FUKalmanBoxTracker_ServiceArea();
    FUKalmanBoxTracker_ServiceArea(const FUKalmanBoxTracker_ServiceArea & rhs);
    ~FUKalmanBoxTracker_ServiceArea(){};

    void updatewithlidar(xt::xarray<float> &bbox, 
                         unsigned long timestamp);

    void updatewithcamera(xt::xarray<float> &bbox,
                          unsigned long timestamp);

    void predict(float dt = 0.1);
    void update(unsigned long timestamp,
                float dt = 0.1);

    state_output_ServiceArea get_state();
    static int _count;

private:
    // 服务区的一些新增函数就是在这个地方申明
    void parse_json(nlohmann::json *fusion_parameter);
    void smooth_xy(float dt);
    void update_wlh();

public:
    int number = 0;
    unsigned long occur_time = 0;
    nlohmann::json *m_fusion_parameter;

    int use_acc_model = 0; // 加速和匀速模型
    std::vector<int>  motor_vehicle_labels;
    std::vector<int>  motor_vehicle_labels_without_car;
    std::vector<int>  camera_create_trackers;
    bool camera_only_update_xy = false;
    bool get_camera_fill_blind = false; // 用相机补盲，陈集服务区应该没做？
    
    float default_blind_area[4] = {0,0,5,5}; // 盲区坐标？

    FUKalmanFilter_ServiceArea _kf;
    xt::xarray<float> _bbox;   // 一维向量
    //表示检测目标的ID
    int id = 0;
    int time_since_update = 0;
    std::vector<xt::xarray<float>> state;
    bool high_speed = false;
    bool low_speed = true;

    float lane_angle = -1000;
    float track_angle = -1000;
    float last_angle = -1000;
    float final_angle =-1000;
    bool track_angle_flag = false;
    bool first_true_angle_switch_flag = false;

    bool true_angle_switch_flag = false;
    std::vector<float> his_angle;

    float speed = 0;
    long int hits = 0;
    std::vector<long int> all_hits;
    int label = 0;
    int lane = -1000;
    float speed_thresh = 3.0;
    xt::xarray<float> update_pixel_location = {-1000, -1000, -1000, -1000};
    float lefttopx = -1000;
    float lefttopy = -1000;
    float rightbottomx = -1000;
    float rightbottomy = -1000;

    TypeFusion_ServiceArea _type_fusion;
    bool use_predict = false;
    xt::xarray<float>  predict_state;

    int lidar_updated = 0;
    int camera_updated = 0;
    int radar_updated = 0;
    int lidar_continous_updated = 0;
    unsigned long last_timestamp = 0;
    // 航向角修正标志位
    int angle_flag  = 0;
    int direction = 1;

    unsigned long lidar_timestamp;
    xt::xarray<float> last_lidar_state; // 一维向量

    unsigned long camera_timestamp;
    xt::xarray<float> last_camera_state; // 一维向量

    unsigned long radar_timestamp;
    xt::xarray<float> last_radar_state; // 一维向量

    xt::xarray<float> fusion_state; // 一维向量
    xt::xarray<float> last_fusion_state; // 一维向量

    // 服务区平滑后处理





public:
    // ******************************** 目标相关变量 ******************************** //
    // public 为了方便调用
    bool wh_locked = false, 
         l_locked = false,
         z_locked = false,
         angle_locked = false,
         coordinate_locked = false; 
    
    float mean_w = -1000,
          mean_l = -1000,
          mean_h = -1000,
          mean_z = -1000,
          angle_rechange = -1000,
          static_temp_x = -1000,
          static_temp_y = -1000,
          static_temp_angle = -1000,
          last_speed = -1000;   // 用于静止时修改角度锁定值

    std::deque<float> his_temp_w,
                      his_temp_l,
                      his_temp_h;


    float lidar_angle = -1000;      // 点云检测直接获得的角度
    float last_frame_angle = -1000;
    int chang_angle_count = 0,
        angle_change_count = 0;
    
    

    std::deque<float> his_speed;    // 存储平滑前的速度，长度应为mean_xy_num(?)
    std::deque<float> his_temp_x;   // 存储平滑前的x坐标，长度应为mean_xy_num
    std::deque<float> his_temp_y;   // 存储平滑前的y坐标，长度应为mean_xy_num
    std::deque<float> his_temp_z;   // 存储平滑前的y坐标，长度应为mean_xy_num
    std::deque<float> his_temp_angle; // c++版本把trk_angle_list和his_angle合并到一起了，十字路口版本的his_angle暂时不动，以免有冲突,合并是否合理，还需要再考虑
    std::deque<int>   his_nor_tan;  // 存储平滑前切向法向距离情况，由于vector<bool>好像不太能用干脆用int好了，后面有问题再改
    std::vector<float> smooth_parking_temp_x; // 存储平滑信息，下同
    std::vector<float> smooth_parking_temp_y;
    std::vector<float> smooth_parking_speed;
    std::vector<float> smooth_parking_angle;


    int blind_area_3_right_into;    // 盲区进入？
    // ********************************************************************************** //

    // ******************************** 融服务区合跟踪函数 ******************************** //
    void update_kalman_coefficient(const float& temp_x,
                                   const float& temp_y,
                                   const float& temp_speed,
                                   const float& temp_angle,
                                   const float& dt);
    // ********************************************************************************** //

   
};

#endif // FUKALMANBOXTRACKER_SERVICEAREA_H