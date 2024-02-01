/*******************************************************
 文件名：FusionTrackAlg.h
 作者：
 描述：存储轨迹，predict，
 版本：v1.0
 日期：2023-4-4
 *******************************************************/

#ifndef FUSIONTRACKALG_H
#define FUSIONTRACKALG_H

#include "IFusionTrackAlg.h"
// class BaseAssociate;
// class Process_lidar;
// class Process_camera;
// class Update_tracks;
// class FUKalmanBoxTracker;
// struct match_out;
// class IouAssociate;

#include "FUKalmanBoxTracker.h"
// #include "BaseAssociate.h"
#include "Process_lidar.h"
#include "Process_camera.h"
#include "Update_tracks.h"
// #include "Process_output.h"
// #include "FunctionHub.h"
// struct match_out;

#include "IouAssociate.h"
#include "DistanceAssociate.h"
#include "json.hpp"

// 融合结果输出数据： [x, y, w, l, theta, z, h, label, speed, id, scores, x_min, y_min, x_max, y_max, data_source, channel, lane,occur_time]
struct FusionTrackResult
{
    /* data */
    float x; //
    float y;
    float w;
    float l;
    float theta;
    float z;
    float h;
    int label;
    float speed;
    int id;
    float score;
    int x_min;
    int y_min;
    int x_max;
    int y_max;
    int data_score;
    int channel;
    int lane;
    float occur_time;
};

class FusionTrackAlg : public IFusionTrackAlg
{

public:
    FusionTrackAlg();
    ~FusionTrackAlg();

    virtual int predict(unsigned long timestamp) override;
    // virtual bool init_parameter(json &parameter) override;

    virtual int set_lidar_data(xt::xarray<float> *pc_result, unsigned long pc_timestamp) override;

    virtual int set_camera_data(std::vector<xt::xarray<float>> *camera_result, std::vector<unsigned long> camera_time) override;

    virtual int process(unsigned long timestamp, xt::xarray<float> *fusion_track_result) override;

    virtual std::vector<FUKalmanBoxTracker>  get_trackers() override;

    // 在cpp文件中new process_camera，process_camera,update_tracks, process_output四个对象并保存在成员变量中
    FusionTrackAlg(nlohmann::json *fusion_parameter);

private:
    // // 对所有的track进行kalman预测
    // void predict();

    // 执行process_lidar关联匹配流程
    int match_lidar(xt::xarray<float>  *_p_pPcResult, unsigned long pc_timestamp);

    // 执行process_camera关联匹配流程
    int match_camera(std::vector<xt::xarray<float>> *p_pCameraResult, std::vector<unsigned long> listCameraTime);

    // 融合轨迹，维护轨迹（类别更新，生成轨迹，位置更新，航向角更新，删除轨迹）
    void fusion_track(unsigned long timestamp, xt::xarray<float> *fusion_track_result);

    // 加载车道航向角
    void load_chedao();

    // // 融合结果后处理
    // std::vector<fusion_output> output();

private:
    std::vector<FUKalmanBoxTracker> m_trackers; // 存储所有的轨迹

    nlohmann::json *m_fusion_parameter = nullptr; // 融合算法参数指针

    std::unordered_map<int, std::pair<int, float>> m_chedao; // 车道航向角

    Process_camera *m_camera_handler = nullptr; // process_camera对象指针
    Process_lidar *m_lidar_handler = nullptr;   // process_lidar对象指针
    Update_tracks *m_update_handler = nullptr;  // update_tracks对象指针

    fusion_match_out *m_match_out = nullptr; // 匹配结果对象指针
    cal_dis_result* m_cal_dis_result = nullptr; // 距离计算结构体指针

    IouAssociate *m_iouassociate_handler = nullptr; // Iou匹配对象指针
    DistanceAssociate *m_disassociate_handler = nullptr; // dis匹配对象指针

    unsigned long last_timestamp = 0;

    float dt;

    float m_xlimit[2], m_ylimit[2];   // 车道限制

    

    // xt::xarray<float> *m_fusion_track_trsult = nullptr; // 发送出去的融合跟踪结果

};
#endif
