/********
 文件名：FusionTrackAlg_ServiceArea.h
 作者：Huahuan
 描述：存储轨迹、预测
 版本：v1.0
 日期：2024.01.10
 *******/

#ifndef FUSIONTRACKALG_H
#define FUSIONTRACKALG_H

#include "IFusionTrackAlg_ServiceArea.h"
#include "FUKalmanBoxTracker_ServiceArea.h"
#include "Process_lidar_ServiceArea.h"
#include "Process_camera_ServiceArea.h"
#include "Update_tracks_ServiceArea.h"
#include "IouAssociate_ServiceArea.h"
#include "DistanceAssociate_ServiceArea.h"
#include "json_ServiceArea.hpp"


struct FusionTrackResult_ServiceArea
{
    /* data*/
    float x;
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
    bool isMoving;
};

class FusionTrackAlg_ServiceArea : public IFusionTrackAlg_ServiceArea
{    
public:
    FusionTrackAlg_ServiceArea();
    ~FusionTrackAlg_ServiceArea();

    virtual int predict(unsigned long timestamp) override;

    virtual int set_lidar_data(xt::xarray<float>    *pc_result,
                               unsigned long        pc_timestamp)
                               override;

    virtual int set_camera_data(std::vector<xt::xarray<float>>  *camera_result,
                                std::vector<unsigned long>      camera_timestamp)
                                override;

    virtual int process(unsigned long       timestamp,
                        xt::xarray<float>   *fusion_track_result,
                        std::map<int, int>   parking_number)
                        override;

    virtual std::vector<FUKalmanBoxTracker_ServiceArea> get_trackers() override;

    // 在cpp文件中new process_camera，process_camera,update_tracks, process_output四个对象并保存在成员变量中
    FusionTrackAlg_ServiceArea(nlohmann::json *fusion_parameter);

private:
    // 执行process_lidar关联匹配流程
    int match_lidar(xt::xarray<float>   *_p_pPcResult, 
                    unsigned long       pc_timestamp);

    // 执行process_camera关联匹配流程
    int match_camera(std::vector<xt::xarray<float>> *p_pCameraResult, 
                     std::vector<unsigned long>     listCameraTime);

    // 融合轨迹、维护轨迹（类别更新、生成轨迹、位置更新、航向角更新、删除轨迹）
    void fusion_track(unsigned long         timestamp,
                      xt::xarray<float>     *fusion_track_result,
                      std::map<int, int>    parking_number);

    // 加载车道航向角
    void load_chedao();

    // 融合结果后处理
    // std::vector<fusion_output_ServiceArea> output();

private:
    
    std::vector<FUKalmanBoxTracker_ServiceArea> m_trackers; // 存储所有的轨迹
    nlohmann::json *m_fusion_parameter = nullptr;
    std::unordered_map<int, std::pair<int, float>> m_chedao;
    Process_camera_ServiceArea *m_camera_handler = nullptr;
    Process_lidar_ServiceArea *m_lidar_handler = nullptr;
    class Update_tracks_ServiceArea *m_update_handler = nullptr;
    fusion_match_out_ServiceArea *m_match_out = nullptr;
    cal_dis_result_ServiceArea *m_cal_dis_result = nullptr;
    IouAssociate_ServiceArea *m_iouassociate_handler = nullptr;
    DistanceAssociate_ServiceArea *m_disassociate_handler = nullptr;

    unsigned long last_timestamp = 0;
    float dt;
    float m_xlimit[2], m_ylimit[2]; // 车道限制


    
};


#endif