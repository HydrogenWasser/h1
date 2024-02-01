#ifndef TwinDisplaySceneAlg_H
#define TwinDisplaySceneAlg_H


#include <unordered_map>
#include <xtensor/xview.hpp>
#include "TFusionResult.h"
#include "ISceneAlgBase.h"
#include "json_ServiceArea.hpp"
#include "xtensor.hpp"
#include "CPostProcess.h"


class TwinDisplaySceneAlg : public ISceneAlgBase
{
// ************************************ 原有成员 ***********************************
public:
    TwinDisplaySceneAlg();
    ~TwinDisplaySceneAlg();
    TwinDisplaySceneAlg(nlohmann::json *fusion_param);

    Fusion_Res_scene RUN(TFusionTrackResult *fusion_track_result);

private:
    // class 转 label
    int Class2label(const std::string   &p_strClass,
                    nlohmann::json      *m_fusion_parameter);

    // 解析json数据
    void parse_json();

private:
    Fusion_Res_scene m_output;                      // 预备发出的场景算法结果
    TFusionTrackResult m_fusion_result;             // 接收到的融合结果
    nlohmann::json *m_fusion_parameter = nullptr;
    std::vector<int> motor_vehicle_labels;
    
private:
    // ******************************** 场景参数 ******************************** //
    std::vector<int> bus_labels;
    std::vector<int> trunk_labels;
    int m_ServiceArea_select;                           // 基站号
    int m_max_age;
    int m_min_hits;
    
    // ********************************************************************************** //

    // ******************************** 场景算法参数 ******************************** //
    std::map<unsigned short, std::vector<int>> m_trackers_life;
    // ********************************************************************************** //

public:
    // ******************************** 函数 ******************************** //
    void pre_process();    // 更新存储的跟踪目标time_since_update和hits
    int process(xt::xarray<float>* scene_result);              // 场景算法流程
    // void post_process();   // 删除无效目标

    // ********************************************************************************** //
};











#endif // TwinDisplaySceneAlg_H