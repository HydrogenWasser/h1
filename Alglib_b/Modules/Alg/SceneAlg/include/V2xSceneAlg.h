#ifndef V2XSCENEALG_H
#define V2XSCENEALG_H
#include "ISceneAlgBase.h"
#include "json.hpp"

#include "xtensor.hpp"

// class FUKalmanBoxTracker;

class V2xSceneAlg : public ISceneAlgBase
{
public:
    V2xSceneAlg(nlohmann::json *fusion_param);
    ~V2xSceneAlg();
    Fusion_Res_scene RUN(TFusionTrackResult *fusion_track_result);



private:
    // int process(std::vector<FUKalmanBoxTracker> *trackers, xt::xarray<float> fusion_track_result, xt::xarray<float> *output_track_result);
    int process(xt::xarray<float> *result);
    // class转label
    int Class2label(const std::string &p_strClass, nlohmann::json *m_fusion_parameter);

    void parse_json();

private:
    TFusionTrackResult m_fusion_result;

    Fusion_Res_scene m_output;

    nlohmann::json *m_fusion_param = nullptr;

    // id : {time_since_update, hits}
    std::unordered_map<unsigned short, std::vector<long int>> id_about_life;  

    int max_age;
    int max_age_new;
    int min_hits;

    std::vector<int> motor_vehicle_labels;
};

#endif