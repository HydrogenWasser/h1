#include "TPcResult.h"
#include "TVideoResult.h"
#include "xtensor.hpp"
#include <json.hpp>
class FUKalmanBoxTracker;

struct FusionTrackResult;

class IFusionTrackAlg
{
public:
    IFusionTrackAlg(){};
    virtual ~IFusionTrackAlg(){};

    // virtual bool init_parameter(json &parameter) = 0;
    virtual int predict(unsigned long timestamp) = 0;
    virtual int set_lidar_data(xt::xarray<float> *pc_result, unsigned long pc_timestamp) = 0;
    virtual int set_camera_data(std::vector<xt::xarray<float>> *camera_result, std::vector<unsigned long> camera_time) = 0;

    virtual int process(unsigned long timestamp, xt::xarray<float> *fusion_track_result) = 0;

    virtual std::vector<FUKalmanBoxTracker> get_trackers() = 0;
    
};