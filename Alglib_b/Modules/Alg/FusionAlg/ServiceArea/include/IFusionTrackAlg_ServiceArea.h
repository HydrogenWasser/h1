/********
 文件名：IFusionTrackAlg_ServiceArea.h
 作者：Huahuan
 描述：FusionTrackAlg的父类
 版本：v1.0
 日期：2024.01.10
 *******/

#include "TPcResult_ServiceArea.h"
#include "TVideoResult_ServiceArea.h"
#include "xtensor.hpp"
#include "json_ServiceArea.hpp"

class FUKalmanBoxTracker_ServiceArea;
struct FusionTrackResult_ServiceArea;

class IFusionTrackAlg_ServiceArea
{
public:
    IFusionTrackAlg_ServiceArea(){};
    virtual ~IFusionTrackAlg_ServiceArea(){};

    virtual int predict(unsigned long timestamp)  = 0;
    
    virtual int set_lidar_data(xt::xarray<float>    *pc_result, 
                               unsigned long        pc_timestamp) = 0;

    virtual int set_camera_data(std::vector<xt::xarray<float>>  *camera_result,
                                std::vector<unsigned long>      camera_time) = 0;

    virtual int process(unsigned long           timestamp, 
                        xt::xarray<float>       *fusion_track_result,
                        std::map<int, int>     parking_number) = 0;

    virtual std::vector<FUKalmanBoxTracker_ServiceArea> get_trackers() = 0;

};

