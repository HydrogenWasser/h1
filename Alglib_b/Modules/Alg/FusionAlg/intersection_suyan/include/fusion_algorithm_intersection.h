//
// Created by root on 5/8/21.
//

#ifndef FUSION_ALG_FUSION_ALGORITHM_INTERSECTION_H
#define FUSION_ALG_FUSION_ALGORITHM_INTERSECTION_H
#include <iostream>
#include <algorithm>
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

// #include "coordinate_map_tunnel.h"

#include "TPcResult.h"
#include "TVideoResult.h"
// #include "box_ops_tunnel.h"
#include "TSelfFusionAlgParam.h"
#include "IFusionAlgBase.h"
// #include "yaml-cpp/yaml.h"
// #include "video_utils_tunnel.h"


#include "FusionTrackAlg.h"




class Fusion_Algorithm_Intersection : public IFusionAlgBase {
public:
    Fusion_Algorithm_Intersection(TSelfFusionAlgParam *m_stAlgParam);
    // Fusion_Algorithm_Intersection();
    ~Fusion_Algorithm_Intersection();
    Fusion_Res RUN(std::vector<xt::xarray<float>> &video, xt::xarray<float> &data, int cam_num);
    // Fusion_Res RUN(std::vector<xt::xarray<float>> &video, xt::xarray<float> &data, int cam_num,unsigned long timestamp);
    // Fusion_Res RUN(TPcResult *p_pPcResult, TVideoResult *p_pVideoResult);

private:

    Fusion_Res _output;
    FusionTrackAlg *m_FusionTrackAlg;
};
    


#endif //FUSION_ALG_FUSION_ALGORITHM_INTERSECTION_H
