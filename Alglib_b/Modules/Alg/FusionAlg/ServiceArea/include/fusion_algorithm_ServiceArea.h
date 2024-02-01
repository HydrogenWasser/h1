/********
 文件名：fusion_algorithm_ServiceArea.h
 作者：Huahuan
 描述：融合算法主函数
 版本：v1.0
 日期：2024.01.10
 *******/

#ifndef FUSION_ALG_FUSION_ALGORITHM_SERVICEAREA_H
#define FUSION_ALG_FUSION_ALGORITHM_SERVICEAREA_H
#include <iostream>
#include <algorithm>
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xview.hpp>

#include "TPcResult_ServiceArea.h"
#include "TVideoResult_ServiceArea.h"
#include "TSelfFusionAlgParam.h"
#include "IFusionAlgBase.h"
#include "FusionTrackAlg_ServiceArea.h"


class Fusion_Algorithm_ServiceArea : public IFusionAlgBase {
public:
    Fusion_Algorithm_ServiceArea(TSelfFusionAlgParam *m_stPcAlgParam);
    ~Fusion_Algorithm_ServiceArea();

    Fusion_Res RUN(std::vector<xt::xarray<float>>   &video_data, 
                   xt::xarray<float>                &pc_data, 
                   int                              cam_num);

private:
    Fusion_Res _output;
     class FusionTrackAlg_ServiceArea *m_FusionTrackAlg;
};

#endif // FUSION_ALG_FUSION_ALGORITHM_ServiceArea_H