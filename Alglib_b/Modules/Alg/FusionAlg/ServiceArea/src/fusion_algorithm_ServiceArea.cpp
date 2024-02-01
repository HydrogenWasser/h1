/********
 文件名：fusion_algorithm_ServiceArea.cpp
 作者：Huahuan
 描述：融合算法主函数
 版本：v1.0
 日期：2024.01.10
 *******/

#include "fusion_algorithm_ServiceArea.h"


Fusion_Algorithm_ServiceArea::Fusion_Algorithm_ServiceArea(TSelfFusionAlgParam *m_stAlgParam)
{   
    m_FusionTrackAlg = new FusionTrackAlg_ServiceArea(&(m_stAlgParam->m_fusion_parameter));
}


Fusion_Algorithm_ServiceArea::~Fusion_Algorithm_ServiceArea()
{
    if (m_FusionTrackAlg != nullptr)
    {
        delete m_FusionTrackAlg;
        m_FusionTrackAlg = nullptr;

    }
}


Fusion_Res Fusion_Algorithm_ServiceArea::RUN(std::vector<xt::xarray<float>> &video_data, 
                                             xt::xarray<float> &pc_data, 
                                             int cam_num)
{
    m_FusionTrackAlg->predict(m_pc_timestamp);                                  // 预测
    m_FusionTrackAlg->set_lidar_data(&pc_data, m_pc_timestamp);                 // 点云更新
    m_FusionTrackAlg->set_camera_data(&video_data, m_camera_timestamp);         // 相机更新

    std::map<int, int>   parking_number;
    xt::xarray<float> fusion_track_result = xt::empty<float>({0, 19});

    std::cout << "Fusion Process begin!" << std::endl;
    m_FusionTrackAlg->process(m_pc_timestamp, 
                              &fusion_track_result,
                              parking_number);
    
    _output.pc_res = fusion_track_result;
    _output.video_res = video_data;

    return _output;
}