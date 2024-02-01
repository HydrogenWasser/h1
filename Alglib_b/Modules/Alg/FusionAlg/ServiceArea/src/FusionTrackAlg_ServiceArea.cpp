/********
 文件名：FusionTrackAlg_ServiceArea.cpp
 作者：Huahuan
 描述：存储轨迹、预测
 版本：v1.0
 日期：2024.01.10
 *******/

#include "FusionTrackAlg_ServiceArea.h"
#include "json_ServiceArea.hpp"


int FUKalmanBoxTracker_ServiceArea::_count = 0;


FusionTrackAlg_ServiceArea::FusionTrackAlg_ServiceArea(nlohmann::json *fusion_parameter)
{
    load_chedao();

    m_match_out = new fusion_match_out_ServiceArea();

    m_cal_dis_result = new cal_dis_result_ServiceArea();

    m_fusion_parameter = fusion_parameter;

    m_lidar_handler = new Process_lidar_ServiceArea(fusion_parameter, 
                                                    &m_chedao,
                                                    m_match_out,
                                                    m_cal_dis_result,
                                                    m_xlimit,
                                                    m_ylimit);

    m_update_handler = new Update_tracks_ServiceArea(fusion_parameter,
                                                     &m_chedao,
                                                     m_xlimit,
                                                     m_ylimit);
    
    m_camera_handler = new Process_camera_ServiceArea(fusion_parameter, 
                                                      &m_chedao,
                                                      m_match_out,
                                                      m_cal_dis_result,
                                                      m_xlimit,
                                                      m_ylimit);

    m_iouassociate_handler = new IouAssociate_ServiceArea();

    m_disassociate_handler = new DistanceAssociate_ServiceArea(m_cal_dis_result);
}


FusionTrackAlg_ServiceArea::~FusionTrackAlg_ServiceArea()
{
    if (m_lidar_handler != nullptr)
    {
        delete m_lidar_handler;
        m_lidar_handler = nullptr;
    }
    if (m_update_handler != nullptr)
    {
        delete m_update_handler;
        m_update_handler = nullptr;
    }
    if (m_iouassociate_handler != nullptr)
    {
        delete m_iouassociate_handler;
        m_iouassociate_handler = nullptr;
    }
    if (m_match_out != nullptr)
    {
        delete m_match_out;
        m_match_out = nullptr;
    }
    if (m_cal_dis_result != nullptr){
        delete m_cal_dis_result;
        m_cal_dis_result = nullptr;
    }
    if (m_disassociate_handler != nullptr){
        delete m_disassociate_handler;
        m_disassociate_handler = nullptr;
    }
}


int FusionTrackAlg_ServiceArea::set_lidar_data(xt::xarray<float> *pc_result, 
                                               unsigned long      pc_timestamp)
{
    int flag;

    if (int((*m_fusion_parameter)["fusion_param"]["Fusion_Lidar"]) ==  1)
        flag = match_lidar(pc_result, pc_timestamp);    
    
    return flag;
}


int FusionTrackAlg_ServiceArea::set_camera_data(std::vector<xt::xarray<float>>  *camera_result, 
                                                std::vector<unsigned long>       camera_time)
{
    int flag;
    if (int((*m_fusion_parameter)["fusion_param"]["Fusion_Video"]) ==  1){
        flag = match_camera(camera_result, camera_time);
    }
    
    return flag;
}


int FusionTrackAlg_ServiceArea::process(unsigned long timestamp, 
                                        xt::xarray<float> *fusion_track_result,
                                        std::map<int, int>   parking_number)
{
   
    fusion_track(timestamp, 
                 fusion_track_result,
                 parking_number);
    return 1;
}


std::vector<FUKalmanBoxTracker_ServiceArea> FusionTrackAlg_ServiceArea::get_trackers()
{
    return m_trackers;
}


int FusionTrackAlg_ServiceArea::predict(unsigned long timestamp)
{   

    dt = float(timestamp - last_timestamp) / 1000;
    auto &trackers = m_trackers; // 改成auto好看点儿
    int trackers_num = trackers.size();
    if (trackers_num > 0){
        
        for (int i = 0; i < trackers_num; i++)
        {   
            trackers[i].predict(dt);
            
            // trackers[i].time_since_update += 1;
            // trackers[i].lidar_updated = 0;
        }
    }
    last_timestamp = timestamp;
    
    return 1;
}


// 处理雷达数据，并进行关联匹配
int FusionTrackAlg_ServiceArea::match_lidar(xt::xarray<float> *_p_pPcResult, 
                                            unsigned long      pc_timestamp)
{
    // 服务区中有一个大车区小车区的分别，根据参数service_area_classification修改类别标签的问题

    m_lidar_handler->process_lidar_data(_p_pPcResult);                                                      // 清空上一帧在类中存储的数据
    
    auto &trackers = m_trackers;

    if (int((*m_fusion_parameter)["fusion_param"]["3rd_match"]) ==  1)
    {
        m_lidar_handler->excute_match_lidar_stage_3rd_match(trackers,
                                                            m_iouassociate_handler,
                                                            m_disassociate_handler);
    }
    else
    {
        m_lidar_handler->excute_match_lidar_stage(trackers, 
                                                  m_iouassociate_handler, 
                                                  m_disassociate_handler);
    }
    
    m_lidar_handler->update_type(trackers, 
                                 pc_timestamp);                                                             // 更新
    
    m_update_handler->create_tracks(m_match_out,                                                            // 创建新track
                                    trackers, 
                                    pc_timestamp, 
                                    1);
    
    return 1;
}


// 处理相机数据，并进行关联匹配
int FusionTrackAlg_ServiceArea::match_camera(std::vector<xt::xarray<float>> *_p_pCameraResult, 
                                             std::vector<unsigned long>      listCameraTime)
{
    m_camera_handler->process_camera_data(_p_pCameraResult);
    auto &trackers = m_trackers;

    m_camera_handler->excute_match_camera_stage(trackers, 
                                                m_iouassociate_handler, 
                                                m_disassociate_handler);

    m_camera_handler->update_type(trackers, listCameraTime[0]);
    
    m_update_handler->create_tracks(m_match_out, 
                                    trackers, 
                                    listCameraTime[0], 0);

    m_camera_handler->getCameraBoxInfo(_p_pCameraResult);

    // # 用以界面显示，目标框，类别，角度等
    return 1;
}


// 雷达或者相机的检测结果更新关联上的目标
void FusionTrackAlg_ServiceArea::fusion_track(unsigned long         timestamp, 
                                              xt::xarray<float>     *fusion_track_result,
                                              std::map<int, int>    parking_number)
{
   
    // m_update_handler->update_type(m_match_out, m_trackers);
    auto &trackers = m_trackers;
    std::cout << "Fusion update begin!" << std::endl;
    m_update_handler->update(trackers, 
                             fusion_track_result, 
                             timestamp, 
                             dt,
                             parking_number); // 0115，新增的目标序号对应车位号
}

void FusionTrackAlg_ServiceArea::load_chedao()
{
    nlohmann::json chedao;
    std::string filepath_json = "Configs/Alg/CppConfigs/fusion/ServiceArea/chedao/chedao.json";
    std::ifstream ifs(filepath_json, std::ios::in);
    ifs >> chedao;

    for (auto item : chedao.items())
    {
        int key = std::atoi(item.key().c_str());
        int lane = std::atoi(std::string(item.value()["lane"][0]).c_str());
        lane = (lane / 1000 * 2 + lane % 1000 / 100 - 1) * 10 + lane % 10;
        float angle = float(item.value()["angle"][0]);
        m_chedao[key] = {lane, angle};
    }

    std::string filepath_csv = "Configs/Alg/CppConfigs/fusion/ServiceArea/chedao/limit.csv";
    std::ifstream file(filepath_csv, std::ios::in);
    std::string line;
    int count = 0, row = 0, col = 0;
    std::vector<int> data;

    while (std::getline(file, line, ','))
    {
        row = count / 3;
        col = count % 3;

        if (row == 0)
        {
            if (col == 1 || col == 2)
                m_xlimit[col - 1] = std::stof(line);
        }
        else if (row == 1)
        {
            if (col == 0 || col == 1)
                m_ylimit[col ] = std::stof(line);
        }
        count++;
    }
    
    file.close();
}

// // 后处理
// std::vector<fusion_output> FusionTrackAlg::output()
// {   std::vector<fusion_output> output;
//     output = m_output_handler->output(m_trackers);
//     return output;
// }