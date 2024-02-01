/*******************************************************
 文件名：DistanceAssociate_SeriviceArea.cpp
 作者：HuaHuan
 描述：距离匹配
 版本：v1.0
 日期：2024-01-11
 *******************************************************/

#include "DistanceAssociate_ServiceArea.h"
#include "FunctionHub_ServiceArea.h"


DistanceAssociate_ServiceArea::DistanceAssociate_ServiceArea(cal_dis_result_ServiceArea *_cal_dis_result)
{
    m_cal_dis_result = _cal_dis_result;
}


DistanceAssociate_ServiceArea::~DistanceAssociate_ServiceArea()
{

}


void DistanceAssociate_ServiceArea::pre_process_associate_data(xt::xarray<float> *_dets, 
                                                               xt::xarray<float> *_trks, 
                                                               int               flag, 
                                                               nlohmann::json    *parameter)
{   
    m_flag = flag;

    if (flag == 1)
    // 雷达距离匹配
    {  
        m_detections = *_dets;
        m_trackers = *_trks;
    }
    else
    // 相机距离匹配
    {   
        m_detections = *_dets;
        m_trackers = *_trks;
    }
    
}


void DistanceAssociate_ServiceArea::cal_cost_matrix(xt::xarray<float> &cost_matrix)
{   
    
    cost_matrix = xt::zeros<float> ({(int)m_detections.shape(0),(int)m_trackers.shape(0)});
    xt::xarray<float> head_dis = xt::zeros<float> ({(int)m_detections.shape(0),(int)m_trackers.shape(0)});
    xt::xarray<float> ano_dis = xt::zeros<float> ({(int)m_detections.shape(0),(int)m_trackers.shape(0)});
    xt::xarray<float> square_dis = xt::zeros<float> ({(int)m_detections.shape(0),(int)m_trackers.shape(0)});

    FunctionHub_ServiceArea::cal_distance(m_detections, m_trackers, head_dis, ano_dis, cost_matrix, m_flag); 

    m_cal_dis_result->square_dis = cost_matrix;
    m_cal_dis_result->ano_dis = ano_dis;
    m_cal_dis_result->head_dis = head_dis;
   
}


void DistanceAssociate_ServiceArea::execute_match(xt::xarray<float>             _cost_matrix, 
                                                  fusion_match_out_ServiceArea  *associate_out, 
                                                  nlohmann::json                *parameter)
{

    
    //  associate_out->_cost_matrix = _cost_matrix;  // 计算出来的距离矩阵存储到associate_out中
    xt::xarray<float> cost;

    if (m_flag == 1){  // 雷达的距离匹配
        float head_thres = float((*parameter)["fusion_param"]["lidar_head_disThreshold"]);
        xt::xarray<float> head_dis_thres_matrix = xt::full_like(m_cal_dis_result->head_dis, head_thres);

        float ano_thres = float((*parameter)["fusion_param"]["lidar_ano_disThreshold"]);
        xt::xarray<float> ano_dis_thres_matrix = xt::full_like(m_cal_dis_result->ano_dis, ano_thres);

        // 把计算得到的距离阈值矩阵存储在结构成员变量中
        m_cal_dis_result->head_dis_thres_matrix = head_dis_thres_matrix; 
        m_cal_dis_result->ano_dis_thres_matrix = ano_dis_thres_matrix;

        xt::xarray<float> final_dis = m_cal_dis_result->head_dis + m_cal_dis_result->ano_dis * 3;
       
        cost = xt::where(final_dis > 60, xt::ones_like(final_dis)*10000, final_dis);
       
    }

    if (m_flag == 0){   // 相机距离匹配
        cost = m_cal_dis_result->square_dis;

        float head_thres = float((*parameter)["fusion_param"]["camera_head_disThreshold"]);
        xt::xarray<float> head_dis_thres_matrix = xt::full_like(m_cal_dis_result->head_dis, head_thres);

        float ano_thres = float((*parameter)["fusion_param"]["camera_ano_disThreshold"]);
        xt::xarray<float> ano_dis_thres_matrix = xt::full_like(m_cal_dis_result->ano_dis, ano_thres);

        // 把计算得到的距离阈值矩阵存储在结构成员变量中
        m_cal_dis_result->head_dis_thres_matrix = head_dis_thres_matrix; 
        m_cal_dis_result->ano_dis_thres_matrix = ano_dis_thres_matrix;
    }
    
    // auto &cost = _cost_matrix;
    auto matched_indices = hungarian_alg_SA(cost);
    associate_out->_cost_matrix = cost;  // 计算出来的距离矩阵存储到associate_out中
    
    associate_out->_matched_indices = matched_indices;
}

void DistanceAssociate_ServiceArea::post_process_associate_data(fusion_match_out_ServiceArea    *associate_out, 
                                                                xt::xarray<float>               *_dets, 
                                                                int                             flag, 
                                                                nlohmann::json                  *parameter)
{
    
    // 把匹配成功的检测索引和轨迹索引用vector存储起来
    std::vector<int> matched_det_index;
    std::vector<int> matched_trk_index;
    for (int i = 0; i < associate_out->_matched_indices.shape(0); i++)
    {
        matched_det_index.push_back(associate_out->_matched_indices(i, 0));
        matched_trk_index.push_back(associate_out->_matched_indices(i, 1));
    }

     // 得到未匹配到的检测索引
    std::vector<int> unmatched_det_index;
    for (int i = 0; i < associate_out->_cost_matrix.shape(0); i++)
    {
        if (std::find(matched_det_index.begin(), matched_det_index.end(), i) == matched_det_index.end())
        {
            unmatched_det_index.push_back(i);
        }
    }

    // 得到未匹配到的轨迹索引
    std::vector<int> unmatched_trk_index;
    for (int i = 0; i < associate_out->_cost_matrix.shape(1); i++)
    {
        if (std::find(matched_trk_index.begin(), matched_trk_index.end(), i) == matched_trk_index.end())
        {
            unmatched_trk_index.push_back(i);
        }
    }

    
    matched_det_index.clear();  // 先清空数据
    matched_trk_index.clear();

    // 得到匹配阈值过滤后的匹配到的轨迹
    for(int i = 0; i < associate_out->_matched_indices.shape(0); i++){

        int det_index = associate_out->_matched_indices(i, 0);
        int trk_index = associate_out->_matched_indices(i, 1);

        if (flag == 1)
        {   
            // 删掉了十字路口的代码
            bool blind_flag = true;
            if (m_blind_area.size() > 0)
            {
                blind_flag = false;
                for (auto item : (*parameter)["fusion_param"]["blindArea"].items())
                {
                    auto current_blind = item.value();
                    if(
                        current_blind[0] < m_detections(det_index, 0) && m_detections(det_index, 0) < current_blind[2] &&   // x
                        current_blind[1] < m_detections(det_index, 1) && m_detections(det_index, 1) < current_blind[3]      // y
                    )
                        blind_flag = true;
                        std::cout << "盲区距离匹配成功" << std::endl;
                }
            }

            if (blind_flag && m_cal_dis_result->head_dis(det_index, trk_index) < 12 && 
                m_cal_dis_result->ano_dis(det_index, trk_index) < 12)                   // 服务区参数
            {
                matched_det_index.push_back(det_index);
                matched_trk_index.push_back(trk_index);
            }
            else if (m_cal_dis_result->head_dis(det_index, trk_index) < 9 && 
                     m_cal_dis_result->ano_dis(det_index, trk_index) < 3)               // 服务区参数
            {
                matched_det_index.push_back(det_index);
                matched_trk_index.push_back(trk_index);
            }
            else
            {
                unmatched_det_index.push_back(det_index);
                unmatched_trk_index.push_back(trk_index);
            }
        }
        else
        {
            if (m_cal_dis_result->head_dis(det_index, trk_index) > (m_cal_dis_result->head_dis_thres_matrix(det_index, trk_index)
                + float((*parameter)["fusion_param"]["head_dis_thres_forword_adjust"]))
                || m_cal_dis_result->head_dis(det_index, trk_index) < (-m_cal_dis_result->head_dis_thres_matrix(det_index, trk_index)
                + float((*parameter)["fusion_param"]["head_dis_thres_backword_adjust"]))
                || m_cal_dis_result->ano_dis(det_index, trk_index) > m_cal_dis_result->ano_dis_thres_matrix(det_index, trk_index))
            {
                unmatched_det_index.push_back(det_index);
                unmatched_trk_index.push_back(trk_index);
            }
            else
            {
                matched_det_index.push_back(det_index);
                matched_trk_index.push_back(trk_index);
            }
        }

    }
         
    std::vector<int> matched_all_dets_index_first = associate_out->_matched_detections_indices;
    std::vector<int> matched_all_trks_index_first = associate_out->_matched_trackers_indices;
    std::vector<int> unmatched_all_dets_index_first = associate_out->_unmatched_detections_indices;
    std::vector<int> unmatched_all_trks_index_first = associate_out->_unmatched_trackers_indices;

    std::vector<int> unmatched_trks_index_second;
    for(auto it=unmatched_det_index.begin(); it != unmatched_det_index.end();  it++){
        unmatched_trks_index_second.push_back(associate_out->_unmatched_trackers_indices[*it]);
    }
    std::vector<int> unmatched_dets_index_second;
    for(auto it = unmatched_det_index.begin(); it != unmatched_det_index.end(); it++){
        unmatched_dets_index_second.push_back(associate_out->_unmatched_detections_indices[*it]);
    }

    std::vector<int> matched_dets_index_second;
    for(auto it=matched_det_index.begin(); it != matched_det_index.end();  it++){
        matched_dets_index_second.push_back(associate_out->_unmatched_detections_indices[*it]);
    }

    std::vector<int> matched_trks_index_second;
    for(auto it=matched_trk_index.begin(); it != matched_trk_index.end();  it++){
        matched_trks_index_second.push_back(associate_out->_unmatched_trackers_indices[*it]);
    }

    std::vector<int> matched_all_dets_index_second;
    for(auto it = matched_det_index.begin(); it!= matched_det_index.end(); it++){
        matched_all_dets_index_second.push_back(unmatched_all_dets_index_first[*it]);
    }

    std::vector<int> matched_all_trks_index_second;
    for(auto it = matched_trk_index.begin(); it!= matched_trk_index.end(); it++){
        matched_all_trks_index_second.push_back(unmatched_all_trks_index_first[*it]);
    }

    std::vector<int> unmatched_all_trks_index_second;
    for(auto it = unmatched_trk_index.begin(); it!= unmatched_trk_index.end(); it++){
        unmatched_all_trks_index_second.push_back(unmatched_all_trks_index_first[*it]);
    }

    std::vector<int> unmatched_all_dets_index_second;
    for(auto it = unmatched_det_index.begin(); it!= unmatched_det_index.end(); it++){
        unmatched_all_dets_index_second.push_back(unmatched_all_dets_index_first[*it]);
    }

    std::vector<int> matched_all_dets_index_ls;
    matched_all_dets_index_ls.insert(matched_all_dets_index_ls.end(), matched_all_dets_index_first.begin(), matched_all_dets_index_first.end());
    matched_all_dets_index_ls.insert(matched_all_dets_index_ls.end(), matched_all_dets_index_second.begin(), matched_all_dets_index_second.end());
    // associate_out->_matched_detections_indices = matched_all_dets_index_ls;

    std::vector<int> matched_all_trks_index_ls;
    matched_all_trks_index_ls.insert(matched_all_trks_index_ls.end(), matched_all_trks_index_first.begin(), matched_all_trks_index_first.end());
    matched_all_trks_index_ls.insert(matched_all_trks_index_ls.end(), matched_all_trks_index_second.begin(), matched_all_trks_index_second.end());
    // associate_out->_matched_detections_indices = matched_all_dets_index_ls;


    std::vector<int> unmatched_all_dets_index_ls;
    unmatched_all_dets_index_ls = unmatched_all_dets_index_second;
    
    

    std::vector<int> unmatched_all_trks_index_ls;
    unmatched_all_trks_index_ls = unmatched_all_trks_index_second;
    

    // std::vector<float> matched_all_trks_id;
    // for(int i = 0; i<matched_all_dets_index_ls.size(); i++){
    //     matched_all_trks_id.push_back(m_trackers(matched_all_dets_index_ls[i], 7));
    // }

    
    // 得到最终的匹配到的索引数组
    if (int(matched_all_dets_index_ls.size()) <= 0)
    {
        associate_out->_matched_indices = xt::empty<int>({0, 2});
    }
    else
    {
        assert(matched_all_dets_index_ls.size() == matched_all_trks_index_ls.size());
        std::vector<std::size_t> shape = {matched_all_dets_index_ls.size(), 1};

        xt::xarray<int> xt_matched_det_index = xt::adapt(matched_all_dets_index_ls, shape);
        xt::xarray<int> xt_matched_trk_index = xt::adapt(matched_all_trks_index_ls, shape);

        associate_out->_matched_indices = xt::concatenate(xt::xtuple(xt_matched_det_index, xt_matched_trk_index), 1);
    }
    

    // 把未匹配到的检测索引和未匹配到的轨迹索引，保存到associate_out
    associate_out->_unmatched_detections_indices = unmatched_all_dets_index_ls;
    associate_out->_unmatched_trackers_indices = unmatched_all_trks_index_ls;

    associate_out->_matched_detections_indices = matched_all_dets_index_ls;
    associate_out->_matched_trackers_indices = matched_all_trks_index_ls;


    // 把未匹配上的检测数据，保存到associate_out.unmatched_detections中
    // xt::xarray<float> dets = *_dets;
    if (unmatched_det_index.size()> 0){
        xt::xarray<float> unmatched_det = xt::zeros<float>({int(unmatched_det_index.size()), int(m_detections.shape(1))});
        for(int i = 0; i < unmatched_det_index.size(); i++){
            
            xt::view(unmatched_det, i, xt::all()) = xt::view(m_detections, unmatched_det_index[i], xt::all());
            
        }
        
        
        associate_out->_unmatched_detections = unmatched_det;
        
    }

}


void DistanceAssociate_ServiceArea::post_process_associate_data_third(fusion_match_out_ServiceArea  *associate_out, 
                                                                      xt::xarray<float>             *_dets, 
                                                                      int                           flag, 
                                                                      nlohmann::json                *parameter, 
                                                                      std::vector<int>              *vaild_elder, 
                                                                      std::vector<int>              *vaild_younger)
{
    
    // 把匹配成功的检测索引和轨迹索引用vector存储起来
    std::vector<int> matched_det_index;
    std::vector<int> matched_trk_index;
    for (int i = 0; i < associate_out->_matched_indices.shape(0); i++)
    {
        matched_det_index.push_back(associate_out->_matched_indices(i, 0));
        matched_trk_index.push_back(associate_out->_matched_indices(i, 1));
    }

     // 得到未匹配到的检测索引
    std::vector<int> unmatched_det_index;
    for (int i = 0; i < associate_out->_cost_matrix.shape(0); i++)
    {
        if (std::find(matched_det_index.begin(), matched_det_index.end(), i) == matched_det_index.end())
        {
            unmatched_det_index.push_back(i);
        }
    }

    // 得到未匹配到的轨迹索引
    std::vector<int> unmatched_trk_index;
    for (int i = 0; i < associate_out->_cost_matrix.shape(1); i++)
    {
        if (std::find(matched_trk_index.begin(), matched_trk_index.end(), i) == matched_trk_index.end())
        {
            unmatched_trk_index.push_back(i);
        }
    }

    
    matched_det_index.clear();  // 先清空数据
    matched_trk_index.clear();

    // 得到匹配阈值过滤后的匹配到的轨迹
    for(int i = 0; i < associate_out->_matched_indices.shape(0); i++){

        int det_index = associate_out->_matched_indices(i, 0);
        int trk_index = associate_out->_matched_indices(i, 1);

        if (int(m_trackers(trk_index, 8)) == 4){
            if (m_cal_dis_result->head_dis(det_index, trk_index) > float((*parameter)["fusion_param"]["lidar_head_disThreshold_person"])
                || m_cal_dis_result->head_dis(det_index, trk_index) < -float((*parameter)["fusion_param"]["lidar_head_disThreshold_person"])
                || m_cal_dis_result->ano_dis(det_index, trk_index) > float((*parameter)["fusion_param"]["lidar_ano_disThreshold_person"])
                || m_cal_dis_result->square_dis(det_index, trk_index) > FunctionHub_ServiceArea::neighbor_search(m_trackers, trk_index, float((*parameter)["fusion_param"]["search_radius_person"]), parameter)[quadrant(det_index, trk_index)]){
                unmatched_det_index.push_back(det_index);
                unmatched_trk_index.push_back(trk_index);
                // cout<<"zb33333333333333333333333333"<<FunctionHub::neighbor_search(m_trackers, trk_index, float((*parameter)["fusion_param"]["search_radius_person"]), parameter)[quadrant(det_index, trk_index)]<<endl;
            }
            else{
                matched_det_index.push_back(det_index);
                matched_trk_index.push_back(trk_index);
            }
        }
        else{
            if (m_cal_dis_result->head_dis(det_index, trk_index) > (m_cal_dis_result->head_dis_thres_matrix(det_index, trk_index)
                + float((*parameter)["fusion_param"]["head_dis_thres_forword_adjust"]))
                || m_cal_dis_result->head_dis(det_index, trk_index) < (-m_cal_dis_result->head_dis_thres_matrix(det_index, trk_index)
                + float((*parameter)["fusion_param"]["head_dis_thres_backword_adjust"]))
                || m_cal_dis_result->ano_dis(det_index, trk_index) > m_cal_dis_result->ano_dis_thres_matrix(det_index, trk_index)
                // || (int(m_trackers(trk_index, 8)) == 1 && int(m_trackers(trk_index, 9)) >= int((*parameter)["fusion_param"]["sort_min_hits"]) && int(m_detections(det_index, 7)) == 0)
                || (std::find(motor_vehicle_labels.begin(), motor_vehicle_labels.end(), int(m_trackers(trk_index, 8))) != motor_vehicle_labels.end() && m_trackers(trk_index, 7) < 1.5 && m_cal_dis_result->ano_dis(det_index, trk_index) > 2.0)
                // && abs(m_cal_dis_result->head_dis(det_index, trk_index)) > 1.0)
                || (int(m_trackers(trk_index, 8)) == 1 && m_cal_dis_result->square_dis(det_index, trk_index) > FunctionHub_ServiceArea::neighbor_search(m_trackers, trk_index, float((*parameter)["fusion_param"]["search_radius_bicycle"]), parameter)[quadrant(det_index, trk_index)])){
                unmatched_det_index.push_back(det_index);
                unmatched_trk_index.push_back(trk_index);
                // cout<<"zb444444444444444444444"<<FunctionHub::neighbor_search(m_trackers, trk_index, float((*parameter)["fusion_param"]["search_radius_bicycle"]), parameter)[quadrant(det_index, trk_index)]<<endl;
            }
            else{
                matched_det_index.push_back(det_index);
                matched_trk_index.push_back(trk_index);
            }
        }

    }
         
    std::vector<int> matched_all_dets_index = associate_out->_matched_detections_indices;
    std::vector<int> matched_elder_trks_index = associate_out->_matched_trackers_indices;
    std::vector<int> unmatched_all_dets_index = associate_out->_unmatched_detections_indices;
    std::vector<int> unmatched_elder_trks_index = associate_out->_unmatched_trackers_indices;

    std::vector<int> matched_all_dets_index_third;
    for(auto it = matched_det_index.begin(); it!= matched_det_index.end(); it++){
        matched_all_dets_index_third.push_back(unmatched_all_dets_index[*it]);
    }

    std::vector<int> unmatched_all_dets_index_third;
    for(auto it = unmatched_det_index.begin(); it!= unmatched_det_index.end(); it++){
        unmatched_all_dets_index_third.push_back(unmatched_all_dets_index[*it]);
    }

    std::vector<int> matched_all_trks_index_from_elder;
    for(int i = 0; i < int((matched_elder_trks_index.size())); i++){
        matched_all_trks_index_from_elder.push_back((*vaild_elder)[matched_elder_trks_index[i]]);
    }

    std::vector<int> unmatched_all_trks_index_from_elder;
    for(int i = 0; i < int((unmatched_elder_trks_index.size())); i++){
        unmatched_all_trks_index_from_elder.push_back((*vaild_elder)[unmatched_elder_trks_index[i]]);
    }

    std::vector<int> matched_all_trks_index_from_younger;
    for(auto it = matched_trk_index.begin(); it!= matched_trk_index.end(); it++){
        matched_all_trks_index_from_younger.push_back((*vaild_younger)[*it]);
    }

    std::vector<int> unmatched_all_trks_index_from_younger;
    for(auto it = unmatched_trk_index.begin(); it!= unmatched_trk_index.end(); it++){
        unmatched_all_trks_index_from_younger.push_back((*vaild_younger)[*it]);
    }


    std::vector<int> matched_all_dets_index_ls;
    matched_all_dets_index_ls.insert(matched_all_dets_index_ls.end(), matched_all_dets_index.begin(), matched_all_dets_index.end());
    matched_all_dets_index_ls.insert(matched_all_dets_index_ls.end(), matched_all_dets_index_third.begin(), matched_all_dets_index_third.end());
    // associate_out->_matched_detections_indices = matched_all_dets_index_ls;

    std::vector<int> matched_all_trks_index_ls;
    matched_all_trks_index_ls.insert(matched_all_trks_index_ls.end(), matched_all_trks_index_from_elder.begin(), matched_all_trks_index_from_elder.end());
    matched_all_trks_index_ls.insert(matched_all_trks_index_ls.end(), matched_all_trks_index_from_younger.begin(), matched_all_trks_index_from_younger.end());
    // associate_out->_matched_detections_indices = matched_all_dets_index_ls;


    std::vector<int> unmatched_all_dets_index_ls;
    unmatched_all_dets_index_ls = unmatched_all_dets_index_third;
    
    

    std::vector<int> unmatched_all_trks_index_ls;
    unmatched_all_trks_index_ls.insert(unmatched_all_trks_index_ls.end(), unmatched_all_trks_index_from_elder.begin(), unmatched_all_trks_index_from_elder.end());
    unmatched_all_trks_index_ls.insert(unmatched_all_trks_index_ls.end(), unmatched_all_trks_index_from_younger.begin(), unmatched_all_trks_index_from_younger.end());
    

    // std::vector<float> matched_all_trks_id;
    // for(int i = 0; i<matched_all_dets_index_ls.size(); i++){
    //     matched_all_trks_id.push_back(m_trackers(matched_all_dets_index_ls[i], 7));
    // }

    
    // 得到最终的匹配到的索引数组
    if (int(matched_all_dets_index_ls.size()) <= 0)
    {
        associate_out->_matched_indices = xt::empty<int>({0, 2});
    }
    else
    {
        assert(matched_all_dets_index_ls.size() == matched_all_trks_index_ls.size());
        std::vector<std::size_t> shape = {matched_all_dets_index_ls.size(), 1};

        xt::xarray<int> xt_matched_det_index = xt::adapt(matched_all_dets_index_ls, shape);
        xt::xarray<int> xt_matched_trk_index = xt::adapt(matched_all_trks_index_ls, shape);

        associate_out->_matched_indices = xt::concatenate(xt::xtuple(xt_matched_det_index, xt_matched_trk_index), 1);
    }
    

    // 把未匹配到的检测索引和未匹配到的轨迹索引，保存到associate_out
    associate_out->_unmatched_detections_indices = unmatched_all_dets_index_ls;
    associate_out->_unmatched_trackers_indices = unmatched_all_trks_index_ls;

    associate_out->_matched_detections_indices = matched_all_dets_index_ls;
    associate_out->_matched_trackers_indices = matched_all_trks_index_ls;


    // 把未匹配上的检测数据，保存到associate_out.unmatched_detections中
    // xt::xarray<float> dets = *_dets;
    if (unmatched_det_index.size()> 0){
        xt::xarray<float> unmatched_det = xt::zeros<float>({int(unmatched_det_index.size()), int(m_detections.shape(1))});
        for(int i = 0; i < unmatched_det_index.size(); i++){
            
            xt::view(unmatched_det, i, xt::all()) = xt::view(m_detections, unmatched_det_index[i], xt::all());
            
        }
        
        
        associate_out->_unmatched_detections = unmatched_det;
        
    }
    else{
        associate_out->_unmatched_detections = xt::empty<float>({0, 0});
    }

}


void DistanceAssociate_ServiceArea::parse_json(nlohmann::json *parameter)
{
    for (auto item : (*parameter)["fusion_param"]["motor_vehicle_labels"].items())
    {
        motor_vehicle_labels.push_back(int(item.value()));
    }

    for (auto item : (*parameter)["fusion_param"]["blindArea"].items())
    {
        std::vector<float> blind_area;
        for (auto v : item.value().items())
		{
			blind_area.push_back(float(v.value()));
		}

        m_blind_area[item.key()] = blind_area;
        blind_area.clear();
    }
}