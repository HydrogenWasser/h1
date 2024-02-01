#include "V2xSceneAlg.h"
#include "CSceneAlg.h"
#include "TPcResult.h"
#include "iostream"
#include "FunctionHub.h"

V2xSceneAlg::V2xSceneAlg(nlohmann::json *fusion_param)
{
    m_fusion_param = fusion_param;
    max_age = int((*fusion_param)["fusion_param"]["sort_max_age"]);
    max_age_new = int((*fusion_param)["fusion_param"]["sort_max_age_new"]);
    min_hits = int((*fusion_param)["fusion_param"]["sort_min_hits"]);

    parse_json();
}

V2xSceneAlg::~V2xSceneAlg()
{
}

Fusion_Res_scene V2xSceneAlg::RUN(TFusionTrackResult *fusion_track_result)
{   
    // 复制一份数据
    m_fusion_result = *fusion_track_result;

    // 后处理结果
    xt::xarray<float> post_result = xt::empty<float>({0, 19});
    int flag = process(&post_result);
    m_output.pc_res = post_result;
    return m_output;
}

int V2xSceneAlg::process(xt::xarray<float> *result)
{
    auto &fusion_result = *result;

    int trackers_num = m_fusion_result.m_vecBox.size();

    std::vector<unsigned short> current_tracker_id;

    for (std::unordered_map<unsigned short, std::vector<long int>>::iterator it = id_about_life.begin(); it!=id_about_life.end(); it++){
        it->second[0] = it->second[0] + 1;
    }

    if (trackers_num > 0){
        fusion_result = xt::zeros<float>({trackers_num, 19});

        std::vector<int> pop_tracker_index;

        // [0, 1, 2, 3,   4,   5, 6,   7,     8,    9,   10,     11,    12,    13,    14,     15,         16,     17,    18]
        // [x, y, w, l, theta, z, h, label, speed, id, scores, x_min, y_min, x_max, y_max, data_source, channel, lane, occur_time]
        for (int i = 0; i < trackers_num; i++){
            fusion_result(i, 0) = float(m_fusion_result.m_vecBox[i].m_sXCoord) / 100.0;
            fusion_result(i, 1) = float(m_fusion_result.m_vecBox[i].m_sYCoord) / 100.0;
            fusion_result(i, 2) = float(m_fusion_result.m_vecBox[i].m_usWidth) / 100.0;
            fusion_result(i, 3) = float(m_fusion_result.m_vecBox[i].m_usLength) / 100.0;
            fusion_result(i, 4) = float(m_fusion_result.m_vecBox[i].m_usCourseAngle);
            fusion_result(i, 5) = float(m_fusion_result.m_vecBox[i].m_sZCoord) / 100.0;
            fusion_result(i, 6) = float(m_fusion_result.m_vecBox[i].m_usHeight) / 100.0;
            fusion_result(i, 7) = float(Class2label(m_fusion_result.m_vecBox[i].m_strClass, m_fusion_param));
            // if (int(m_fusion_result.m_vecBox[i].m_ucSource) == 0 || int(m_fusion_result.m_vecBox[i].m_ucSource) == 2 || int(m_fusion_result.m_vecBox[i].m_ucSource) == 3 || int(m_fusion_result.m_vecBox[i].m_ucSource) == 5){
            //     fusion_result(i, 7) = 12;
            // }  //zb, debug
            fusion_result(i, 8) = float(m_fusion_result.m_vecBox[i].m_usSpeed) / 100.0;
            fusion_result(i, 9) = float(m_fusion_result.m_vecBox[i].m_usSingleId);
            fusion_result(i, 10) = float(m_fusion_result.m_vecBox[i].m_ucConfidence) / 100.0;
            fusion_result(i, 11) = float(m_fusion_result.m_vecBox[i].m_usVideoInfo[0]);
            fusion_result(i, 12) = float(m_fusion_result.m_vecBox[i].m_usVideoInfo[1]);
            fusion_result(i, 13) = float(m_fusion_result.m_vecBox[i].m_usVideoInfo[2]);
            fusion_result(i, 14) = float(m_fusion_result.m_vecBox[i].m_usVideoInfo[3]);
            fusion_result(i, 15) = float(m_fusion_result.m_vecBox[i].m_ucSource);
            fusion_result(i, 16) = float(m_fusion_result.m_vecBox[i].m_ucReserve[0]);
            fusion_result(i, 17) = float(m_fusion_result.m_vecBox[i].m_ucLanId);
            // fusion_result(i, 18) = float(m_fusion_result.m_vecBox[i].m_ulOccurTime);
            fusion_result(i, 18) = float(m_fusion_result.m_ulBufTimeStamp[0]);

            if (fusion_result(i, 3) < 3.0 && std::find(motor_vehicle_labels.begin(), motor_vehicle_labels.end(), int(fusion_result(i, 7))) != motor_vehicle_labels.end()){
                fusion_result(i, 7) = 1;
            }
            if (fusion_result(i, 3) > 4.0 && (int(fusion_result(i, 7)) == 1 || int(fusion_result(i, 7)) == 4)){
                fusion_result(i, 7) = 0;
            }
            if (fusion_result(i, 3) > 7.0 && int(fusion_result(i, 7)) == 0){
                fusion_result(i, 7) = 2;
            }
            if (fusion_result(i, 3) < 5.5 && int(fusion_result(i, 7)) == 6){
                fusion_result(i, 7) = 0;
            }

            unsigned short id = m_fusion_result.m_vecBox[i].m_usSingleId;
            current_tracker_id.push_back(id);
            if (id_about_life.find(id) == id_about_life.end()){  // new tracker
                std::vector<long int> life = {0, 0};
                id_about_life.emplace(id, life);
            }else{  // living tracker
                if ((int(m_fusion_result.m_vecBox[i].m_ucSource) == 1 && int((*m_fusion_param)["fusion_param"]["Fusion_Video"]) && !int((*m_fusion_param)["fusion_param"]["Fusion_Lidar"])) 
                || int(m_fusion_result.m_vecBox[i].m_ucSource) == 2 || int(m_fusion_result.m_vecBox[i].m_ucSource) == 4 || int(m_fusion_result.m_vecBox[i].m_ucSource) == 5 || int(m_fusion_result.m_vecBox[i].m_ucSource) == 7){  // tracking success
                    id_about_life[id][0] = 0;
                    id_about_life[id][1] += 1;
                }     
            }
          
            // uint32_t time_since_update = (uint32_t)m_fusion_result.m_vecBox[i].m_uiTimeSinceUpdate;
            // uint64_t hits = (uint64_t)m_fusion_result.m_vecBox[i].m_ulHits;

            if (id_about_life[id][0] > max_age || id_about_life[id][1] <= min_hits || (id_about_life[id][0] > id_about_life[id][1] && id_about_life[id][0] > 2)){
                pop_tracker_index.push_back(i);
            }        

        }

        std::vector<unsigned short> delete_track_id;
        for (std::unordered_map<unsigned short, std::vector<long int>>::iterator it = id_about_life.begin(); it!=id_about_life.end(); it++){
            if (std::find(current_tracker_id.begin(), current_tracker_id.end(), it->first) == current_tracker_id.end()){  // delete tracker
                // id_about_life.erase(it->first);
                delete_track_id.push_back(it->first);
            }

        }
        if (int(delete_track_id.size()) > 0){
            for (auto it = delete_track_id.begin(); it != delete_track_id.end(); it++){
                id_about_life.erase(*it);
            }
        }
        
        std::cout << "liluo----------> rsu_fusion_result 0000: " << fusion_result.shape(0) << std::endl;
        if (int(pop_tracker_index.size()) > 0){
           fusion_result = xt::view(fusion_result, xt::drop(pop_tracker_index));
           std::cout << "liluo----------> rsu_fusion_result 1111: " << fusion_result.shape(0) << std::endl;
        }

        // 融合结果nms
        std::vector<int> nms_delete_index;
        FunctionHub::result_nms(fusion_result, nms_delete_index, 0.3, 1, false);

        if (nms_delete_index.size()>0)
        {
            fusion_result = xt::view(fusion_result, xt::drop(nms_delete_index));
        }

    }else{
        fusion_result = xt::empty<float>({0, 19});
    }
    
    return 1;
}

int V2xSceneAlg::Class2label(const std::string &p_strClass, nlohmann::json *m_fusion_parameter)
{
   for (int i = 0; i < 12; i++)
    {
        if (p_strClass == (*m_fusion_parameter)["fusion_param"]["m_strFusionClass"][i]){
            return i;
        }
    }
    
    return 0;
}

void V2xSceneAlg::parse_json()
{
    for (auto item : (*m_fusion_param)["fusion_param"]["motor_vehicle_labels"].items())
    {
        motor_vehicle_labels.push_back(int(item.value()));
    }
}