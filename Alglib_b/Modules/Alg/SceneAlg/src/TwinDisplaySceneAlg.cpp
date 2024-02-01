#include "TwinDisplaySceneAlg.h"
#include "FunctionHub.h"
#include <unistd.h>

TwinDisplaySceneAlg::TwinDisplaySceneAlg(nlohmann::json *fusion_param)
{
    m_fusion_parameter = fusion_param;
    parse_json();
}

TwinDisplaySceneAlg::TwinDisplaySceneAlg()
{

}

TwinDisplaySceneAlg::~TwinDisplaySceneAlg()
{

}


int TwinDisplaySceneAlg::Class2label(const std::string &p_strClass, nlohmann::json *m_fusion_parameter)
{
    for (int i = 0; i < 12; i++)
    {
        if (p_strClass == (*m_fusion_parameter)["fusion_param"]["m_strFusionClass"][i]){
            return i;
        }
    }
    
    return 0;
}

void TwinDisplaySceneAlg::parse_json()
{
    for (auto item : (*m_fusion_parameter)["fusion_param"]["bus_labels"].items()){
        bus_labels.push_back(item.value());
    }

    for (auto item : (*m_fusion_parameter)["fusion_param"]["trunk_labels"].items()){
        trunk_labels.push_back(item.value());
    }

    for (auto item : (*m_fusion_parameter)["fusion_param"]["motor_vehicle_labels"].items())
    {
        motor_vehicle_labels.push_back(int(item.value()));
    }

    m_ServiceArea_select = (int)(*m_fusion_parameter)["fusion_param"]["ServiceArea_select"];
    m_max_age = (int)(*m_fusion_parameter)["fusion_param"]["sort_max_age"];
    m_min_hits = (int)(*m_fusion_parameter)["fusion_param"]["sort_min_hits"];
    
}


Fusion_Res_scene TwinDisplaySceneAlg::RUN(TFusionTrackResult *fusion_track_result)
{   
    //  复制一份数据， 每一种算法拷贝一份数据
    m_fusion_result = *fusion_track_result;

    // 场景算法输出
    xt::xarray<float> scene_result = xt::empty<float>({0, 19});

    
    pre_process();
    std::cout << "SceneAlg Pre Process done!!!!!" << std::endl;
    int flag = process(&scene_result);
    std::cout << "SceneAlg Process done!!!!!" << std::endl;
    
    m_output.pc_res = scene_result;
    return m_output;
}



void TwinDisplaySceneAlg::pre_process()
{      
    auto & fusion_result_trackers = m_fusion_result.m_vecBox;
    int track_num = fusion_result_trackers.size();

    if (track_num > 0)
    {
        for (auto iter = m_trackers_life.begin(); iter != m_trackers_life.end(); iter ++)
        {
            // auto temp_tracker = m_trackers_life[i]
            if (std::find_if(fusion_result_trackers.begin(), 
                             fusion_result_trackers.end(), 
                             [=](TFusionTrackBoxInfo &rhs)
                             {
                                return rhs.m_usSingleId == iter->first;
                             }
                            ) != fusion_result_trackers.end())
                continue;
            else
                m_trackers_life.erase(iter->first);
        }
    }
    

    for (int i = 0; i < track_num; i ++)
    {
        auto tracker = fusion_result_trackers[i];
        
        int id = (int) tracker.m_usSingleId;
        int source = (int) tracker.m_ucSource;    // source 具体信息查询参见Update_tracks_ServiceArea.cpp
        if (m_trackers_life.find(id) != m_trackers_life.end())
        {    // 轨迹已经存在于记录中
            if (source != 0)
            {
                m_trackers_life[id][0] = 0;     // time_since_update
                m_trackers_life[id][1] += 1;    // hits
            }
        }
        else
        {
            m_trackers_life.emplace(id, std::vector<int>{0, 0});
        }
    }
    

}


int TwinDisplaySceneAlg::process(xt::xarray<float>* scene_result)
{
    // 0,     1,     2,     3,     4,     5,    6,     7,     8,     9,     10,          11
    // x,     y,     w,     l,   theta,   z,    h,   label,  speed,  id,   scores,     x_min, 
    //  12,        13,        14,           15,            16,        17,          18
    //y_min,     x_max,     y_max,     data_source,     channel,     lane,     occur_time
    
    
    // result
    auto & fusion_result_trackers = m_fusion_result.m_vecBox;
    int track_num = fusion_result_trackers.size(); // 
    xt::xarray<float> &result = *scene_result;
    result = xt::zeros<float>({track_num, 19});
    std::vector<int> trk_send;
    std::vector<int> trk_drop;
    // label的修改
    if (track_num > 0)
    {
        for (int i = 0; i < fusion_result_trackers.size(); i ++)
        {
            auto tracker = fusion_result_trackers[i];
            float x = tracker.m_sXCoord / 100,
                y = tracker.m_sYCoord / 100,
                z = tracker.m_sZCoord / 100,
                w = tracker.m_usWidth / 100,
                l = tracker.m_usLength / 100,
                h = tracker.m_usHeight / 100,
                speed = tracker.m_usSpeed / 100,
                angle = tracker.m_usCourseAngle,
                label = (float) Class2label(tracker.m_strClass, m_fusion_parameter),
                score = float(tracker.m_ucConfidence) / 100, 
                x_min = float(tracker.m_usVideoInfo[0]), 
                y_min = float(tracker.m_usVideoInfo[1]),
                x_max = float(tracker.m_usVideoInfo[2]), 
                y_max = float(tracker.m_usVideoInfo[3]), 
                data_source = float(tracker.m_ucSource),
                channel = float(tracker.m_ucReserve[0]), 
                lane = float(tracker.m_ucLanId), 
                occur_time = float(m_fusion_result.m_ulBufTimeStamp[0]),
                id = (float) tracker.m_usSingleId;

            // label 处理
            int label_output = (int) label;
            if (m_ServiceArea_select == 1)
            {
                if (std::find(bus_labels.begin(), bus_labels.end(), label_output) != bus_labels.end())
                    label_output = 2;
                else if (std::find(trunk_labels.begin(), trunk_labels.end(), label_output) != trunk_labels.end())
                    label_output = 6;
            }
            else if (m_ServiceArea_select == 2)
            {
                if (std::find(bus_labels.begin(), bus_labels.end(), label_output) != bus_labels.end())
                    label_output = 2;
                else if (std::find(trunk_labels.begin(), trunk_labels.end(), label_output) != trunk_labels.end())
                {
                    if(l < 16) label_output = trunk_labels[1];
                    else label_output = trunk_labels[0];
                }
            }
            else if (m_ServiceArea_select == 3)
            {
                if (std::find(bus_labels.begin(), bus_labels.end(), label_output) != bus_labels.end())
                {
                    if(l >= 12) label_output = 1;
                    else if (7 < l && l < 12) label_output = 2;
                    else label_output = 3;
                }
                else if (std::find(trunk_labels.begin(), trunk_labels.end(), label_output) != trunk_labels.end())
                {
                    if(l >= 16) label_output = 4;
                    else if (l < 16) label_output = 5;
                    else label_output = 6;
                }
            }

            xt::xarray<float> track = {x,
                                       y,
                                       w,
                                       l,
                                       angle,
                                       z,
                                       h,
                                       (float) label_output,
                                       speed,
                                       id,
                                       score,
                                       x_min,
                                       y_min,
                                       x_max,
                                       y_max,
                                       data_source,
                                       channel,
                                       lane,
                                       occur_time}; 

            xt::view(result, i, xt::all()) = xt::view(track, xt::all(), xt::all());

            // pop 和 send
            int time_since_update = m_trackers_life[id][0],
                hits = m_trackers_life[id][1];
        
            // float x = tracker.m_sXCoord,
            //       y = tracker.m_sYCoord;

            bool create_tracker_area_flag;

            if (m_ServiceArea_select == 1)
                create_tracker_area_flag = (70 > x && x > -60)? true : false;
            else if (m_ServiceArea_select == 2)
                create_tracker_area_flag = (100 > y && y > -100)? true : false;
            else if (m_ServiceArea_select == 3)
                create_tracker_area_flag = (100 > x && x > -100)? true : false;

            if ((time_since_update < m_max_age * (speed < 1 && 
                                                  std::find(motor_vehicle_labels.begin(), motor_vehicle_labels.end(), label_output) != motor_vehicle_labels.end())? 
                                                  3 : 1 ) &&
                (hits > m_min_hits * (create_tracker_area_flag && speed < 1)? 5 : 1))
            {
                trk_send.push_back(i);
            }
            else
            {
                trk_drop.push_back(i); // 暂时没啥用
            }
             
        
        }

        if (int(trk_send.size()) > 0){
            result = xt::view(result, xt::keep(trk_send)); // 去掉不需要发送的
        }
        
        // 融合结果nms
        std::vector<int> nms_delete_index;
        FunctionHub::result_nms(result, nms_delete_index, 0.3, 1, false);

        if (nms_delete_index.size()>0)
            result = xt::view(result, xt::drop(nms_delete_index));

        // 
        if (m_ServiceArea_select == 1)
        {
            auto selected_index = xt::where(xt::equal(xt::col(result, 7), 0) ||
                                            xt::equal(xt::col(result, 7), 2) ||
                                            xt::equal(xt::col(result, 7), 6))[0];
            auto selected_result = xt::view(result, xt::keep(selected_index));
            result = selected_result;
        }
    }
    else
    {
        result = xt::empty<float>({0, 19});    
    }

    return 0;
}


// void TwinDisplaySceneAlg::post_process()
// {
//     int track_num = m_trackers_life.size();
//     for (int i = 0; i < track_num; i ++)
//     {
//         auto tracker = m_trackers_life[i];

//         int id = (int) tracker.m_usSingleId;
//         int source = (int) tracker.m_ucSource;    // source 具体信息查询参见Update_tracks_ServiceArea.cpp
//         if (m_trackers_life.find(id) != m_trackers_life.end())
//         {// 轨迹已经存在于记录中
//             if (source != 0)
//             {
//                 m_trackers_life[id][0] = 0;     // time_since_update
//                 m_trackers_life[id][1] += 1;    // hits
//             }
//         }
//         else
//         {
//             m_trackers_life.emplace(id, std::vector<int>{0, 0});
//         }
//     }
// }