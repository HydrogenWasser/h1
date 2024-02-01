#include <Update_tracks.h>

Update_tracks::Update_tracks(nlohmann::json *parameter, std::unordered_map<int, std::pair<int, float>> *chedao, float x_limit[], float y_limit[])
{
    m_fusion_parameter = parameter;
    m_chedao = chedao;

    m_height = int((y_limit[1] - y_limit[0]) * 10);
    m_width = int((x_limit[1] - x_limit[0]) * 10);

    m_x_limit = x_limit;
    m_y_limit = y_limit;

    parse_json();
    // m_fusion_track_trsult = fusion_track_trsult;
}

Update_tracks::~Update_tracks()
{
}

// void Update_tracks::update_type(match_out* _match_out, std::vector<FUKalmanBoxTracker>* _m_trackers){

// }

void Update_tracks::create_tracks(fusion_match_out *_match_out, std::vector<FUKalmanBoxTracker> &_m_trackers, unsigned long timestamp, int flag)
{

    auto unmatched_detections = _match_out->_unmatched_detections;
    // std::vector<FUKalmanBoxTracker> trackers = *_m_trackers;
    int unmatched_dets_num = unmatched_detections.shape(0);
    

    if (unmatched_dets_num > 0)
    {
        for (int i = 0; i < unmatched_dets_num; i++)
        {
            xt::xarray<float> bbox = xt::view(unmatched_detections, i, xt::all());
            

            if (flag == 1)
            { // 雷达
                float lane_angle = FunctionHub::get_lane_info(bbox(0), bbox(1), m_width, m_height, m_x_limit, m_y_limit, m_chedao).second;
                if (lane_angle != -1000)
                {
                    bbox(4) = lane_angle;
                }


                // float fake_object_1_x = -71.50;
                // float fake_object_1_y = -3.30;
                // float fake_object_2_x = -3.06;
                // float fake_object_2_y = -6.72;
                // float fake_object_3_x = -26.33;
                // float fake_object_3_y = -5.83;

                // float fake_object_4_x = -2.01;
                // float fake_object_4_y = -6.43;

                // float fake_object_5_x = 46.19;
                // float fake_object_5_y = -20.26;

                // float fake_object_6_x = 8.67;
                // float fake_object_6_y = -8.38;

                // float fake_object_7_x = 40.82;
                // float fake_object_7_y = -26.40;

                // float fake_object_8_x = -192.5;
                // float fake_object_8_y = -33.32;


                // bool real_flag_1 = true;
                // bool real_flag_2 = true;
                // bool real_flag_3 = true;
                // bool real_flag_4 = true;
                // bool real_flag_5 = true;
                // bool real_flag_6 = true;
                // bool real_flag_7 = true;
                // bool real_flag_8 = true;


                // if ((abs(bbox(0) - fake_object_1_x)<8)&&(abs(bbox(1) - fake_object_1_y)<3.5)){
                //     real_flag_1 = false;
                // }
                // if ((abs(bbox(0) - fake_object_2_x)<3.5)&&(abs(bbox(1) - fake_object_2_y)<3.5)){
                //     real_flag_2 = false;
                // }
                // if ((abs(bbox(0) - fake_object_3_x)<3.5)&&(abs(bbox(1) - fake_object_3_y)<3.5)){
                //     real_flag_3 = false;
                // }
                // if ((abs(bbox(0) - fake_object_4_x)<3.5)&&(abs(bbox(1) - fake_object_4_y)<3.5)){
                //     real_flag_4 = false;
                // }
                // if ((abs(bbox(0) - fake_object_5_x)<3.5)&&(abs(bbox(1) - fake_object_5_y)<3.5)){
                //     real_flag_5 = false;
                // }
                // if ((abs(bbox(0) - fake_object_6_x)<3.5)&&(abs(bbox(1) - fake_object_6_y)<3.5)){
                //     real_flag_6 = false;
                // }
                // if ((abs(bbox(0) - fake_object_7_x)<10)&&(abs(bbox(1) - fake_object_7_y)<3.5)){
                //     real_flag_7 = false;
                // }
                // if ((abs(bbox(0) - fake_object_8_x)<3.5)&&(abs(bbox(1) - fake_object_8_y)<3.5)){
                //     real_flag_8 = false;
                // }


                // if ((real_flag_1&&real_flag_2&&real_flag_3 && real_flag_4&&real_flag_5&&real_flag_6 && real_flag_7&&real_flag_8)){
                //     _m_trackers.push_back(FUKalmanBoxTracker(bbox, timestamp, m_fusion_parameter, true, false, false));
                // }else{
                //     std::cout << "liluo-----------------------> fake object: " << "x: " << bbox(0) << ",  y: " << bbox(1) << std::endl;
                // }

                _m_trackers.push_back(FUKalmanBoxTracker(bbox, timestamp, m_fusion_parameter, true, false, false));
            }

            if (flag == 0)
            { // 相机 label:bbox(7)
                if (std::find(camera_create_trackers.begin(), camera_create_trackers.end(), int(bbox(7))) != camera_create_trackers.end())
                {
                    float lane_angle = FunctionHub::get_lane_info(bbox(0), bbox(1), m_width, m_height, m_x_limit, m_y_limit, m_chedao).second;
                    if (lane_angle != -1000)
                    {
                        bbox(4) = lane_angle;
                    }
                    _m_trackers.push_back(FUKalmanBoxTracker(bbox, timestamp, m_fusion_parameter, false, true, false));
                }
            }
        }
    }
}

void Update_tracks::update(std::vector<FUKalmanBoxTracker> &_m_trackers, xt::xarray<float> *fusion_track_result, unsigned long timestamp, float dt)
{

    int trackers_num = _m_trackers.size();
    auto &res = *fusion_track_result;    // 融合结果
    
    
    std::vector<int> pop_tracker_index;  // 删除轨迹的索引
    std::vector<int> live_tracker_index; // 需要保留的轨迹索引

    std::vector<long int> all_hits;
    res = xt::zeros<float>({trackers_num, 22});
    int output_object_index = trackers_num -1;
    if (trackers_num > 0)
    {
        for (int i = 0; i < trackers_num; i++)
        {

            _m_trackers[i].update(timestamp, dt);

            // 查询轨迹的车道号和车道角
            std::pair<int, float> lane_lane_angle;
            lane_lane_angle = FunctionHub::get_lane_info(_m_trackers[i]._kf._x(0, 0), _m_trackers[i]._kf._x(1, 0), m_width, m_height, m_x_limit, m_y_limit, m_chedao);
            _m_trackers[i].lane = lane_lane_angle.first;
            _m_trackers[i].lane_angle = lane_lane_angle.second;

            // 存储轨迹的状态值
            xt::xarray<float> current_state = {_m_trackers[i]._kf._x(0, 0), _m_trackers[i]._kf._x(1, 0), _m_trackers[i].speed};
            _m_trackers[i].state.push_back(current_state);

            std::vector<xt::xarray<float>> last_thirty_state;
            if (int(_m_trackers[i].state.size()) > 30)
            {
                last_thirty_state.reserve(30);
                std::copy(_m_trackers[i].state.end() - 30, _m_trackers[i].state.end(), std::back_inserter(last_thirty_state));
                _m_trackers[i].state = last_thirty_state;
            }

            std::pair<float, bool> angle_and_angle_flag;
            angle_and_angle_flag = FunctionHub::cal_angle_new(&_m_trackers[i], m_fusion_parameter);
            _m_trackers[i].track_angle = angle_and_angle_flag.first;
            bool old_track_angle_flag = _m_trackers[i].track_angle_flag;
            _m_trackers[i].track_angle_flag = angle_and_angle_flag.second;

            if (_m_trackers[i].first_true_angle_switch_flag == false && old_track_angle_flag == false && _m_trackers[i].track_angle_flag == true)
            {
                _m_trackers[i].first_true_angle_switch_flag = true;
                _m_trackers[i].true_angle_switch_flag = true;
            }
            else
            {
                if (_m_trackers[i].hits > 50)
                {
                    _m_trackers[i].first_true_angle_switch_flag = true;
                }
                _m_trackers[i].true_angle_switch_flag = false;
            }

            if (_m_trackers[i].track_angle_flag == false && _m_trackers[i].last_angle != -1000.0)
            {
                _m_trackers[i].track_angle = _m_trackers[i].last_angle;
            }

            // 根据配置文件确定车道角是否给航向角
            if ((*m_fusion_parameter)["fusion_param"]["flag_lane_angle2track_angle"])
            {
                if (_m_trackers[i].lane_angle != -1000 && (std::find(motor_vehicle_labels.begin(), motor_vehicle_labels.end(), _m_trackers[i]._type_fusion.fusion_label) != motor_vehicle_labels.end()))
                {
                    _m_trackers[i].track_angle = _m_trackers[i].lane_angle;
                }
            }

            // // 航向角平滑
            // if ((*m_fusion_parameter)["fusion_param"]["flag_smooth_angle"] && (!(_m_trackers[i].true_angle_switch_flag)))
            // {
            //     FunctionHub::smooth_angle(&_m_trackers[i], &motor_vehicle_labels);
            // }
            // 航向角平滑
            if ((*m_fusion_parameter)["fusion_param"]["flag_smooth_angle"] )
            {
                FunctionHub::smooth_angle(&_m_trackers[i], &motor_vehicle_labels);
            }

            // 保存最新的30帧的航向角
            std::vector<float> last_thirty_angle;
            if (int(_m_trackers[i].his_angle.size()) > 30)
            {
                last_thirty_angle.reserve(30);
                std::copy(_m_trackers[i].his_angle.end() - 30, _m_trackers[i].his_angle.end(), std::back_inserter(last_thirty_angle));
                _m_trackers[i].his_angle = last_thirty_angle;
            }
            std::cout << "-----------------------------------------------------------------------" << std::endl;
            std::cout << "---------------------------temp angle----->" << _m_trackers[i].track_angle << std::endl;
            std::cout << "---------------------------last angle----->" << _m_trackers[i].last_angle << std::endl;
            _m_trackers[i].final_angle = _m_trackers[i].track_angle;
            _m_trackers[i].his_angle.push_back(_m_trackers[i].final_angle);
            _m_trackers[i].last_angle = _m_trackers[i].final_angle;

            if ((_m_trackers[i].time_since_update > int((*m_fusion_parameter)["fusion_param"]["sort_max_age_new"]))) //   || ((_m_trackers[i].time_since_update > _m_trackers[i].hits) && (_m_trackers[i].time_since_update>2) )
            {
                pop_tracker_index.push_back(i);
            }
            else
            {

                live_tracker_index.push_back(i);
            }

            all_hits.push_back(_m_trackers[i].hits);
            
            // data_source
            int src = 0;
            if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 0){
                src = 1;
            }else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 0)
            {
                src = 2;
            }else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 1){
                src = 3;
            }else if(_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 0){
                src = 4;
            }else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 1){
                src = 5;
            }else if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 1){
                src = 6;
            }else if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 1){
                src = 7;
            }
            


            // bbox:shape:(n, 1)
            // _kf._x: shape:(n, 1)
            // 融合结果输出数据： [x, y, w, l, theta, z, h, label, speed, id, scores, x_min, y_min, x_max, y_max, data_source, channel, lane,occur_time, time_sicne_update, hits, flag_delete_track]
            xt::xarray<float> conv = {
                _m_trackers[i]._kf._x(0, 0),   // 0: x
                _m_trackers[i]._kf._x(1, 0),   // 1: y
                _m_trackers[i]._bbox(2),     // 2: w  bbox:shape:(n, 1)
                _m_trackers[i]._bbox(3),     // 3: l
                _m_trackers[i].final_angle, // 4: angle
                _m_trackers[i]._bbox(5),     // 5: z
                _m_trackers[i]._bbox(6),      // 6: h
                float(_m_trackers[i]._type_fusion.fusion_label),         // 7: label
                _m_trackers[i].speed,                  // 8: speed
                float(_m_trackers[i].id + 1), // 9: id
                float(_m_trackers[i]._type_fusion.fusion_score),                  // 10: score
                _m_trackers[i].update_pixel_location(0),                  // 11: x_min
                _m_trackers[i].update_pixel_location(1),                  // 12: y_min
                _m_trackers[i].update_pixel_location(2),                  // 13: x_max
                _m_trackers[i].update_pixel_location(3),                  // 14: y_max 
                float(src),                  // 15: data_source
                0,                  // 16: channel
                float(_m_trackers[i].lane),                  // 17: lane
                float(_m_trackers[i].occur_time),                  // 18: occur_time
                float(_m_trackers[i].time_since_update),  // 19: time_since_update
                float(_m_trackers[i].hits),               // 20: hits
                0                                         // 21: delete track flag
       

            };

            conv.reshape({1, conv.size()});
            // std::cout<<conv<<std::endl;
            // std::cout<<res.shape()[0]<<res.shape()[1]<<std::endl;
            xt::view(res,i,xt::all()) = xt::view(conv,0,xt::all());
        }


        // 删除轨迹
        if (pop_tracker_index.size() >0){
            for (int i = int(pop_tracker_index.size()-1); i>=0; i--)
            {
                int index = pop_tracker_index[i];
                _m_trackers.erase(_m_trackers.begin()+index);
            }
        }


        // 融合结果nms 融合结果nms V2
        // std::vector<int> nms_delete_index;
        // FunctionHub::result_nms(res, nms_delete_index, 0.3, 1, false);

        // if (nms_delete_index.size()>0)
        // {
        //     res = xt::view(res, xt::drop(nms_delete_index));
        // }

    }
    else
    {
        
        res = xt::empty<float>({0, 22});
        
    }

    std::cout << "liluo------------>"
              << "fusion_track_result_nums: " << res.shape(0) << std::endl;
}

void Update_tracks::parse_json()
{
    for (auto item : (*m_fusion_parameter)["fusion_param"]["camera_create_trackers"].items())
    {
        camera_create_trackers.push_back(item.value());
    }

    for (auto item : (*m_fusion_parameter)["fusion_param"]["motor_vehicle_labels"].items())
    {
        motor_vehicle_labels.push_back(item.value());
    }
}
