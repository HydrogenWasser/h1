/********
 文件名：Update_tracks_ServiceArea.cpp
 作者：Huahuan
 描述：更新轨迹
 版本：v1.0
 日期：2024.01.11
 *******/

#include "Update_tracks_ServiceArea.h"

// typedef std::unordered_map<int, std::pair<int, float>> chedao_type;


Update_tracks_ServiceArea::Update_tracks_ServiceArea(nlohmann::json *parameter,
                                                     std::unordered_map<int, std::pair<int, float>> *chedao,
                                                     float          x_limit[],
                                                     float          y_limit[])
{
    m_fusion_parameter = parameter;
    m_chedao = chedao;

    m_height = int((y_limit[1] - y_limit[0]) * 10);
    m_width = int((x_limit[1] - x_limit[0]) * 10);

    m_x_limit = x_limit;
    m_y_limit = y_limit;

    parse_json();
}


void Update_tracks_ServiceArea::create_tracks(fusion_match_out_ServiceArea                   *_match_out,
                                              std::vector<FUKalmanBoxTracker_ServiceArea>    &_m_trackers,
                                              unsigned long                                  timestamp,
                                              int                                            flag)
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
                float lane_angle = FunctionHub_ServiceArea::get_lane_info(bbox(0), 
                                                                          bbox(1), 
                                                                          m_width, 
                                                                          m_height, 
                                                                          m_x_limit, 
                                                                          m_y_limit, 
                                                                          m_chedao).second;
                                                                          
                if (lane_angle != -1000)
                {
                    bbox(4) = lane_angle;
                }


                float fake_object_1_x = -71.50;
                float fake_object_1_y = -3.30;
                float fake_object_2_x = -3.06;
                float fake_object_2_y = -6.72;
                float fake_object_3_x = -26.33;
                float fake_object_3_y = -5.83;

                float fake_object_4_x = -2.01;
                float fake_object_4_y = -6.43;

                float fake_object_5_x = 46.19;
                float fake_object_5_y = -20.26;

                float fake_object_6_x = 8.67;
                float fake_object_6_y = -8.38;

                float fake_object_7_x = 40.82;
                float fake_object_7_y = -26.40;

                float fake_object_8_x = -192.5;
                float fake_object_8_y = -33.32;


                bool real_flag_1 = true;
                bool real_flag_2 = true;
                bool real_flag_3 = true;
                bool real_flag_4 = true;
                bool real_flag_5 = true;
                bool real_flag_6 = true;
                bool real_flag_7 = true;
                bool real_flag_8 = true;


                if ((abs(bbox(0) - fake_object_1_x) < 8)   && (abs(bbox(1) - fake_object_1_y) < 3.5)){
                    real_flag_1 = false;
                }
                if ((abs(bbox(0) - fake_object_2_x) < 3.5) && (abs(bbox(1) - fake_object_2_y) < 3.5)){
                    real_flag_2 = false;
                }
                if ((abs(bbox(0) - fake_object_3_x) < 3.5) && (abs(bbox(1) - fake_object_3_y) < 3.5)){
                    real_flag_3 = false;
                }
                if ((abs(bbox(0) - fake_object_4_x) < 3.5) && (abs(bbox(1) - fake_object_4_y) < 3.5)){
                    real_flag_4 = false;
                }
                if ((abs(bbox(0) - fake_object_5_x) < 3.5) && (abs(bbox(1) - fake_object_5_y) < 3.5)){
                    real_flag_5 = false;
                }
                if ((abs(bbox(0) - fake_object_6_x) < 3.5) && (abs(bbox(1) - fake_object_6_y) < 3.5)){
                    real_flag_6 = false;
                }
                if ((abs(bbox(0) - fake_object_7_x) < 10)  && (abs(bbox(1) - fake_object_7_y) < 3.5)){
                    real_flag_7 = false;
                }
                if ((abs(bbox(0) - fake_object_8_x) < 3.5) && (abs(bbox(1) - fake_object_8_y) < 3.5)){
                    real_flag_8 = false;
                }


                if ((real_flag_1 && real_flag_2 && real_flag_3 && real_flag_4 && 
                     real_flag_5 && real_flag_6 && real_flag_7 && real_flag_8))
                {
                    _m_trackers.push_back(FUKalmanBoxTracker_ServiceArea(bbox, 
                                                                         timestamp, 
                                                                         m_fusion_parameter, 
                                                                         true, 
                                                                         false, 
                                                                         false));
                }
                else
                {
                    std::cout << "liluo-----------------------> fake object: " 
                              << "x: " << bbox(0) 
                              << ",  y: " << bbox(1) 
                              << std::endl;
                }

                // _m_trackers.push_back(FUKalmanBoxTracker(bbox, timestamp, m_fusion_parameter, true, false, false));
            }

            if (flag == 0)
            { // 相机 label:bbox(7)
                if (std::find(camera_create_trackers.begin(), camera_create_trackers.end(), int(bbox(7))) != camera_create_trackers.end())
                {
                    float lane_angle = FunctionHub_ServiceArea::get_lane_info(bbox(0), 
                                                                              bbox(1), 
                                                                              m_width, 
                                                                              m_height, 
                                                                              m_x_limit, 
                                                                              m_y_limit, 
                                                                              m_chedao).second;

                    if (lane_angle != -1000) bbox(4) = lane_angle;

                    _m_trackers.push_back(FUKalmanBoxTracker_ServiceArea(bbox, 
                                                                         timestamp, 
                                                                         m_fusion_parameter, 
                                                                         false, 
                                                                         true, 
                                                                         false));
                }
            }
        }
    }
}


void Update_tracks_ServiceArea::update(std::vector<FUKalmanBoxTracker_ServiceArea>   &_m_trackers, 
                                       xt::xarray<float>                             *fusion_track_result, 
                                       unsigned long                                 timestamp, 
                                       float                                         dt,
                                       std::map<int, int>                           parking_space_number)
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

            _m_trackers[i].update(timestamp, dt); // tracker更新

            // 查询轨迹的车道号和车道角
            std::pair<int, float> lane_lane_angle; // 对应multi_sensor_track.py的4414行
            lane_lane_angle = FunctionHub_ServiceArea::get_lane_info(_m_trackers[i]._kf._x(0, 0), 
                                                                     _m_trackers[i]._kf._x(1, 0), 
                                                                     m_width, 
                                                                     m_height, 
                                                                     m_x_limit, 
                                                                     m_y_limit, 
                                                                     m_chedao);
            _m_trackers[i].lane = lane_lane_angle.first;
            _m_trackers[i].lane_angle = lane_lane_angle.second;

            // 存储轨迹的状态值
            xt::xarray<float> current_state = {_m_trackers[i]._kf._x(0, 0), 
                                               _m_trackers[i]._kf._x(1, 0), 
                                               _m_trackers[i].speed};

            _m_trackers[i].state.push_back(current_state);
            std::vector<xt::xarray<float>> last_thirty_state;

            if (int(_m_trackers[i].state.size()) > 30)
            {
                last_thirty_state.reserve(30);
                std::copy(_m_trackers[i].state.end() - 30, 
                          _m_trackers[i].state.end(), 
                          std::back_inserter(last_thirty_state));
                _m_trackers[i].state = last_thirty_state;
            }

            std::pair<float, bool> angle_and_angle_flag;

            angle_and_angle_flag = FunctionHub_ServiceArea::cal_angle_new(&_m_trackers[i], 
                                                                          m_fusion_parameter); // 对应4443行，计算角度

            _m_trackers[i].track_angle = angle_and_angle_flag.first; // 计算角
            bool old_track_angle_flag = _m_trackers[i].track_angle_flag;
            _m_trackers[i].track_angle_flag = angle_and_angle_flag.second;

            
            if (_m_trackers[i].first_true_angle_switch_flag == false && 
                old_track_angle_flag == false && 
                _m_trackers[i].track_angle_flag == true)
            {
                _m_trackers[i].first_true_angle_switch_flag = true;
                _m_trackers[i].true_angle_switch_flag = true;
            }
            else
            {
                if (_m_trackers[i].hits > 50)
                    _m_trackers[i].first_true_angle_switch_flag = true;

                _m_trackers[i].true_angle_switch_flag = false;
            }

            if (_m_trackers[i].track_angle_flag == false && _m_trackers[i].last_angle != -1000.0)
            {
                _m_trackers[i].track_angle = _m_trackers[i].last_angle;
            }

            
            if ((*m_fusion_parameter)["fusion_param"]["flag_lane_angle2track_angle"])
            {
                auto entrance_coordinate = (*m_fusion_parameter)["fusion_param"]["entrance_coordinate"];
                if (entrance_coordinate[0] < current_state[0] && current_state[0] < entrance_coordinate[1] && // 在入口处用车道角覆盖，配置文件应提供入口范围坐标
                    entrance_coordinate[2] < current_state[1] && current_state[1] < entrance_coordinate[3])
                {
                    _m_trackers[i].track_angle = _m_trackers[i].lane_angle;
                }
            }

            // 航向角平滑
            if ((*m_fusion_parameter)["fusion_param"]["flag_smooth_angle"] )
            {
                FunctionHub_ServiceArea::smooth_angle(&_m_trackers[i], 
                                                      &motor_vehicle_labels);
            }

            // 保存最新的30帧的航向角
            std::vector<float> last_thirty_angle;
            if (int(_m_trackers[i].his_angle.size()) > 30)
            {
                last_thirty_angle.reserve(30);

                std::copy(_m_trackers[i].his_angle.end() - 30, 
                          _m_trackers[i].his_angle.end(), 
                          std::back_inserter(last_thirty_angle));

                _m_trackers[i].his_angle = last_thirty_angle;
            }

            std::cout << "-----------------------------------------------------------------------" << std::endl;
            std::cout << "---------------------------temp angle----->" << _m_trackers[i].track_angle << std::endl;
            std::cout << "---------------------------last angle----->" << _m_trackers[i].last_angle << std::endl;

            _m_trackers[i].final_angle = _m_trackers[i].track_angle;
            _m_trackers[i].his_angle.push_back(_m_trackers[i].final_angle);
            _m_trackers[i].last_angle = _m_trackers[i].final_angle;

            

            all_hits.push_back(_m_trackers[i].hits);
            
            // data_source
            int src = 0;
            if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 0)
            {
                src = 1;
            }
            else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 0)
            {
                src = 2;
            }
            else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 1){
                src = 3;
            }
            else if(_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 0){
                src = 4;
            }
            else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 1){
                src = 5;
            }
            else if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 1){
                src = 6;
            }
            else if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 1){
                src = 7;
            }

            // deal_lock_smooth
            m_parking_space_number = parking_space_number;
            auto smooth__result = deal_lock_smooth(_m_trackers[i], dt);

            bool isMoving = smooth__result.isMoving;
            int thresh_pop = (_m_trackers[i].hits > 150 && isMoving == false)? 10 : 1;
            if ((_m_trackers[i].time_since_update > int((*m_fusion_parameter)["fusion_param"]["sort_max_age_new"]) * thresh_pop) ||
                (_m_trackers[i].time_since_update > _m_trackers[i].hits && _m_trackers[i].time_since_update > 3)) 
            {
                pop_tracker_index.push_back(i);
            }
            else
            {
                live_tracker_index.push_back(i);
            }
            
            // bbox:shape:(n, 1)
            // _kf._x: shape:(n, 1)
            // 融合结果输出数据： [x, y, w, l, theta, z, h, label, speed, id, scores, x_min, y_min, x_max, y_max, data_source, channel, lane,occur_time, time_sicne_update, hits, flag_delete_track]
            xt::xarray<float> conv = {
                smooth__result.x,                                           // 0: x
                smooth__result.y,                                           // 1: y
                smooth__result.w,                                           // 2: w  bbox:shape:(n, 1)
                smooth__result.l,                                           // 3: l
                smooth__result.theta,                                       // 4: angle
                smooth__result.z,                                           // 5: z
                smooth__result.h,                                           // 6: h
                float(_m_trackers[i]._type_fusion.fusion_label),            // 7: label
                smooth__result.speed,                                       // 8: speed
                float(_m_trackers[i].id + 1),                               // 9: id
                float(_m_trackers[i]._type_fusion.fusion_score),            // 10: score
                _m_trackers[i].update_pixel_location(0),                    // 11: x_min
                _m_trackers[i].update_pixel_location(1),                    // 12: y_min
                _m_trackers[i].update_pixel_location(2),                    // 13: x_max
                _m_trackers[i].update_pixel_location(3),                    // 14: y_max 
                float(src),                                                 // 15: data_source
                0,                                                          // 16: channel
                float(_m_trackers[i].lane),                                 // 17: lane
                float(_m_trackers[i].occur_time),                           // 18: occur_time
                float(_m_trackers[i].time_since_update),                    // 19: time_since_update
                float(_m_trackers[i].hits),                                 // 20: hits
                0                                                           // 21: delete track flag
            };

            conv.reshape({1, conv.size()});
            // std::cout<<conv<<std::endl;
            // std::cout<<res.shape()[0]<<res.shape()[1]<<std::endl;
             xt::view(res,i,xt::all()) = xt::view(conv, 0, xt::all());
        }


        // 删除轨迹
        if (pop_tracker_index.size() >0){
            for (int i = int(pop_tracker_index.size()-1); i>=0; i--)
            {
                int index = pop_tracker_index[i];
                _m_trackers.erase(_m_trackers.begin() + index);
            }
        }

        res = xt::view(res, xt::keep(live_tracker_index)); // 去掉被删除的轨迹，然后发送所有轨迹
        

    }
    else
    {
        
        res = xt::empty<float>({0, 22});
        
    }

    std::cout << "liluo------------>"
              << "fusion_track_result_nums: " << res.shape(0) 
              << std::endl;
}


void Update_tracks_ServiceArea::parse_json()
{
    for (auto item : (*m_fusion_parameter)["fusion_param"]["camera_create_trackers"].items())
    {
        camera_create_trackers.push_back(item.value());
    }

    for (auto item : (*m_fusion_parameter)["fusion_param"]["motor_vehicle_labels"].items())
    {
        motor_vehicle_labels.push_back(item.value());
    }
    
    // ***************** 服务区新增参数 ***************** // 
    m_mean_xy_num = (*m_fusion_parameter)["fusion_param"]["mean_xy_num"]; 
    m_driving_frame_num = (*m_fusion_parameter)["fusion_param"]["driving_frame_num"];
    m_smooth_whipping_xy = (*m_fusion_parameter)["fusion_param"]["smooth_whipping_xy"];
    m_parking_speed_limit = (*m_fusion_parameter)["fusion_param"]["parking_speed_limit"];
    m_smooth_parking_xy = (*m_fusion_parameter)["fusion_param"]["smooth_parking_xy"];
    m_high_speed_for_changing_angle = (*m_fusion_parameter)["fusion_param"]["high_speed_for_changing_angle"];
    m_parking_angle_limit = (*m_fusion_parameter)["fusion_param"]["parking_angle_limit"];
    m_limit_l_dis = (*m_fusion_parameter)["fusion_param"]["limit_l_dis"];
    m_flag_use_chewei_angle = (*m_fusion_parameter)["fusion_param"]["flag_use_chewei_angle"];

    for (auto item : (*m_fusion_parameter)["fusion_param"]["blindArea"].items())
    {
        std::vector<float> blind_area;
        for (auto v : item.value().items())
		{
			blind_area.push_back(float(v.value()));
		}

        m_blind_area[item.key()] = blind_area;
        blind_area.clear();
    }

    for (auto item : (*m_fusion_parameter)["fusion_param"]["chewei_left_right_num"].items())
    {
        m_chewei_left_right_num.push_back(item.value());
    }
    
    for (auto item : (*m_fusion_parameter)["fusion_param"]["static_coefficient"].items())
    {
        m_static_coefficient.push_back(item.value());
    }
}

FusionTrackResult_ServiceArea Update_tracks_ServiceArea::deal_lock_smooth(FUKalmanBoxTracker_ServiceArea& trk,
                                                                          float                          dt)
{
    auto temp_x = trk._kf._x(0, 0); // x
    auto temp_y = trk._kf._x(1, 0); // y
    auto temp_z = trk._bbox(5);     // z

    auto temp_w = trk._bbox(2);     // w
    auto temp_l = trk._bbox(3);     // l
    auto temp_h = trk._bbox(6);     // h

    auto temp_speed = trk.speed;    // speed

    float temp_angle = -1000;   // angle
    
    // 从检测角和计算角中选择合适的角度
    if (temp_speed < 1.5 && trk.lidar_angle != -1000 && trk._type_fusion.fusion_score >= 0.4)
    {
        temp_angle = trk.lidar_angle;
        // std::cout <<
    }
    else
        temp_angle = trk.track_angle;

    auto origin_angle = temp_angle;

    bool isMoving = judge_motion_status(trk,
                                        temp_x, 
                                        temp_y, 
                                        temp_angle);

    z_lock(trk, temp_z);

    deal_wlh_lock(trk,
                  temp_w,
                  temp_l,
                  temp_h);

    bool angle_reversed = deal_angle_lock(trk, temp_angle, isMoving);

    if (isMoving)
    {
        auto blind_area = m_blind_area["3"]; // 这个理应放到json里面

        deal_blind(trk,
                   dt,
                   temp_x,
                   temp_y,
                   temp_speed,
                   temp_angle,
                   blind_area);
    }


    if (m_flag_use_chewei_angle)
    {
        auto modified_temp_angle = modify_chewei_angle(trk,
                                                       temp_angle,
                                                       isMoving,
                                                       temp_x,
                                                       temp_y);
        
        if (cos((modified_temp_angle - temp_angle) / 180 * PI) <= 0)
            temp_angle = fmod(modified_temp_angle + 180, 360);
        else
            temp_angle = modified_temp_angle;
    }

    if (isMoving)
    {
        if (angle_reversed)
        {
            trk.his_temp_angle.clear();
            trk.his_temp_angle.push_back(temp_angle);
        }
        else
        {
            if (trk.his_temp_angle.size() > 0)
                trk.his_temp_angle.back() = temp_angle; // 可以设置temp_angle为back()的引用，保持变动，后续再修改
            else
                trk.his_temp_angle.push_back(temp_angle);
        }
        temp_angle = FunctionHub_ServiceArea::cal_mean_angle_with_weight(trk.his_temp_angle);
        trk.his_temp_angle.back() = temp_angle; // 写的有点啰嗦，修改方式如上

        // 清空静止锁定参数
        trk.angle_locked = false;
        trk.coordinate_locked = false;
        trk.smooth_parking_temp_x.clear();
        trk.smooth_parking_temp_y.clear();
        trk.smooth_parking_speed.clear();
        trk.smooth_parking_angle.clear();
        trk.angle_rechange = -1000;
        trk.angle_change_count = 0;
    }
    else
    {
        if (trk.smooth_parking_temp_x.size() <= m_static_coefficient[0])
        {

            trk.smooth_parking_temp_x.push_back(temp_x);
            trk.smooth_parking_temp_y.push_back(temp_y);
            trk.smooth_parking_speed.push_back(temp_speed);
            trk.smooth_parking_angle.push_back(temp_angle);

            temp_x = FunctionHub_ServiceArea::cal_mean_container(trk.smooth_parking_temp_x);
            trk.static_temp_x = temp_x;
            temp_y = FunctionHub_ServiceArea::cal_mean_container(trk.smooth_parking_temp_y);
            trk.static_temp_y = temp_y;
            temp_speed = FunctionHub_ServiceArea::cal_mean_container(trk.smooth_parking_speed);
            temp_angle = FunctionHub_ServiceArea::cal_mean_angle_with_weight(trk.smooth_parking_angle);
            trk.static_temp_angle = temp_angle;
        
        }
        else
        {
            temp_x = trk.static_temp_x;
            temp_y = trk.static_temp_y;
            temp_speed = 0;
            temp_angle = trk.static_temp_angle;
        }

        // angle reset
        float diff_cos = cos((temp_angle - origin_angle) / 180 * PI);

        if (cos(angle_change_parameter[0] / 180 * PI) <= diff_cos &&
            diff_cos <= cos(angle_change_parameter[1] / 180 * PI))
        {
            trk.angle_rechange = trk.angle_rechange == -1000? origin_angle : trk.angle_rechange;
            float diff_cos_for_counting = cos((trk.angle_rechange - origin_angle) / 180 * PI);

            if (cos(angle_change_parameter[2] / 180 * PI) <= diff_cos_for_counting && diff_cos_for_counting <= 1)
                trk.angle_change_count += 1;
            else
            {
                trk.angle_change_count = 0;
                trk.angle_rechange = -1000;
            }
        }

        if (trk.angle_change_count >= m_static_coefficient[1])
        {
            trk.static_temp_angle = trk.angle_rechange;
            temp_angle = trk.static_temp_angle;
            trk.angle_rechange = -1000;
            trk.angle_change_count = 0;
        }

    }

    trk.update_kalman_coefficient(temp_x,
                                  temp_y,
                                  temp_speed,
                                  temp_angle,
                                  dt);

    trk.speed = temp_speed;
    trk.last_speed = temp_speed;
    trk.last_angle = temp_angle;

    FusionTrackResult_ServiceArea result;
    result.x = temp_x;
    result.y = temp_y;
    result.z = temp_z;

    result.w = temp_w;
    result.l = temp_l;
    result.h = temp_h;

    result.theta = temp_angle;
    result.speed = temp_speed;
    result.isMoving = isMoving;

    return result;
}


bool Update_tracks_ServiceArea::judge_motion_status(FUKalmanBoxTracker_ServiceArea&  trk,
                                                    const float&                    temp_x,
                                                    const float&                    temp_y,
                                                    const float&                    temp_angle)
{
    bool speed_flag,        // 速度标志位
         dist_flag,         // 距离标志位
         dist_nor_tan_flag; // 切向法向速度标志位

    if (trk.his_speed.size() >= m_driving_frame_num)
    {
        // 速度
        speed_flag = std::all_of(trk.his_speed.begin(), 
                                 trk.his_speed.end(), 
                                 [=](float &item){return item > m_parking_speed_limit;});
        // 切向和法向速度
        dist_nor_tan_flag = std::all_of(trk.his_nor_tan.begin(), 
                                        trk.his_nor_tan.end(), 
                                        [](int &item){return item > 0;});

        speed_flag = speed_flag && dist_nor_tan_flag;
        
    }
    else
        speed_flag = true; // 满足一定数据量之前应当都认为在运动中
        
    if (trk.his_temp_x.size() > m_mean_xy_num)
    {
        // 距离
        float mean_x = std::accumulate(trk.his_temp_x.begin(), trk.his_temp_x.end(), 0.0) / trk.his_temp_x.size();
        float mean_y = std::accumulate(trk.his_temp_y.begin(), trk.his_temp_y.end(), 0.0) / trk.his_temp_y.size();
        float distance = pow((mean_x - temp_x), 2) + pow((mean_y - temp_y), 2);
        dist_flag = distance >= m_smooth_whipping_xy? true : false;
    }
    else
        dist_flag = true;


    // 处理新增元素
    if (trk.his_temp_x.size() > m_mean_xy_num)
    {
        float distance = sqrt(
            pow((trk.his_temp_x.back() - temp_x), 2) + 
            pow((trk.his_temp_y.back() - temp_y), 2)
            );
        float dis_tan = abs(distance * cos((temp_angle - trk.his_temp_angle.back()) / 180 * PI));
        float dis_nor = abs(distance * sin((temp_angle - trk.his_temp_angle.back()) / 180 * PI));
    
        trk.his_nor_tan.push_back(dis_tan > dis_nor? 1 : 0);
    }
    else
        trk.his_nor_tan.push_back(0);

    trk.his_speed.push_back(trk.speed);  // 这里没有用temp_speed
    trk.his_temp_x.push_back(temp_x);
    trk.his_temp_y.push_back(temp_y);

    // 只保留driving_frame_num个历史数据
    if (trk.his_speed.size() > m_driving_frame_num)
    {
        trk.his_speed.pop_front();
        trk.his_nor_tan.pop_front();
    }

    // 只保留mean_xy_num个历史数据
    if (trk.his_temp_x.size() > m_mean_xy_num)
    {
        trk.his_temp_x.pop_front();
        trk.his_temp_y.pop_front();
    }
        
    return speed_flag && dist_flag;
}


void Update_tracks_ServiceArea::z_lock(FUKalmanBoxTracker_ServiceArea& trk,
                                       float& temp_z)
{
    if (trk.z_locked)
        temp_z = trk.mean_z;
    else if (trk.his_temp_z.size() >= m_smooth_parking_xy * 3)
    {
        temp_z = std::accumulate(trk.his_temp_z.begin(), trk.his_temp_z.end(), 0.0) / trk.his_temp_z.size();
        trk.mean_z = temp_z;
        trk.z_locked = true;
    }
    else
    {
        trk.his_temp_z.push_back(temp_z);
        temp_z = std::accumulate(trk.his_temp_z.begin(), trk.his_temp_z.end(), 0.0) / trk.his_temp_z.size();
    }
}


bool Update_tracks_ServiceArea::deal_angle_lock(FUKalmanBoxTracker_ServiceArea& trk,
                                                float&                          temp_angle, 
                                                const bool                      isMoving)
{
    bool angle_reversed = false;

    if (trk.his_temp_angle.size() > 0)
    {
        if (trk.speed <= m_high_speed_for_changing_angle)
        {
            if (cos((trk.his_temp_angle.back() -temp_angle) / 180 * PI <= 0))
                temp_angle =  fmod((temp_angle + 180), 360);
            
            if (isMoving && 
                cos(m_parking_angle_limit) >= cos((temp_angle - trk.his_temp_angle.back()) / 180 * PI))
            {
                float diff_cos = cos((temp_angle - trk.last_frame_angle) / 180 * PI);

                if (cos(10 / 180 * PI) <= diff_cos && diff_cos <= cos(0 / 180 * PI))
                    trk.chang_angle_count += 1;
                else
                    trk.chang_angle_count = 1;

                trk.last_frame_angle= temp_angle;

                if (trk.chang_angle_count == 3)
                    trk.chang_angle_count = 0; // 清空历史，应用当前角度
                else
                    temp_angle = trk.his_temp_angle.back();
            }
        }
        else
            temp_angle = cos((temp_angle - trk.track_angle) / 180 * PI) <= 0 ? fmod((temp_angle + 180), 360) : temp_angle;
    }

    trk.his_temp_angle.push_back(temp_angle);

    if (trk.his_temp_angle.size() >= m_smooth_parking_xy)
        trk.his_temp_angle.pop_front();

    if (trk.his_temp_angle.size() >= 2 &&
        cos((trk.his_temp_angle.back() - trk.his_temp_angle[trk.his_temp_angle.size()-2]) / 180 * PI) <= 0)
        angle_reversed = true;

    return angle_reversed;
}

void Update_tracks_ServiceArea::deal_wlh_lock(FUKalmanBoxTracker_ServiceArea& trk,
                                              float& temp_w, 
                                              float& temp_l, 
                                              float& temp_h)
{
    // 计算 wlh
    if (trk.wh_locked)
        temp_w = trk.mean_w;
    else if (trk.his_temp_w.size() >= m_smooth_parking_xy * 3)
    {
        temp_w = std::accumulate(trk.his_temp_w.begin(), trk.his_temp_w.end(), 0.0) / trk.his_temp_w.size();
        trk.mean_w = temp_w;

        temp_h = std::accumulate(trk.his_temp_h.begin(), trk.his_temp_h.end(), 0.0) / trk.his_temp_h.size();
        trk.mean_h = temp_h;

        trk.wh_locked = true;
    }
    else
    {
        trk.his_temp_w.push_back(temp_w);
        temp_w = std::accumulate(trk.his_temp_w.begin(), trk.his_temp_w.end(), 0.0) / trk.his_temp_w.size();

        trk.his_temp_h.push_back(temp_h);
        temp_h = std::accumulate(trk.his_temp_h.begin(), trk.his_temp_h.end(), 0.0) / trk.his_temp_h.size();
    }

    // 计算 l
    if (trk.l_locked)
        temp_l = trk.mean_l;
    else if (trk.his_temp_l.size() >= m_smooth_parking_xy * 3)
    {
        temp_l = std::accumulate(trk.his_temp_l.begin(), trk.his_temp_l.end(), 0.0) / trk.his_temp_l.size();
        trk.mean_l = temp_l;

        trk.l_locked = true;
    }
    else
    {
        trk.his_temp_l.push_back(temp_l);
        
        if (trk.his_temp_l.size() <= m_smooth_parking_xy * 2 ||
            trk.his_temp_l.size() < m_limit_l_dis)
            temp_l = std::accumulate(trk.his_temp_l.begin(), trk.his_temp_l.end(), 0.0) / trk.his_temp_l.size();
    }
}


void Update_tracks_ServiceArea::deal_blind(FUKalmanBoxTracker_ServiceArea&   trk,
                                           float                            dt,
                                           float&                           temp_x,
                                           float&                           temp_y,
                                           const float                      temp_speed,
                                           float&                           temp_angle,
                                           std::vector<float>&              blind_area)
{
    if (temp_x > blind_area[0] && temp_x < blind_area[1] &&
        temp_y > blind_area[1] && temp_y < blind_area[3])
        trk.blind_area_3_right_into += 1;

    if (trk.time_since_update != 0)
    {
        if (trk.blind_area_3_right_into > 50)
        {
            trk.track_angle = 65; // 这个地方要改成对应的lane_angle
            temp_angle = 65;
            auto x_axis_angle = fmod(-1 * temp_angle -90, 360) / 180 * PI;
            temp_x = trk.his_temp_x.back() + trk.his_speed.back() * cos(x_axis_angle) * dt; // 待修改，因为temp_x混用
            temp_y = trk.his_temp_y.back() + trk.his_speed.back() * sin(x_axis_angle) * dt; // 待修改
        }
    }

    if ((trk.time_since_update > int((*m_fusion_parameter)["fusion_param"]["sort_max_age"])))
        trk.blind_area_3_right_into = 0;
}


float Update_tracks_ServiceArea::modify_chewei_angle(FUKalmanBoxTracker_ServiceArea& trk,
                                                    float temp_angle, 
                                                    const bool isMoving,
                                                    const float& temp_x,
                                                    const float& temp_y)
{
    auto trk_id = trk.id;
    std::vector<int> left_id_list;
    std::vector<int> right_id_list;

    if (m_parking_space_number.find(trk_id + 1) != m_parking_space_number.end())
    {
        auto parking_num = int(m_parking_space_number.find(trk_id + 1)->first);
        if (parking_num != 0)
        {
            if (parking_num == m_chewei_left_right_num[0])
            {
                auto right_parking_num = parking_num + 1;
                auto left_parking_num = parking_num;

                left_id_list.push_back(trk_id + 1);

                for (auto iter : m_parking_space_number)
                {
                    if (iter.second == right_parking_num)
                        right_id_list.push_back(iter.first);
                }
            }
            else if (parking_num == m_chewei_left_right_num[1])
            {
                auto right_parking_num = parking_num;
                auto left_parking_num = parking_num - 1;

                right_id_list.push_back(trk_id + 1);

                for (auto iter : m_parking_space_number)
                {
                    if (iter.second == right_parking_num)
                        left_id_list.push_back(iter.first);
                }
            }
            else
            {
                auto right_parking_num = parking_num + 1;
                auto left_parking_num = parking_num - 1;

                for (auto iter : m_parking_space_number)
                {
                    if (iter.second == right_parking_num)
                        right_id_list.push_back(iter.first);
                }

                for (auto iter : m_parking_space_number)
                {
                    if (iter.second == right_parking_num)
                        left_id_list.push_back(iter.first);
                }
            }
        }

        if (left_id_list.size() != 0 || right_id_list.size() != 0)
            temp_angle = m_chewei_angle[(parking_num - 1)][5];

        if (isMoving == false &&
            m_special_chewei_num.find(m_parking_space_number.find(trk_id + 1)->second) != m_special_chewei_num.end())
            temp_angle = m_chewei_angle[(parking_num - 1)][5];
    }

    return temp_angle;
}






















