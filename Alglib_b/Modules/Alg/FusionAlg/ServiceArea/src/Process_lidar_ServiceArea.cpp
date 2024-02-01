/********
 文件名：IouAssociate_ServiceArea.cpp
 作者：Huahuan
 描述：IOU匹配
 版本：v1.0
 日期：2024.01.12
 *******/
#include "Process_lidar_ServiceArea.h"
#include "FunctionHub_ServiceArea.h"

typedef std::unordered_map<int, std::pair<int, float>> chedao_type;


Process_lidar_ServiceArea::Process_lidar_ServiceArea(nlohmann::json                 *parameter,
                                                     chedao_type                    *chedao,
                                                     fusion_match_out_ServiceArea   *match_out,
                                                     cal_dis_result_ServiceArea     *cal_dis_result, 
                                                     float                          x_limit[], 
                                                     float                          y_limit[])
{
    m_fusion_parameter = parameter;
    m_chedao = chedao;
    m_associate_out = match_out;
    m_cal_dis_result = cal_dis_result;

    m_height = int((y_limit[1] - y_limit[0]) * 10);
    m_width = int((x_limit[1] - x_limit[0]) * 10);
}


Process_lidar_ServiceArea::~Process_lidar_ServiceArea()
{

}


void Process_lidar_ServiceArea::process_lidar_data(xt::xarray<float> *_p_pPcResult)
{
    // 先清除上一帧match_out的数据
    xt::xarray<float> empty_data = xt::empty<float>({0, 0});

    m_associate_out->_cost_matrix = empty_data;
    m_associate_out->_matched_indices = empty_data;

    m_associate_out->_matched_detections_indices.clear();
    m_associate_out->_matched_trackers_indices.clear();

    m_associate_out->_unmatched_detections_indices.clear();
    m_associate_out->_unmatched_trackers_indices.clear();

    m_associate_out->_unmatched_detections = empty_data;
    m_associate_out->_unmatched_trackers = empty_data;
    m_associate_out->_matched_all_trks_id.clear();

    // 先清除上一帧的距离计算结果
    m_cal_dis_result->ano_dis = xt::empty<float>({0, 0});
    m_cal_dis_result->head_dis = xt::empty<float>({0, 0});
    m_cal_dis_result->square_dis = xt::empty<float>({0, 0});
    m_cal_dis_result->ano_dis_thres_matrix = xt::empty<float>({0, 0});
    m_cal_dis_result->head_dis_thres_matrix = xt::empty<float>({0, 0});

    lidar_box = *_p_pPcResult;
}


void Process_lidar_ServiceArea::excute_match_lidar_stage(std::vector<FUKalmanBoxTracker_ServiceArea>    &_trackers, 
                                                                     IouAssociate_ServiceArea           *iou_associate, 
                                                                     DistanceAssociate_ServiceArea      *dis_associate)
{
    track_box = xt::empty<float>({0, 8});
    int trackers_num = _trackers.size();
    
    if (trackers_num > 0)
    {
        // x, y, z, w, l, h, theta, speed;
        track_box = xt::zeros<float>({trackers_num, 8});
        std::vector<int> invalid_trackers_index;
        
        for (int i = 0; i < trackers_num; i ++)
        {
            track_box(i, 0) = _trackers[i]._kf._x(0, 0); // x
            track_box(i, 1) = _trackers[i]._kf._x(1, 0); // y
            track_box(i, 2) = _trackers[i]._bbox(5);  // z
            track_box(i, 3) = _trackers[i]._bbox(2);  // w
            track_box(i, 4) = _trackers[i]._bbox(3);  // l
            track_box(i, 5) = _trackers[i]._bbox(6);  // h

            float heading_angle;

            if (_trackers[i].last_angle != None && _trackers[i].last_angle != -1000)
                heading_angle = _trackers[i].last_angle;
            else
                heading_angle = _trackers[i]._bbox(4);  // angle

            track_box(i, 6) = heading_angle;      // theta
            track_box(i, 7) = _trackers[i].speed; // speed

            if (xt::any(xt::isnan(_trackers[i]._kf._x)))
                invalid_trackers_index.push_back(i);
        }

        // 去除无效的轨迹数据
        if (int(invalid_trackers_index.size()) > 0)
            track_box = xt::view(track_box, xt::drop(invalid_trackers_index));
    }

    /* ============================================================================================= */
    // python版本使用的associate_detections_to_trackers函数，
    // associate_detections_to_trackers返回值是 matches, 
    //                                         unmatched_detections_post, 
    //                                         unmatched_trackers_post, 
    //                                         cost_matrix
    /* =======================================  待完善  ============================================= */
    // 在这里判断检测结果或者轨迹结果是否有数据，如果检测或者没有轨迹数据，不用执行下面的步骤
    if (int(lidar_box.shape(0)) == 0)
    {   // 无检测数据情况
        // 直接更该匹配结果
        int track_box_num = track_box.shape(0);
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        m_associate_out->_unmatched_detections_indices.clear(); // = xt::empty<int>({0, 0});
        std::vector<int> track_index(track_box_num);
        iota(track_index.begin(), track_index.end(), 0);
        m_associate_out->_unmatched_trackers_indices = track_index; //xt::arange<int>(0, track_box_num);
        m_associate_out->_unmatched_detections = xt::empty<float>({0, 0});
    }

    if (int(track_box.shape(0)) == 0) // 无轨迹数据
    {
        // 直接更改匹配结果  xt::empty<int>({0, 2})
        int lidar_box_num = lidar_box.shape(0);
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        std::vector<int> lidar_index(lidar_box_num);
        iota(lidar_index.begin(), lidar_index.end(), 0);
        m_associate_out->_unmatched_detections_indices = lidar_index; //xt::arange<int>(0, lidar_box.shape(0));
        m_associate_out->_unmatched_trackers_indices.clear(); // = xt::empty<int>({0, 0});
        m_associate_out->_unmatched_detections = lidar_box;
    }

    if (int(lidar_box.shape(0)) != 0 && int(track_box.shape(0)) != 0)
    {
        // 执行IOU匹配过程
        // 1：关联之前进行数据预处理，服务区没有这个数据预处理 
        iou_associate->pre_process_associate_data(&lidar_box, &track_box, 1, m_fusion_parameter);

        // 计算距离
        // 2：计算cost值
        xt::xarray<float> cost_iou;
        // xt::xarray<float> cost_iou = iou_associate->cal_cost_matrix();
        iou_associate->cal_cost_matrix(cost_iou); 
        
        // 3: 执行匹配
        iou_associate->execute_match(cost_iou, m_associate_out, m_fusion_parameter); // 这里获取matched_indices
        // 4: 关联之后后处理,对应python版associate_detections_to_trackers
        iou_associate->post_process_associate_data(m_associate_out, &lidar_box, 1, m_fusion_parameter);
    }

    // 获取un_matched_indices
    std::vector<int> matched_all_dets_first_index = m_associate_out->_matched_detections_indices;
    std::vector<int> matched_all_trks_first_index = m_associate_out->_matched_trackers_indices;

    std::vector<int> unmatched_all_dets_first_index = m_associate_out->_unmatched_detections_indices;
    std::vector<int> unmatched_all_trks_first_index = m_associate_out->_unmatched_trackers_indices;

    xt::xarray<float> dets_for_second = m_associate_out->_unmatched_detections;

    xt::xarray<float> trks_for_second; 

    if (int(unmatched_all_trks_first_index.size()) > 0)
    {
        trks_for_second = xt::zeros<float>({int(unmatched_all_trks_first_index.size()), int(track_box.shape(1))});
        for(int i = 0; i < int((unmatched_all_trks_first_index.size())); i++)
        {
            xt::view(trks_for_second, i, xt::all()) = xt::view(track_box, unmatched_all_trks_first_index[i], xt::all());
        } 
    }
    else
    {
        trks_for_second = xt::empty<float>({0, 8});        
    }

    xt::xarray<int> matched_second_indices;
    xt::xarray<float> cost_matrix_second;

    if(int(trks_for_second.shape(0)) == 0){   // 第二阶段无轨迹数据

        matched_second_indices = xt::empty<int>({0, 2});

        std::vector<int> unmatched_detections_index_second(int(dets_for_second.shape(0)));
        iota(unmatched_detections_index_second.begin(), unmatched_detections_index_second.end(), 0);

        cost_matrix_second = xt::empty<float>({0, 0});

        m_associate_out->_cost_matrix = cost_matrix_second;
        

        m_associate_out->_unmatched_detections_indices = unmatched_detections_index_second;
    }
    if (int(dets_for_second.shape(0)) == 0){

        
        matched_second_indices = xt::empty<int>({0, 2});

        std::vector<int> unmatched_trackers_index_second(int(trks_for_second.shape(0)));
        iota(unmatched_trackers_index_second.begin(), unmatched_trackers_index_second.end(), 0);

        cost_matrix_second = xt::empty<float>({0, 0});
        m_associate_out->_cost_matrix = cost_matrix_second;

        // m_associate_out->_unmatched_trackers_indices = unmatched_trackers_index_second;
    }
    
    if(int(trks_for_second.shape(0)) != 0 && int(dets_for_second.shape(0)) != 0){

        dis_associate->pre_process_associate_data(&dets_for_second, 
                                                  &trks_for_second, 
                                                  1, 
                                                  m_fusion_parameter);

        dis_associate->cal_cost_matrix(cost_matrix_second);

        dis_associate->execute_match(cost_matrix_second, 
                                     m_associate_out, 
                                     m_fusion_parameter);

        dis_associate->post_process_associate_data(m_associate_out, 
                                                   &dets_for_second, 
                                                   1, 
                                                   m_fusion_parameter);
    }
}


void Process_lidar_ServiceArea::update_type(std::vector<FUKalmanBoxTracker_ServiceArea> &_m_trackers, 
                                            unsigned long                               pc_timestamp)
{
    
    int trackers_num = _m_trackers.size();
     
    if (trackers_num > 0)
    {
        for (int i = 0; i < trackers_num; i++)
        {
            if (std::find(m_associate_out->_unmatched_trackers_indices.begin(), m_associate_out->_unmatched_trackers_indices.end(), i) == m_associate_out->_unmatched_trackers_indices.end())
            {   // 表示i不在_unmatched_trackers_indices中,即检测和轨迹匹配成功了，就更新轨迹
                auto detection_indices = xt::where(xt::col(m_associate_out->_matched_indices, 1) > i - 1 && xt::col(m_associate_out->_matched_indices, 1) < i + 1)[0];
                if (int(detection_indices.size()) >0){
                    int index = detection_indices[0];
                    xt::xarray<float> bbox = xt::view(lidar_box, int(m_associate_out->_matched_indices(index, 0)), xt::all());
                    _m_trackers[i].updatewithlidar(bbox, pc_timestamp);
                }
               
            }
        }
    }
}


void Process_lidar_ServiceArea::excute_match_lidar_stage_3rd_match(std::vector<FUKalmanBoxTracker_ServiceArea>  &_trackers, 
                                                                   IouAssociate_ServiceArea                     *iou_associate, 
                                                                   DistanceAssociate_ServiceArea                *dis_associate)
{
    track_box = xt::empty<float>({0, 10});
    younger_track_box = xt::empty<float>({0, 10});
    elder_track_box = xt::empty<float>({0, 10});
    std::vector<int> vaild_younger;
    std::vector<int> vaild_elder;
    int trackers_num = _trackers.size();
    if (trackers_num > 0){
       
        
        // x, y, z, w，   l,  h,   theta, speed
        track_box = xt::zeros<float>({trackers_num, 10});
        std::vector<int> vaild_trackers_index;
        std::vector<int> younger_trackers_index;
        std::vector<int> elder_trackers_index;
        for (int i = 0; i < trackers_num; i++)
        {
           
            track_box(i, 0) = _trackers[i]._kf._x(0, 0); // x
            track_box(i, 1) = _trackers[i]._kf._x(1, 0); // y
            track_box(i, 2) = _trackers[i]._bbox(5);  // z
            track_box(i, 3) = _trackers[i]._bbox(2);  // w
            track_box(i, 4) = _trackers[i]._bbox(3);  // l
            track_box(i, 5) = _trackers[i]._bbox(6);  // h

            float heading_angle;
            if (_trackers[i].last_angle != None && _trackers[i].last_angle != -1000)
            {
                heading_angle = _trackers[i].last_angle;
            }
            else
            {
                heading_angle = _trackers[i]._bbox(4);  // angle
            }
            track_box(i, 6) = heading_angle;      // theta
            track_box(i, 7) = _trackers[i].speed; // speed
            track_box(i, 8) = float(_trackers[i]._type_fusion.fusion_label); // label
            track_box(i, 9) = _trackers[i].hits;
           
            if (!(xt::any(xt::isnan(_trackers[i]._kf._x))))
                vaild_trackers_index.push_back(i);
            
            if (_trackers[i].hits < int((*m_fusion_parameter)["fusion_param"]["young_hits_thresh"]))
                younger_trackers_index.push_back(i);
            else
                elder_trackers_index.push_back(i);
        }

        // /* ============================================================================================= */
        // // 去除无效的轨迹数据
        // if (int(invaild_trackers_index.size()) > 0)
        // {
        //     track_box = xt::view(track_box, xt::drop(invaild_trackers_index));
        // }
        /* ============================================================================================= */
        // 按hits划分跟踪结果
        vaild_younger.reserve(std::min(vaild_trackers_index.size(), younger_trackers_index.size()));
        vaild_elder.reserve(std::min(vaild_trackers_index.size(), elder_trackers_index.size()));
        set_intersection(vaild_trackers_index.begin(), vaild_trackers_index.end(), younger_trackers_index.begin(), younger_trackers_index.end(), back_inserter(vaild_younger));
        set_intersection(vaild_trackers_index.begin(), vaild_trackers_index.end(), elder_trackers_index.begin(), elder_trackers_index.end(), back_inserter(vaild_elder));
        younger_track_box = xt::view(track_box, xt::keep(vaild_younger));
        elder_track_box = xt::view(track_box, xt::keep(vaild_elder));
    }
    /* ============================================================================================= */

    /* =======================================  待完善  ============================================= */
    // Step1: elder_track与det进行IOU匹配
    // 在这里判断检测结果或者轨迹结果是否有数据，如果检测或者没有轨迹数据，不用执行下面的步骤
    if (int(lidar_box.shape(0)) == 0)
    { // 无检测数据情况
        // 直接更该匹配结果
        int track_box_num = elder_track_box.shape(0);
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        m_associate_out->_unmatched_detections_indices.clear(); // = xt::empty<int>({0, 0});
        std::vector<int> track_index(track_box_num);
        iota(track_index.begin(), track_index.end(), 0);
        m_associate_out->_unmatched_trackers_indices = track_index; //xt::arange<int>(0, track_box_num);
        m_associate_out->_unmatched_detections = xt::empty<float>({0, 0});
    }
    if (int(elder_track_box.shape(0)) == 0) // 无轨迹数据
    {
        // 直接更改匹配结果  xt::empty<int>({0, 2})
        int lidar_box_num = lidar_box.shape(0);
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        std::vector<int> lidar_index(lidar_box_num);
        iota(lidar_index.begin(), lidar_index.end(), 0);
        m_associate_out->_unmatched_detections_indices = lidar_index; //xt::arange<int>(0, lidar_box.shape(0));
        m_associate_out->_unmatched_trackers_indices.clear(); // = xt::empty<int>({0, 0});
        m_associate_out->_unmatched_detections = lidar_box;
    }
    if (int(lidar_box.shape(0)) != 0 && int(elder_track_box.shape(0)) != 0)
    {
        // 执行IOU匹配过程
        // 1：关联之前进行数据预处理
        iou_associate->pre_process_associate_data(&lidar_box, &elder_track_box, 1, m_fusion_parameter);
        // 2：计算cost值
        xt::xarray<float> cost_iou;
        // xt::xarray<float> cost_iou = iou_associate->cal_cost_matrix();
        iou_associate->cal_cost_matrix(cost_iou);
        
        // 3: 执行匹配
        iou_associate->execute_match(cost_iou, m_associate_out, m_fusion_parameter);
        // 4: 关联之后后处理
        iou_associate->post_process_associate_data(m_associate_out, &lidar_box, 1, m_fusion_parameter);
    }

    // Step2: elder_track与det进行dis匹配
    
    // std::vector<int> matched_all_dets_first_index = m_associate_out->_matched_detections_indices;
    // std::vector<int> matched_elder_trks_first_index = m_associate_out->_matched_trackers_indices;

    // std::vector<int> unmatched_all_dets_first_index = m_associate_out->_unmatched_detections_indices;
    std::vector<int> unmatched_elder_trks_first_index = m_associate_out->_unmatched_trackers_indices;

    xt::xarray<float> dets_for_second = m_associate_out->_unmatched_detections;

    xt::xarray<float> elder_trks_for_second; 
    if (int(unmatched_elder_trks_first_index.size()) > 0){
        elder_trks_for_second = xt::zeros<float>({int(unmatched_elder_trks_first_index.size()), int(elder_track_box.shape(1))});
        for(int i = 0; i < int((unmatched_elder_trks_first_index.size())); i++){
            xt::view(elder_trks_for_second, i, xt::all()) = xt::view(elder_track_box, unmatched_elder_trks_first_index[i], xt::all());
        }
    }else{
        
        elder_trks_for_second = xt::empty<float>({0, 8});
        
    }

    xt::xarray<float> cost_matrix_second;

    if(int(elder_trks_for_second.shape(0)) == 0){   // 第二阶段无轨迹数据


        // std::vector<int> unmatched_detections_index_second(int(dets_for_second.shape(0)));
        // iota(unmatched_detections_index_second.begin(), unmatched_detections_index_second.end(), 0);

        cost_matrix_second = xt::empty<float>({0, 0});
        m_associate_out->_cost_matrix = cost_matrix_second;
        
        // m_associate_out->_unmatched_detections_indices = unmatched_detections_index_second;
    }
    if (int(dets_for_second.shape(0)) == 0){

        
        // std::vector<int> unmatched_trackers_index_second(int(elder_trks_for_second.shape(0)));
        // iota(unmatched_trackers_index_second.begin(), unmatched_trackers_index_second.end(), 0);

        cost_matrix_second = xt::empty<float>({0, 0});
        m_associate_out->_cost_matrix = cost_matrix_second;

        // m_associate_out->_unmatched_trackers_indices = unmatched_trackers_index_second;
    }
    
    if(int(elder_trks_for_second.shape(0)) != 0 && int(dets_for_second.shape(0)) != 0){

        dis_associate->pre_process_associate_data(&dets_for_second, &elder_trks_for_second, 1, m_fusion_parameter);

        dis_associate->cal_cost_matrix(cost_matrix_second);

        dis_associate->execute_match(cost_matrix_second, m_associate_out, m_fusion_parameter);

        dis_associate->post_process_associate_data(m_associate_out, &dets_for_second, 1, m_fusion_parameter);
    }

    // Step3: younger_track与det进行dis匹配

    std::vector<int> unmatched_all_dets_index = m_associate_out->_unmatched_detections_indices;
    std::vector<int> unmatched_elder_trks_index = m_associate_out->_unmatched_trackers_indices;

    std::vector<int> matched_all_dets_index = m_associate_out->_matched_detections_indices;
    std::vector<int> matched_elder_trks_index = m_associate_out->_matched_trackers_indices;

    // xt::xarray<int> matched_eldertrks_alldets_index = m_associate_out->_matched_indices;

    xt::xarray<float> dets_for_third = m_associate_out->_unmatched_detections;
    xt::xarray<float> younger_trks_for_third = younger_track_box;

    if (int(dets_for_third.shape(0)) == 0) // 没有待匹配的检测目标
    {
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});

        // _matched_indices不变，但要把elder_trks的idx还原到原track_box的idx
        if (int(matched_all_dets_index.size()) <= 0){
            m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        }
        else{
            std::vector<int> matched_all_trks_index; 
            for(int i = 0; i < int((matched_elder_trks_index.size())); i++){
                matched_all_trks_index.push_back(vaild_elder[matched_elder_trks_index[i]]);
            }
            assert(matched_all_dets_index.size() == matched_all_trks_index.size());
            std::vector<std::size_t> shape = {matched_all_dets_index.size(), 1};
            xt::xarray<int> xt_matched_det_index = xt::adapt(matched_all_dets_index, shape);
            xt::xarray<int> xt_matched_trk_index = xt::adapt(matched_all_trks_index, shape);
            m_associate_out->_matched_indices = xt::concatenate(xt::xtuple(xt_matched_det_index, xt_matched_trk_index), 1);
            // _matched_trackers_indices同样要把elder_trks的idx还原到原track_box的idx
            m_associate_out->_matched_trackers_indices = matched_all_trks_index;
        }

        m_associate_out->_unmatched_detections_indices.clear();

        // _unmatched_trackers_indices=unmatched_elder_trks_index+vaild_younger,
        // 同样要把elder_trks的idx还原到原track_box的idx
        std::vector<int> track_index;
        for(int i = 0; i < int((unmatched_elder_trks_index.size())); i++){
            track_index.push_back(vaild_elder[unmatched_elder_trks_index[i]]);
        }
        track_index.insert(track_index.end(), vaild_younger.begin(), vaild_younger.end());
        m_associate_out->_unmatched_trackers_indices = track_index;

        m_associate_out->_unmatched_detections = xt::empty<float>({0, 0});
    }

    if (int(younger_trks_for_third.shape(0)) == 0)  // 没有待匹配的跟踪目标
    {
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});

        // _matched_indices不变，但要把elder_trks的idx还原到原track_box的idx
        if (int(matched_all_dets_index.size()) <= 0){
            m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        }
        else{
            std::vector<int> matched_all_trks_index; 
            for(int i = 0; i < int((matched_elder_trks_index.size())); i++){
                matched_all_trks_index.push_back(vaild_elder[matched_elder_trks_index[i]]);
            }
            assert(matched_all_dets_index.size() == matched_all_trks_index.size());
            std::vector<std::size_t> shape = {matched_all_dets_index.size(), 1};
            xt::xarray<int> xt_matched_det_index = xt::adapt(matched_all_dets_index, shape);
            xt::xarray<int> xt_matched_trk_index = xt::adapt(matched_all_trks_index, shape);
            m_associate_out->_matched_indices = xt::concatenate(xt::xtuple(xt_matched_det_index, xt_matched_trk_index), 1);
            // _matched_trackers_indices同样要把elder_trks的idx还原到原track_box的idx
            m_associate_out->_matched_trackers_indices = matched_all_trks_index;
        }
        
        // _unmatched_trackers_indices同样要把elder_trks的idx还原到原track_box的idx
        std::vector<int> track_index;
        for(int i = 0; i < int((unmatched_elder_trks_index.size())); i++){
            track_index.push_back(vaild_elder[unmatched_elder_trks_index[i]]);
        }
        m_associate_out->_unmatched_trackers_indices = track_index;
    }
    
    xt::xarray<float> cost_matrix_third;

    if(int(younger_trks_for_third.shape(0)) != 0 && int(dets_for_third.shape(0)) != 0){

        dis_associate->pre_process_associate_data(&dets_for_third, &younger_trks_for_third, 1, m_fusion_parameter);

        dis_associate->cal_cost_matrix(cost_matrix_third);

        dis_associate->execute_match(cost_matrix_third, m_associate_out, m_fusion_parameter);

        dis_associate->post_process_associate_data_third(m_associate_out, &dets_for_third, 1, m_fusion_parameter, &vaild_elder, &vaild_younger);
    }

}