/*******************************************************
 文件名：process_camera_ServiceArea.h
 作者：HuaHuan
 描述：图像检测数据和轨迹关联匹配
 版本：v1.0
 日期：2024-01-11
 *******************************************************/

#include "Process_camera_ServiceArea.h"

typedef std::unordered_map<int, std::pair<int, float>> chedao_type;


Process_camera_ServiceArea::Process_camera_ServiceArea(nlohmann::json               *parameter,
                                                       chedao_type                  *chedao,
                                                       fusion_match_out_ServiceArea *match_out, 
                                                       cal_dis_result_ServiceArea   *cal_dis_result, 
                                                       float                        x_limit[], 
                                                       float                        y_limit[])
{
    m_fusion_parameter = parameter;
    m_chedao = chedao;
    m_associate_out = match_out;
    m_cal_dis_result = cal_dis_result;
    load_csv("Configs/Alg/CppConfigs/fusion/ServiceArea/reflect/camera_reflect_limit.csv");

    m_height = int((y_limit[1] - y_limit[0]) * 10);
    m_width = int((x_limit[1] - x_limit[0]) * 10);
}


Process_camera_ServiceArea::~Process_camera_ServiceArea()
{

}


void Process_camera_ServiceArea::process_camera_data(std::vector<xt::xarray<float>> *_p_pCameraResult)
{
    xt::xarray<float> empty_data = xt::empty<float>({0, 0});
    // 先清除上一帧match_out的数据
    m_associate_out->_cost_matrix = empty_data;
    m_associate_out->_matched_indices = empty_data;
    
    m_associate_out->_matched_detections_indices.clear();
    m_associate_out->_matched_trackers_indices.clear();
    
    m_associate_out->_unmatched_detections_indices.clear();
    m_associate_out->_unmatched_trackers_indices.clear();

    m_associate_out->_unmatched_detections = empty_data;
    m_associate_out->_unmatched_trackers = empty_data;

    m_associate_out->_matched_all_trks_id.clear();

    // 先清除上一帧的 距离计算 结果
    m_cal_dis_result->ano_dis = empty_data;
    // std::cout << "liluo-----------> 111111111111  000000000000" << std::endl;
    m_cal_dis_result->head_dis = empty_data;
    // std::cout << "liluo-----------> 111111111111  111111111111111: "<< std::endl << m_cal_dis_result->square_dis << std::endl;
    m_cal_dis_result->square_dis = empty_data;
    // std::cout << "liluo-----------> 111111111111  2222222222222" << std::endl;
    m_cal_dis_result->ano_dis_thres_matrix = empty_data;
    // std::cout << "liluo-----------> 111111111111  3333333333333" << std::endl;
    m_cal_dis_result->head_dis_thres_matrix = empty_data;
    // std::cout << "liluo-----------> 111111111111  44444444444444" << std::endl;
    std::vector<xt::xarray<float>> video_for_fusion_all_channel = *_p_pCameraResult;
    // std::cout << "liluo-----------> 111111111111  555555555555" << std::endl;
    int camera_dev_num = video_for_fusion_all_channel.size();
    

    // 把n路的视频检测结果放到xt::xarray<float> camera_box 中
    if (video_for_fusion_all_channel.size() == camera_dev_num)
    {
        int all_channel_box_num = 0;

        for (auto it = video_for_fusion_all_channel.begin(); it != video_for_fusion_all_channel.end(); it++)
        {
            all_channel_box_num += it->shape(0);
        }

        camera_box = xt::zeros<float>({all_channel_box_num, 17});
        int last_box_num = 0;

        for (auto it = video_for_fusion_all_channel.begin(); it != video_for_fusion_all_channel.end(); it++)
        {
            int each_box_num = it->shape(0);

            xt::view(camera_box, xt::range(last_box_num, 
                                           last_box_num + each_box_num), 
                                           xt::all())
                = xt::view(*it, xt::all(), xt::all());

            last_box_num += each_box_num;
        }
    }

    // nms
    std::vector<int> delete_index;

    FunctionHub_ServiceArea::result_nms(camera_box, 
                                        delete_index, 
                                        float((*m_fusion_parameter)["fusion_param"]["camera_nms_min_threshold"]), 
                                        float((*m_fusion_parameter)["fusion_param"]["camera_nms_max_threshold"]), 
                                        true);
}


void Process_camera_ServiceArea::excute_match_camera_stage(std::vector<FUKalmanBoxTracker_ServiceArea>  &_trackers, 
                                                           IouAssociate_ServiceArea                     *iou_associate, 
                                                           DistanceAssociate_ServiceArea                *dis_associate)
{
    track_box = xt::empty<float>({0, 8});

    int trackers_num = _trackers.size();

    if (trackers_num > 0){
        // x, y, z, w，   l,  h,   theta, id
        
        track_box = xt::zeros<float>({trackers_num, 8});
        std::vector<int> invaild_trackers_index;
        for (int i = 0; i < trackers_num; i++)
        {
            

            track_box(i, 0) = _trackers[i]._kf._x(0, 0); // x
            track_box(i, 1) = _trackers[i]._kf._x(1, 0); // y
            track_box(i, 2) = _trackers[i]._bbox(5);     // z
            track_box(i, 3) = _trackers[i]._bbox(2);     // w
            track_box(i, 4) = _trackers[i]._bbox(3);     // l
            track_box(i, 5) = _trackers[i]._bbox(6);     // h
            
            float heading_angle;

            if (_trackers[i].last_angle != None && _trackers[i].last_angle != -1000)
            {
                heading_angle = _trackers[i].last_angle;
            }
            else
            {
                heading_angle = _trackers[i]._bbox(4); // angle
            }

            track_box(i, 6) = heading_angle;      // theta
            track_box(i, 7) = _trackers[i].id + 1; // id

            if (xt::any(xt::isnan(_trackers[i]._kf._x)))
                invaild_trackers_index.push_back(i);
        }
        // 去除无效的轨迹数据
        if (int(invaild_trackers_index.size()) > 0)
        {
            track_box = xt::view(track_box, xt::drop(invaild_trackers_index));
        }

    }
    
    // 在这里判断检测结果或者轨迹结果是否有数据，如果检测或者没有轨迹数据，不用执行下面的步骤
    if (int(camera_box.shape(0)) == 0)
    { // 无检测数据情况
        // 直接更该匹配结果
        int track_box_num = track_box.shape(0);
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        m_associate_out->_unmatched_detections_indices.clear(); // = xt::empty<int>({0, 0});
        std::vector<int> track_index(track_box_num);
        iota(track_index.begin(), track_index.end(), 0);
        m_associate_out->_unmatched_trackers_indices = track_index; // xt::arange<int>(0, track_box_num);
        m_associate_out->_unmatched_detections = xt::empty<float>({0, 0});
    }
    else if (int(track_box.shape(0)) == 0) // 无轨迹数据
    {
        // 直接更改匹配结果  xt::empty<int>({0, 2})
        int camera_box_num = camera_box.shape(0);
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        std::vector<int> camera_index(camera_box_num);
        iota(camera_index.begin(), camera_index.end(), 0);
        m_associate_out->_unmatched_detections_indices = camera_index; // xt::arange<int>(0, lidar_box.shape(0));
        m_associate_out->_unmatched_trackers_indices.clear();          // = xt::empty<int>({0, 0});
        m_associate_out->_unmatched_detections = camera_box;
        
    }
    else if (int(camera_box.shape(0)) != 0 && int(track_box.shape(0)) != 0)
    {
        // 执行IOU匹配过程
        // 1：关联之前进行数据预处理
        iou_associate->pre_process_associate_data(&camera_box, &track_box, 0, m_fusion_parameter);
        // 2：计算cost值
        xt::xarray<float> cost_iou;
        
        iou_associate->cal_cost_matrix(cost_iou);

        // 3: 执行匹配
        iou_associate->execute_match(cost_iou, m_associate_out, m_fusion_parameter);
        // 4: 关联之后后处理
        iou_associate->post_process_associate_data(m_associate_out, &camera_box, 0, m_fusion_parameter);
    }
    // std::cout<<"zly4444444444444444444444444444444"<<std::endl;
    // 执行距离匹配
    std::vector<int> matched_all_dets_first_index = m_associate_out->_matched_detections_indices;
    std::vector<int> matched_all_trks_first_index = m_associate_out->_matched_trackers_indices;

    std::vector<int> unmatched_all_dets_first_index = m_associate_out->_unmatched_detections_indices;
    std::vector<int> unmatched_all_trks_first_index = m_associate_out->_unmatched_trackers_indices;

    xt::xarray<float> dets_for_second = m_associate_out->_unmatched_detections;

    xt::xarray<float> trks_for_second;
    if (int(unmatched_all_trks_first_index.size()) > 0)
    {
        trks_for_second = xt::zeros<float>({int(unmatched_all_trks_first_index.size()), int(track_box.shape(1))});
        for (int i = 0; i < int((unmatched_all_trks_first_index.size())); i++)
        {
            xt::view(trks_for_second, i, xt::all()) = xt::view(track_box, unmatched_all_trks_first_index[i], xt::all());
        }
    }
    else
    {

        trks_for_second = xt::empty<float>({0, 8});
    }

    xt::xarray<int> matched_second_indices; // 声明第二阶段匹配索引
    xt::xarray<float> cost_matrix_second;   // 声明第二阶段损失矩阵

    if (int(trks_for_second.shape(0)) == 0)
    { // 第二阶段无轨迹数据

        matched_second_indices = xt::empty<int>({0, 2});

        std::vector<int> unmatched_detections_index_second(int(dets_for_second.shape(0)));
        iota(unmatched_detections_index_second.begin(), unmatched_detections_index_second.end(), 0);

        cost_matrix_second = xt::empty<float>({0, 0});

        m_associate_out->_cost_matrix = cost_matrix_second;

        m_associate_out->_unmatched_detections_indices = unmatched_detections_index_second;
        
    }

    if (int(dets_for_second.shape(0)) == 0)
    { // 第二阶段无检测数据

        matched_second_indices = xt::empty<int>({0, 2});

        std::vector<int> unmatched_trackers_index_second(int(trks_for_second.shape(0)));
        iota(unmatched_trackers_index_second.begin(), unmatched_trackers_index_second.end(), 0);

        cost_matrix_second = xt::empty<float>({0, 0});
        m_associate_out->_cost_matrix = cost_matrix_second;

        // m_associate_out->_unmatched_trackers_indices = unmatched_trackers_index_second;
    }

    if (int(trks_for_second.shape(0)) != 0 && int(dets_for_second.shape(0)) != 0)
    {

        dis_associate->pre_process_associate_data(&dets_for_second, 
                                                  &trks_for_second, 
                                                  0, 
                                                  m_fusion_parameter);

        dis_associate->cal_cost_matrix(cost_matrix_second);

        dis_associate->execute_match(cost_matrix_second, 
                                     m_associate_out, 
                                     m_fusion_parameter);

        // annotation by zqj todo
        dis_associate->post_process_associate_data(m_associate_out, 
                                                   &dets_for_second, 
                                                   0, 
                                                   m_fusion_parameter);
    }

    std::vector<int> matched_all_trks_id;   // 存储所有匹配上的目标轨迹id
    for(int i = 0; i< m_associate_out->_matched_trackers_indices.size(); i++){
        matched_all_trks_id.push_back(int(track_box(m_associate_out->_matched_trackers_indices[i], 7)));
    }
    m_associate_out->_matched_all_trks_id = matched_all_trks_id;

    // 把和轨迹匹配上的相机检测结果id改成轨迹id
    int matched_num = m_associate_out->_matched_detections_indices.size();
    for (int i = 0; i < matched_num; i++){
        int box_index = m_associate_out->_matched_detections_indices[i];
        camera_box(box_index, 9) = matched_all_trks_id[i];
    }
}   


void Process_camera_ServiceArea::update_type(std::vector<FUKalmanBoxTracker_ServiceArea> &_trackers, 
                                             unsigned long                               timestamp)
{
    int trackers_num = _trackers.size();

    if (trackers_num > 0)
    {
        for (int i = 0; i < trackers_num; i++)
        {

            if (std::find(m_associate_out->_unmatched_trackers_indices.begin(), m_associate_out->_unmatched_trackers_indices.end(), i) == m_associate_out->_unmatched_trackers_indices.end())
            { // 表示i不在_unmatched_trackers_indices中,即检测和轨迹匹配成功了，就更新轨迹
                auto detection_indices = xt::where(xt::col(m_associate_out->_matched_indices, 1) > i - 1 && xt::col(m_associate_out->_matched_indices, 1) < i + 1)[0];
                if (int(detection_indices.size()) > 0)
                {
                    int index = detection_indices[0];
                    // _m_trackers[i].label = _m_trackers[i]._bbox(7);
                    xt::xarray<float> bbox = xt::view(camera_box, int(m_associate_out->_matched_indices(index, 0)), xt::all());
                    _trackers[i].updatewithcamera(bbox, timestamp);
                    // _m_trackers[i].last_lidar_state =  xt::view(lidar_box, int(m_associate_out->_matched_indices(index, 0)), xt::all());
                    // _m_trackers[i].lidar_updated = 1;
                }
            }
        }
    }
}


void Process_camera_ServiceArea::load_csv(std::string _file_path)
/*
    加载csv,把csv存储的数据转换成xt::xarray
*/
{
    std::ifstream file(_file_path, std::ios::in);
    if (!file.good())
    {
        perror("can't open file");
    }
    std::string line;
    int row = 0;
    int column = 0;
    std::vector<int> data;
    while (std::getline(file, line))
    {
        /* code */
        column = 0;
        row++;

        // if(row == 1){
        // 	continue;
        // }

        std::stringstream ss(line);
        float value;

        // std::vector<float> row_data;

        while (ss >> value)
        {
            /* code */
            // std::cout << column << std::endl;
            data.push_back(value);
            if (ss.peek() == ',')
            {
                ss.ignore();
            }
            column++;
        }

        // data.push_back(row_data);
    }
    file.close();

    // for(int i = 0; i<data.size(); i++){
    // 	for (int j = 0; j<data[i].size(); j++){
    // 		std::cout << data[i][j] << " ";
    // 	}
    // 	std::cout << "\n";
    // }

    m_camera_reflect_limit = xt::adapt(data, {row, column});
}


void Process_camera_ServiceArea::getCameraBoxInfo(std::vector<xt::xarray<float>> *_p_pCameraResult)
{
    int nVideoCount = _p_pCameraResult->size();
    
    float image_size = float((*m_fusion_parameter)["fusion_param"]["IMAGE_SIZE"]);

    // 直接发送相机原始检测结果
    if ((*m_fusion_parameter)["fusion_param"]["video_for_show_flag"]){
        for (int i = 0; i < nVideoCount; i++){

            xt::xarray<float> current_box = (*_p_pCameraResult)[i];
            // xt::view(current_box, xt::all(), 11) *= (image_size / 1920.0);
            // xt::view(current_box, xt::all(), 12) *= (image_size / 1080.0);
            // xt::view(current_box, xt::all(), 13) *= (image_size / 1920.0);
            // xt::view(current_box, xt::all(), 14) *= (image_size / 1080.0);

            _p_pCameraResult->at(i) = current_box;
        }
    }else{  // 给相机的检测结果加上匹配结果
        for (int i = 0; i < nVideoCount; i++){
            int index = i+1;
            auto reverse_index = xt::where(xt::col(camera_box, 16) >= index && xt::col(camera_box, 16) <= index)[0];
            xt::xarray<float> video_for_fusion_single_channel = xt::view(camera_box, xt::keep(reverse_index));

            
            // xt::view(video_for_fusion_single_channel, xt::all(), 11) *= (image_size / 1920.0);
            // xt::view(video_for_fusion_single_channel, xt::all(), 12) *= (image_size / 1080.0);
            // xt::view(video_for_fusion_single_channel, xt::all(), 13) *= (image_size / 1920.0);
            // xt::view(video_for_fusion_single_channel, xt::all(), 14) *= (image_size / 1080.0);

            auto matched_index = xt::where(xt::col(video_for_fusion_single_channel, 9) > 0)[0];
            xt::view(video_for_fusion_single_channel, xt::keep(matched_index), 15) = 2; // 2: 融合
            _p_pCameraResult->at(i) = video_for_fusion_single_channel;
        }
    }
}
