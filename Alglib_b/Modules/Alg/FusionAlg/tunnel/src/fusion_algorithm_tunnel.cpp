//
// Created by root on 5/8/21.
//

#include "fusion_algorithm_tunnel.h"
#include "hungarian_bigraph_matcher.h"
#include <iostream>
#include <xtensor/xnorm.hpp>
#include "hungarian.h"

Fusion_Algorithm_TunnelAll::Fusion_Algorithm_TunnelAll(TSelfFusionAlgParam *m_stAlgParam){
    // 读取融合算法配置文件
    YAML::Node fusion_cfg = YAML::LoadFile(m_stAlgParam->m_strRootPath + m_stAlgParam->m_strFusionCfgPath);   

    m_stAlgParam->m_stFusionAlgParam.m_vecPcClass = {fusion_cfg["TUNNEL_FUSION"]["PC_CLASS_NAMES"][0].as<std::string>(),
                                                    fusion_cfg["TUNNEL_FUSION"]["PC_CLASS_NAMES"][1].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["PC_CLASS_NAMES"][2].as<std::string>(),
                                                    fusion_cfg["TUNNEL_FUSION"]["PC_CLASS_NAMES"][3].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["PC_CLASS_NAMES"][4].as<std::string>(),
                                                    fusion_cfg["TUNNEL_FUSION"]["PC_CLASS_NAMES"][5].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["PC_CLASS_NAMES"][6].as<std::string>()};

    m_stAlgParam->m_stFusionAlgParam.m_vecVideoClass = {fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][0].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][1].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][2].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][3].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][4].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][5].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][6].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][7].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][8].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][9].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][10].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][11].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][12].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][13].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][14].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][15].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][16].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][17].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["VIDEO_CLASS_NAMES"][18].as<std::string>()};

    m_stAlgParam->m_stFusionAlgParam.m_vecFusionClass = {fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][0].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][1].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][2].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][3].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][4].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][5].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][6].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][7].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][8].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][9].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][10].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][11].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][12].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][13].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][14].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][15].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][16].as<std::string>(),
                                                        fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][17].as<std::string>(), fusion_cfg["TUNNEL_FUSION"]["FUSION_CLASS_NAMES"][18].as<std::string>()};
    // 融合算法参数
    std::string tunnel_config_path = m_stAlgParam->m_strRootPath + fusion_cfg["TUNNEL_FUSION_CONF_PATH"].as<std::string>();

    m_pWlhEstimator = std::shared_ptr<WLH_Estimator>(new WLH_Estimator);
    for (auto i = 0; i < m_stAlgParam->m_stCameraParam.m_vecCameraDev.size(); ++i)
    {
        auto& l_stCameraDev = m_stAlgParam->m_stCameraParam.m_vecCameraDev[i];
        auto& l_stSuiDaoCameraParam = m_stAlgParam->m_stCameraParam.m_vecCameraDev[i].m_stSuiDaoVideoParam;
        auto& l_stLidarDev = m_stAlgParam->m_stLidarParam.m_vecLidarDev[0];

        //相机内外参
        stCameraParam l_stCameraParam;
        std::vector<std::size_t> shape = {3,3};
        l_stCameraParam.m_xarrInParam = xt::adapt(l_stCameraDev.m_vecInParameter, shape);
        shape = {3};
        l_stCameraParam.m_xarrRotateMatrix = xt::adapt(l_stCameraDev.m_vecRotateMatrix, shape);
        shape = {3};
        l_stCameraParam.m_xarrTranslationMatrix = xt::adapt(l_stCameraDev.m_vecTranslationMatrix, shape);
        shape = {5};
        l_stCameraParam.m_xarrDistMatrix = xt::adapt(l_stCameraDev.m_vecDistMatrix, shape);

        m_vecAlgHandles.emplace_back(new Fusion_Algorithm_tunnel(tunnel_config_path, i+1, m_stAlgParam->m_ucStationId, l_stCameraParam));
        m_vecLaneJudger.emplace_back(new Box_Bottom_Lane_Judger(tunnel_config_path, i+1,
                                                                l_stSuiDaoCameraParam.m_nBorderSx,
                                                                l_stSuiDaoCameraParam.m_nBorderSy,
                                                                l_stSuiDaoCameraParam.m_nBorderEx,
                                                                l_stSuiDaoCameraParam.m_nBorderEy));
        m_vecCoordinateEstimator.emplace_back(new Match_Pairs_Coordinate_Estimator(tunnel_config_path, i+1,
                                                                                    l_stSuiDaoCameraParam.m_nBorderY,
                                                                                    m_stAlgParam->m_ucStationId,
                                                                                    l_stSuiDaoCameraParam.m_fGroundZ,
                                                                                    l_stLidarDev.m_dLon,
                                                                                    l_stLidarDev.m_dLat,
                                                                                    l_stLidarDev.m_fAngle));

        m_vecCameraMap.emplace_back(new Camera_Map(l_stCameraParam.m_xarrInParam, l_stCameraParam.m_xarrRotateMatrix, l_stCameraParam.m_xarrTranslationMatrix, l_stCameraParam.m_xarrDistMatrix));
        m_vecFromAndTo.emplace_back(m_stAlgParam->m_stCameraParam.m_vecCameraDev[i].m_stSuiDaoVideoParam.m_nFromAndTo);
    }
}

/*
*Parameters
*    video : 保存"视频检测框左上角x,视频检测框左上角y,视频检测框右下角x,视频检测框右下角y,
*            检测框类别, 检测框置信度"                                 #idx : 6         
*            的列表
*            没有单位，均为像素值
*    data :  保存"点云检测框中心点x,点云检测框中心点y,点云检测框中心点z,点云检测框宽,点云检测框长, 点云检测框高, 航向角, 
*            类别, 速度, 点云跟踪ID号, 置信度，                                 
*            角点1的x, 角点1的y,角点1的z, 角点2的x, 角点2的y, 角点2的z, 角点3的x, 角点3的y, 角点3的z, 角点4的x, 
*            角点4的y, 角点4的z, 角点5的x, 角点5的y, 角点5的z, 角点6的x, 角点6的y, 角点6的z
*            角点7的x, 角点7的y, 角点7的z, 角点8的x, 角点8的y, 角点8的z"的列表      #idx : 35
*            单位为米
 *   _trk:x y w l angle(rad) z h label speed(-1) blanket conf blanket ...
*    mapSign : 是否使用二维点反映射到激光雷达坐标系功能
*/

Fusion_Res Fusion_Algorithm_TunnelAll::RUN(std::vector<xt::xarray<float>> &video,
                xt::xarray<float> &data,
                int cam_num){
    //clear history data
    
    _video_all_ret.clear();
    // std::cout<< "tunnel start" << std::endl;

    //点云结果
    // 点云11, x,y,z,w,l,h,角度，类别，速度，ID，置信度
    xt::xarray<float> temp_lidar1 = xt::view(data, xt::all(), xt::range(0, 2));  //x,y
    xt::xarray<float> temp_lidar2 = xt::view(data, xt::all(), xt::range(3, 5));  //w,l
    xt::xarray<float> temp_lidar3 = xt::view(data, xt::all(), xt::range(6, 7));  //角度
    xt::xarray<float> temp_lidar4 = xt::view(data, xt::all(), xt::range(2, 3));  //z
    xt::xarray<float> temp_lidar5 = xt::view(data, xt::all(), xt::range(5, 6));  //h
    xt::xarray<float> temp_lidar6 = xt::view(data, xt::all(), xt::range(7, 11));  //类别，速度，ID，置信度
    xt::xarray<float> temp_empty = xt::zeros<int>({(int)data.shape(0), 10});
    _pc_ret = xt::concatenate(xt::xtuple(temp_lidar1,temp_lidar2,temp_lidar3,temp_lidar4,temp_lidar5,temp_lidar6,temp_empty), 1);

    int l_nVideoTargetTotalNum = 0;
    // auto camera_id = 0;
    // for (auto &v : video_results.vecResult()) 
    
    for (int j = 0; j < cam_num; ++j)
    {
        // auto camera_id = v.ucCameraId();
        //分类切片
        xt::xarray<float> l_xarrBox = xt::view(video[j], xt::all(), xt::range(0, 4));
        //目标框原始图像坐标 
        xt::xarray<float> l_xarrOrginBox = l_xarrBox;
        xt::view(l_xarrOrginBox, xt::all(), 0) *= (1920 / 640.0);
        xt::view(l_xarrOrginBox, xt::all(), 1) *= (1080 / 640.0);
        xt::view(l_xarrOrginBox, xt::all(), 2) *= (1920 / 640.0);
        xt::view(l_xarrOrginBox, xt::all(), 3) *= (1080 / 640.0);
        l_xarrOrginBox = xt::clip(l_xarrOrginBox, xt::xarray<int>({0, 0, 0, 0}), xt::xarray<int>({1920 - 1, 1080 - 1, 1920 - 1, 1080 - 1}));
        //类别
        xt::xarray<int> l_xarrClass = xt::col(video[j], 4);
        //置信度
        xt::xarray<float> l_xarrConfidence = xt::zeros<float>({int(video[j].shape(0)), 1});
        l_xarrConfidence = xt::col(video[j], 5);
        //目标单基站检测的ID
        xt::xarray<int> l_xarrSingleId = xt::zeros<int>({int(video[j].shape(0)), 1});
        l_xarrSingleId = xt::col(video[j], 6);
        //目标宽长高
        xt::xarray<float> l_xarrWHL = xt::zeros<int>({int(video[j].shape(0)), 3});
        m_pWlhEstimator->get_wlh(l_xarrClass, l_xarrWHL);
        xt::xarray<int> l_xarrLanNos = xt::zeros<int>({int(video[j].shape(0)), 1});
        l_xarrLanNos = m_vecLaneJudger[j]->get_lane_no(l_xarrOrginBox, m_vecFromAndTo[j]);
        //目标空间坐标
        xt::xarray<float> l_xarrXYZ = xt::zeros<int>({int(video[j].shape(0)), 3});
        l_xarrXYZ = m_vecCoordinateEstimator[j]->get_xyz(l_xarrOrginBox, l_xarrClass, l_xarrLanNos);
        //条件过滤
        int l_nCount = 0;
        for (int i = 0; i < l_xarrXYZ.shape(0); ++i) 
        {
            if (l_xarrXYZ(i, 0) != 0)
            {
                xt::view(l_xarrBox, l_nCount) = xt::view(l_xarrBox, i);
                l_xarrConfidence(l_nCount) = l_xarrConfidence(i);
                l_xarrClass(l_nCount) = l_xarrClass(i);
                l_xarrSingleId(l_nCount) = l_xarrSingleId(i);
                xt::view(l_xarrWHL, l_nCount) = xt::view(l_xarrWHL, i);
                l_xarrLanNos(l_nCount) = l_xarrLanNos(i);
                xt::view(l_xarrXYZ, l_nCount) = xt::view(l_xarrXYZ, i);
                l_nCount++;
            }
        }
        l_xarrBox = xt::view(l_xarrBox, xt::range(0, l_nCount));
        l_xarrConfidence = xt::view(l_xarrConfidence, xt::range(0, l_nCount));
        l_xarrClass = xt::view(l_xarrClass, xt::range(0, l_nCount));
        l_xarrSingleId = xt::view(l_xarrSingleId, xt::range(0, l_nCount));
        l_xarrWHL = xt::view(l_xarrWHL, xt::range(0, l_nCount));
        l_xarrLanNos = xt::view(l_xarrLanNos, xt::range(0, l_nCount));
        l_xarrXYZ = xt::view(l_xarrXYZ, xt::range(0, l_nCount));

        auto &p_refAlgHandle = m_vecAlgHandles[j];
        xt::xarray<float> l_xarrVideo = StackDet(l_xarrBox, l_xarrClass, l_xarrConfidence, l_xarrSingleId);
        xt::xarray<float> tempp = xt::zeros<float>({int(data.shape(0)), 5});
        if (data.shape(1) < 40)
        {
            data = xt::concatenate(xt::xtuple(data, tempp), 1);
        }
        // std::cout<< "***" << j << " fusion before _pc_ret.shape:   "  << _pc_ret.shape(0) << ", " << _pc_ret.shape(1) << std::endl;

        // p_refAlgHandle->fusion(l_xarrVideo, data, pc_result.unFrameId(), l_xarrXYZ, l_xarrWHL, l_xarrSingleId, l_xarrLanNos);
        xt::xarray<float> video_ret = xt::zeros<float>({(int)(l_xarrVideo.shape(0)), 8});
        // std::cout<< "***" << j << " fusion before video_ret.shape:   "  << video_ret.shape(0) << ", " << video_ret.shape(1) << std::endl;
        p_refAlgHandle->fusion(l_xarrVideo, data, l_xarrXYZ, l_xarrWHL, l_xarrSingleId, l_xarrLanNos, video_ret, _pc_ret);

        // std::cout<< "***" << j << "fusion after _pc_ret.shape:   "  << _pc_ret.shape(0) << ", " << _pc_ret.shape(1) << std::endl;
        // std::cout<< "***" << j << " fusion after video_ret.shape:   "  << video_ret.shape(0) << ", " << video_ret.shape(1) << std::endl;
        
        l_nVideoTargetTotalNum = l_nVideoTargetTotalNum + video[j].shape(0);
        _video_all_ret.push_back(video_ret);

        // setFusionResult(pc_result, v, p_refAlgHandle->output_result());
        // camera_id++;
    }  
    // std::cout<< "all _pc_ret.shape:   "  << _pc_ret.shape(0) << ", " << _pc_ret.shape(1) << std::endl;
    _output.pc_res = _pc_ret;
    // std::cout<< "all _video_ret.size:   "  << _video_ret.size() << std::endl;
    _output.video_res = _video_all_ret;
    // std::cout<< "tunnel end "  << std::endl;
    return _output;              
}

Fusion_Algorithm_tunnel::Fusion_Algorithm_tunnel(std::string config_path, int cam_id, int station_id, const stCameraParam& p_refCameraParam):
    _cam_id(cam_id),
    _config_path(config_path),
    _station_id(station_id),
    // _frame(0),
    _frame_dict(0),
    _frame_update(2000)
{
    _coordinate_map = Coordinate_Map_tunnel(_config_path, p_refCameraParam, _cam_id, _ground_height, _station_id);
    _load_config();
    _getRotationVector = {0, 0, 0};
    _getRotationVector = _getRotationVector * M_PI / 180.0;

    plane_matrix();
    _count = 0;
    _middle_line_2d = _coordinate_map._middle_line_2d;
    _bianxian_2d = _coordinate_map._bianxian_2d;
    _new_mid_img_xy_pc_xylonlat = _coordinate_map._middle_xyz_to_xy;
    _dingwei_count = 0;

    _class_size["car"] = xt::xarray<float>{{1.95017717, 4.60718145, 1.72270761}};
    _class_size["bicycle"] = xt::xarray<float>{{0.60058911, 1.68452161, 1.27192197}};
    _class_size["middle_truck"] = xt::xarray<float>{{2.94046906, 11.1885991, 3.47030982}};
    _class_size["motorcycle"] = xt::xarray<float>{{0.76279481, 2.09973778, 1.44403034}};
    _class_size["person"] = xt::xarray<float>{{0.66344886, 0.7256437, 1.75748069}};
    _class_size["middle_bus"] = xt::xarray<float>{{2.4560939, 6.73778078, 2.73004906}};
}

void Fusion_Algorithm_tunnel::plane_matrix() {
    _R = _coordinate_map.eulerAnglesToRotationMatrix(_getRotationVector);
    xt::xarray<double> inverse_R = xt::linalg::inv(_R);
    xt::xarray<double> orginal_N = {0, 0, 1};
    orginal_N = orginal_N.reshape({3,1});
    xt::xarray<double> new_N = xt::linalg::dot(inverse_R, orginal_N);
    new_N = new_N.reshape({1, new_N.size()});
    _para_plane = -1 * new_N / (new_N(0, 2) * _ground_height);
}

void Fusion_Algorithm_tunnel::_load_config() {
    switch (_cam_id) {
        case 1:
            _iou_threshold = 0.01;
            _default_angle = 90;
            _map_mode = 3;      // map mode 1:city 2:tunnel 3:lan
            _map_trans_x = 0;
            _map_trans_y = 0.5;
            _target_safe_dis = 50;
            _ground_height = -4.9;

//            _horizontal_offset_1 = 0;
//            _vertical_offset_1 = 0;
//            _horizontal_offset_2 = 0;
//            _vertical_offset_2 = 0;
//            _horizontal_offset_long_object = 0;
//            _vertical_offset_long_object = 0.15;
//            _default_angle = 90;
            break;
        case 2:
            _iou_threshold = 0.01;
            _default_angle = 90;
            _map_mode = 3;      // map mode 1:city 2:tunnel 3:lan
            _map_trans_x = 0;
            _map_trans_y = 0.5;
            _target_safe_dis = 60;
            _ground_height = -4.8;
            break;
        case 3:
            _iou_threshold = 0.01;
            _default_angle = 90;
            _map_mode = 3;      // map mode 1:city 2:tunnel 3:lan
            _map_trans_x = 0;
            _map_trans_y = 0.5;
            _target_safe_dis = 75;
            _ground_height = -4.7;
            break;
        case 4:
            _iou_threshold = 0;
            _default_angle = 0;
            _map_mode = 3;      // map mode 1:city 2:tunnel 3:lan
            _map_trans_x = 0;
            _map_trans_y = 0.5;
            _target_safe_dis = 75;
            _ground_height = -4.7;
            break;
    }
}

// void Fusion_Algorithm_tunnel::fusion(xt::xarray<float> &video, xt::xarray<float> &data, int frameId, xt::xarray<float> &xyz,
//                               xt::xarray<float> &wlh, xt::xarray<int> &ids, xt::xarray<int> &lane_no, bool mapSign) 
void Fusion_Algorithm_tunnel::fusion(xt::xarray<float> &video, xt::xarray<float> &data, xt::xarray<float> &xyz,
                                    xt::xarray<float> &wlh, xt::xarray<int> &ids, xt::xarray<int> &lane_no, 
                                    xt::xarray<float> &video_ret, xt::xarray<float> &pc_ret,
                                    bool mapSign) 
{
    // auto l_s = std::chrono::high_resolution_clock::now();
    // auto l_sall = std::chrono::high_resolution_clock::now();

    _output_index.clear();

    _output_infor = xt::empty<float>({0, 39});
    _lidtovid_inf = xt::empty<float>({0, 82});
    _last_lidar_position = xt::empty<float>({0, 10});
    if (_map_mode == 1){
        _infor_list = xt::empty<float>({0, 49});
    }
    else if (_map_mode == 2 or _map_mode == 3) {
        _infor_list = xt::empty<float>({0, 81});
    }

    if (video.shape().size() != 2){
        video = xt::empty<float>({0, 7});
    }
    if (data.shape().size() != 2){
        data = xt::empty<float>({0, 40});
    }
    if (xyz.shape().size() != 2){
        xyz = xt::empty<float>({0, 3});
    }
    if (wlh.shape().size() != 2){
        wlh = xt::empty<float>({0, 3});
    }

    _boxes_list = xt::view(video, xt::all(), xt::range(0, 4));
    _scores_list = xt::view(video, xt::all(), 5);
    _labels_list = xt::cast<int>(xt::view(video, xt::all(), 4));
    _ids_list = xt::view(video, xt::all(), 6);
    _data = data;
    xt::xarray<int> match_flag_video = xt::zeros<int>({_boxes_list.shape(0)});

    // auto l_e = std::chrono::high_resolution_clock::now();
    // auto l_ulT8 = std::chrono::duration<double, std::milli>(l_e - l_s).count();
    // l_s = std::chrono::high_resolution_clock::now();
    // std::cout << " =========================== fusion delay 1= " << l_ulT8 << std::endl;

    // _frame = frameId;
    data_analysis();
    _camlidar_org = _camlidar;

    _count_fusion = 0;

    ids = ids.reshape({ids.size(), 1});
    lane_no = lane_no.reshape({lane_no.size(), 1});

    if (xyz.shape(0) == 0){
        _lan_target = xt::empty<float>({0, 5});
    } else{
        _lan_target = xt::hstack(xt::xtuple(xyz, wlh, lane_no, ids));
    }

    // l_e = std::chrono::high_resolution_clock::now();
    // l_ulT8 = std::chrono::duration<double, std::milli>(l_e - l_s).count();
    // l_s = std::chrono::high_resolution_clock::now();
    // std::cout << " =========================== fusion delay 2= " << l_ulT8 << std::endl;

    if (_lidar_box_list.shape(0) > 0){
        _lidar_box_list = _lidar_box_list.reshape({int(_lidar_box_list.size() / 4), 4});
        xt::xarray<float> cost;
        if (_boxes_list.shape(0) != 0){
            _boxes_list = _boxes_list.reshape({int(_boxes_list.size() / 4), 4});
            cost = box_ops_tunnel::iou_jit(_lidar_box_list, _boxes_list);
        } else{
            _boxes_list = xt::empty<float>({0, 4});
            cost = xt::empty<float>({int (_lidar_box_list.shape(0)), 0});
        }
        // l_e = std::chrono::high_resolution_clock::now();
        // l_ulT8 = std::chrono::duration<double, std::milli>(l_e - l_s).count();
        // l_s = std::chrono::high_resolution_clock::now();
        // std::cout << " =========================== fusion delay 3= " << l_ulT8 << std::endl;

        xt::xarray<float> cost_matrix = cost;
        xt::xarray<int> matches = run_hungarian_match<float>(cost_matrix);


        // auto l_e = std::chrono::high_resolution_clock::now();
        // auto l_ulT8 = std::chrono::duration<double, std::milli>(l_e - l_s).count();
        // l_s = std::chrono::high_resolution_clock::now();
        // std::cout << " =========================== fusion delay 4= " << l_ulT8 << std::endl;

        std::vector<int> indicss;
        for (int i = 0; i < _indicss.size(); ++i) {
            if (_indicss(i))
                indicss.push_back(i);
        }

        std::vector<int> delete_matches;
        for (int i = 0; i < _camlidar.shape(0); ++i) {
            _camlidar(i, 7) = fusion_utils::lidar_class_transform(_camlidar(i, 7));
        }
        for (int j = 0; j < _labels_list.size(); ++j) {
            _labels_list(j) = fusion_utils::video_class_transform(_labels_list(j));
        }

        // l_e = std::chrono::high_resolution_clock::now();
        // l_ulT8 = std::chrono::duration<double, std::milli>(l_e - l_s).count();
        // l_s = std::chrono::high_resolution_clock::now();
        // std::cout << " =========================== fusion delay 5= " << l_ulT8 << std::endl;

        // std::cout << "matches:" << matches << std::endl;
        // std::cout << "cost:" << cost << std::endl;
        
        for (int k = 0; k < matches.shape(0); ++k) {
            int i = matches(k, 0);
            int j = matches(k, 1);
            if (cost(i, j) > _iou_threshold){
                // _match_flag_video[_cam_id-1](j, 0) = 1;
                // std::cout << "i:" << i << ", j:" << j << std::endl;
                _count_fusion++;
                match_flag_video(j) = 1;
                float lidar_class = _camlidar(i, 7);
                float video_class = _labels_list(j);
                _lidar_confidience = _camlidar(i, 10);
                _video_confidience = _scores_list(j);

                float final_class = fusion_utils::judge_class(video_class, lidar_class, _video_confidience, _lidar_confidience);
                float label_max = final_class;
                _camlidar(i, 7) = label_max;
                _labels_list(j) = int(label_max);
                xt::xarray<float> video_input_temp;

                if (int (_map_mode) == 3){
                    xt::xarray<float> central_points = xt::view(_lan_target, j, xt::range(0, 3));
                    xt::xarray<float> size = xt::view(_lan_target, j, xt::range(3, 6));
                    xt::xarray<float> video_input_temp_temp = {float (_default_angle / 180 * M_PI),
                                                               float (_labels_list(j)),
                                                               0,
                                                               _ids_list(j),
                                                               _scores_list(j),
                                                               0, 0, 0, 0, 0};
                    video_input_temp = xt::hstack(xt::xtuple(central_points, size, video_input_temp_temp));
                }
                else{
                    // TODO: not use until now
                }

                _output_index.push_back({i, int (indicss[i])});



                // zhanghan, 28/3/2022
                // 原始输出 _output_infor
                // xt::xarray<float> boxes_list_temp = xt::view(_boxes_list, j);
                // xt::xarray<float> output_infor_temp = {_scores_list(j), float (_labels_list(j)), _ids_list(j)};
                // xt::xarray<float> camlidar_org_temp = xt::view(_camlidar_org, i, xt::range(0, 11));
                // xt::xarray<float> temp = xt::concatenate(xt::xtuple(boxes_list_temp, output_infor_temp, camlidar_org_temp, video_input_temp), 0);
                // temp = temp.reshape({1, 39});
                // _output_infor = xt::concatenate(xt::xtuple(_output_infor, temp), 0);

                pc_ret(int (indicss[i]), 16) = _boxes_list(j, 0);
                pc_ret(int (indicss[i]), 17) = _boxes_list(j, 1);
                pc_ret(int (indicss[i]), 18) = _boxes_list(j, 2);
                pc_ret(int (indicss[i]), 19) = _boxes_list(j, 3);
                pc_ret(int (indicss[i]), 20) = _cam_id;

                _dict_match = {_ids_list(j), _camlidar(i, 9)};
                if (_vidmatch_history.count(int(_ids_list(j))) == 0){
                    _vidmatch_history[int(_ids_list(j))] = {};
                }
                if ( _vidmatch_history.count(int(_camlidar(i, 9))) == 0){
                    _vidmatch_history[int(_ids_list(j))][int(_camlidar(i, 9))] = 0;
                }
                _vidmatch_history[int(_ids_list(j))][int(_camlidar(i, 9))] += 1;
            }
            else {
                delete_matches.push_back(k);
            }
        }
        // **********************修改后的输出 start*********************
        // std::cout<< "video_ret.shape" << video_ret.shape(0) << video_ret.shape(1) << std::endl;
        if (video.shape(0) > 0){
            // x1, y1, x2, y2, score, label, match_flag, id
            for (int i = 0; i<_boxes_list.shape(0); ++i){
                video_ret(i, 0) = _boxes_list(i, 0);
                video_ret(i, 1) = _boxes_list(i, 1);
                video_ret(i, 2) = _boxes_list(i, 2);
                video_ret(i, 3) = _boxes_list(i, 3);
                video_ret(i, 4) = _scores_list(i);
                video_ret(i, 5) = _labels_list(i);
                video_ret(i, 6) = match_flag_video(i);
                video_ret(i, 7) = _ids_list(i);
            }
            // std::cout<< "end video_ret.shape" << video_ret.shape(0) << video_ret.shape(1) << std::endl;
        }

        // **********************修改后的输出 end ********************

        // l_e = std::chrono::high_resolution_clock::now();
        // l_ulT8 = std::chrono::duration<double, std::milli>(l_e - l_s).count();
        // l_s = std::chrono::high_resolution_clock::now();
        // std::cout << " =========================== fusion delay 6= " << l_ulT8 << std::endl;

        xt::xarray<int> matches_new = xt::zeros<int>({int(matches.shape(0) - delete_matches.size()), 2});
        int count = 0;
        for (int i = 0; i < matches.shape(0); ++i) {
            if (std::find(delete_matches.begin(), delete_matches.end(), i) == delete_matches.end()){
                xt::view(matches_new, count) = xt::view(matches, i);
                count++;
            }
        }

        if (_frame_dict > _frame_update){
            _vidmatch_history.clear();
            _id_with_chedao.clear();
            _frame_dict = _frame_dict - _frame_update;
            // _lidar_pos_history.clear();
        }

        std::vector<int> matched_lidar(matches_new.shape(0));
        for (int i = 0; i < matches_new.shape(0); ++i) {
            matched_lidar[i] = matches_new(i, 0);
        }

        for (int i = 0; i < _lidar_box_list.shape(0); ++i) {
            if (std::find(matched_lidar.begin(), matched_lidar.end(), i) == matched_lidar.end()){
                int video_class = int(_camlidar(i, 7));
                xt::xarray<float> video_input_temp = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0};
                xt::xarray<float> lid2vid_inf_temp = {
                        _lidar_box_list(i, 0),_lidar_box_list(i, 1),_lidar_box_list(i, 2),_lidar_box_list(i, 3),
                        0.4,
                        float (video_class),
                        _camlidar(i, 9), 0
                };

                xt::xarray<float> camlidar_temp = xt::view(_camlidar, i);
                xt::xarray<float> temp = {float (i), float (indicss[i])};
                xt::xarray<float> camlidar_org_temp = xt::view(_camlidar_org, i, xt::range(0, 16));
                xt::xarray<float> lid2vid_inf = xt::hstack(xt::xtuple(lid2vid_inf_temp, camlidar_temp, temp, camlidar_org_temp, video_input_temp));

                lid2vid_inf = lid2vid_inf.reshape({1, 82});
                _lidtovid_inf = xt::concatenate(xt::xtuple(_lidtovid_inf, lid2vid_inf), 0);;
            }
        }

        // l_e = std::chrono::high_resolution_clock::now();
        // l_ulT8 = std::chrono::duration<double, std::milli>(l_e - l_s).count();
        // l_s = std::chrono::high_resolution_clock::now();
        // std::cout << " =========================== fusion delay 7= " << l_ulT8 << std::endl;


        if (mapSign){
            std::vector<int> matched_video(matches_new.shape(0));
            for (int i = 0; i < matches_new.shape(0); ++i) {
                matched_video[i] = matches_new(i, 1);
            }
            for (int j = 0; j < _boxes_list.shape(0); ++j) {
                if (std::find(matched_video.begin(), matched_video.end(), j) == matched_video.end()){
                    if (_map_mode == 1){
                        xt::xarray<float> points_remap = {(_boxes_list(j, 0) + _boxes_list(j, 2)) / 2, _boxes_list(j, 3)};
                        xt::xarray<float> size = _class_size[vid2lidsize(int (_labels_list(j)))];
                        auto res_map = _coordinate_map.map_2dto3d_transform(points_remap, _cam_id, _para_plane, size);
                        xt::xarray<float> res = res_map.first;
                        float z = res_map.second;
                        xt::xarray<float> rectify_point_temp = {res(0, 0), res(1, 0), z};
                        rectify_point_temp = rectify_point_temp.reshape({rectify_point_temp.size(), 1});
                        xt::xarray<float> rectify_point = xt::linalg::dot(_R, rectify_point_temp);
                        rectify_point = rectify_point.reshape({1, rectify_point.size()});
                        xt::xarray<float> central_points = {rectify_point(0, 0), rectify_point(0, 1) + size(0) / 2 , float (size(2) / 2 + _ground_height)};
                        central_points = central_points.reshape({int(central_points.size() / 3), 3});
                        size = size.reshape({int (size.size() / 3), 3});
                        float angles = float (_default_angle / 180.0 * M_PI);
                        xt::xarray<float> corner_data = box_ops_tunnel::center_to_corner_box3d(central_points, size, angles);
                        corner_data = corner_data.reshape({int(corner_data.size() / 24), 24});
                        xt::xarray<float> vid2lid_inf_temp = {central_points(0, 0), central_points(0, 1), central_points(0, 2),
                                                              size(0), size(1), size(2),
                                                              angles, float (_labels_list(j)),0,_ids_list(j),
                                                              0, 0, 0, 0, 0, 0,
                                                              };
                        corner_data = xt::view(corner_data, 0, xt::all());
                        corner_data = corner_data.reshape({24,});
                        xt::xarray<float> vid2lid_inf_temp2 = {_boxes_list(j, 0), _boxes_list(j, 1), _boxes_list(j, 2), _boxes_list(j, 3),
                                                               _scores_list(j), float (_labels_list(j)), _ids_list(j), 0, 0};
                        xt::xarray<float> vid2lid_inf = xt::hstack(xt::xtuple(vid2lid_inf_temp, corner_data, vid2lid_inf_temp2));
                        vid2lid_inf = vid2lid_inf.reshape({1, 49});
                        _infor_list = xt::concatenate(xt::xtuple(_infor_list, vid2lid_inf), 0);

                    }
                    else if (_map_mode == 2){
                        xt::xarray<float> boxes_list_temp = xt::view(_boxes_list, j);
                        xt::xarray<float> point_2d = _coordinate_map.cal_chafen_diff2(boxes_list_temp, _cam_id);

                        if (int(point_2d(2)) != 2){
                            if (point_2d(1) > 200){
                                if (_id_with_chedao.count(int(_ids_list(j))) == 0){
                                    _id_with_chedao[int(_ids_list(j))] = {};
                                }
                                else if (_id_with_chedao[int(_ids_list(j))].size() < 6){
                                    _id_with_chedao[int(_ids_list(j))].push_back(point_2d(3));
                                }
                                else {
                                    std::vector<float> list_chedao = _id_with_chedao[int(_ids_list(j))];
                                    float plural_chedao = fusion_utils::find_most_times_ele(list_chedao);
                                    if (point_2d(3) != plural_chedao){
                                        point_2d(3) = plural_chedao;
                                    }
                                }
                            }
                            xt::xarray<float> size = _class_size[vid2lidsize(_labels_list(j))];
                            xt::xarray<float> central_points = get_nearest_coordniate_point(point_2d);
                            central_points(2) = size(2) / 2 + _ground_height;
                            central_points(0) += _map_trans_x;
                            central_points(1) += _map_trans_y;
                            central_points = central_points.reshape({1, 3});
                            size = size.reshape({1, 3});
                            float angle = _default_angle / 180.0 * M_PI;
                            xt::xarray<float> corner_data = box_ops_tunnel::center_to_corner_box3d(central_points, size, angle);
                            corner_data = corner_data.reshape({24,});
                            xt::xarray<float> video_input_temp = {angle, float (_labels_list(j)), 0, _ids_list(j),
                                                                  _scores_list(j), 0, 0, 0, 0, 0};
                            size = size.reshape({size.size(),});
                            central_points = central_points.reshape({central_points.size(),});
                            xt::xarray<float> video_input = xt::hstack(xt::xtuple(central_points, size, video_input_temp));
                            xt::xarray<float> lidar_input = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};




                            xt::xarray<float> vid2lid_inf_temp = {central_points(0, 0), central_points(0, 1), central_points(0, 2),
                                                                  size(0), size(1), size(2),
                                                                  angle, float (_labels_list(j)),0,_ids_list(j),
                                                                  0, 0, 0, 0, 0, 0,
                            };

                            xt::xarray<float> vid2lid_inf_temp2 = {boxes_list_temp(j, 0), boxes_list_temp(j, 1), boxes_list_temp(j, 2), boxes_list_temp(j, 3),
                                                                   _scores_list(j), float (_labels_list(j)), _ids_list(j), 0, 0};
                            xt::xarray<float> vid2lid_inf = xt::hstack(xt::xtuple(vid2lid_inf_temp, corner_data, vid2lid_inf_temp2, lidar_input, video_input));
                            vid2lid_inf = vid2lid_inf.reshape({1, 81});
//                            _infor_list = xt::concatenate(xt::xtuple(_infor_list, vid2lid_inf), 0);
                            if (std::abs(central_points(0)) <= _target_safe_dis){
                                _infor_list = xt::concatenate(xt::xtuple(_infor_list, vid2lid_inf), 0);
                            }
                        }
                    }
                    else {
                        xt::xarray<float> central_points = xt::view(_lan_target, j, xt::range(0, 3));
                        xt::xarray<float> size = xt::view(_lan_target, j, xt::range(3, 6));
                        central_points = central_points.reshape({1, 3});
                        size = size.reshape({1, 3});
                        float angles = float (_default_angle / 180.0 * M_PI);
                        xt::xarray<float> corner_data = box_ops_tunnel::center_to_corner_box3d(central_points, size, angles);
                        corner_data = corner_data.reshape({24,});
                        xt::xarray<float> lidar_input = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                        xt::xarray<float> video_input = {central_points(0, 0), central_points(0, 1), central_points(0, 2),
                                                         size(0), size(1), size(2),
                                                         angles, float (_labels_list(j)),0,_ids_list(j),_scores_list(j),
                                                              0, 0, 0, 0, 0};
                         xt::xarray<float> video_input_temp = {_boxes_list(j, 0), _boxes_list(j, 1), _boxes_list(j, 2), _boxes_list(j, 3),
                                            _scores_list(j), float (_labels_list(j)), _ids_list(j), 0, 0};
                        xt::xarray<float> vid2lid_inf = xt::hstack(xt::xtuple(video_input, corner_data, video_input_temp, lidar_input, video_input));
                        vid2lid_inf = vid2lid_inf.reshape({1, 81});
                        _infor_list = xt::concatenate(xt::xtuple(_infor_list, vid2lid_inf), 0);
                    }

                }
            }
        }

        for (int i = 0; i < _camlidar.shape(0); ++i) {
            xt::xarray<float> camlidar_temp = xt::view(_camlidar,i, xt::range(0, 10));
            camlidar_temp = camlidar_temp.reshape({1, 10});
            _last_lidar_position = xt::concatenate(xt::xtuple(_last_lidar_position, camlidar_temp), 0);
        }
        // for (int i = 0; i < _camlidar.shape(0); ++i) {
        //     _lidar_pos_history[int(_camlidar(i, 9))] = {float (_frame), _camlidar(i, 0), _camlidar(i, 1)};
        // }
        _frame_dict++;
    }

    // l_e = std::chrono::high_resolution_clock::now();
    // l_ulT8 = std::chrono::duration<double, std::milli>(l_e - l_s).count();
    // l_s = std::chrono::high_resolution_clock::now();
    // std::cout << " =========================== fusion delay 8= " << l_ulT8 << std::endl;

    // auto l_eall = std::chrono::high_resolution_clock::now();
    // auto l_uall = std::chrono::duration<double, std::milli>(l_eall - l_sall).count();
    // std::cout << " =========================== fusion delay all= " << l_uall << std::endl;
}

void Fusion_Algorithm_tunnel::data_analysis(bool direct, bool cvSign) {
    xt::xarray<float> DirLidar = _data;
    std::vector<int> img_size = {1920, 1080};
    xt::xarray<float> dirLidar_input = xt::view(DirLidar, xt::all(), xt::range(11, 35));
    auto lidar_box_indices = cv_map(dirLidar_input, _cam_id, img_size, _coordinate_map);

    _lidar_box_list = lidar_box_indices.first;
    _indicss = lidar_box_indices.second;
//    camlidar = xt::empty<float>({int (DirLidar.shape(0)), int (DirLidar.shape(1))});
    int count = 0;
    for (int i = 0; i < DirLidar.shape(0); ++i) {
        if (_indicss(i) == 1) {
            xt::view(DirLidar, count) = xt::view(DirLidar, i);
            count += 1;
        }
    }
    _camlidar = xt::view(DirLidar, xt::range(0, count));
}


std::pair<xt::xarray<float>, xt::xarray<int>> cv_map(xt::xarray<float> &points,
                                                      int cameraID,
                                                      std::vector<int> img_size,
                                                      Coordinate_Map_tunnel coordinate_map){                                                         
    if (points.shape(0) == 0){
        xt::xarray<float> temp1 = xt::empty<float>({0, 4});
        xt::xarray<float> temp2 = xt::empty<float>({0});
        return {temp1, temp2};
    }

    xt::xarray<float> indimg_lists = xt::empty<float>({0, 4});

    xt::xarray<int> indices = xt::zeros<int>({points.shape(0),});
    points = points.reshape({int (points.size() / 3), 3});

    xt::xarray<float> xyz_cam = coordinate_map.lidar_to_cam(points);
    xt::xarray<int> filter_cam = xt::zeros<int>({xyz_cam.shape(0)});
    for (int i = 0; i < xyz_cam.shape(0); ++i) {
        if (xyz_cam(i, 2) > 0)
            filter_cam(i) = 1;
        else
            filter_cam(i) = 0;
    }

    xt::xarray<float> r = xt::norm(xt::view(points, xt::all(), xt::range(0, 2)));
    r = xt::view(r, xt::all(), 0) + xt::view(r, xt::all(), 1);
    r = xt::sqrt(r);
    r = r.reshape({r.size()});


    xt::xarray<int> filter_r = xt::zeros<int>({xyz_cam.shape(0)});
    for (int i = 0; i < r.shape(0); ++i) {
        if (r(i) > 4)
            filter_r(i) = 1;
        else
            filter_r(i) = 0;
    }

    xt::xarray<int> filter_ = filter_r * filter_cam;
    xt::xarray<float> img2d = coordinate_map.project_points(points);
    xt::xarray<int> filter_img = xt::zeros<int>({img2d.shape(0)});
    xt::xarray<int> filter_large = xt::zeros<int>({img2d.shape(0)});

    for (int i = 0; i < img2d.shape(0); ++i) {
        if (img2d(i, 0) > 0 && img2d(i, 0) < img_size[0] &&
            img2d(i, 1) > 0 && img2d(i, 1) < img_size[1])
            filter_img(i) = 1;
        else
            filter_img(i) = 0;
        if (std::abs(img2d(i, 0)) > 1e7 & std::abs(img2d(i, 1)) > 1e7){
            filter_large(i) = 1;
        } else{
            filter_large(i) = 0;
        }
    }

    xt::xarray<float> zero_tensor = {{0, 0, 639, 639}};
    for (int i = 0; i < int (img2d.shape(0) / 8); ++i) {
        if (xt::sum(xt::view(filter_, xt::range(8 * i, 8 * i + 8)))(0) < 8) // one false == True
            continue;
        if (xt::sum(xt::view(filter_img, xt::range(8 * i, 8 * i + 8)))(0) == 0) // all false == true
            continue;
        if (xt::sum(xt::view(filter_large, xt::range(8 * i, 8 * i + 8)))(0) > 0)    // one true == True
            continue;
        indices(i) = 1;
        xt::xarray<float> indimg_list = xt::view(img2d, xt::range(8 * i, 8 * i + 8));

        auto lidar_box = lidar_box_calculation(indimg_list, coordinate_map);
        if (xt::amax(xt::abs(zero_tensor - lidar_box))(0) > 0){
            lidar_box = lidar_box.reshape({1, 4});
            indimg_lists = xt::concatenate(xt::xtuple(indimg_lists, lidar_box), 0);
        }
    }
    return {indimg_lists, indices};

}

xt::xarray<float> lidar_box_calculation(xt::xarray<float> &img2d_list, Coordinate_Map_tunnel &coodinate){
    xt::xarray<float> x = xt::view(img2d_list, xt::all(), 0) / (float(coodinate._project_frame_W) / float (coodinate._input_frame_W));

    xt::xarray<float> y = xt::view(img2d_list, xt::all(), 1) / (float (coodinate._project_frame_H) / float (coodinate._input_frame_H));
    xt::xarray<float> x_sort = xt::sort(x);
    xt::xarray<float> y_sort = xt::sort(y);

    for(int i = 0; i < img2d_list.shape(0);++i){
        if (x_sort(i) < 0){
            x_sort(i) = 0;
        } else if (x_sort(i) > coodinate._input_frame_W - 1){
            x_sort(i) = coodinate._input_frame_W - 1;
        }
        if (y_sort(i) < 0){
            y_sort(i) = 0;
        } else if (y_sort(i) > coodinate._input_frame_H - 1){
            y_sort(i) = coodinate._input_frame_H;
        }
    }
    float xmin = int ((x_sort[0] + x_sort[3]) / 2);
    float ymin = int ((y_sort[0] + y_sort[3]) / 2);
    float xmax = int ((x_sort[6] + x_sort[7]) / 2);
    float ymax = int ((y_sort[6] + y_sort[7]) / 2);

    float ymax2;
    if (x(0) < 0 or x(3) < 0 or x(4) < 0 or x(7) < 0){
        ymax2 = std::max((y(0) - (y(3) - y(0)) / (x(3) - x(0)) * x(0)), (y(4) - (y(7) - y(4)) / (x(7) - x(4)) * x(4)));
        ymax2 = std::min(float (639.0), ymax2);
        ymax = (ymax + ymax2) / 2;
    }
    if (x(0) > 639 or x(3) > 639 or x(4) > 639 or x(7) > 639){
        ymax2 = std::max((y(0) + (y(3) - y(0)) / (x(3) - x(0)) * (640 - x(0))), (y(4) + (y(7) - y(4)) / (x(7) - x(4)) * (640 - x(4))));
        ymax2 = std::min(float (coodinate._input_frame_H - 1), ymax2);
        ymax = (ymax + ymax2) / 2;
    }

    xt::xarray<float> lidar_box = {xmin, ymin, xmax, ymax};
    return lidar_box;

}

float IOU_calc(xt::xarray<float> box1, xt::xarray<float> box2, bool wh){
    float xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2;

    if (!wh){
        xmin1 = box1(0, 0);
        ymin1 = box1(0, 1);
        xmax1 = box1(0, 2);
        ymax1 = box1(0, 3);
        xmin2 = box2(0, 0);
        ymin2 = box2(0, 1);
        xmax2 = box2(0, 2);
        ymax2 = box2(0, 3);
    }
    else{
        xmin1 = box1(0, 0);
        ymin1 = box1(0, 1);
        xmax1 = box1(0, 0) + box1(0, 2);
        ymax1 = box1(0, 1) + box1(0, 3);
        xmin2 = box2(0, 0);
        ymin2 = box2(0, 1);
        xmax2 = box2(0, 0) + box2(0, 2);
        ymax2 = box2(0, 3) + box2(0, 3);
    }

    float xx1 = xmin1 > xmin2 ? xmin1 : xmin2;
    float yy1 = ymin1 > ymin2 ? ymin1 : ymin2;
    float xx2 = xmax1 < xmax2 ? xmax1 : xmax2;
    float yy2 = ymax1 < ymax2 ? ymax1 : ymax2;

    float area1 = (xmax1 - xmin1) * (ymax1 - ymin1);
    float area2 = (xmax2 - xmin2) * (ymax2 - ymin2);
    float inter_area = std::max(0.0f, xx2 - xx1) * std::max(0.0f, yy2 - yy1);
    float iou = inter_area / (area1 + area2 - inter_area + 1e-6);
    return iou;

}

bool class_match(int lidar_class, int video_class){
    bool sign;
    if (video_class == 0 && lidar_class == 4)
        sign = true;
    else if (video_class == 1 and lidar_class == 1)
        sign = true;
    else if (video_class == 2 and lidar_class == 0)
        sign = true;
    else if (video_class == 3 and lidar_class == 3)
        sign = true;
    else if (video_class == 5 and lidar_class == 2)
        sign = true;
    else if (video_class == 7 and lidar_class == 6)
        sign = true;
    else
        sign = false;
    return sign;

}

int vid2lid_class(int camera_id){
    int lidar_id;
    if (camera_id == 0)
        lidar_id = 4;
    else if (camera_id == 1)
        lidar_id = 1;
    else if (camera_id == 2)
        lidar_id = 0;
    else if (camera_id == 3)
        lidar_id = 3;
    else if (camera_id == 5)
        lidar_id = 2;
    else
        lidar_id = 6;
    return lidar_id;
}

int lid2vid_class(int lidar_id){
    int camera_id;
    if (lidar_id == 4)
        camera_id = 0;
    else if (lidar_id == 1)
        camera_id = 1;
    else if (lidar_id == 0)
        camera_id = 2;
    else if (lidar_id == 3)
        camera_id = 3;
    else if (lidar_id == 2)
        camera_id = 5;
    else
        camera_id = 7;
    return camera_id;
}

std::string vid2lidsize(int camera_id){
    if (camera_id == 0)
        return "car";
    else if (camera_id == 1)
        return "bicycle";
    else if (camera_id == 2)
        return "middle_bus";
    else if (camera_id == 3)
        return "motorcycle";
    else if (camera_id == 4)
        return "person";
    else if (camera_id == 5)
        return "middle_bus";
    else
        return "middle_truck";
}

// Fusion_Res_tunnel Fusion_Algorithm_tunnel::output_result() {
//     Fusion_Res_tunnel output;
//     output.camlidar = _camlidar;
//     output.indicss = _indicss;
//     output.infor_list = _infor_list;
//     output.output_index = _output_index;
//     output.lidtovid_inf = _lidtovid_inf;
//     output.output_infor = _output_infor;

//     return output;
// }

xt::xarray<float> Fusion_Algorithm_tunnel::get_nearest_coordniate_point(xt::xarray<float> &point_2d) {
    xt::xarray<float> img_xy_xy_lon_lat = _new_mid_img_xy_pc_xylonlat[int (point_2d(3))];
    xt::xarray<float> index_temp = xt::view(img_xy_xy_lon_lat, xt::all(), xt::range(0, 2));
    xt::view(index_temp, xt::all(), 0) -= point_2d(0);
    xt::view(index_temp, xt::all(), 1) -= point_2d(1);
    index_temp *= -1;

    index_temp = xt::norm(index_temp);
    index_temp = xt::sum(index_temp, 1);
    index_temp = xt::sqrt(index_temp);

    int index = xt::argmin(index_temp)[0];
    xt::xarray<float> xyz = {img_xy_xy_lon_lat(index, 2), img_xy_xy_lon_lat(index, 3), float (_ground_height)};

    return xyz;
}