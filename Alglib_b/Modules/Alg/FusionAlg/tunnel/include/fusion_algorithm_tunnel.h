//
// Created by root on 5/8/21.
//

#ifndef FUSION_ALG_FUSION_ALGORITHM_TUNNEL_H
#define FUSION_ALG_FUSION_ALGORITHM_TUNNEL_H
#include <iostream>
#include <algorithm>
#include "coordinate_map_tunnel.h"
// #include "km.h"
#include "box_ops_tunnel.h"
#include "TSelfFusionAlgParam.h"
#include "IFusionAlgBase.h"
#include "yaml-cpp/yaml.h"
#include "video_utils_tunnel.h"    
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

std::pair<xt::xarray<float>, xt::xarray<int>> cv_map(xt::xarray<float> &points,
                                                      int cameraID,
                                                      std::vector<int> img_size,
                                                      Coordinate_Map_tunnel coordinate_map);

xt::xarray<float> lidar_box_calculation(xt::xarray<float> &img2d_list, Coordinate_Map_tunnel &coodinate);

float IOU_calc(xt::xarray<float> box1, xt::xarray<float> box2, bool wh = false);

bool class_match(int lidar_class, int video_class);

int vid2lid_class(int camera_id);

int lid2vid_class(int lidar_id);

std::string vid2lidsize(int camera_id);

// struct Fusion_Res_tunnel {
//     xt::xarray<float> camlidar;
//     xt::xarray<float> indicss;
//     xt::xarray<float> infor_list;
//     std::vector<std::vector<int>> output_index;
//     xt::xarray<float> lidtovid_inf;
//     xt::xarray<float> output_infor;

// };

class Fusion_Algorithm_tunnel{
public:
    Fusion_Algorithm_tunnel(std::string config_path, int cam_id, int station_id, const stCameraParam& p_refCameraParam);
    Fusion_Algorithm_tunnel();
    void _load_config();
    // now
    // void fusion(xt::xarray<float> &video,
    //             xt::xarray<float> &data,
    //             int frameId,
    //             xt::xarray<float> &xyz,
    //             xt::xarray<float> &wlh,
    //             xt::xarray<int> &ids,
    //             xt::xarray<int> &lane_no,
    //             bool mapSign = true);
    void fusion(xt::xarray<float> &video,
                xt::xarray<float> &data,
                xt::xarray<float> &xyz,
                xt::xarray<float> &wlh,
                xt::xarray<int> &ids,
                xt::xarray<int> &lane_no,
                xt::xarray<float> &video_ret,
                xt::xarray<float> &pc_ret,
                bool mapSign = true);
    // void fusion(xt::xarray<float> &video, xt::xarray<float> &data, bool mapSign);
    void data_analysis(bool direct = false, bool cvSign = true);

    void plane_matrix();

    // Fusion_Res_tunnel output_result();
    xt::xarray<float> get_nearest_coordniate_point(xt::xarray<float> &point_2d);

private:
    float _iou_threshold;
    float _horizontal_offset_1;
    float _vertical_offset_1;
    float _horizontal_offset_2;
    float _vertical_offset_2;
    float _horizontal_offset_long_object;
    float _vertical_offset_long_object;
    float _default_angle;
    int _map_mode;
    double _map_trans_x;
    double _map_trans_y;
    int _target_safe_dis;
    xt::xarray<double> _getRotationVector;
    xt::xarray<double> _R;
    xt::xarray<float> _para_plane;
    int _count;
    std::map<int, xt::xarray<float>> _middle_line_2d;
    std::map<int, xt::xarray<float>> _bianxian_2d;
    std::map<int, xt::xarray<float>> _new_mid_img_xy_pc_xylonlat;
    int _dingwei_count;


    int _cam_id;
    int _station_id;
    std::string _config_path;
    // int _frame = 0;
    double _ground_height = -4.7;


    Coordinate_Map_tunnel _coordinate_map;

    xt::xarray<float> _boxes_list;
    xt::xarray<float> _scores_list;
    xt::xarray<int> _labels_list;
    xt::xarray<float> _ids_list;
    xt::xarray<float> _data;
    xt::xarray<float> _lidar_box_list;
    xt::xarray<float> _camlidar;
    xt::xarray<float> _camlidar_org;
    xt::xarray<float> _indicss;
    int _count_fusion;
    xt::xarray<float> _lan_target;
    float _lidar_confidience;
    float _video_confidience;
    std::vector<float> _dict_match;
    std::map<int, std::map<int, float>> _vidmatch_history;
    int _frame_dict;
    int _frame_update;
    std::map<int, std::vector<float>> _id_with_chedao;
    xt::xarray<float> _last_lidar_position;
    // std::map<int, xt::xarray<float>> _lidar_pos_history;


    xt::xarray<float> _lidar_temp;
    xt::xarray<float> _lidar_zsc;

    std::vector<std::vector<int>> _output_index;
    xt::xarray<float> _output_infor;
    xt::xarray<float> _lidtovid_inf;
    xt::xarray<float> _infor_list;
    std::unordered_map<std::string, xt::xarray<float>> _class_size;
};


class Fusion_Algorithm_TunnelAll : public IFusionAlgBase 
{
public:
    Fusion_Algorithm_TunnelAll(TSelfFusionAlgParam *m_stAlgParam);

    Fusion_Res RUN(std::vector<xt::xarray<float>> &video,
                xt::xarray<float> &data,
                int cam_num);

private:
    std::vector<std::shared_ptr<Fusion_Algorithm_tunnel>>               m_vecAlgHandles; 
    std::shared_ptr<WLH_Estimator>                                      m_pWlhEstimator; 
    std::vector<std::shared_ptr<Box_Bottom_Lane_Judger>>                m_vecLaneJudger; 
    std::vector<std::shared_ptr<Match_Pairs_Coordinate_Estimator>>      m_vecCoordinateEstimator; 
    std::vector<std::shared_ptr<Camera_Map>>                            m_vecCameraMap; 
    std::vector<int>                                                    m_vecFromAndTo;
    Fusion_Res                                                          _output;
    // xt::xarray<float>                                                   _video_ret;
    std::vector<xt::xarray<float>>                                      _video_all_ret;
    xt::xarray<float>                                                   _pc_ret;
};

#endif //FUSION_ALG_FUSION_ALGORITHM_H
