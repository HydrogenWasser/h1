//
// Created by root on 5/8/21.
//

#ifndef FUSION_ALG_FUSION_ALGORITHMC_ROSSING_H
#define FUSION_ALG_FUSION_ALGORITHMC_ROSSING_H
#include <iostream>
#include <algorithm>
#include <xtensor/xmath.hpp>

#include "TSelfFusionAlgParam.h"
#include "coordinate_map_crossing.h"
#include "box_ops_crossing.h"
//#include "label.h"
#include "post_nms_crossing.h"
#include "fusion_sort_crossing.h"
#include "IFusionAlgBase.h"
#include "yaml-cpp/yaml.h"

//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>


// struct Fusion_Res {
//     xt::xarray<float> camlidar;
//     xt::xarray<float> indicss;
//     xt::xarray<float> infor_list;
//     std::vector<std::vector<int>> output_index;
//     xt::xarray<float> lidtovid_inf;
//     xt::xarray<float> output_infor;
//     xt::xarray<int> PeopleCount;
//     xt::xarray<float> pc_res;
//     std::vector<xt::xarray<float>> video_res;
//     std::vector<xt::xarray<float>> lidar_box_vector;

// };


class Fusion_Algorithm_crossing : public IFusionAlgBase {
public:
    Fusion_Algorithm_crossing(TSelfFusionAlgParam *m_stAlgParam);
    Fusion_Algorithm_crossing();
    ~Fusion_Algorithm_crossing();

    //融合配置文件
    void load_config(std::string config_path,xt::xarray<float> &ClassConf,xt::xarray<float> &WarpMatrixConf);
    //图像孪生加载
    void WarPersepctiveMatrix(xt::xarray<float> &corner_2d, xt::xarray<float> &corner_3d,xt::xarray <float> &warpMatrix);

    //图像处理
    void VideoSrcdataHandle(xt::xarray<float> VideoData, int l);
    //图像功能
    void VideoFunction(int l);
    //点云映射
    void data_analysis(xt::xarray<float> &lidar_box_list,
                       xt::xarray<float> &camlidar,
                       xt::xarray<int> &indicss,
                       int camID, bool direct = false, bool cvSign = true);
    void data_fusion(xt::xarray<float> &data, int camID, float eps, xt::xarray<int> &matches, xt::xarray<float> &cost);
                       
    //融合处理
    void DealFusionKM(xt::xarray<float> _lidar_box_list,
                                        float eps,
                                        xt::xarray<int> &matches,
                                        std::vector<int> &indicss,
                                        xt::xarray<float> &cost,
                                        int l);
    //三维点云映射
    std::pair<xt::xarray<float>, xt::xarray<int>> cv_map(xt::xarray<float> &points,
                                                         int cameraID,
                                                         std::vector<int> img_size,
                                                         Coordinate_Map_crossing coordinate_map);
    //融合结果处理
    void FusionResHandle(xt::xarray<int> matches, xt::xarray<float> cost, xt::xarray<float> &data, std::vector<int> indicss, int l);

    //融合跟踪进程
    Fusion_Res RUN(std::vector<xt::xarray<float>> &video,
                xt::xarray<float> &data,
                int cam_num);

    //结果输出
    // Fusion_Res output_result();

    xt::xarray<float> littleTargetMap(xt::xarray<float> &ret);


private:
    FSort_crossing *_sort;
    std::vector<Coordinate_Map_crossing> _coordinate_map;
    std::string _config_path;

    int _cam_num;
    int _cam_id;
    xt::xarray<float> _data;

    //行人统计孪生
    int _CountPeopleflag;
    int _PeopleCoorflag;
    std::vector<cv::Mat> _img_people_coor;
    std::vector<cv::Mat> _img_people_count;
    float _GroundHeight;
    std::vector<xt::xarray<float>> warpMatrix_vec;
    xt::xarray<int> _people_count;
    xt::xarray<float> _video_coordinate;

    //融合匹配结果
    std::vector<int> _match_flag_lidar;
    std::vector<xt::xarray<int>> _match_flag_video;
    std::vector<std::vector<int>> _matched_video;
    std::vector<std::vector<int>> _matched_video_untrk;
    std::vector<std::vector<std::vector<int>>> _matches_res;
    std::vector<std::vector<std::vector<int>>> _matches_res_untrk;

    //图像及点云检测数据
    std::vector<xt::xarray<float>> _boxes_list;
    std::vector<xt::xarray<float>> _score;
    std::vector<xt::xarray<float>> _labels_list;
    std::vector<xt::xarray<float>> _cam_box_id;
    std::vector<xt::xarray<float>> _lidar_box_vector;

    //点云映射结果
    xt::xarray<float> _lidar_box_list;
    xt::xarray<float> _camlidar;
    xt::xarray<int> _indicss;

    //图像压缩及原始尺寸
    int _input_frame_W;
    int _input_frame_H;
    int _project_frame_W;
    int _project_frame_H;

    std::unordered_map<std::string, xt::xarray<float>> _class_size;
    xt::xarray<float> class_size;
    std::vector<xt::xarray<float>> tiles_vec;
    float _iou_threshold;

    xt::xarray<float> _unmatchedTrkData;

    int _frame_count;
    std::vector<xt::xarray<float>> _video_ret;
    xt::xarray<float> _pc_ret;
    
    Fusion_Res _output;
};
#endif 


