//
// Created by root on 10/24/22.
//

#ifndef TESTALGLIB_FUSION_SORT_CROSSING_H
#define TESTALGLIB_FUSION_SORT_CROSSING_H

#endif //TESTALGLIB_FUSION_SORT_H

#include "TSelfFusionAlgParam.h"
#include "FUKalmanBoxTracker_crossing.h"
#include "hungarian_bigraph_matcher.h"
#include "hungarian.h"
#include "box_ops_crossing.h"
#include "coordinate_map_crossing.h"
#include "label_crossing.h"
#include "yaml-cpp/yaml.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

struct fumatch_out_crossing{
    xt::xarray<int> matches;
    xt::xarray<int> unmatched_detections;
    xt::xarray<int> unmatched_trackers;
    xt::xarray<float> cost_matrix;
    std::vector<int> unmatched_detections_vector;
    std::vector<int> unmatched_trackers_vector;
};

class FSort_crossing{
public:
    FSort_crossing(TSelfFusionAlgParam *m_stAlgParam);
    FSort_crossing();

    void sort(xt::xarray<float> &data, std::vector<xt::xarray<float>> &video_ret, int frame_count);

    fumatch_out_crossing associate_detections_to_trackers(xt::xarray<float> &detections,xt::xarray<float> &trackers, int flag);

    void cal_distance(xt::xarray<float> &detections, xt::xarray<float> &trackers, int flag, xt::xarray<float> &L_distance,xt::xarray<float> &N_distance,xt::xarray<float> &O_distance);

    std::vector<FUKalmanBoxTracker_crossing> distanceMatch(xt::xarray<float> dets, xt::xarray<float>trks, std::vector<int> unmatched_dets, std::vector<int> unmatched_trks);

    void trackAngleHandle(xt::xarray<float> x_temp, xt::xarray<float> d, int i, bool useLanebmp);

    void result_nms(xt::xarray<float> &pc_ret_nms,std::vector<int> to_del3);

    

public:
    int _frame_count;
    std::vector<xt::xarray<float>> _video_ret;
    xt::xarray<float> _pc_ret;

private:

    std::string _config_path;
    cv::Mat _lane_bmp;
    xt::xarray<int> _laneParam;
    bool _useLanebmp;


    std::vector<FUKalmanBoxTracker_crossing> _trackers;



    float _dis_thresh;
    float _angle_judge;
    float _iou_threshold;

    int _max_age;
    int _max_age_new;
    int _min_hits;
    float _nms_min;
    float _nms_max;
    float _caldis_A;
    float _caldis_B;


};