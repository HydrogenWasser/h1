//
// Created by root on 4/26/21.
//

#ifndef SORT_TRACKER_TRACK_H
#define SORT_TRACKER_TRACK_H

#include <iostream>
#include <xtensor/xarray.hpp>
#include "kalman_filter.h"
#include<algorithm>
//#include "use_deepsort.h"

#define WEIGHT_D 0.1
#define QUEUE_SIZE 5
#define SPEED_SMOOTH_SIGN false
#define TENTATIVE 1
#define CONFIRMED 2
#define DELETED 3
#define SET_DIDNOT_MATCH_TIME 4
#define MIN_CONF_INCREMENT 0
#define QUEUE_SIZE 5
#define MAX_H_DOT 10
#define IN_BBOX_W 1080
#define IN_BBOX_H 1920
#define OFFSET_T 0
#define OFFSET_L 20
#define OFFSET_B 0
#define OFFSET_R 0
#define OUT_UNMATCHED_TIMES 3
#define ACCELERATION_SIGN true
#define ENCODER 1
#define HOG 2
#define COLOR 3
#define NONE_MODE 0
#define FEATURE_MODE 0
#define MIN_IOU 0.01
#define MAX_AGE 5
#define N_INIT 3



struct match_out {
    xt::xarray<int> matches;
    std::vector<int> unmatched_detections;
    std::vector<int> unmatched_tracks;
};

class Detection {
public:
    Detection();

    Detection(xt::xarray<float> &tlwh, float confidence, float class_idx, xt::xarray<float> &feature, int colors,
              int track_color_score);

    xt::xarray<float> to_tlbr();

    xt::xarray<float> to_xyah();

    xt::xarray<float> _tlwh;
    xt::xarray<float> _feature;
    float _confidence;
    float _class_idx;

    int _color_bbox;
    float _color_score;
};

class Track {
public:
    Track(xt::xarray<float> &mean, xt::xarray<float> &covariance, int track_id,
          int n_init, int max_age, int class_idx, float confidence, int frame_W,
          int frame_H, int border_left, int border_top, int border_right, int border_bottom,
          int feature,
          int color_track = 0,
          int track_color_score = 0,
          float min_conf_increment = MIN_CONF_INCREMENT,
          float queue_size = QUEUE_SIZE);

    Track();

    void predict(KalmanFilterOfVideo &kf);

    void update(KalmanFilterOfVideo &kf, Detection &detection);

    bool is_confirmed();

    bool is_tentative();

    bool is_deleted();

    xt::xarray<float> to_tlwh();

    xt::xarray<float> to_tlbr();

    void mark_missed();

    xt::xarray<float> _mean;
    xt::xarray<float> _covariance;
    int _track_id;
    int _hits;
    int _age;
    int _time_since_update;
    int _state;
    int _n_init;
    int _max_age;
    int _class_idx;
    float _confidence;
    float _min_conf_increment;
    float _queue_size;
    xt::xarray<float> _now_v;
    float _min_area;
    float _last_area;
    xt::xarray<float> _last_box;
    bool _is_occlusion;
    std::vector<xt::xarray<float>> _history_pos;
    std::vector<xt::xarray<float>> _history_v;
    xt::xarray<float> _last_matched_v;

    std::vector<float> _weight_of_history_v;
    xt::xarray<float> _norm_weight_v;
    std::vector<float> _weight_of_history_v_d;
    xt::xarray<float> _norm_weight_v_d;
    xt::xarray<float> _acce = {};
    std::vector<xt::xarray<float>> _history_acce;
    std::vector<float> _weight_of_history_a;
    xt::xarray<float> _norm_weight_a;
    std::vector<float> _weight_of_history_a_d;
    xt::xarray<float> _norm_weight_a_d;

    int _frame_W;
    int _frame_H;
    int _border_left;
    int _border_top;
    int _border_right;
    int _border_bottom;
};

class Tracker {
public:
    Tracker();

    Tracker(const int &metric, const int &frame_W, const int &frame_H, const int &border_left, const int &border_top, 
            const int &border_right, const int &border_bottom, float max_iou_distance = 1 - MIN_IOU, int max_age = MAX_AGE,
            int n_init = N_INIT);

    void predict();

    void update(std::vector<Detection> &detections);

    match_out match(std::vector<Detection> &detections);

    void initiate_track(Detection &detection);

    void _del_outframe_track();

    int _metric;
    float _max_iou_distance;
    int _max_age;
    int _n_init;
    KalmanFilterOfVideo _kf;
    std::vector<Track> _tracks;
    float _next_id;
    int _last_matches_num;

    int _frame_W;
    int _frame_H;
    int _border_left;
    int _border_top;
    int _border_right;
    int _border_bottom;


};


#endif //SORT_TRACKER_TRACK_H
