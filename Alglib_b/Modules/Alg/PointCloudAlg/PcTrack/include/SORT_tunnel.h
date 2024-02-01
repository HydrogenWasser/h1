//
// Created by root on 2/11/22.
//

#ifndef SORT_TRACKER_SORT_TUNNEL_H
#define SORT_TRACKER_SORT_TUNNEL_H
#include "TSelfPcAlgParam.h"
#include "KalmanFilter_tunnel.h"
#include "IPcAlgBase.h"
#include <iostream>
#include <xtensor/xmath.hpp>

//#define pi 3.1415926
#ifndef None_PI
#define None_PI -10000.1
#endif

struct state_output{
    xt::xarray<float> x;
    xt::xarray<float> bbox;
    float speed;
    std::vector<xt::xarray<float>> angle_box;
};


namespace sort{
    struct match_out{
        xt::xarray<int> matches;
        xt::xarray<int> unmatched_detections;
        xt::xarray<int> unmatched_trackers;
        xt::xarray<float> cost_matrix;
        std::vector<int> unmatched_detections_vector;
        std::vector<int> unmatched_trackers_vector;
    };

    match_out associate_detections_to_trackers(xt::xarray<float> &detections,
                                               xt::xarray<float> &trackers,
                                               int flag,
                                               float width_coe);

    std::pair<xt::xarray<float>, xt::xarray<float>> iou_jit_new(xt::xarray<float> &boxes,
                                                                xt::xarray<float> &query_boxes,
                                                                float eps = 0.0);

    std::pair<xt::xarray<float>, xt::xarray<float>> get_iou(xt::xarray<float> &boxes,
                                                                  xt::xarray<float> &query_boxes, int flag);

    std::pair<xt::xarray<float>, xt::xarray<float>> rotate_nms_cc(xt::xarray<float> &dets,
                                                                xt::xarray<float> &trackers);

    xt::xarray<float> center_to_corner_box2d(xt::xarray<float> &centers,
                                             xt::xarray<float> &dims,
                                             xt::xarray<float> &angles,
                                             float origin = 0.5);

    xt::xarray<float> corners_nd(xt::xarray<float> &dims, float origin = 0.5);

    xt::xarray<float> rotation_2d(xt::xarray<float> & corners, const xt::xarray<float> & angles);

    xt::xarray<float> corner_to_standup_nd(xt::xarray<float> &boxes_corner);

    float cal_angle(std::vector<xt::xarray<float>> &state_list, float thresh);

    std::pair<xt::xarray<float>, xt::xarray<float>> cal_lateral_dis(xt::xarray<float> &detections,
                                                                    xt::xarray<float> &trackers,
                                                                    int flag);

    float median_sizeof_car(std::vector<float> &source);

    std::pair<int, int> majority_element(std::vector<int>& nums);

    bool cmp2(std::pair<int, int> &a, std::pair<int, int> &b);
}

class KalmanBoxTracker{
public:
    static int _count;
    KalmanBoxTracker(xt::xarray<float> bbox);
    KalmanBoxTracker();
//    ~KalmanBoxTracker();

    void update(xt::xarray<float> &bbox, float delta_thresh);
    std::pair<xt::xarray<float>, xt::xarray<float>> predict();
    state_output get_state();
    int _time_since_update = 0;

    float _speed;
    int _label = 0;
    int _speed_thresh;
    std::vector<xt::xarray<float>> _angle_box;
    std::vector<float> _angle_list;
    float _detect_angle;
    float _lane_angle;
    float _last_angle;
    std::vector<xt::xarray<float>> _state;
    float _track_angle;
    bool _high_speed;
    bool _low_speed;
    float _final_angle;
    std::vector<float> _head_angle;
    int _hits;
    int _id;
    float _l_max = -1000;
    float _w_max = -1000;
    float _h_max = -1000;
    int _lane = -1000;
    int _new_label = -1;
    int _out_label = -1;
    float _l_confidence = -10.0;
    float _w_confidence = -10.0;
    float _h_confidence = -10.0;
    float _l_unconfidence = -10.0;
    float _w_unconfidence = -10.0;
    float _h_unconfidence = -10.0;
    std::vector<float> _l_list;
    std::vector<float> _w_list;
    std::vector<float> _h_list;
    std::vector<int> _label_list;
    std::vector<float> _l_list_all;
    std::vector<float> _w_list_all;
    std::vector<float> _h_list_all;
    xt::xarray<float> _bbox;
    KalmanFilter _kf;

private:
    int _use_acc_model = 0;



    bool _min_q;
    bool _max_q;


    std::vector<xt::xarray<float>> _history;

    std::map<int, xt::xarray<float>> _label_dict;

    bool _dynamic;
    bool _static;
    std::vector<xt::xarray<float>> _state_judge;
    std::vector<int> _label_box;
};

class Sort : public IPcAlgBase{
public:
    Sort(TSelfPcAlgParam * AlgParams);
    Sort(std::vector<float> fix_truck, int max_age=4, int min_hits=2);
    Sort();

    xt::xarray<float> RUN(xt::xarray<float> &dets);
    std::vector<float> _fix_truck;
    int _max_age;
    int _max_age_new;
    int _min_hits;
    std::vector<KalmanBoxTracker> _trackers;
    int _frame_count;
    float _dis_thresh;
    float _angle_judge;

    float _delta_thresh;
    float _width_coe;
};

#endif //SORT_TRACKER_SORT_TUNNEL_H
