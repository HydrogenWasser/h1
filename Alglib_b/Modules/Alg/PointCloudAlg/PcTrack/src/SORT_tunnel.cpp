//
// Created by root on 2/11/22.
//
#include <iostream>
#include <cmath>
#include <algorithm>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xsort.hpp>
#include <utility>
#include <xtensor/xmath.hpp>
#include <xtensor/xcontainer.hpp>
#include <vector>
#include "SORT_tunnel.h"
#include "cal_iou.h"
#include "hungarian_bigraph_matcher.h"
#include "xtensor/xbuilder.hpp"
#include "xtensor/xtensor_forward.hpp"
#include "hungarian.h"

int KalmanBoxTracker::_count = 0;

KalmanBoxTracker::KalmanBoxTracker(xt::xarray<float> bbox) {
    _use_acc_model = 0;
    if (bbox(7) == 0 or bbox(7) == 2 or bbox(7) == 5 or bbox(7) == 6 or bbox(7) == 8 or bbox(7) == 9) {
        _use_acc_model = 1;
        _kf = KalmanFilter(8, 4);
        _kf._F = xt::xarray<float>({
                                           {1, 0, 0, 0, 0.1, 0,   0.005, 0},
                                           {0, 1, 0, 0, 0,   0.1, 0,     0.005},
                                           {0, 0, 1, 0, 0,   0,   0,     0},
                                           {0, 0, 0, 1, 0,   0,   0,     0},
                                           {0, 0, 0, 0, 1,   0,   0.1,   0},
                                           {0, 0, 0, 0, 0,   1,   0,     0.1},
                                           {0, 0, 0, 0, 0,   0,   1,     0},
                                           {0, 0, 0, 0, 0,   0,   0,     1}});
        _kf._H = xt::xarray<float>({
                                           {1, 0, 0, 0, 0, 0, 0, 0},
                                           {0, 1, 0, 0, 0, 0, 0, 0},
                                           {0, 0, 1, 0, 0, 0, 0, 0},
                                           {0, 0, 0, 1, 0, 0, 0, 0}});
        _kf._Q *= 0.1; // init value
        _kf._R *= 1;
        _kf._P *= 10;
        xt::view(_kf._P, xt::range(4, 6), xt::range(4, 6)) *= 1000;
    } else {
        _use_acc_model = 0;
        _kf = KalmanFilter(7, 4);
        _kf._F = xt::xarray<float>({{1, 0, 0, 0, 0.1, 0, 0},
                                    {0, 1, 0, 0, 0, 0.1, 0},
                                    {0, 0, 1, 0, 0, 0, 1},
                                    {0, 0, 0, 1, 0, 0, 0},
                                    {0, 0, 0, 0, 1, 0, 0},
                                    {0, 0, 0, 0, 0, 1, 0},
                                    {0, 0, 0, 0, 0, 0, 1}});
        _kf._H = xt::xarray<float>({
                                           {1, 0, 0, 0, 0, 0, 0},
                                           {0, 1, 0, 0, 0, 0, 0},
                                           {0, 0, 1, 0, 0, 0, 0},
                                           {0, 0, 0, 1, 0, 0, 0}});
        _kf._Q(_kf._Q.shape(0) - 1, _kf._Q.shape(1) - 1) *= 0.01; // init value
        xt::view(_kf._Q, xt::range(4, _kf._Q.shape(0)), xt::range(4, _kf._Q.shape(1))) *= 0.01;
        _kf._R *= 10;
        _kf._P *= 10;
        xt::view(_kf._P, xt::range(4, 6), xt::range(4, 6)) *= 1000;
    }
    xt::xarray<float> bbox_ = xt::view(bbox, xt::range(0, 4));
    xt::view(_kf._x, xt::range(0, 4)) = bbox_.reshape({bbox_.size(), 1});
    _bbox = bbox;
    _time_since_update = 0;
    _min_q = true;
    _max_q = false;
    _final_angle = None_PI;
    _id = _count;
    _count += 1;

    _label_dict = {{0, xt::xarray<float>({1.80, 4.31, 1.59})},
                   {1, xt::xarray<float>({0.68, 1.76, 1.68})},
                   {2, xt::xarray<float>({2.95, 10.96, 3.24})},
                   {3, xt::xarray<float>({1.43, 2.73, 1.89})},
                   {4, xt::xarray<float>({0.64, 0.78, 1.73})},
                   {5, xt::xarray<float>({3.01, 14.96, 3.91})},
                   {6, xt::xarray<float>({2.53, 7.20, 3.08})}
    };

    _high_speed = false;
    _low_speed = false;
    _dynamic = false;
    _static = false;

    _lane_angle = None_PI;
    _track_angle = None_PI;
    _detect_angle = None_PI;
    _last_angle = None_PI;

    _hits = 0;
    _speed = 0;
    _label = int(bbox(7));
    _speed_thresh = 3;

}

void KalmanBoxTracker::update(xt::xarray<float> &bbox, float delta_thresh) {
    ////std::cout<<bbox<<"\n";
    int reduce = 0;
    if (_kf._x.shape(0) == 8) {
        if ((_label == 2 or _label == 5 or _label == 6 or _label == 8 or _label == 9) and _hits > 10) {
            reduce = 1;
        }
    }
    _time_since_update = 0;
    std::vector<xt::xarray<float>>::iterator it;

    for (it = _history.begin(); it != _history.end();) {
        it = _history.erase(it);
    }
    _hits += 1;
    xt::xarray<float> bbox_ = xt::view(bbox, xt::range(0, 4));
    bbox_ = bbox_.reshape({bbox_.size(), 1});
    _kf.update(bbox_, reduce, delta_thresh);

    if (std::abs(bbox(0)) < 30) {
        _l_list.push_back(bbox(3));
        _w_list.push_back(bbox(2));
        _h_list.push_back(bbox(6));
        _label_list.push_back(bbox(7));
    }
    _label_box.push_back(bbox(7));
    int l_length = _l_list.size();
    if (l_length > 0) {
        _l_confidence = sort::median_sizeof_car(_l_list);
        _w_confidence = sort::median_sizeof_car(_w_list);
        _h_confidence = sort::median_sizeof_car(_h_list);
    }
    if (bbox(0) < -30 and _l_confidence > 0) {
        bbox(3) = _l_confidence;
        auto label_cnt_pair = sort::majority_element(_label_list);
        _out_label = label_cnt_pair.first;
    }

    _bbox = bbox;
    _lane = bbox(bbox.size() - 5);
    _l_list_all.push_back(bbox(3));
    _w_list_all.push_back(bbox(2));
    _h_list_all.push_back(bbox(6));
    int length_all = _l_list_all.size();
    if (length_all > 30) {
        for (int i = 0; i < _l_list_all.size() - 30; ++i) {
            _l_list_all.erase(_l_list_all.begin()); // delete the first N
            _w_list_all.erase(_w_list_all.begin()); // delete the first N
            _h_list_all.erase(_h_list_all.begin()); // delete the first N
        }
    }
    if (length_all > 0){
        _l_unconfidence = sort::median_sizeof_car(_l_list_all);
        _w_unconfidence = sort::median_sizeof_car(_w_list_all);
        _h_unconfidence = sort::median_sizeof_car(_h_list_all);
    }
    int len_label_box = _label_box.size();
    if (len_label_box > 50) {
        for (int i = 0; i < len_label_box - 50; ++i)
            _label_box.erase(_label_box.begin()); // delete the first N
    }
//    ////std::cout << "_label_box: " << _label_box << std::endl;
    if (len_label_box > 0) {
        auto more_label_pair = sort::majority_element(_label_box);
        ////std::cout<<more_label_pair.first<<"\n";
        ////std::cout<<more_label_pair.second<<"\n";
        _label = more_label_pair.first;
        if (_label == 0 or _label == 1 or _label == 3 or _label == 4) {
            _bbox(2) = _label_dict[_label](0);
            _bbox(3) = _label_dict[_label](1);
            _bbox(6) = _label_dict[_label](2);
        }
        if ( float(more_label_pair.second) / float(_label_box.size()) > 0.7) {
            _bbox(7) = _label;
            if (more_label_pair.second > 35) {
                _label_box[_label_box.size() - 1] = more_label_pair.first;
            }
        }
    } else {
        _label = _bbox(7);
    }
    _speed = std::sqrt(std::pow(_kf._x(4, 0), 2) + std::pow(_kf._x(5, 0), 2));
//    ////std::cout << "speed:" << _speed << ", _bbox: " << _bbox << ", kf_x_: " << _kf._x << ", shape: " << _kf._x.shape(0)
//              << ", " << _kf._x.shape(1) << std::endl;
    if (_speed > _speed_thresh) {
        _high_speed = true;
        _low_speed = false;
    } else {
        _high_speed = false;
        _low_speed = true;
    }
    int l_state = _state_judge.size();
    if (l_state > 9) {
        float diff_x = _state_judge[l_state - 1](0) - _state_judge[0](0);
        float diff_y = _state_judge[l_state - 1](1) - _state_judge[0](1);
        float diff_dis = std::sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2));
        if (_speed < 3 and diff_dis < 1.5) {
            _static = true;
            _dynamic = false;
        } else {
            _static = false;
            _dynamic = true;
        }
    }
    if (_label == 0 or _label == 2 or _label == 5 or _label == 6 or _label == 8 or _label == 9) {
        if (!_high_speed and _min_q) {
            _kf._Q *= 0.1;
            _min_q = false;
            _max_q = true;
        }
        if (_high_speed and _max_q) {
            _kf._Q *= 10;
            _min_q = true;
            _max_q = false;
        }
    }
    _angle_box.push_back(_bbox);
    int l_angle_box = _angle_box.size();
    if (l_angle_box > 10) {
        for (int i = 0; i < l_angle_box - 10; ++i) {
            _angle_box.erase(_angle_box.begin());
        }
    }
    ////std::cout<<_bbox<<"\n";
}

std::pair<xt::xarray<float>, xt::xarray<float>> KalmanBoxTracker::predict() {
    if (_speed > 2 or _time_since_update < 3) {
        _kf.predict();
    }
//    ////std::cout << "kf.x shape: " << _kf._x.shape(0) << "," << _kf._x.shape(1) << std::endl;
    _speed = std::sqrt(std::pow(_kf._x(4, 0), 2) + std::pow(_kf._x(5, 0), 2));
    _time_since_update += 1;
    xt::xarray<float> output_history = xt::view(_kf._x, xt::range(0, 4));
    output_history.reshape({1, 4});
    _history.push_back(output_history);

    _state_judge.push_back(_kf._x);
    int l_state_judge = _state_judge.size();
    if (l_state_judge > 10) {
        for (int i = 0; i < l_state_judge - 10; ++i)
            _state_judge.erase(_state_judge.begin()); // delete the first N
    }
    std::pair<xt::xarray<float>, xt::xarray<float>> p1{_history[_history.size() - 1], _bbox};
    return p1;
}

state_output KalmanBoxTracker::get_state() {
    xt::xarray<float> output_x = xt::view(_kf._x, xt::range(0, 4));
    ////std::cout<<output_x<<"\n";
//    for (int i = 0; i < _angle_box.size(); ++i) {
//        ////std::cout<<_angle_box[i]<<"\n";
//    }

    if (_speed < 0.5 and _angle_box.size() > 1) {
//        ////std::cout<<_angle_box[0]<<"\n";
        xt::xarray<float> x_mean = xt::zeros<float>({1, int(_angle_box[0].size())});
        ////std::cout<<x_mean<<"\n";
        for (int i = 0; i < _angle_box.size(); ++i) {
            x_mean = xt::concatenate(xt::xtuple(x_mean, _angle_box[i].reshape({1, _angle_box[i].size()})), 0);
        }
        ////std::cout<<x_mean<<"\n";
        output_x = xt::mean(xt::view(x_mean, xt::range(1, _angle_box.size() + 1), xt::range(0, 4)), 0);
        ////std::cout<<output_x<<"\n";
        output_x.reshape({1, 4});
    }
    ////std::cout<<_bbox<<"\n";
    state_output res;
    res.x = output_x;
    res.bbox = _bbox;
    res.speed = _speed;
    res.angle_box = _angle_box;
    return res;
}


std::pair<xt::xarray<float>, xt::xarray<float>> sort::get_iou(xt::xarray<float> &dets_corners,
                                                              xt::xarray<float> &track_corners, int flag) {
    xt::xarray<float> dets_standup, trackers_standup, standup_iou, standup_iou_new;
    if (flag == 1) {
        dets_standup = sort::corner_to_standup_nd(dets_corners);
        trackers_standup = sort::corner_to_standup_nd(track_corners);
    } else {
        dets_standup = sort::corner_to_standup_nd(dets_corners);
        trackers_standup = dets_standup;
    }
    auto iou_result = sort::iou_jit_new(dets_standup, trackers_standup, 0);
    standup_iou = iou_result.first;
    standup_iou_new = iou_result.first;
    if (flag != 1) {
        for (int i = 0; i < standup_iou.shape(0); ++i) {
            standup_iou(i, i) = 0;
        }
    }
    auto re_cal = xt::where(standup_iou > 0);
    std::vector<int> no_care;
    if (re_cal.size() == 0) {
        return {standup_iou, standup_iou_new};
    } else {
        for (int i = 0; i < re_cal[0].size(); ++i) {
            if (re_cal[0][i] == re_cal[1][i]) {
                continue;
            }
            if (std::find(no_care.begin(), no_care.end(), re_cal[0][i]) != no_care.end()) {
                continue;
            }
            std::vector<xt::xarray<float>> interpoly;
            xt::xarray<float> dets_corners_bak = xt::view(dets_corners, re_cal[0][i]);
            xt::xarray<float> trks_corners_bak = xt::view(track_corners, re_cal[1][i]);
            bool res_flag = cal_iou::PolygonClip(dets_corners_bak, trks_corners_bak, interpoly);
            float IOU, IOU_new;
            if (res_flag) {
                std::vector<xt::xarray<float>> list2 = interpoly;
                for (int l = 0; l < list2.size() - 1; ++l) {
                    for (int j = l; j < list2.size(); ++j) {
                        auto res_equal = xt::equal(list2[l], list2[j]);
                        bool res_bool;
                        for (int k = 0; k < res_equal.size(); ++k) {
                            res_bool = res_equal(k) and res_equal(k + 1);
                        }
                        if (res_bool) {
                            interpoly.erase(interpoly.begin() + j);
                        }
                    }
                }
                float area = cal_iou::CalArea(interpoly);
                std::vector<xt::xarray<float>> dets_corners_bak_vec = {dets_corners_bak};
                std::vector<xt::xarray<float>> trks_corners_bak_vec = {trks_corners_bak};
                float area1 = cal_iou::CalArea(dets_corners_bak_vec);
                float area2 = cal_iou::CalArea(trks_corners_bak_vec);
                IOU = area / (area2 + area1 - area);
                IOU_new = area / area1;
            } else {
                IOU = 0;
                IOU_new = 0;
            }
            no_care.push_back(re_cal[1][i]);
            if (flag == 1) {
                standup_iou(re_cal[0][i], re_cal[1][i]) = IOU;
                standup_iou_new(re_cal[0][i], re_cal[1][i]) = IOU_new;
            } else {
                standup_iou(re_cal[0][i], re_cal[1][i]) = IOU;
                standup_iou(re_cal[1][i], re_cal[0][i]) = IOU;
                standup_iou_new(re_cal[0][i], re_cal[1][i]) = IOU_new;
                standup_iou_new(re_cal[1][i], re_cal[0][i]) = IOU_new;
            }
        }

    }
    return {standup_iou, standup_iou_new};
}

std::pair<xt::xarray<float>, xt::xarray<float>> sort::iou_jit_new(xt::xarray<float> &boxes,
                                                                  xt::xarray<float> &query_boxes, float eps) {
    int N = boxes.shape(0);
    int K = query_boxes.shape(0);
    xt::xarray<float> overlaps = xt::zeros<float>({N, K});
    xt::xarray<float> overlaps_new = xt::zeros<float>({N, K});
    for (int k = 0; k < K; ++k) {
        float box_area = (query_boxes(k, 2) - query_boxes(k, 0) + eps) * (query_boxes(k, 3) - query_boxes(k, 1) + eps);
        for (int n = 0; n < N; ++n) {
            float iw = std::min(boxes(n, 2), query_boxes(k, 2)) - std::max(boxes(n, 0), query_boxes(k, 0)) + eps;
            if (iw > 0) {
                float ih = std::min(boxes(n, 3), query_boxes(k, 3)) - std::max(boxes(n, 1), query_boxes(k, 1)) + eps;
                if (ih > 0) {
                    float ua =
                            (boxes(n, 2) - boxes(n, 0) + eps) * (boxes(n, 3) - boxes(n, 1) + eps) + box_area - iw * ih;
                    overlaps(n, k) = iw * ih / ua;
                    overlaps_new(n, k) = iw * ih / box_area;
                }
            }
        }
    }
//    std::pair<xt::xarray<float>, xt::xarray<float>> res{overlaps, overlaps_new};
    return {overlaps, overlaps_new};
}


sort::match_out sort::associate_detections_to_trackers(xt::xarray<float> &detections,
                                                       xt::xarray<float> &trackers,
                                                       int flag,
                                                       float width_coe) {
    sort::match_out associate_out;
    if (!trackers.size() or !detections.size()) {
        associate_out.matches = xt::empty<int>({0, 2});
        if (!trackers.size() and detections.size()){
            associate_out.unmatched_detections = xt::arange<int>(0, detections.shape(0));
            associate_out.unmatched_trackers = xt::empty<int>({0, 5}); ///??? 5
            for (int i = 0; i < detections.shape(0); ++i) {
                associate_out.unmatched_detections_vector.push_back(i);
            }
        }else if (trackers.size() and !detections.size()){
            associate_out.unmatched_detections = xt::empty<int>({0, 5});
            associate_out.unmatched_trackers = xt::arange<int>(0, trackers.shape(0));
            for (int i = 0; i < trackers.shape(0); ++i) {
                associate_out.unmatched_trackers_vector.push_back(i);
            }
        }
        else{
            associate_out.unmatched_detections = xt::empty<int>({0, 5});
            associate_out.unmatched_trackers = xt::empty<int>({0, 5});
        }
        associate_out.cost_matrix = xt::xarray<float>({0});
        return associate_out;
    }
//    head_dis, ano_dis
    xt::xarray<int> unsort_index;
    xt::xarray<double> cost_matrix;
    float iou_threshold = 0.000001;
    ////std::cout<<detections<<"\n";
    ////std::cout<<trackers<<"\n";
    auto dist_pair = sort::cal_lateral_dis(detections, trackers, 0);
    ////std::cout<<dist_pair.first<<"\n";
    ////std::cout<<dist_pair.second<<"\n";

    if (flag == 1) {
        xt::col(detections, 2) *= width_coe;
        xt::col(trackers, 2) *= width_coe;
        auto iou_res = sort::rotate_nms_cc(detections, trackers);
//    ////std::cout<<iou_res<<"\n";
        xt::xarray<float> iou_matrix = iou_res.first;
        xt::xarray<float> iou_matrix_new = iou_res.second;
//        ////std::cout << "bef ioumatch cost_matrix shape: row-"<<iou_matrix.shape(0)<<", col-" << iou_matrix.shape(1) << std::endl;
        ////std::cout << "bef ioumatch cost_matrix: " << iou_matrix << "\n";
        /** ================== KM alg ======================*/
        // TODO: change it to Hungarian alg! 2021.04.14
//        km::KuhnMunkres km_alg;
        cost_matrix = iou_matrix;
        iou_matrix *= -1;
//        ////std::cout<<iou_matrix<<"\n";
//        km_alg.set_matrix(iou_matrix);
//        km_alg.km();
//        unsort_index = km_alg.get_connect_result();
        unsort_index = run_hungarian_match<float>(iou_matrix);
        ////std::cout<<unsort_index<<"\n";
        xt::col(detections, 2) /= width_coe;
        xt::col(trackers, 2) /= width_coe;

    } else {

        xt::xarray<float> final_dis = dist_pair.first + 3 * dist_pair.second;
        auto index = xt::where(final_dis > 30);
        for (int i = 0; i < index[0].size(); ++i) {
            final_dis(index[0][i], index[1][i]) = 9999;
        }
        ////std::cout<<final_dis<<"\n";
//        for (int i = 0; i < final_dis.shape(0); ++i) {
//            for (int j = 0; j < final_dis.shape(1); ++j) {
//                if (final_dis(i, j) > 60){
//                    final_dis(i, j) = 10000;
//                }
//            }
//        }

//        km::KuhnMunkres km_alg;
//
        cost_matrix = final_dis;
//        ////std::cout<< final_dis<<"\n";
//        km_alg.set_matrix(final_dis);
//        km_alg.km();
//        unsort_index = km_alg.get_connect_result();
        unsort_index = run_hungarian_match<float>(final_dis);


        ////std::cout<<unsort_index<<"\n";
    }
    auto sort_index = xt::argsort(xt::view(unsort_index, xt::all(), 0));
    auto indeices = unsort_index;
    for (int i = 0; i < sort_index.size(); ++i) {
        xt::view(indeices, i) = xt::view(unsort_index, sort_index(i));
    }
//    ////std::cout << "indeices_sorted: " << indeices << std::endl;
    std::vector<int> match_0, match_1;
    for (int i = 0; i < indeices.shape(0); ++i) {
        match_0.push_back(indeices(i, 0));
        match_1.push_back(indeices(i, 1));
    }

    std::vector<int> unmatched_detections;
    for (int j = 0; j < detections.shape(0); ++j) {
        if (std::find(match_0.begin(), match_0.end(), j) == match_0.end())
            unmatched_detections.push_back(j);
    }
    std::vector<int> unmatched_trackers;
    for (int k = 0; k < trackers.shape(0); ++k) {
        if (std::find(match_1.begin(), match_1.end(), k) == match_1.end())
            unmatched_trackers.push_back(k);
    }
//    for (int i = 0; i < unmatched_trackers.size(); ++i) {
//        ////std::cout << "unmatched_trackers_inner: " << unmatched_trackers[i] << std::endl;
//    }
//    for (int i = 0; i < unmatched_detections.size(); ++i) {
//        ////std::cout << "unmatched_detections_inner: " << unmatched_detections[i] << std::endl;
//    }


    std::vector<int> matches_0, matches_1;
    for (int m = 0; m < indeices.shape(0); ++m) {
        int a = indeices(m, 0), b = indeices(m, 1);
        if (flag == 1) {
//            ////std::cout<<iou_matrix(4, 0)<<"\n";
//            ////std::cout<< iou_matrix_new(a, b) << "\n";
//            ////std::cout<<iou_matrix(a, b)<<"\n";
//            ////std::cout<<iou_matrix_new(a, b)<<"\n";
//            ////std::cout<<trackers(b, trackers.shape(1) - 1)<<"\n";
            if ((cost_matrix(a, b) < iou_threshold) || (dist_pair.second(a, b) > 2)) {
                unmatched_detections.push_back(a);
                unmatched_trackers.push_back(b);
            } else {
                matches_0.push_back(indeices(m, 0));
                matches_1.push_back(indeices(m, 1));
            }
        } else {
            if ((dist_pair.first(a, b) > 10) || (dist_pair.second(a, b) > 1.5)) {
                unmatched_detections.push_back(a);
                unmatched_trackers.push_back(b);
            } else {
                matches_0.push_back(indeices(m, 0));
                matches_1.push_back(indeices(m, 1));
            }
        }

    }
//    for (int i = 0; i < unmatched_trackers.size(); ++i) {
//        ////std::cout << "unmatched_trackers_inner_2: " << unmatched_trackers[i] << std::endl;
//    }
//    for (int i = 0; i < unmatched_detections.size(); ++i) {
//        ////std::cout << "unmatched_detections_inner_2: " << unmatched_detections[i] << std::endl;
//    }
    if (!matches_0.size())
        associate_out.matches = xt::empty<int>({0, 2});
    else {
        assert(matches_0.size() == matches_1.size());
        std::vector<std::size_t> shape = {matches_0.size(), 1};
        xt::xarray<int> xt_match0 = xt::adapt(matches_0, shape);
        xt::xarray<int> xt_match1 = xt::adapt(matches_1, shape);
        associate_out.matches = xt::concatenate(xt::xtuple(xt_match0, xt_match1), 1);
    }
    associate_out.unmatched_detections_vector = unmatched_detections;
    associate_out.unmatched_trackers_vector = unmatched_trackers;
    std::vector<std::size_t> shape_d = {unmatched_detections.size()};
    std::vector<std::size_t> shape_t = {unmatched_trackers.size()};
    associate_out.unmatched_detections = xt::adapt(unmatched_detections, shape_d);
    associate_out.unmatched_trackers = xt::adapt(unmatched_trackers, shape_t);
    associate_out.cost_matrix = cost_matrix;
    ////std::cout<<associate_out.matches<<"\n";
    ////std::cout<<associate_out.unmatched_detections<<"\n";
    ////std::cout<<associate_out.unmatched_trackers<<"\n";
    ////std::cout<<associate_out.cost_matrix<<"\n";
    return associate_out;
}

Sort::Sort(std::vector<float> fix_truck, int max_age, int min_hits) :
        _fix_truck(fix_truck),
        _max_age(max_age),
        _min_hits(min_hits) {
    _max_age_new = 30;
    _frame_count = 0;
    _dis_thresh = 3;
    _angle_judge = 10;
}

 /***************************************************************
 * @file       SORT_tunnel.cpp
 * @brief      点云跟踪对象的构造函数
 * @input      AlgParams: 参数结构体指针
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
Sort::Sort(TSelfPcAlgParam * AlgParams):
    // _fix_truck(AlgParams->m_stPcAlgParam.m_vecFixTruckParam),
    _max_age(AlgParams->m_stPcAlgParam.m_fTrackMaxAge),
    _min_hits(AlgParams->m_stPcAlgParam.m_fTrackMinHits),
    _delta_thresh(AlgParams->m_stPcAlgParam.m_DeltaThresh),
    _width_coe(AlgParams->m_stPcAlgParam.m_fConeThreshold)
{
    _fix_truck = {-70, 70, -3.9, 0, 0, -70, 70, 50000};
    _max_age_new = 30;
    _frame_count = 0;
    _dis_thresh = 3;
    _angle_judge = 10;
}

Sort::Sort() {}

/***************************************************************
 * @file       SORT_tunnel.cpp
 * @brief      点云跟踪主函数
 * @input      dets:        点云检测结果，尺寸N * 9 (x, y, z, w, l, h, yaw, cls, conf)， N可为0.单位: 米、弧度
 * @return     点云跟踪结果， 尺寸N * 16. N可为0. 单位: 米、角度( **注意** )
 *             [0]  : x
 *             [1]  : y
 *             [2]  : w
 *             [3]  : l
 *             [4]  : yaw
 *             [5]  : z
 *             [6]  : h
 *             [7]  : cls
 *             [8]  : speed
 *             [9]  : id
 *             [10] : conf
 *             [11] : hit_num
 *             [12] : kf_z 
 *             [13] : kf_h
 *             [14] : kf_cls 
 *             [15] : kf_conf
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
// xt::xarray<float> Sort::update(xt::xarray<float> &dets) {
xt::xarray<float> Sort::RUN(xt::xarray<float> &dets) {
    
    // [x, y, z, w, l, h, yaw, cls, conf] ---> [x, y, w, l, yaw, z, h, cls, conf]
    std::vector<int> index = {0, 1, 3, 4, 6, 2, 5, 7, 8};
    xt::xarray<float> input_tracker = xt::zeros<float>({dets.shape(0), dets.shape(1)});
    for (int j = 0; j < index.size(); ++j) {
        xt::view(input_tracker, xt::all(), j) = xt::view(dets, xt::all(), index[j]);
    }
    xt::col(input_tracker, 4)= xt::col(input_tracker, 4)*180/3.14;

    dets = input_tracker;

    auto t_0_st = std::chrono::steady_clock::now();
    _frame_count += 1;
    int l = _trackers.size();
    xt::xarray<float> trks = xt::zeros<float>({l, 8});
    std::vector<int> to_del;
    xt::xarray<float> ret = xt::empty<float>({0, 16});
    //std::cout << "dets cnts: " << dets.shape(0) << std::endl;
    //std::cout << "tracker cnts: " << l << std::endl;
    for (int i = 0; i < l; ++i) {
        auto trk_pre = _trackers[i].predict();
        xt::xarray<float> pos = trk_pre.first;
        xt::xarray<float> bbox = trk_pre.second;
        pos = xt::view(pos, 0);
        xt::view(trks, i) = xt::xarray<float>({pos(0), pos(1), bbox(2), bbox(3), bbox(4),
                                               float(_trackers[i]._time_since_update),
                                               float(_trackers[i]._lane),
                                               float(_trackers[i]._label)});
        if (xt::any(xt::isnan(pos)))
            to_del.push_back(i);
    }
    //std::cout << trks << std::endl;

    std::reverse(to_del.begin(), to_del.end());
    for (int i = 0; i < to_del.size(); ++i) {
        _trackers.erase(_trackers.begin() + to_del[i]);
    }
    //std::cout << "iou_dets: " << dets << std::endl;
    //std::cout << "iou_trks: " << trks << std::endl;
    auto t_1_st = std::chrono::steady_clock::now();
    auto match_res = sort::associate_detections_to_trackers(dets, trks, 1, _width_coe);
    auto matched = match_res.matches;
    auto unmatched_dets = match_res.unmatched_detections_vector;
    auto unmatched_trks = match_res.unmatched_trackers_vector;
    auto cost_matrix = match_res.cost_matrix;
    auto t_2_dist_st = std::chrono::steady_clock::now();
    std::sort(unmatched_dets.begin(), unmatched_dets.end());
//    //std::cout << "iou_matched: " << matched << std::endl;
//    //std::cout << "iou_cost_matrix: " << cost_matrix << std::endl;
//    //std::cout << "unmatched_dets: ";
//    for (int i = 0; i < unmatched_dets.size(); ++i) {
//        //std::cout<<unmatched_dets[i] <<"  ";
//    }
//    //std::cout<<"\n";
//    //std::cout << "unmatched_trks: " << match_res.unmatched_trackers << std::endl;
//    ////std::cout << "all trks: " << trks << std::endl;
//    ////std::cout<<dets<<std::endl;
//    std::vector<int> unmatched_trks_vec(unmatched_trks.size());
//    std::copy(unmatched_trks.cbegin(), unmatched_trks.cend(), unmatched_trks_vec.begin());
//    ////std::cout << _trackers.size() << std::endl;
    for (int i = 0; i < _trackers.size(); ++i) {
        if (std::find(unmatched_trks.begin(), unmatched_trks.end(), i) == unmatched_trks.end()) {
            auto index = xt::where(xt::col(matched, 1) > i - 1 &&
                                   xt::col(matched, 1) < i + 1)[0];
            xt::xarray<float> temp_bbox = xt::view(dets, matched(index[0], 0));
//            ////std::cout<<temp_bbox<<"\n";
            _trackers[i].update(temp_bbox, _delta_thresh);
        }
    }

    xt::xarray<float> un_match_object_big, un_match_object_small, un_match_track_big;
    un_match_object_big = xt::zeros_like(dets);
    un_match_object_small = xt::zeros_like(dets);
    un_match_track_big = xt::zeros_like(trks);
    int detBig = 0;
    int detSmall = 0;
    int trkBig = 0;
    std::vector<int> un_match_object_big_idx, un_match_track_big_idx, un_match_object_small_idx;
    for (int i = 0; i < unmatched_dets.size(); ++i) {
        int labelDet = int(dets(unmatched_dets[i], 7));
        if (labelDet == 0 || labelDet == 2 || labelDet == 5 ||
            labelDet == 6 || labelDet == 8 || labelDet == 9) {
            un_match_object_big_idx.push_back(unmatched_dets[i]);
            xt::view(un_match_object_big, detBig) = xt::view(dets, unmatched_dets[i]);
            detBig++;
        } else if (labelDet == 1 || labelDet == 3 || labelDet == 4) {
            un_match_object_small_idx.push_back(unmatched_dets[i]);
            xt::view(un_match_object_small, detSmall) = xt::view(dets, unmatched_dets[i]);
            detSmall++;
        }
    }
    un_match_object_big = xt::view(un_match_object_big, xt::range(0, detBig));
    un_match_object_small = xt::view(un_match_object_small, xt::range(0, detSmall));
    ////std::cout<<un_match_object_big<<"\n";
    ////std::cout<<un_match_object_small<<"\n";
    for (int i = 0; i < unmatched_trks.size(); ++i) {
        int labelTrk = int(trks(unmatched_trks[i], 7));
        if (labelTrk == 0 || labelTrk == 2 || labelTrk == 5 || labelTrk == 6 ||
            labelTrk == 8 || labelTrk == 9) {
            un_match_track_big_idx.push_back(unmatched_trks[i]);
            xt::view(un_match_track_big, trkBig) = xt::view(trks, unmatched_trks[i]);
            trkBig++;
        }
    }
    un_match_track_big = xt::view(un_match_track_big, xt::range(0, trkBig));
    //std::cout << "dist_dets: " << un_match_object_big << std::endl;
    //std::cout << "dist_trks: " << un_match_track_big << std::endl;
    auto t_2_st = std::chrono::steady_clock::now();
    auto match_res_bak = sort::associate_detections_to_trackers(un_match_object_big, un_match_track_big, 0, _width_coe);
    auto matched_bak = match_res_bak.matches;
    auto unmatched_dets_bak = match_res_bak.unmatched_detections_vector;
    auto unmatched_trks_bak = match_res_bak.unmatched_trackers_vector;
    auto cost_matrix_bak = match_res_bak.cost_matrix;

    //std::cout << "dist_matched_bak: " << matched_bak << std::endl;
    //std::cout << "dist_cost_matrix: " << cost_matrix_bak << std::endl;

    for (int i = 0; i < matched_bak.shape(0); ++i) {
        int d = un_match_object_big_idx[matched_bak(i, 0)];
        int t = un_match_track_big_idx[matched_bak(i, 1)];
        xt::xarray<float> temp_bbox = xt::view(dets, d);
        ////std::cout<<temp_bbox<<"\n";
        _trackers[t].update(temp_bbox, _delta_thresh);
    }
    std::sort(unmatched_dets_bak.begin(), unmatched_dets_bak.end());
    //std::cout<<"unmatched_dets_bak: \n";
    for (int j = 0; j < unmatched_dets_bak.size(); ++j) {
        //std::cout<<unmatched_dets_bak[j]<<"  ";
    }
    //std::cout<<"\n";
//    std::vector<std::size_t> shape = { unmatched_dets_bak.size() };
//    xt::xarray<int> a = xt::adapt(unmatched_dets_bak, shape);
//    xt::dump_npy("/data/Documents/project_cpp/sort_lidar/sort/res/unmatched_dets_bak/" + std::to_string(frame) + ".npy", a);
//    xt::dump_npy("/data/Documents/project_cpp/sort_lidar/sort/res/un_match_object_small/" + std::to_string(frame) + ".npy", un_match_object_small);
////    //std::cout<<"\n";
    for (int i = 0; i < unmatched_dets_bak.size(); ++i) {
        xt::xarray<float> bbox_for_kalma = xt::view(un_match_object_big, unmatched_dets_bak[i], xt::all());
        _trackers.push_back(KalmanBoxTracker(bbox_for_kalma));
    }

    for (int i = 0; i < un_match_object_small.shape(0); ++i) {
        xt::xarray<float> bbox_for_kalma = xt::view(un_match_object_small, i, xt::all());
        _trackers.push_back(KalmanBoxTracker(bbox_for_kalma));
    }
    auto t_3_st = std::chrono::steady_clock::now();
    int num_tra = _trackers.size();
//    std::reverse(_trackers.begin(), _trackers.end());

//    xt::xarray<float> trks_out = xt::zeros<float>({num_tra, 9});
//    for (int i = 0; i < num_tra; ++i) {
//        xt::view(trks_out, i) = xt::xarray<float>({_trackers[i]._kf._x(0, 0), _trackers[i]._kf._x(1, 0),
//                                               _trackers[i]._kf._x(2, 0), _trackers[i]._kf._x(3, 0),
//                                               _trackers[i]._bbox(4),
//                                               float(_trackers[i]._time_since_update),
//                                               float(_trackers[i]._lane),
//                                               float(_trackers[i]._label),
//                                               float(_trackers[i]._speed)});
//    }
//    ////std::cout << "tracker done: " << trks_out << std::endl;

    std::vector<int> to_del2;
    for (int i = _trackers.size() - 1; i >= 0; --i) {
        state_output res = _trackers[i].get_state();

        auto d = res.x;
        auto x_temp = res.bbox;
        auto trk_speed = res.speed;
        auto angle_box = res.angle_box;

        ////std::cout<<d<<"\n";
        ////std::cout<<x_temp<<"\n";


        d = d.reshape({d.size()});
        d.reshape({d.shape(0)});
        assert(xt::adapt(x_temp.shape()).size() == 1);
        if (_trackers[i]._out_label > -1) {
            if (_trackers[i]._out_label == 2) {
                if (_trackers[i]._l_confidence > 0) {
                    if (_trackers[i]._l_confidence < 7) {
                        _trackers[i]._label = 8;
                        _trackers[i]._new_label = 8;
                    }
                } else if (_trackers[i]._l_unconfidence > 0) {
                    if (_trackers[i]._l_unconfidence < 7) {
                        _trackers[i]._label = 8;
                        _trackers[i]._new_label = 8;
                    }
                }
            } else if (_trackers[i]._out_label == 5 or _trackers[i]._out_label == 6) {
                if (_trackers[i]._l_confidence > 0) {
                    if (_trackers[i]._l_confidence > 12) {
                        _trackers[i]._label = 5;
                        _trackers[i]._new_label = 5;
                    } else if (_trackers[i]._l_confidence < 6) {
                        _trackers[i]._label = 9;
                        _trackers[i]._new_label = 9;
                    } else {
                        _trackers[i]._label = 6;
                        _trackers[i]._new_label = 6;
                    }
                } else if (_trackers[i]._l_unconfidence > 0) {
                    if (_trackers[i]._l_unconfidence > 12) {
                        _trackers[i]._label = 5;
                        _trackers[i]._new_label = 5;
                    } else if (_trackers[i]._l_unconfidence < 6) {
                        _trackers[i]._label = 9;
                        _trackers[i]._new_label = 9;
                    } else {
                        _trackers[i]._label = 6;
                        _trackers[i]._new_label = 6;
                    }
                }
            }
        } else {
            if (_trackers[i]._label == 2) {
                if (_trackers[i]._l_confidence > 0) {
                    if (_trackers[i]._l_confidence < 7) {
                        _trackers[i]._label = 8;
                        _trackers[i]._new_label = 8;
                    }
                } else if (_trackers[i]._l_unconfidence > 0) {
                    if (_trackers[i]._l_unconfidence < 7) {
                        _trackers[i]._label = 8;
                        _trackers[i]._new_label = 8;
                    }
                }
            } else if (_trackers[i]._label == 5 or _trackers[i]._label == 6) {
                if (_trackers[i]._l_confidence > 0) {
                    if (_trackers[i]._l_confidence > 12) {
                        _trackers[i]._label = 5;
                        _trackers[i]._new_label = 5;
                    } else if (_trackers[i]._l_confidence < 6) {
                        _trackers[i]._label = 9;
                        _trackers[i]._new_label = 9;
                    } else {
                        _trackers[i]._label = 6;
                        _trackers[i]._new_label = 6;
                    }
                } else if (_trackers[i]._l_unconfidence > 0) {
                    if (_trackers[i]._l_unconfidence > 12) {
                        _trackers[i]._label = 5;
                        _trackers[i]._new_label = 5;
                    } else if (_trackers[i]._l_unconfidence < 6) {
                        _trackers[i]._label = 9;
                        _trackers[i]._new_label = 9;
                    } else {
                        _trackers[i]._label = 6;
                        _trackers[i]._new_label = 6;
                    }
                }
            }
        }
        xt::xarray<float> out_x = xt::view(_trackers[i]._kf._x, xt::range(0, 2), xt::all());
        out_x.reshape({out_x.size(), 1});
        _trackers[i]._state.push_back(out_x);
        int l_state = _trackers[i]._state.size();
        if (_trackers[i]._high_speed) {
            if (l_state > 10) {
                for (int j = 0; j < l_state - 10; ++j) {
                    _trackers[i]._state.erase(_trackers[i]._state.begin() + 0);
                }
            }
        } else {
            if (l_state > 30) {
                for (int j = 0; j < l_state - 30; ++j) {
                    _trackers[i]._state.erase(_trackers[i]._state.begin() + 0);
                }
            }
        }
        _trackers[i]._lane_angle = _trackers[i]._bbox(4);
        _trackers[i]._track_angle = sort::cal_angle(_trackers[i]._state, 3);
        if (_trackers[i]._track_angle != None_PI) {
            if (std::abs(_trackers[i]._lane_angle - _trackers[i]._track_angle) > 90) {
                _trackers[i]._final_angle = _trackers[i]._track_angle;
            } else {
                _trackers[i]._final_angle = _trackers[i]._lane_angle;
            }
        } else {
            if (_trackers[i]._last_angle == None_PI) {
                _trackers[i]._final_angle = _trackers[i]._lane_angle;
            } else {
                if (std::abs(_trackers[i]._lane_angle - _trackers[i]._last_angle) > 90) {
                    _trackers[i]._final_angle = _trackers[i]._last_angle;
                } else {
                    _trackers[i]._final_angle = _trackers[i]._lane_angle;
                }
            }
        }
        _trackers[i]._last_angle = _trackers[i]._final_angle;

        if (_trackers[i]._time_since_update < _max_age_new and _trackers[i]._hits >= _min_hits) {
            float head_final = std::fmod(x_temp(4), 360.0);
            head_final = head_final >= 0 ? head_final : 360.0 + head_final;
            if (_trackers[i]._time_since_update < _max_age) {
                if (_trackers[i]._hits < _fix_truck[7] and (d(0) < _fix_truck[5] or d(0) > _fix_truck[6]));
                else {
                    int l_bbox = x_temp.size();
                    xt::xarray<float> d_conv = {d(0),
                                                d(1),
                                                x_temp(2),
                                                x_temp(3),
                                                head_final,
                                                x_temp(5),
                                                x_temp(6),
                                                static_cast<float>(_trackers[i]._label),
                                                static_cast<float>(_trackers[i]._speed),
                                                static_cast<float>(_trackers[i]._id + 1),
                                                x_temp(8),
                                                static_cast<float>(_trackers[i]._hits),
                                                x_temp(l_bbox - 4),
                                                x_temp(l_bbox - 3),
                                                x_temp(l_bbox - 2),
                                                x_temp(l_bbox - 1)};
                    d_conv.reshape({1, 16});
                    ////std::cout<< d_conv<<"\n";
                    ret = xt::concatenate(xt::xtuple(ret, d_conv), 0);
                }
            }
        }
        num_tra -= 1;
        if (_trackers[i]._time_since_update > _max_age_new) {
            to_del2.push_back(num_tra);
        }
    }
    for (auto num : to_del2) {
        _trackers.erase(_trackers.begin() + num);
    }
    to_del2.clear();

    return ret;
}