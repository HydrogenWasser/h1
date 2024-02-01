//
// Created by root on 4/26/21.
//

#include "track.h"
#include "use_deepsort.h"
#include "linear_assignment.h"

Detection::Detection(xt::xarray<float> &tlwh, float confidence, float class_idx, xt::xarray<float> &feature, int colors,
                     int track_color_score) :
        _tlwh(tlwh),
        _confidence(confidence),
        _class_idx(class_idx),
        _color_bbox(colors),
        _color_score(float(track_color_score)) {
    if (FEATURE_MODE != NONE_MODE)
        _feature = feature;
}

//Detection::Detection(xt::xarray<float> &tlwh, float confidence, float class_idx,
//                               xt::xarray<float> &feature):
//        _tlwh(tlwh),
//        _confidence(confidence),
//        _feature(feature),
//        _class_idx(class_idx){
//}

Detection::Detection() {}

xt::xarray<float> Detection::to_xyah() {
    xt::xarray<float> ret = _tlwh;
    xt::view(ret, xt::range(0, 2)) += (xt::view(ret, xt::range(2, ret.size())) / 2);
    ret(2) /= ret(3);
    return ret;
}

xt::xarray<float> Detection::to_tlbr() {
    xt::xarray<float> ret = _tlwh;
//    std::cout<<ret<<"\n";
    ret(2) += ret(0);
    ret(3) += ret(1);
    return ret;
}

Track::Track(xt::xarray<float> &mean, xt::xarray<float> &covariance, int track_id, int n_init, int max_age,
             int class_idx, float confidence, int frame_W, int frame_H, int border_left, int border_top,
             int border_right, int border_bottom, int feature, int color_track, int track_color_score,
             float min_conf_increment, float queue_size) :
        _mean(mean),
        _covariance(covariance),
        _track_id(track_id),
        _hits(1),
        _age(1),
        _time_since_update(0), \
        _frame_W(frame_W),
        _frame_H(frame_H),
        _border_bottom(border_bottom),
        _border_right(border_right),
        _border_top(border_top),
        _border_left(border_left),
        _state(TENTATIVE),
        _n_init(n_init),
        _max_age(max_age),
        _class_idx(class_idx),
        _confidence(confidence),
        _min_conf_increment(min_conf_increment),
        _queue_size(queue_size),
        _last_area(0.0),
        _now_v({0, 0}),
        _min_area(59.91124),
        _is_occlusion(false) {
    _history_pos = {};
    _history_v = {};
    _weight_of_history_v_d = {};
    _weight_of_history_v = {};
    _acce = xt::empty<float>({0});
    _last_box = xt::empty<float>({0});

}

void Track::predict(KalmanFilterOfVideo &kf) {
#if 0
    _history_pos.push_back(xt::view(_mean, xt::range(0, 2)));

    if (_history_pos.size() > _queue_size + 1)
        _history_pos.erase(_history_pos.begin());

    if (_history_pos.size() > 1){
        _now_v = _history_pos[_history_pos.size() - 1] - _history_pos[_history_pos.size() - 2];
        if (_time_since_update == 0)
            _last_matched_v = xt::view(_mean, xt::range(4, 6));
        _history_v.push_back(xt::view(_mean, xt::range(4, 6)));
        if (_history_v.size() > _queue_size)
            _history_v.erase(_history_v.begin());
        if (_weight_of_history_v.size() < _queue_size){
            _weight_of_history_v.push_back(float (1 + WEIGHT_D * _weight_of_history_v.size()));
            _norm_weight_v = xt::adapt(_weight_of_history_v, {_weight_of_history_v.size()});
            _norm_weight_v = _norm_weight_v / xt::sum(_norm_weight_v);
            _weight_of_history_v_d.push_back(float (1 - WEIGHT_D * _weight_of_history_v.size()));
            _norm_weight_v_d = xt::adapt(_weight_of_history_v_d);
            _norm_weight_v_d = _norm_weight_v_d / xt::sum(_norm_weight_v_d);

        }
    }
    if (_history_v.size() > 1 && _time_since_update == 0){
        _history_acce.push_back(_history_v[_history_v.size() - 1] - _history_v[_history_v.size() - 2]);
        if (_history_acce.size() > _queue_size - 1)
            _history_acce.erase(_history_acce.begin());
        if (_weight_of_history_a.size() < _queue_size - 1){
            _weight_of_history_a.push_back(1 + WEIGHT_D * _weight_of_history_a.size());
            _norm_weight_a = xt::adapt(_weight_of_history_a, {_weight_of_history_a.size()});
            _norm_weight_a = _norm_weight_a / xt::sum(_norm_weight_a);
            _weight_of_history_a_d.push_back(1 - WEIGHT_D * _weight_of_history_a.size());
            _norm_weight_a_d = xt::adapt(_weight_of_history_a_d, {_weight_of_history_a_d.size()});
            _norm_weight_a_d = _norm_weight_a_d / xt::sum(_norm_weight_a_d);
        }
    }

#endif
    kf.predict(_mean, _covariance);
//    std::cout<<_mean<<"\n";
//    std::cout<<_covariance<<"\n";
#if 0
    if (ACCELERATION_SIGN){
        if (_time_since_update == 1){
            xt::xarray<float> weight = _is_occlusion ? _norm_weight_a_d : _norm_weight_a;
            assert(weight.size() == _history_acce.size());
            weight = weight.reshape({1, weight.size()});
            _acce = xt::empty<float>({0, int (_history_acce[0].size())});
            xt::xarray<float> temp = xt::empty<float>({0, int (_history_acce[0].size())});
            for (int i = 0; i < _history_acce.size(); ++i) {
                temp = xt::concatenate(xt::xtuple(temp, _history_acce[i].reshape({1, _history_acce[i].size()})));
            }

            _acce = xt::linalg::dot(weight, temp);
        }

        if (_acce.shape(0) != 0){
            _acce.reshape({_acce.size()});
            xt::xarray<float> v_by_acce = _last_matched_v * (_last_matched_v + _acce * _time_since_update);
            if (v_by_acce(0) > 0 && v_by_acce(1) > 0)
                xt::view(_mean, xt::range(0, 2)) += (_acce * _time_since_update + 0.5 * _acce);
        }

    }
    if (_mean(2) * _mean(3) * _mean(3) < _min_area)
        _mean(3) = std::sqrt(_min_area / _mean(2));

    if (_mean(7) < -1 * MAX_H_DOT)
        _mean(7) = -1 * MAX_H_DOT;
    else if (_mean(7) > MAX_H_DOT)
        _mean(7) = MAX_H_DOT;
#endif
    _age += 1;
    _time_since_update += 1;
}

void Track::update(KalmanFilterOfVideo &kf, Detection &detection) {
    xt::xarray<float> measurement = detection.to_xyah();
//    std::cout<<measurement<<"\n";

//    std::cout<<"before update : \n"<<_mean<<"\n";
    kf.update(_mean, _covariance, measurement);
//    std::cout<<_mean<<"\n";

    if (FEATURE_MODE != NONE_MODE) {

    }


    _hits += 1;
    _time_since_update = 0;
    if (_state == TENTATIVE && _hits >= _n_init)
        _state = CONFIRMED;

    if (detection._confidence - _confidence >= _min_conf_increment || detection._confidence == 1) {
        _confidence = detection._confidence;
        _class_idx = detection._class_idx;
    }

    xt::xarray<float> box = detection.to_xyah();
    xt::view(_mean, xt::range(0, 4)) = box;
//    std::cout<<_mean<<"\n";
#if 0
    float area = box(2) * std::pow(box(3), 2);
    area = floor(area * 100 + 0.5) / 100;
    if (_last_box.shape(0) != 0){
        std::vector<float> weight;
        xt::xarray<float> this_box;
        float h;
        float alph;
        if (area < float (0.8 * _last_area)){
            _is_occlusion = true;
            if (_history_v.size() > 1){
                xt::xarray<float> temp = xt::empty<float>({0, int (_history_v[0].size())});
                for (int i = 0; i < _history_v.size() - 1; ++i) {
                    _history_v[i] = _history_v[i].reshape({1, _history_v[i].size()});
                    temp = xt::concatenate(xt::xtuple(temp, _history_v[i]));
                }


                xt::view(_mean, xt::range(0, 2)) = _history_pos[_history_pos.size() - 2] + xt::mean(temp);
                if (_acce.shape(0) != 0 && _history_acce.size() >= 1){
                    xt::xarray<float> v_by_acce = _last_matched_v * (_last_matched_v + _acce);
                    if (v_by_acce(0) > 0 && v_by_acce(1) > 0)
                        xt::view(_mean, xt::range(0, 2)) += 1.5 * _acce;

                }
            }

            weight = {0.8, 0.2};
            this_box = _last_box;
            h = weight[0] * _mean(3) + weight[1] * this_box(3);
            alph = (weight[0] * _mean(2) * _mean(3) + weight[1] * this_box(2) * this_box(3)) / h;
            _mean(2) = alph;
            _mean(3) = h;
        }
        else{
            _is_occlusion = false;
            weight = {0.4, 0.6};
            if (detection._confidence > 0.7){
                this_box = _last_box;
                h = weight[0] * _mean(3) + weight[1] * this_box(3);
                alph = (weight[0] * _mean(2) * _mean(3) + weight[1] * this_box(2) * this_box(3)) / h;
                _mean(2) = alph;
                _mean(3) = h;
            }
        }
    }
    _last_area = floor(area * 100 + 0.5) / 100; //
    _last_box = box;
    _history_pos[_history_pos.size() - 1] = xt::view(_mean, xt::range(0, 2));
    if (_history_pos.size() > 1){
        _now_v = _history_pos[_history_pos.size() - 1] - _history_pos[_history_pos.size() - 2];
        _history_v[_history_v.size() - 1] = xt::view(_mean, xt::range(4, 6));
    }

    if (_history_v.size() > 1 && _time_since_update == 0)
        _history_acce[_history_acce.size() - 1] = _history_v[_history_v.size() - 1] - _history_v[_history_v.size() - 2];

    if (_mean(2) * _mean(3) * _mean(3) < _min_area)
        _mean(3) = std::sqrt(_min_area / _mean(2));

    if (_mean(7) < -1 * MAX_H_DOT)
        _mean(7) = -1 * MAX_H_DOT;
    else if (_mean(7) > MAX_H_DOT)
        _mean(7) = MAX_H_DOT;
//    std::cout<<"after update : \n"<<_mean<<"\n";
//    std::cout<<" ";
#endif
}

bool Track::is_tentative() {
    return _state == TENTATIVE;
}

bool Track::is_confirmed() {
    return _state == CONFIRMED;
}

bool Track::is_deleted() {
    return _state == DELETED;
}

xt::xarray<float> Track::to_tlwh() {
//    std::cout<<_mean<<"\n";
    xt::xarray<float> ret = xt::view(_mean, xt::range(0, 4));

    ret(2) = ret(2) * ret(3);
    ret(0) = ret(0) - ret(2) / 2.0;
    ret(1) = ret(1) - ret(3) / 2.0;

    return ret;
}

xt::xarray<float> Track::to_tlbr() {

    xt::xarray<float> ret = to_tlwh();

    ret(2) = ret(0) + ret(2);
    ret(3) = ret(1) + ret(3);


    return ret;
}

void Track::mark_missed() {
    if (_state == TENTATIVE)
        _state = DELETED;
    else if (_time_since_update > _max_age)
        _state = DELETED;
    float sx, sy, ex, ey;
    sx = _mean(0) - _mean(2) * _mean(3) / 2;
    sy = _mean(1) - _mean(3) / 2;
    ex = _mean(0) + _mean(2) * _mean(3) / 2;
    ey = _mean(1) + _mean(3) / 2;
    if (_time_since_update > 1 and
        (sx < _border_left or sy < _border_top or ex > _frame_W - _border_right or ey > _frame_H - _border_bottom)) {
        _state = DELETED;
    }

}
Tracker::Tracker(const int &metric, const int &frame_W, const int &frame_H, const int &border_left, const int &border_top, 
                 const int &border_right, const int &border_bottom, float max_iou_distance, int max_age, int n_init):
        _metric(metric),
        _max_iou_distance(max_iou_distance),
        _max_age(max_age),
        _n_init(n_init),
        _next_id(1),
        _last_matches_num(0),
        _frame_W(frame_W),
        _frame_H(frame_H),
        _border_left(border_left),
        _border_top(border_top),
        _border_right(border_right),
        _border_bottom(border_bottom) 
{
    _kf = KalmanFilterOfVideo();
    _tracks = {};
}


void Tracker::predict() {
    for (int i = 0; i < _tracks.size(); ++i) {
        _tracks[i].predict(_kf);
    }


}

Tracker::Tracker() {}

void Tracker::update(std::vector<Detection> &detections) {
    match_out associate_out = match(detections);
    xt::xarray<float> matches = associate_out.matches;
    std::vector<int> unmatched_tracks = associate_out.unmatched_tracks;
    std::vector<int> unmatched_detections = associate_out.unmatched_detections;
//    std::cout<<matches<<"\n";
    for (int i = 0; i < matches.shape(0); ++i) {
        int track_idx = matches(i, 0), detection_idx = matches(i, 1);

        _tracks[track_idx].update(_kf, detections[detection_idx]);
    }
    _del_outframe_track();
    for (auto track_idx : unmatched_tracks) {
        _tracks[track_idx].mark_missed();
    }
    for (auto detection_idx : unmatched_detections) {
        initiate_track(detections[detection_idx]);
    }
    std::vector<int> to_del;
    for (int i = 0; i < _tracks.size(); ++i) {
        if (_tracks[i].is_deleted())
            to_del.push_back(i);
    }
    std::reverse(to_del.begin(), to_del.end());
    for (auto i : to_del) {
        _tracks.erase(_tracks.begin() + i);
    }

}

match_out Tracker::match(std::vector<Detection> &detections) {
    match_out res;

    std::vector<int> confirmed_tracks;
    std::vector<int> unconfirmed_tracks;
    for (int i = 0; i < _tracks.size(); ++i) {
        if (_tracks[i].is_confirmed())
            confirmed_tracks.push_back(i);
        if (!_tracks[i].is_confirmed())
            unconfirmed_tracks.push_back(i);
    }
//    std::vector<int> matched_a = {};
    std::vector<int> unmatched_tracks_a = confirmed_tracks;
    std::vector<int> unmatched_detections;
    for (int i = 0; i < detections.size(); ++i) {
        unmatched_detections.push_back(i);
    }
    std::vector<int> iou_track_candidates = unconfirmed_tracks;
    for (int k : unmatched_tracks_a) {
        if (_tracks[k]._time_since_update <= SET_DIDNOT_MATCH_TIME + 1)
            iou_track_candidates.push_back(k);
    }
//    unmatched_tracks_a.clear();
    std::vector<int> to_erase;
    for (int k : unmatched_tracks_a) {
        if (_tracks[k]._time_since_update <= SET_DIDNOT_MATCH_TIME + 1)
            to_erase.push_back(k);
    }
    std::reverse(to_erase.begin(), to_erase.end());
    for (int k : to_erase) {
        unmatched_tracks_a.erase(unmatched_tracks_a.begin() + k);
    }

    auto match_res = min_cost_matching(_max_iou_distance, _tracks, detections, iou_track_candidates,
                                       unmatched_detections);

//    xt::xarray<float> matches = match_res.matches;
    unmatched_tracks_a.insert(unmatched_tracks_a.end(), match_res.unmatched_tracks.begin(),
                              match_res.unmatched_tracks.end());
    std::sort(unmatched_tracks_a.begin(), unmatched_tracks_a.end());
    res.matches = match_res.matches;
    res.unmatched_tracks = unmatched_tracks_a;
    res.unmatched_detections = match_res.unmatched_detections;

//    std::cout<<res.matches<<"\n";
    return res;
}


void Tracker::initiate_track(Detection &detection) {
    xt::xarray<float> det_xtah = detection.to_xyah();
    auto init_out = _kf.initiate(det_xtah);
//    Track::Track(xt::xarray<float> &mean, xt::xarray<float> &covariance, int track_id, int n_init, int max_age,
//            int class_idx, float confidence, int frame_W, int frame_H, int border_left, int border_top,
//            int border_right, int border_bottom, int feature, int color_track, int track_color_score,
//            float min_conf_increment, float queue_size)
    _tracks.push_back(Track(init_out.first,
                            init_out.second,
                            _next_id,
                            _n_init,
                            _max_age,
                            detection._class_idx,
                            detection._confidence,
                            _frame_W,
                            _frame_H,
                            _border_left,
                            _border_top,
                            _border_right,
                            _border_bottom,
                            0,
                            0,
                            -10));
    _next_id += 1;
}

void Tracker::_del_outframe_track() {
    for (auto &track : _tracks) {

        xt::xarray<float> bbox = track.to_tlbr();

        if (bbox(0) > IN_BBOX_H - OFFSET_B ||
            bbox(1) > IN_BBOX_W - OFFSET_R ||
            bbox(2) < 0 + OFFSET_T ||
            bbox(3) < 0 + OFFSET_L)
            track._state = DELETED;
    }
}