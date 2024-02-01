//
// Created by root on 4/26/21.
//

#include "linear_assignment.h"
// #include "km.h"
#include <xtensor/xsort.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xstrides.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <algorithm>
#include "hungarian_bigraph_matcher.h"
#include "hungarian.h"
#if 0
kmOfVideo::KMNode::KMNode(int id, float exception, int match, bool visit):
        _id(id),
        _exception(exception),
        _match(match),
        _visit(visit){

}

kmOfVideo::KuhnMunkres::KuhnMunkres() {}

kmOfVideo::KuhnMunkres::~KuhnMunkres() {}

void kmOfVideo::KuhnMunkres::set_matrix(xt::xarray<float> &x_y_values) {
    xt::xarray<int> cost = xt::zeros_like(x_y_values);
    for (int i = 0; i < x_y_values.shape(0); ++i) {
        for (int j = 0; j < x_y_values.shape(1); ++j) {
            cost(i, j) = std::floor(x_y_values(i, j) * 10000 + 0.5);
        }
    }

    cost *= -1 ;
    _x_length = cost.shape(0);
    _y_length = cost.shape(1);
    if (_x_length < _y_length){
        _index_x = 0;
        _index_y = 1;
    } else{
        _index_x = 1;
        _index_y = 0;
        cost = xt::transpose(cost);
//        std::cout<<xt::view(x_y_values, 1)<<"\n";
        std::swap(_x_length, _y_length);
    }

    for (int i = 0; i < _x_length; ++i) {
        _xnodes.push_back(kmOfVideo::KMNode(i));
    }

    for (int j = 0; j < _y_length; ++j) {
        _ynodes.push_back(kmOfVideo::KMNode(j));
    }

    _matrix = cost;
    for (int k = 0; k < _x_length; ++k) {
//        std::cout<<xt::amax(xt::view(_matrix, k), 0)(0)<<"\n";
//        std::cout<<xt::view(_matrix, k)<<"\n";
        _xnodes[k]._exception = xt::amax(xt::view(_matrix, k), 0)(0);
    }
}

void kmOfVideo::KuhnMunkres::set_false(std::vector<kmOfVideo::KMNode> &nodes) {
    for (int i = 0; i < nodes.size(); ++i) {
        nodes[i]._visit = false;
    }
}

bool kmOfVideo::KuhnMunkres::dfs(int &i) {
    _xnodes[i]._visit = true;
    for (int j = 0; j < _y_length; ++j) {
        if (!_ynodes[j]._visit){
//            std::cout<<_matrix(i, j)<<"\n";
            int t = _xnodes[i]._exception + _ynodes[j]._exception - _matrix(i, j);
            if (std::abs(t) < ZERO_THRESHOLD){

                _ynodes[j]._visit = true;

                if (_ynodes[j]._match == MATCH_VAL || dfs(_ynodes[j]._match)){
                    _xnodes[i]._match = j;
                    _ynodes[j]._match = i;
                    return true;
                }
            } else{
                if (t >= ZERO_THRESHOLD)
                    _minz = std::min(_minz, t);
            }
        }
    }
    return false;
}

void kmOfVideo::KuhnMunkres::change_exception(std::vector<kmOfVideo::KMNode> &nodes, int &change) {
    for (int i = 0; i < nodes.size(); ++i) {
        if (nodes[i]._visit)
            nodes[i]._exception += change;
    }
}

void kmOfVideo::KuhnMunkres::km() {
    for (int i = 0; i < _x_length; ++i) {
        while (true){
            _minz = MIN_Z;
            set_false(_xnodes);
            set_false(_ynodes);
            if (dfs(i))
                break;
            auto change = -1 * _minz;
            change_exception(_xnodes, change);
            change_exception(_ynodes, _minz);
        }
    }
}

xt::xarray<int> kmOfVideo::KuhnMunkres::get_connect_result() {
    xt::xarray<int> ret = xt::xarray<int>({{0, 0}});
    for (int i = 0; i < _x_length; ++i) {
        int j = _xnodes[i]._match;
        int x_id = _xnodes[i]._id;
        int y_id = _ynodes[j]._id;
        if (_index_x == 1 and _index_y == 0)
            std::swap(x_id, y_id);
        ret = xt::concatenate(xt::xtuple(ret, xt::xarray<int>({{x_id, y_id}})),0);
    }

    return xt::view(ret, xt::range(1, ret.shape(0)));
}


float kmOfVideo::KuhnMunkres::get_max_value_result() {
    float ret = 0;
    for (int i = 0; i < _x_length; ++i) {
        int j = _xnodes[i]._match;
        ret += _matrix(i, j);
    }
    return ret;
}
#endif


hungary_out min_cost_matching(float max_distance, std::vector<Track> &tracks,
                              std::vector<Detection> &detections,
                              std::vector<int> &track_indices,
                              std::vector<int> &detection_indices){
    hungary_out res;

    if (track_indices.size() == 0){
        for (int i = 0; i < tracks.size(); ++i) {
            track_indices.push_back(i);
        }
    }

    if (detection_indices.size() == 0){
        for (int i = 0; i < detections.size(); ++i) {
            detection_indices.push_back(i);
        }
    }

    if (detection_indices.size() == 0 || track_indices.size() == 0){
        res.matches = xt::empty<int>({0, 2});
        res.unmatched_tracks = track_indices;
        res.unmatched_detections = detection_indices;
        return res;
    }
    xt::xarray<float> cost_matrix = iou_cost(tracks, detections, track_indices, detection_indices);
    for (int i = 0; i < cost_matrix.shape(0); ++i) {
        for (int j = 0; j < cost_matrix.shape(1); ++j) {
            if (cost_matrix(i, j) > max_distance)
                cost_matrix(i, j) = max_distance + 1e-5;
        }
    }
//    std::cout<<cost_matrix<<"\n";
    xt::xarray<float> cost_matrix_min  = cost_matrix;
    // TODO: change it to Hungarian alg! 2021.04.14
    // kmOfVideo::KuhnMunkres km_alg;
    // km_alg.set_matrix(cost_matrix_min);
    // km_alg.km();
    // xt::xarray<int> indices_unsort = km_alg.get_connect_result();
    xt::xarray<int> indices_unsort = run_hungarian_match<float>(cost_matrix_min);

//    std::cout<<indices_unsort<<"\n";

    auto sort_index = xt::argsort(xt::view(indices_unsort, xt::all(), 0));
    auto indices = indices_unsort;
    for (int i = 0; i < sort_index.size(); ++i) {
        xt::view(indices, i) = xt::view(indices_unsort, sort_index(i));
    }

//    std::cout<<indices<<"\n";
    std::vector<int> match_0, match_1;
    for (int i = 0; i < indices.shape(0); ++i) {
        match_0.push_back(indices(i, 0));
        match_1.push_back(indices(i, 1));
    }
    std::vector<int> unmatched_tracks;
    std::vector<int> unmatched_detections;

    for (int i = 0; i < detection_indices.size(); ++i) {
        if (std::find(match_1.begin(), match_1.end(), i) == match_1.end())
            unmatched_detections.push_back(detection_indices[i]);
    }


    for (int i = 0; i < track_indices.size(); ++i) {
        if (std::find(match_0.begin(), match_0.end(), i) == match_0.end())
            unmatched_tracks.push_back(track_indices[i]);
    }

    std::vector<int> matches_0, matches_1;
    for (int i = 0; i < indices.shape(0); ++i) {
        auto track_idx = track_indices[indices(i, 0)];
        auto detection_idx = detection_indices[indices(i, 1)];
        if (cost_matrix(indices(i, 0), indices(i, 1)) > max_distance){
            unmatched_tracks.push_back(track_idx);
            unmatched_detections.push_back(detection_idx);
        }
        else{
            matches_0.push_back(track_idx);
            matches_1.push_back(detection_idx);
        }
    }
    assert(matches_0.size() == matches_1.size());
    std::vector<std::size_t> shape = {matches_0.size(), 1};
    xt::xarray<int> xt_match0 = xt::adapt(matches_0, shape);
    xt::xarray<int> xt_match1 = xt::adapt(matches_1, shape);

//    std::sort(unmatched_tracks.begin(), unmatched_tracks.end());
    std::sort(unmatched_detections.begin(), unmatched_detections.end());

    res.matches = xt::concatenate(xt::xtuple(xt_match0, xt_match1), 1);
    res.unmatched_tracks = unmatched_tracks;
    res.unmatched_detections = unmatched_detections;
//    std::cout<<res.matches<<"\n";
    return res;
}

xt::xarray<float> iou(xt::xarray<float> &bbox, xt::xarray<float> &candidates){

    xt::xarray<float> bbox_tl = xt::view(bbox, xt::range(0, 2));

    xt::xarray<float> bbox_br = xt::view(bbox, xt::range(0, 2)) +
                                xt::view(bbox, xt::range(2, 4));
    xt::xarray<float> bbox_wh = xt::view(bbox, xt::range(2, 4));

    xt::xarray<float> candidates_tl = xt::view(candidates, xt::all(), xt::range(0, 2));

    xt::xarray<float> candidates_br = xt::view(candidates, xt::all(), xt::range(0, 2)) +
            xt::view(candidates, xt::all(), xt::range(2, 4));

    for (int i = 0; i < candidates_tl.shape(0); ++i) {
        candidates_tl(i, 0) = candidates_tl(i, 0) > bbox_tl(0) ? candidates_tl(i, 0) : bbox_tl(0);
        candidates_tl(i, 1) = candidates_tl(i, 1) > bbox_tl(1) ? candidates_tl(i, 1) : bbox_tl(1);
    }

    for (int i = 0; i < candidates_br.shape(0); ++i) {
        candidates_br(i, 0) = candidates_br(i, 0) < bbox_br(0) ? candidates_br(i, 0) : bbox_br(0);
        candidates_br(i, 1) = candidates_br(i, 1) < bbox_br(1) ? candidates_br(i, 1) : bbox_br(1);
    }

    xt::xarray<float> wh = candidates_br - candidates_tl;

    for (int i = 0; i < wh.shape(0); ++i) {
        wh(i, 0) = wh(i, 0) > 0 ? wh(i, 0) : 0;
        wh(i, 1) = wh(i, 1) > 0 ? wh(i, 1) : 0;
    }

    xt::xarray<float> area_intersection = xt::prod(wh, 1);

    xt::xarray<float> area_bbox = xt::prod(xt::view(bbox, xt::range(2, bbox.size())));

    xt::xarray<float> area_candidates = xt::prod(xt::view(candidates, xt::all(), xt::range(2, candidates.shape(1))), 1);
    return area_intersection / (area_bbox + area_candidates - area_intersection);

}

xt::xarray<float> iou_cost(std::vector<Track> &tracks,
                           std::vector<Detection> &detections,
                           std::vector<int> &track_indices,
                           std::vector<int> &detection_indices){
    if (track_indices.size() == 0){
        for (int i = 0; i < tracks.size(); ++i) {
            track_indices.push_back(i);
        }
    }
    if (detection_indices.size() == 0){
        for (int i = 0; i < detections.size(); ++i) {
            detection_indices.push_back(i);
        }
    }

    xt::xarray<float> cost_matrix = xt::zeros<float>({track_indices.size(), detection_indices.size()});
    xt::xarray<float> boxes = xt::empty<float>({0, 4});
    for (int i = 0; i < track_indices.size(); ++i) {
        int row = i;
        int track_idx = track_indices[i];
        xt::xarray<float> bbox = tracks[track_idx].to_tlbr();
        bbox = bbox.reshape({1, bbox.size()});
        boxes = xt::concatenate(xt::xtuple(boxes, bbox), 0);
    }
//    std::cout<<boxes<<"\n";
    xt::xarray<float> candidates = xt::empty<float>({0, 4});
    for (int i = 0; i < detection_indices.size(); ++i) {
        xt::xarray<float> temp = detections[i].to_tlbr();
        temp = temp.reshape({1, 4});
        candidates = xt::concatenate(xt::xtuple(candidates, temp), 0);
    }
    cost_matrix = iou_jit(boxes, candidates);
    cost_matrix = 1 - cost_matrix;

    for (int i = 0; i < track_indices.size(); ++i){
        if (tracks[track_indices[i]]._time_since_update > SET_DIDNOT_MATCH_TIME + 1){
            xt::view(cost_matrix, i) = INFTY_COST;
            continue;
        }
    }
//    std::cout<<cost_matrix<<"\n";
    return cost_matrix;
}

xt::xarray<float> iou_jit(xt::xarray<float> &boxes, xt::xarray<float> &query_boxes, float eps){
    int N = boxes.shape(0);
    int K = query_boxes.shape(0);
    xt::xarray<float> overlaps = xt::zeros<float>({N, K});
    for (int k = 0; k < K; ++k) {
        float box_area = (query_boxes(k, 2) - query_boxes(k, 0) + eps) * (query_boxes(k, 3) - query_boxes(k, 1) + eps);
        for (int n = 0; n < N; ++n) {
            float iw = std::min(boxes(n, 2), query_boxes(k, 2)) - std::max(boxes(n, 0), query_boxes(k, 0)) + eps;
            if (iw > 0){
                float ih = std::min(boxes(n, 3), query_boxes(k, 3)) - std::max(boxes(n, 1), query_boxes(k, 1)) + eps;
                if (ih > 0){
                    float ua = (boxes(n, 2) - boxes(n, 0) + eps) * (boxes(n, 3) - boxes(n, 1) + eps) + box_area - iw * ih;
                    overlaps(n, k) = iw * ih / ua;
                }
            }
        }
    }
    return overlaps;
}
