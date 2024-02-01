//
// Created by root on 4/26/21.
//

#ifndef SORT_TRACKER_LINEAR_ASSIGNMENT_H
#define SORT_TRACKER_LINEAR_ASSIGNMENT_H

#include <iostream>
#include <vector>
#include "track.h"
// #include "km.h"

#define ZERO_THRESHOLD 0.00000001
#define MATCH_VAL -7522752
#define MIN_Z 9900000
#define INFTY_COST 10000
#define MAX_AREA_RATE 4
struct hungary_out{
    xt::xarray<int> matches;
    std::vector<int> unmatched_tracks;
    std::vector<int> unmatched_detections;
};

hungary_out min_cost_matching(float max_distance, std::vector<Track> &tracks,
                              std::vector<Detection> &detections,
                              std::vector<int> &track_indices,
                              std::vector<int> &detection_indices);

xt::xarray<float> iou_cost(std::vector<Track> &tracks,
                           std::vector<Detection> &detections,
                           std::vector<int> &track_indices,
                           std::vector<int> &detection_indices);

xt::xarray<float> iou(xt::xarray<float> &bbox, std::vector<xt::xarray<float>> &candidates);
xt::xarray<float> iou_jit(xt::xarray<float> &boxes, xt::xarray<float> &query_boxes, float eps = 0.0);

#if 0
namespace kmOfVideo {
    class KMNode {
    public:
        KMNode(int id, float exception = 0, int match = MATCH_VAL, bool visit = false);

        int _id;
        int _exception;
        int _match;
        bool _visit;
    };

    class KuhnMunkres{
    public:
        KuhnMunkres();
        ~KuhnMunkres();
        void set_matrix(xt::xarray<float> &x_y_values);
        void km();
        bool dfs(int &i);
        void set_false(std::vector<kmOfVideo::KMNode> &nodes);
        void change_exception(std::vector<kmOfVideo::KMNode> &nodes, int &change);
        xt::xarray<int> get_connect_result();
        float get_max_value_result();

    private:
        xt::xarray<int> _matrix;
        std::vector<kmOfVideo::KMNode> _xnodes;
        std::vector<kmOfVideo::KMNode> _ynodes;
        int _minz = -100000000;
        int _x_length = 0;
        int _y_length = 0;
        int _index_x = 0;
        int _index_y;
    };

}
#endif

#endif //SORT_TRACKER_LINEAR_ASSIGNMENT_H
