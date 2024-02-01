//
// Created by root on 4/25/21.
//

#ifndef SORT_TRACKER_USE_DEEPSORT_H
#define SORT_TRACKER_USE_DEEPSORT_H
#include <memory>
#include <vector>
#include <xtensor/xview.hpp>
// #include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include "kalman_filter.h"
#include "track.h"
#include "TSelfVideoAlgParam.h"
#include "IVideoAlgBase.h"


// namespace deepsort {


struct deepsort_out{
    xt::xarray<float> out_boxes;
    std::vector<float> out_score;
    std::vector<int> out_class_idxs;
    std::vector<int> out_track_id;
};


class Use_DeepSort : public IVideoAlgBase{
public:
    // Use_DeepSort(bool count_sign = false);
    Use_DeepSort(std::string model_filename, int frame_W, int frame_H, int border_left, int border_top, int border_right, int border_bottom);
    Use_DeepSort(TSelfVideoAlgParam* Alg_params);
    // deepsort::deepsort_out run_deepsort(int frame_index,
    //                                     xt::xarray<float> &tlwh_boxes,
    //                                     xt::xarray<float> &scores,
    //                                     xt::xarray<float> &class_idxs);
    // deepsort::deepsort_out run_deepsort(xt::xarray<float> &tlwh_boxes,
    //                                     xt::xarray<float> &scores,
    //                                     xt::xarray<int> &class_idxs);
    std::vector<xt::xarray<float>> RUN(std::vector<cv::Mat> & img_batch) {}; 
    xt::xarray<float> RUN(xt::xarray<float> &input);
    // void get_features(int frame_index,
    //                   xt::xarray<float> &tlwh_boxes,
    //                   xt::xarray<float> &scores,
    //                   xt::xarray<float> &class_idxs);

private:
    void get_features(xt::xarray<float> &tlwh_boxes,
                        xt::xarray<float> &scores,
                        xt::xarray<int> &class_idxs);

    void predict();
    void update();

    Tracker _tracker;
    std::vector<Detection> _detection;
    std::vector<int> _tracks_history;
    std::vector<int> _all_deepsort_info;
    std::vector<int> _all_deepsort_info_feature;
    std::vector<int> _all_deepsort_info_predict;
    std::vector<int> _all_deepsort_info_update;
    bool _count_sign;


    int _colors;
    int _track_color_score;
//        self.track_debug
};
// }

void post_nms_WJ_15class(xt::xarray<float> &boxes, xt::xarray<float> &scores, xt::xarray<float> &labels, int max_boxes = 50);
void remain_specific_class(xt::xarray<float> &boxes, xt::xarray<float> &scores, xt::xarray<float> &labels);
void nms_different_class(xt::xarray<float> &boxes, xt::xarray<float> &scores, xt::xarray<float> &labels, int max_boxes);
void remove_people_in_the_vehicle(xt::xarray<float> &boxes, xt::xarray<float> &scores, xt::xarray<float> &labels);
void remove_people_above_the_bycycle_motor(xt::xarray<float> &boxes, xt::xarray<float> &scores, xt::xarray<float> &labels);
std::vector<int> py_nms(xt::xarray<float> &boxes, xt::xarray<float> &scores, int max_boxes = 50, float iou_thresh=0.5);

#endif //SORT_TRACKER_USE_DEEPSORT_H
