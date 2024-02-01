//
// Created by root on 4/25/21.
//

#include "use_deepsort.h"
#include "track.h"
#include "xtensor/xslice.hpp"
#include "xtensor/xtensor_forward.hpp"
#include "xtensor/xview.hpp"
#include <xtensor/xnpy.hpp>
#include <xtensor/xsort.hpp>


//deepsort::Use_DeepSort::Use_DeepSort(bool count_sign):
//                                    _count_sign(count_sign){
//    _tracker = Tracker(0.0);
//}

Use_DeepSort::Use_DeepSort(std::string model_filename, int frame_W, int frame_H, int border_left,
                                     int border_top, int border_right, int border_bottom) {
    if (FEATURE_MODE == ENCODER) {

    } else if (FEATURE_MODE == HOG) {

    } else if (FEATURE_MODE == COLOR) {

    } else {
        _tracker = Tracker(0, frame_W, frame_H, border_left, border_top, border_right, border_bottom);
    }

}

Use_DeepSort::Use_DeepSort(TSelfVideoAlgParam* Alg_params) {
    if (FEATURE_MODE == ENCODER) {

    } else if (FEATURE_MODE == HOG) {

    } else if (FEATURE_MODE == COLOR) {

    } else {
        _tracker = Tracker(0, 1920, 1080, 20, 20, 20, 20);
        // _tracker = Tracker(0, frame_W, frame_H, border_left, border_top, border_right, border_bottom);
    }

}

//void deepsort::Use_DeepSort::get_features(int frame_index,
//                                          xt::xarray<float> &tlwh_boxes,
//                                          xt::xarray<float> &scores,
//                                          xt::xarray<float> &class_idxs) {
//    xt::xarray<float> features = xt::ones<float>({int(tlwh_boxes.shape(0)), 1});
//
//    if (tlwh_boxes.shape(0) != 0) {
//        _detection.clear();
//        for (int i = 0; i < tlwh_boxes.shape(0); ++i) {
//            xt::xarray<float> bbox = xt::view(tlwh_boxes, i);
//            float score = scores(i);
//            float class_idx = class_idxs(i);
//            _detection.push_back(Detection(bbox, score, class_idx, features));
//        }
//    } else
//        _detection.clear();
//
//}

void Use_DeepSort::predict() {
    _tracker.predict();
}

void Use_DeepSort::update() {
    _tracker.update(_detection);
}

// deepsort::deepsort_out deepsort::Use_DeepSort::run_deepsort(xt::xarray<float> &tlwh_boxes, xt::xarray<float> &scores,
//                                                             xt::xarray<int> &class_idxs) {
//     deepsort::deepsort_out out;
// //    std::cout<<frame<<"\n";
// //    std::cout<<tlwh_boxes<<"\n";
// //    std::cout<<scores<<"\n";
// //    std::cout<<class_idxs<<"\n";
//     get_features(tlwh_boxes, scores, class_idxs);
//     predict();
//     update();

//     xt::xarray<float> out_boxes = xt::empty<float>({0, 4});
//     std::vector<float> out_score;
//     std::vector<int> out_class_idx;
//     std::vector<int> out_track_id;

//     for (auto &track : _tracker._tracks) {
//         if (track.is_confirmed() && track._time_since_update > OUT_UNMATCHED_TIMES) {
//             continue;
//         }
// //        std::cout<<track._mean<<"\n";
//         auto bbox = track.to_tlbr();

//         bbox = bbox.reshape({1, bbox.size()});
//         out_boxes = xt::concatenate(xt::xtuple(out_boxes, bbox), 0);
//         out_score.push_back(track._confidence);
//         out_class_idx.push_back(track._class_idx);
//         out_track_id.push_back(track._track_id);
//     }


//     out.out_boxes = out_boxes;
//     out.out_score = out_score;
//     out.out_track_id = out_track_id;
//     out.out_class_idxs = out_class_idx;

//     return out;
// }
/*
 * return : t_l_x, t_l_y, r_b_x, r_b_y, conf, cls, id
 */ 
// xt::xarray<float> deepsort::Use_DeepSort::RUN(xt::xarray<float> & input) {
xt::xarray<float> Use_DeepSort::RUN(xt::xarray<float> & input) {
    /*
        add by chenxiangyang.
        将所有的中间处理都放到算法内部执行
    */ 
    xt::xarray<float> tlwh_boxes = xt::view(input, xt::all(), xt::range(0, 4));
    xt::xarray<float> scores = xt::view(input, xt::all(), 4);
    xt::xarray<float> class_id = xt::view(input, xt::all(), 5);
    scores = scores.reshape({scores.size()});
    class_id = class_id.reshape({class_id.size()});
    post_nms_WJ_15class(tlwh_boxes, scores, class_id);

    // 转换到 1920 * 1080
    // xt::view(tlwh_boxes, xt::all(), 0) *= (1920 / 416.0);
    // xt::view(tlwh_boxes, xt::all(), 1) *= (1080 / 416.0);
    // xt::view(tlwh_boxes, xt::all(), 2) *= (1920 / 416.0);
    // xt::view(tlwh_boxes, xt::all(), 3) *= (1080 / 416.0);
    tlwh_boxes = xt::clip(tlwh_boxes, xt::xarray<int>({0, 0, 0, 0}), xt::xarray<int>({1920 - 1, 1080 - 1, 1920 - 1, 1080 - 1}));
    xt::xarray<int> class_idxs = xt::cast<int>(class_id);


    deepsort_out out;


    get_features(tlwh_boxes, scores, class_idxs);
    predict();
    update();

    xt::xarray<float> out_boxes = xt::empty<float>({0, 7});
    // xt::xarray<float> out_boxes = xt::empty<float>({0, 4});
    
    // std::vector<float> out_score;
    // std::vector<int> out_class_idx;
    // std::vector<int> out_track_id;

    for (auto &track : _tracker._tracks) {
        if (track.is_confirmed() && track._time_since_update > OUT_UNMATCHED_TIMES) {
            continue;
        }
//        std::cout<<track._mean<<"\n";
        auto bbox = track.to_tlbr();
        
        xt::xarray<float> temp = {bbox(0), bbox(1), bbox(2), bbox(3), track._confidence, float(track._class_idx), float(track._track_id)};
        temp = temp.reshape({1, temp.size()});

        out_boxes = xt::concatenate(xt::xtuple(out_boxes, temp), 0);
        // out_score.push_back(track._confidence);
        // out_class_idx.push_back(track._class_idx);
        // out_track_id.push_back(track._track_id);
    }
    
    //转换回416x416
    // std::cout << out_boxes << "\n";
    xt::view(out_boxes, xt::all(), 0) *= (640 / 1920.0);
    xt::view(out_boxes, xt::all(), 1) *= (640 / 1080.0);
    xt::view(out_boxes, xt::all(), 2) *= (640 / 1920.0);
    xt::view(out_boxes, xt::all(), 3) *= (640 / 1080.0);
    xt::xarray<float> temp = xt::view(out_boxes, xt::all(), xt::range(0, 4));
    temp = xt::clip(temp, xt::xarray<int>({0, 0, 0, 0}), xt::xarray<int>({640 - 1, 640 - 1, 640 - 1, 640 - 1}));
    xt::view(out_boxes, xt::all(), xt::range(0, 4)) = temp;

    // 输出格式： t_l_x, t_l_y, r_b_x, r_b_y, conf, cls, id
    // std::cout << out_boxes << "\n";
    return out_boxes;
}

//deepsort::deepsort_out deepsort::Use_DeepSort::run_deepsort(int frame_index,
//                                                            xt::xarray<float> &tlwh_boxes, xt::xarray<float> &scores,
//                                                            xt::xarray<float> &class_idxs) {
//    deepsort::deepsort_out out;
//    //    if (_count_sign)
//    get_features(frame_index, tlwh_boxes, scores, class_idxs);
//    predict(frame_index);
//    update(frame_index);
//
//    xt::xarray<float> out_boxes = xt::empty<float>({0, 4});
//    std::vector<float> out_score;
//    std::vector<int> out_class_idx;
//    std::vector<int> out_track_id;
//
//    for (auto &track : _tracker._tracks) {
//        if (track.is_confirmed() && track._time_since_update > OUT_UNMATCHED_TIMES) {
//            continue;
//        }
////        std::cout<<track._mean<<"\n";
//        auto bbox = track.to_tlbr();
////        std::cout<<bbox<<"\n";
//        bbox = bbox.reshape({1, bbox.size()});
//        out_boxes = xt::concatenate(xt::xtuple(out_boxes, bbox), 0);
//        out_score.push_back(track._confidence);
//        out_class_idx.push_back(track._class_idx);
//        out_track_id.push_back(track._track_id);
//    }
//
//
//    out.out_boxes = out_boxes;
//    out.out_score = out_score;
//    out.out_track_id = out_track_id;
//    out.out_class_idxs = out_class_idx;
//
//    return out;
//}

void Use_DeepSort::get_features(xt::xarray<float> &tlwh_boxes,
                                          xt::xarray<float> &scores, xt::xarray<int> &class_idxs){

    xt::xarray<float> features;
    if (FEATURE_MODE == ENCODER) {

    } else if (FEATURE_MODE == HOG) {

    } else if (FEATURE_MODE == COLOR) {

    } else {
        features = xt::ones<float>({int(tlwh_boxes.shape(0)), 1});
        _colors = 0;
        _track_color_score = -10;
    }

    if (tlwh_boxes.shape(0) != 0) {
        if (FEATURE_MODE == ENCODER or
                FEATURE_MODE == COLOR){
            _detection.clear();
            for (int i = 0; i < tlwh_boxes.shape(0); ++i) {
                xt::xarray<float> bbox = xt::view(tlwh_boxes, i);
                float score = scores(i);
                float class_idx = class_idxs(i);
                _detection.push_back(Detection(bbox, score, class_idx, features, _colors, _track_color_score));
            }
        }
        else{
            _detection.clear();
            for (int i = 0; i < tlwh_boxes.shape(0); ++i) {
                xt::xarray<float> bbox = xt::view(tlwh_boxes, i);
                float score = scores(i);
                float class_idx = class_idxs(i);
                _detection.push_back(Detection(bbox, score, class_idx, features, _colors, _track_color_score));
            }
        }
    } else
        _detection.clear();

}

void remain_specific_class(xt::xarray<float> &boxes, xt::xarray<float> &scores, xt::xarray<float> &labels) {
    std::vector<int> labels_list_WJ = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14};
    int count = 0;
    for (int i = 0; i < labels.shape(0); ++i) {
        if (std::find(labels_list_WJ.begin(), labels_list_WJ.end(), labels(i)) != labels_list_WJ.end()) {
            xt::view(boxes, count, xt::all()) = xt::view(boxes, i, xt::all());
            scores(count) = scores(i);
            labels(count) = labels(i);
            ++count;
        }
    }
    boxes = xt::view(boxes, xt::range(0, count), xt::all());
    labels = xt::view(labels, xt::range(0, count));
    scores = xt::view(scores, xt::range(0, count));
}

std::vector<int> py_nms(xt::xarray<float> &boxes, xt::xarray<float> &scores, int max_boxes, float iou_thresh) {
    assert(boxes.shape(1) == 4 and scores.shape().size() == 1);
    xt::xarray<float> x1 = xt::view(boxes, xt::all(), 0);
    xt::xarray<float> y1 = xt::view(boxes, xt::all(), 1);
    xt::xarray<float> x2 = xt::view(boxes, xt::all(), 2);
    xt::xarray<float> y2 = xt::view(boxes, xt::all(), 3);

    xt::xarray<float> area = (x2 - x1 + 1) * (y2 - y1 + 1);
//    std::cout<<area<<"\n";

    xt::xarray<int> order_ = xt::argsort(scores);
    xt::xarray<int> order = order_;
    for(int i = 0; i < order_.size(); ++i){
        order(i) = order_(order_.size() - i - 1);
    }

//    std::cout<<order<<"\n";
    std::vector<int> keep;

    while (order.shape(0) > 0) {
        int i = order(0);
        keep.push_back(i);
        xt::xarray<float> xx1, yy1, xx2, yy2;

        // =========  xx1
        std::vector<float> temp;
        for (int j = 1; j < order.shape(0); ++j){
            temp.push_back(x1(order(j)));
        }
        std::vector<std::size_t> shape = {temp.size()};
        xt::xarray<float> temp_xx1 = xt::adapt(temp, shape);
        xx1 = xt::maximum(x1(i), temp_xx1);

        // ========= yy1
        temp.clear();
        for (int j = 1; j < order.shape(0); ++j){
            temp.push_back(y1(order(j)));
        }
        shape = {temp.size()};
        xt::xarray<float> temp_yy1 = xt::adapt(temp, shape);
        yy1 = xt::maximum(y1(i), temp_yy1);

        // ========= xx2
        temp.clear();
        for (int j = 1; j < order.shape(0); ++j){
            temp.push_back(x2(order(j)));
        }
        shape = {temp.size()};
        xt::xarray<float> temp_xx2 = xt::adapt(temp, shape);
        xx2 = xt::minimum(x2(i), temp_xx2);

        // ========= yy2
        temp.clear();
        for (int j = 1; j < order.shape(0); ++j){
            temp.push_back(y2(order(j)));
        }
        shape = {temp.size()};
        xt::xarray<float> temp_yy2 = xt::adapt(temp, shape);
        yy2 = xt::minimum(y2(i), temp_yy2);

//        std::cout<<"xx1 : "<<xx1<<"\n";
//        std::cout<<"yy1 : "<<yy1<<"\n";
//        std::cout<<"xx2 : "<<xx2<<"\n";
//        std::cout<<"yy2 : "<<yy2<<"\n";


        xt::xarray<float> w = xt::maximum(0.0, xx2 - xx1 + 1);
        xt::xarray<float> h = xt::maximum(0.0, yy2 - yy1 + 1);
        xt::xarray<float> inter = w * h;
//        std::cout<<"inter : "<<inter<<"\n";

        xt::xarray<float> areas_temp = xt::zeros<float>({order.size() - 1});
        for (int j = 1; j < order.size(); ++j){
            areas_temp(j - 1) = area(order(j));
        }
        xt::xarray<float> ovr = inter / (area(i) + areas_temp - inter);
//        std::cout<<"ovr : "<<ovr<<"\n";

        std::vector<unsigned long> inds = xt::where(ovr <= iou_thresh)[0];

        int count = 0;
        for (int j = 0; j < inds.size(); ++j) {
            order(count++) = order(inds[j] + 1);
        }

        order = xt::view(order, xt::range(0, count));
//        std::cout<<order<<"\n";
    }

    if (keep.size() <= max_boxes) {
        return keep;
    } else {
        std::vector<int>::const_iterator first = keep.cbegin();
        std::vector<int>::const_iterator last = keep.cbegin() + max_boxes;
        std::vector<int> keep_(first, last);
        return keep_;
    }
}


void nms_different_class(xt::xarray<float> &boxes, xt::xarray<float> &scores, xt::xarray<float> &labels, int max_boxes) {
    std::vector<int> indicss = py_nms(boxes, scores, max_boxes);
    std::sort(indicss.begin(), indicss.end());
    int count = 0;
    for (int i = 0; i < indicss.size(); ++i) {
        xt::view(boxes, count, xt::all()) = xt::view(boxes, i, xt::all());
        scores(count) = scores(i);
        labels(count) = labels(i);
        ++count;
    }
    boxes = xt::view(boxes, xt::range(0, count));
    scores = xt::view(scores, xt::range(0, count));
    labels = xt::view(labels, xt::range(0, count));
}

void remove_people_in_the_vehicle(xt::xarray<float> &boxes, xt::xarray<float> &scores, xt::xarray<float> &labels) {
    xt::xarray<bool> indices = xt::ones_like(labels);
    std::vector<unsigned long> not_people_boxes = xt::where(labels > 0 or labels < 0)[0];
    for (int i = 0; i < boxes.shape(0); ++i) {
        if (int(labels(i)) == 0) {
            xt::xarray<float> this_people_box = xt::view(boxes, i);
            for (int j = 0; j < not_people_boxes.size(); ++j) {
                xt::xarray<float> this_not_people_box = xt::view(boxes, not_people_boxes[j]);
                if (this_not_people_box(0) < this_people_box(0) and this_not_people_box(1) - 50 < this_people_box(1)
                    and this_not_people_box(2) > this_people_box(2) and this_not_people_box(3) > this_people_box(3)) {
                    indices(i) = false;
                }
            }
        }
    }
    int count = 0;
    for (int i = 0; i < indices.shape(0); ++i) {
        if (indices(i)) {
            xt::view(boxes, count) = xt::view(boxes, i);
            scores(count) = scores(i);
            labels(count) = labels(i);
            ++count;
        }
    }
    boxes = xt::view(boxes, xt::range(0, count));
    scores = xt::view(scores, xt::range(0, count));
    labels = xt::view(labels, xt::range(0, count));
}

void remove_people_above_the_bycycle_motor(xt::xarray<float> &boxes, xt::xarray<float> &scores, xt::xarray<float> &labels) {
    xt::xarray<bool> indices = xt::ones_like(labels);
    std::vector<int> classes = {2, 3, 4};
    for (int i = 0; i < boxes.shape(0); ++i) {
        if (int(labels(i)) == 0) {
            xt::xarray<float> this_people_box = xt::view(boxes, i);
            // std::cout<<this_people_box<<"\n";
            for (int j = 0; j < boxes.shape(0); ++j) {
                if (std::find(classes.begin(), classes.end(), int(labels(j))) == classes.end()) {
                    continue;
                }
                xt::xarray<float> this_not_people_box = xt::view(boxes, j);
                // std::cout<<this_not_people_box<<"\n";
                bool a = this_not_people_box(1) <= this_people_box(3) and this_people_box(3) <= this_not_people_box(3);
                // std::cout<<a<<"\n";
                if ((this_not_people_box(1) <= this_people_box(3) and this_people_box(3) <= this_not_people_box(3)) and
                    (((this_not_people_box(0) <= this_people_box(2) and this_people_box(2) <= this_not_people_box(2)) or
                            (this_not_people_box(0) <= this_people_box(0) and this_people_box(0) <= this_not_people_box(2))) or
                     ((this_people_box(0) <= this_not_people_box(2) and this_not_people_box(2) <= this_people_box(2)) or
                             (this_people_box(0) <= this_not_people_box(0) and this_not_people_box(0) <= this_people_box(2))))) {
                    indices(i) = false;
                    boxes(j, 1) = (boxes(i, 1) + boxes(j, 1)) / 2;
                    scores(j) = 1;
                    break;
                }
            }
        }
    }
    // std::cout<<indices<<"\n";
    int count = 0;
    for (int i = 0; i < indices.shape(0); ++i) {
        if (indices(i)) {
            xt::view(boxes, count) = xt::view(boxes, i);
            scores(count) = scores(i);
            labels(count) = labels(i);
            ++count;
        }
    }
    boxes = xt::view(boxes, xt::range(0, count));
    scores = xt::view(scores, xt::range(0, count));
    labels = xt::view(labels, xt::range(0, count));
}

void post_nms_WJ_15class(xt::xarray<float> &boxes, xt::xarray<float> &scores, xt::xarray<float> &labels, int max_boxes) {
    remain_specific_class(boxes, scores, labels);
    nms_different_class(boxes, scores, labels, max_boxes);
    remove_people_in_the_vehicle(boxes, scores, labels);
    remove_people_above_the_bycycle_motor(boxes, scores, labels);

}

