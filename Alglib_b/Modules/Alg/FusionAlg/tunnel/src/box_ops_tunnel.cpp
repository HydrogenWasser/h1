//
// Created by root on 5/11/21.
//

#include "box_ops_tunnel.h"

xt::xarray<float> box_ops_tunnel::corners_nd(xt::xarray<float> &dims, float origin) {
    int ndim = dims.shape(1);
//    std::cout<<dims<<"\n";

    auto shape = xt::ones<int>({ndim}) * 2;
    xt::xarray<float> corners_norm;
    if (ndim == 2){
        corners_norm = xt::zeros<float>({4,2});
        corners_norm(1, 1) = 1;
        corners_norm(2, 0) = 1;
        corners_norm(3, 0) = 1;
        corners_norm(2, 1) = 1;
    } else{
        corners_norm = xt::zeros<float>({8,3});
        corners_norm(4, 0) = 1;
        corners_norm(5, 0) = 1;
        corners_norm(6, 0) = 1;
        corners_norm(7, 0) = 1;
        corners_norm(2, 1) = 1;
        corners_norm(3, 1) = 1;
        corners_norm(6, 1) = 1;
        corners_norm(7, 1) = 1;
        corners_norm(1, 2) = 1;
        corners_norm(2, 2) = 1;
        corners_norm(5, 2) = 1;
        corners_norm(6, 2) = 1;
    }
    corners_norm = corners_norm - origin;
    dims = dims.reshape({int (dims.size() / ndim), 1, ndim});
    corners_norm = corners_norm.reshape({1, int(std::pow(2, ndim)), ndim});
    xt::xarray<float> res = xt::ones<float>({int (dims.shape(0)),int(std::pow(2, ndim)), ndim});
    for (int i = 0; i < res.shape(0); ++i) {
//        std::cout<<xt::view(dims, i) <<"\n";
//        std::cout<<xt::view(corners_norm, 0)<<"\n";
        xt::view(res, i) = xt::view(dims, i) * xt::view(corners_norm, 0);
    }
//    std::cout<<res<<"\n";
    return res;
}

xt::xarray<float> box_ops_tunnel::rotation_2d(xt::xarray<float> &corners, const xt::xarray<float> &angles) {

    xt::xarray<float> rot_sin = xt::sin(angles);
    xt::xarray<float> rot_cos = xt::cos(angles);
    rot_cos.reshape({1, rot_cos.size()});
    rot_sin.reshape({1, rot_sin.size()});

    xt::xarray<float> a1 = xt::concatenate(xt::xtuple(rot_cos, -1 * rot_sin), 0);
    a1.reshape({1, a1.shape(0), a1.shape(1)});
    xt::xarray<float> a2 = xt::concatenate(xt::xtuple(rot_sin, rot_cos), 0);
    a2.reshape({1, a2.shape(0), a2.shape(1)});
    xt::xarray<float> rot_mat_T = xt::concatenate(xt::xtuple(a1, a2), 0);
//    std::cout<<rot_mat_T<<"\n";

//    xt::dump_npy("/data/second_cpp2/npy_test_corners.npy", corners);
//    xt::dump_npy("/data/second_cpp2/npy_test_rot_math_t.npy", rot_mat_T);
    for (int i = 0; i < corners.shape(0); ++i) {
        xt::xarray<float> a_ = xt::view(corners, i);
        xt::xarray<float> b_ = xt::view(rot_mat_T, xt::all(), xt::all(), i);
        xt::view(corners, i) = xt::linalg::dot(a_, b_);
    }
//    std::cout<<corners<<"\n";
    return corners;
}

xt::xarray<float> box_ops_tunnel::center_to_corner_box2d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                               xt::xarray<float> &angles, float origin) {
//    std::cout<<dims<<"\n";
    xt::xarray<float> corners = box_ops_tunnel::corners_nd(dims, origin);

    if (angles.size())
        corners = box_ops_tunnel::rotation_2d(corners, angles);

    centers.reshape({-1, 1, 2});
    corners += centers;
//    std::cout<<corners<<"\n";
    return corners;
}

xt::xarray<float> box_ops_tunnel::center_to_corner_box3d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                                  float &angles, float origin, int axis) {
    xt::xarray<float> corners = box_ops_tunnel::corners_nd(dims, origin);
    if (angles != float (None))
        box_ops_tunnel::rotation_3d_in_axis(corners, angles, axis);
//    std::cout<<corners<<"\n";
    centers = centers.reshape({int (centers.size() / 3), 1, 3});
//    std::cout<<corners + centers<<"\n";
    return corners + centers;
}

xt::xarray<float> box_ops_tunnel::corner_to_standup_nd(xt::xarray<float> &boxes_corner) {
    xt::xarray<int> box_shape = xt::adapt(boxes_corner.shape());
    assert(box_shape.size() == 3);
    xt::xarray<float> standup_boxes1 = xt::amin(boxes_corner, 1);
//    std::cout<<standup_boxes1<<"\n";
    xt::xarray<float> standup_boxes2 = xt::amax(boxes_corner, 1);
//    std::cout<<standup_boxes2<<"\n";
//    std::cout<<xt::concatenate(xt::xtuple(standup_boxes1, standup_boxes2), 1)<<"\n";
    return xt::concatenate(xt::xtuple(standup_boxes1, standup_boxes2), 1);
}

std::pair<xt::xarray<float>, xt::xarray<float>> box_ops_tunnel::rotate_nms_cc(xt::xarray<float> &dets,
                                                                    xt::xarray<float> &trackers) {
    xt::xarray<float> t1 = xt::view(trackers, xt::all(), xt::range(0, 2));
    xt::xarray<float> t2 = xt::view(trackers, xt::all(), xt::range(2, 4));
    xt::xarray<float> t3 = xt::view(trackers, xt::all(), 4);
    xt::xarray<float> trackers_corners = box_ops_tunnel::center_to_corner_box2d(t1, t2, t3);
    xt::xarray<float> trackers_standup = box_ops_tunnel::corner_to_standup_nd(trackers_corners);
//    std::cout<<trackers_standup<<"\n";
    xt::xarray<float> d1 = xt::view(dets, xt::all(), xt::range(0, 2));
    xt::xarray<float> d2 = xt::view(dets, xt::all(), xt::range(2, 4));
    xt::xarray<float> d3 = xt::view(dets, xt::all(), 4);
    xt::xarray<float> dets_corners = box_ops_tunnel::center_to_corner_box2d(d1, d2, d3);
    xt::xarray<float> dets_standup = box_ops_tunnel::corner_to_standup_nd(dets_corners);
//    std::cout<<dets_standup<<"\n";
    return box_ops_tunnel::iou_jit_new(dets_standup, trackers_standup, 0.0);
}


float box_ops_tunnel::cal_angle(std::vector<xt::xarray<float>> &state_list, float &thresh) {
//    for (auto iii : state_list){
//        std::cout<<iii<<"\n";
//    }
    auto dis_x = state_list[state_list.size() - 1](0, 0) - state_list[0](0, 0);
    auto dis_y = state_list[state_list.size() - 1](0, 1) - state_list[0](0, 1);
    auto dis_len = std::sqrt(dis_x * dis_x + dis_y * dis_y);
    float dis_angle;
    if (dis_len > thresh){
        dis_angle = std::acos(dis_x / dis_len) * 180 / M_PI - 180;
        if (dis_y > 0)
            dis_angle = 90 - dis_angle;
        else
            dis_angle = 90 - (360 - dis_angle);
        dis_angle = std::fmod(dis_angle, 360.0);
//        dis_angle = dis_angle >= 0 ? dis_angle : 360.0 + dis_angle;
    } else{
        dis_angle = None;
    }
    return dis_angle;
}

std::pair<xt::xarray<float>, xt::xarray<float>> box_ops_tunnel::iou_jit_new(xt::xarray<float> &boxes,
                                                                     xt::xarray<float> &query_boxes, float eps) {
    int N = boxes.shape(0);
    int K = query_boxes.shape(0);
    xt::xarray<float> overlaps = xt::zeros<float>({N, K});
    xt::xarray<float> overlaps_new = xt::zeros<float>({N, K});
    for (int k = 0; k < K; ++k) {
        float box_area = (query_boxes(k, 2) - query_boxes(k, 0) + eps) * (query_boxes(k, 3) - query_boxes(k, 1) + eps);
        for (int n = 0; n < N; ++n) {
            float iw = std::min(boxes(n, 2), query_boxes(k, 2)) - std::max(boxes(n, 0), query_boxes(k, 0)) + eps;
            if (iw > 0){
                float ih = std::min(boxes(n, 3), query_boxes(k, 3)) - std::max(boxes(n, 1), query_boxes(k, 1)) + eps;
                if (ih > 0){
                    float ua = (boxes(n, 2) - boxes(n, 0) + eps) * (boxes(n, 3) - boxes(n, 1) + eps) + box_area - iw * ih;
                    overlaps(n, k) = iw * ih / ua;
                    overlaps_new(n, k) = iw * ih / box_area;
                }
            }
        }
    }
    return {overlaps, overlaps_new};
}
xt::xarray<float> box_ops_tunnel::iou_jit(xt::xarray<float> &boxes, xt::xarray<float> &query_boxes, float eps){
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
void box_ops_tunnel::rotation_3d_in_axis(xt::xarray<float> &points, float &angles, int axis) {
    float rot_sin = std::sin(angles);
    float rot_cos = std::cos(angles);
    float ones = 1.0;
    float zeros = 0.0;
    xt::xarray<float> rot_mat_T;
    if (axis == 1){
        rot_mat_T = xt::xarray<float>{rot_cos, zeros, -1 * rot_sin,
                                      zeros, ones, zeros,
                                      rot_sin, zeros, rot_cos};

    }
    else if (axis == 2 or axis == -1){
        rot_mat_T = xt::xarray<float>{rot_cos, -1 * rot_sin, zeros,
                                      rot_sin, rot_cos, zeros,
                                      zeros, zeros, ones};
    }
    else{
        throw "axis should in range";
    }

    rot_mat_T = rot_mat_T.reshape({3,3,1});

//    std::cout<<points<<"\n";
//    std::cout<<rot_mat_T<<"\n";

    for (int i = 0; i < points.shape(0); ++i) {
        xt::xarray<float> a_ = xt::view(points, i);
        xt::xarray<float> b_ = xt::view(rot_mat_T, xt::all(), xt::all(), i);
        xt::view(points, i) = xt::linalg::dot(a_, b_);
    }
}


float fusion_utils::lidar_class_transform(float lidar_class) {
    float output_class;
    if (int (lidar_class) == 0){  //car
        output_class = 0;
    }
    else if (int(lidar_class) == 1){  // bicycle
            output_class = 1;
    }
    else if (int(lidar_class) == 2){  // large_bus
        output_class = 5;
    }
    else if (int(lidar_class) == 3){  // motorbike
        output_class = 3;
    }
    else if (int(lidar_class) == 4){  // person
        output_class = 4;
    }
    else if (int(lidar_class) == 5){  // large_truck
        output_class = 8;
    }
    else if (int(lidar_class) == 6){  // middle_truck
        output_class = 7;
    }
    else if (int(lidar_class) == 8){  // middle_bus
        output_class = 2;
    }
    else{                             // little_truck
        output_class = 6;
    }
    return output_class;
}
float fusion_utils::video_class_transform(float video_class) {
    float output_class;
    if (int(video_class) == 0){
        output_class = 4;
    }
    else if (int(video_class) == 1 or int(video_class) == 5 or int(video_class) == 6 or int(video_class) == 8 or int(video_class) == 9){
        output_class = 0;
    }
    else if (int(video_class) == 10){
        output_class = 2;
    }
    else if (int(video_class) == 2){
        output_class = 1;
    }
    else if (int(video_class) == 3 or int(video_class) == 4){
        output_class = 3;
    }
    else{
        output_class = 7;
    }
    return output_class;
}

float fusion_utils::judge_class(float &video_class, float &lidar_class, float &video_confidience,
                                float &lidar_confidience) {
    float final_class;
    if (int(video_class) == int(lidar_class)){
        final_class = video_class;
    }
    else{
        if (int(video_class) == 0){
            if ((int(lidar_class) == 2 or int(lidar_class) == 7 or int(lidar_class) == 8) and lidar_confidience <= 0.5){
                final_class = video_class;
            }
            else if ((int(lidar_class) == 5 or int(lidar_class) == 7) and lidar_confidience >= 0.7){
                final_class = lidar_class;
            }
            else if (int(lidar_class) == 1 or int(lidar_class) == 3 or int(lidar_class) == 4){
                final_class = video_class;
            }
            else {
                final_class = video_class;
            }
        }
        else if (int(video_class) == 2){
            if (int(lidar_class) == 0 and lidar_confidience >= 0.5){
                final_class = lidar_class;
            }
            else if (int(lidar_class) == 0 and lidar_confidience < 0.4){
                final_class = video_class;
            }
            else if (int(lidar_class) == 0 and lidar_confidience >= 0.4){
                final_class = lidar_class;
            }
            else if (int(lidar_class) == 7 and lidar_confidience >= 0.75){
                final_class = lidar_class;
            }
            else if (int(lidar_class) == 8 and lidar_confidience >= 0.97){
                final_class = video_class;
            }
            else if (int(lidar_class) == 2 or int(lidar_class) == 5){
                final_class = lidar_class;
            } else{
                final_class = video_class;
            }
        }
        else if (int(video_class) == 1 or int(video_class) == 3){
            if (int(video_class) == 1 and int (lidar_class) == 0 and lidar_confidience >= 0.7){
                final_class = lidar_class;
            }
            else if (int(lidar_class) == 4){
                final_class = video_class;
            }
            else if (int (lidar_class) == 5 or int (lidar_class) == 6 or int (lidar_class) == 7 or int (lidar_class) == 8){
                final_class = lidar_class;
            }
            else if (int (video_class) == 3 and int (lidar_class) == 0 and (video_confidience <= 0.75 or lidar_confidience >= 0.7)){
                final_class = lidar_class;
            }
            else {
                final_class = video_class;
            }
        }
        else if (int(video_class) == 4){
            if (int (lidar_class) == 1 or int (lidar_class) == 3){
                final_class = lidar_class;
            }
            else if (int (lidar_class) == 0 and lidar_confidience <= 0.7){
                final_class = 3;
            }
            else {
                final_class = video_class;
            }
        }
        else if (int (video_class) == 7){
            if (int (lidar_class) == 6 or int (lidar_class) == 7 or int (lidar_class) == 8){
                final_class = lidar_class;
            }
            else if (int (lidar_class) == 0 and lidar_confidience <= 0.4){
                final_class = video_class;
            }
            else if (int (lidar_class) == 0 and lidar_confidience >= 0.6){
                final_class = lidar_class;
            }
            else if (int (lidar_class) == 5 and lidar_confidience >= 0.55){
                final_class = lidar_class;
            }
            else {
                final_class = video_class;
            }
        }
        else {
            final_class = video_class;
        }
    }
    return final_class;
}

float fusion_utils::find_most_times_ele(const std::vector<float> &vecElesParent) {
    float max_times_ele_index = 0;
    int temp_max_count = 0;
    std::map<float, int> info;
    for (size_t i = 0; i < vecElesParent.size(); ++i) {
        info[vecElesParent[i]]++;
    }
    std::map<float, int>::iterator iter;
    for(iter = info.begin(); iter != info.end(); iter++) {
        if (iter->second > temp_max_count){
            max_times_ele_index = iter->first;
            temp_max_count = iter->second;
        }
    }


    return max_times_ele_index;
}