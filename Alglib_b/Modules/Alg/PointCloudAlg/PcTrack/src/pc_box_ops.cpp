//
// Created by root on 4/14/21.
//
#include <math.h>
#include "SORT_tunnel.h"
#include <utility>
#include <xtensor/xnpy.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xstrides.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xpad.hpp>

#define PI_PI 3.1415926
xt::xarray<float> sort::corners_nd(xt::xarray<float> &dims, float origin) {
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
    dims = dims.reshape({-1, 1, ndim});
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

xt::xarray<float> sort::rotation_2d(xt::xarray<float> &corners, const xt::xarray<float> &angles) {

    xt::xarray<float> rot_sin = xt::sin(angles);
    xt::xarray<float> rot_cos = xt::cos(angles);
    rot_cos.reshape({1, -1});
    rot_sin.reshape({1, -1});

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

xt::xarray<float> sort::center_to_corner_box2d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                               xt::xarray<float> &angles, float origin) {
//    std::cout<<dims<<"\n";
    xt::xarray<float> corners = sort::corners_nd(dims, origin);

    if (angles.size())
        corners = sort::rotation_2d(corners, angles);

    centers.reshape({-1, 1, 2});
    corners += centers;
//    std::cout<<corners<<"\n";
    return corners;
}

xt::xarray<float> sort::corner_to_standup_nd(xt::xarray<float> &boxes_corner) {
    xt::xarray<int> box_shape = xt::adapt(boxes_corner.shape());
    assert(box_shape.size() == 3);
    xt::xarray<float> standup_boxes1 = xt::amin(boxes_corner, 1);
//    std::cout<<standup_boxes1<<"\n";
    xt::xarray<float> standup_boxes2 = xt::amax(boxes_corner, 1);
//    std::cout<<standup_boxes2<<"\n";
//    std::cout<<xt::concatenate(xt::xtuple(standup_boxes1, standup_boxes2), 1)<<"\n";
    return xt::concatenate(xt::xtuple(standup_boxes1, standup_boxes2), 1);
}

std::pair<xt::xarray<float>, xt::xarray<float>> sort::rotate_nms_cc(xt::xarray<float> &dets,
                                                                    xt::xarray<float> &trackers) {
    xt::xarray<float> t1 = xt::view(trackers, xt::all(), xt::range(0, 2));
    xt::xarray<float> t2 = xt::view(trackers, xt::all(), xt::range(2, 4));
    xt::xarray<float> t3 = (xt::view(trackers, xt::all(), 4)) * PI_PI / 180;
    xt::xarray<float> trackers_corners = sort::center_to_corner_box2d(t1, t2, t3);

    xt::xarray<float> d1 = xt::view(dets, xt::all(), xt::range(0, 2));
    xt::xarray<float> d2 = xt::view(dets, xt::all(), xt::range(2, 4));
    xt::xarray<float> d3 = (xt::view(dets, xt::all(), 4)) * PI_PI / 180;
    xt::xarray<float> dets_corners = sort::center_to_corner_box2d(d1, d2, d3);
//    std::cout << dets_corners << std::endl;
    auto iou_res = sort::get_iou(dets_corners, trackers_corners, 1);
    return iou_res;
}


float sort::cal_angle(std::vector<xt::xarray<float>> &state_list, float thresh) {
//    for (auto iii : state_list){
//        std::cout<<iii<<"\n";
//    }
    auto dis_x = state_list[state_list.size() - 1](0, 0) - state_list[0](0, 0);
    auto dis_y = state_list[state_list.size() - 1](0, 1) - state_list[0](0, 1);
    auto dis_len = std::sqrt(dis_x * dis_x + dis_y * dis_y);
    float dis_angle;
    if (dis_len > thresh){
        dis_angle = std::acos(dis_x / dis_len) * 180 / PI_PI - 180;
        if (dis_y > 0)
            dis_angle = 90 - dis_angle;
        else
            dis_angle = 90 - (360 - dis_angle);
        dis_angle = std::fmod(dis_angle, 360.0);
//        dis_angle = dis_angle >= 0 ? dis_angle : 360.0 + dis_angle;
    } else{
        dis_angle = None_PI;
    }
    return dis_angle;
}

std::pair<xt::xarray<float>, xt::xarray<float>> sort::cal_lateral_dis(xt::xarray<float> &detections,
                                                                      xt::xarray<float> &trackers,
                                                                      int flag){
    xt::xarray<float> det_xy = xt::view(detections, xt::all(), xt::range(0, 2));
    xt::xarray<float> tra_xy = xt::view(trackers, xt::all(), xt::range(0, 2));
    xt::xarray<float> asso_vector_x, asso_vector_y, theta;
    int tracker_row = tra_xy.shape(0);
    int detects_row = det_xy.shape(0);
    xt::xarray<float> tra_xy_0 = xt::col(tra_xy, 0);
    xt::xarray<float> tra_xy_1 = xt::col(tra_xy, 1);
    xt::xarray<float> det_xy_0 = xt::col(det_xy, 0);
    xt::xarray<float> det_xy_1 = xt::col(det_xy, 1);
    tra_xy_0.reshape({1, tracker_row});
    tra_xy_1.reshape({1, tracker_row});
    det_xy_0.reshape({detects_row, 1});
    det_xy_1.reshape({detects_row, 1});
    asso_vector_x = det_xy_0 - tra_xy_0;
    asso_vector_y = det_xy_1 - tra_xy_1;
    if (flag == 1){
        theta = xt::col(trackers, 6) * 180 / PI_PI;
    }
    else{
        theta = xt::col(trackers, 4);
    }
    theta = theta.reshape({1, trackers.shape(0)});
    theta = (-1 * theta - 90) * PI_PI / 180;
    xt::xarray<float> head_vector_x = xt::cos(theta);
    xt::xarray<float> head_vector_y = xt::sin(theta);
    xt::xarray<float> head_dis_new, ano_dis_new;
    std::pair<xt::xarray<float>, xt::xarray<float>> dist_pair;
    xt::xarray<float> new_head_vector_x = xt::tile(head_vector_x, asso_vector_x.shape(0));
    xt::xarray<float> new_head_vector_y = xt::tile(head_vector_y, asso_vector_y.shape(0));
    head_dis_new = xt::abs(asso_vector_x * new_head_vector_x + asso_vector_y * new_head_vector_y);
    ano_dis_new = xt::abs(asso_vector_x * new_head_vector_y - asso_vector_y * new_head_vector_x);
    dist_pair = {head_dis_new, ano_dis_new};
    return dist_pair;
}

float sort::median_sizeof_car(std::vector<float> &source){
    float median_source;
    std::sort(source.begin(), source.end());
    int length = source.size();
    if (length % 2 == 0){
        median_source = (source[length / 2 - 1] + source[length / 2]) / 2;
    }
    else{
        median_source = source[(length - 1) / 2];
    }
    return median_source;
}

bool sort::cmp2(std::pair<int, int>&a, std::pair<int, int> &b) {
    return a.second < b.second;//对于second的升序
}

std::pair<int, int> sort::majority_element(std::vector<int>& nums) {
    int n = nums.size();
    std::map<int, int> mapping;
    for(int num: nums)
    {
        ++mapping[num];
    }
    int hit = 0, id = 0;
    for (auto & label: nums) {
        if (mapping[label] > hit){
            hit = mapping[label];
            id = label;
        }
    }
//    std::vector<std::pair<int, int>> vecs, vecs_bak;
//    for (auto it = mapping.begin();it!= mapping.end();it++) {
//        vecs.push_back(std::make_pair(it->first, it->second));
//    }
////    对于second升序排列
//    std::sort(vecs_bak.begin(), vecs_bak.end(), sort::cmp2);
//    std::pair<int, int> res = {-1, -1};
//    int l = vecs_bak.size();
//    auto res_pos = vecs_bak[l-1];
//    for (int i = 0; i < vecs.size(); ++i) {
//        if (res_pos.second == vecs[i].second){
//            res = vecs[i];
//            break;
//        }
//    }
    return {id, hit};
}

