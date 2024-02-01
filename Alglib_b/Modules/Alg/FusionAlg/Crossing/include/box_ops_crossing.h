//
// Created by root on 5/11/21.
//

#ifndef FUSION_ALG_BOX_OPS_H
#define FUSION_ALG_BOX_OPS_H
#include <xtensor/xnpy.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xstrides.hpp>
#include <xtensor-blas/xlinalg.hpp>

#define None -10000.1

namespace box_ops_crossing {
    xt::xarray<float> corners_nd(xt::xarray<float> &dims, float origin = 0.5);
    xt::xarray<float> rotation_2d(xt::xarray<float> &corners, const xt::xarray<float> &angles);
    xt::xarray<float> center_to_corner_box2d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                                   xt::xarray<float> &angles, float origin = 0.5);

    xt::xarray<float> center_to_corner_box3d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                             float &angles, float origin = 0.5, int axis = 2);
    void rotation_3d_in_axis(xt::xarray<float> &points, float &angles, int axis = 0);


    xt::xarray<float> corner_to_standup_nd(xt::xarray<float> &boxes_corner);

    std::pair<xt::xarray<float>, xt::xarray<float>> rotate_nms_cc(xt::xarray<float> &dets,
                                                                xt::xarray<float> &trackers);

    float cal_angle(std::vector<xt::xarray<float>> &state_list, float &thresh);


    std::pair<xt::xarray<float>, xt::xarray<float>> iou_jit_new(xt::xarray<float> &boxes,
                                                                xt::xarray<float> &query_boxes,
                                                                float eps = 0.0);

}

#endif //FUSION_ALG_BOX_OPS_H
