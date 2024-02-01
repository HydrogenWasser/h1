//
// Created by root on 1/6/22.
//

#ifndef FUSION_ALG_POST_NMS_H
#define FUSION_ALG_POST_NMS_H
#include <xtensor/xnpy.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xstrides.hpp>
#include <xtensor-blas/xlinalg.hpp>


namespace post_nms_crossing{
    void postNms(xt::xarray<float> &tlwh_boxes,
                  xt::xarray<float> &scores,
                  xt::xarray<float> &class_idxs);

    void remove_people_in_the_vehicle(xt::xarray<float> &tlwh_boxes,
                                      xt::xarray<float> &scores,
                                      xt::xarray<float> &class_idxs);

    void remove_big_box_in_the_images(xt::xarray<float> &tlwh_boxes,
                                      xt::xarray<float> &scores,
                                      xt::xarray<float> &class_idxs);

    void remove_people_above_bicycle_motor(xt::xarray<float> &tlwh_boxes,
                                           xt::xarray<float> &scores,
                                           xt::xarray<float> &class_idxs);
}



#endif //FUSION_ALG_POST_NMS_H
