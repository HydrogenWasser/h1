//
// Created by root on 4/25/21.
//

// #ifndef SORT_TRACKER_KALMAN_FILTER_H
// #define SORT_TRACKER_KALMAN_FILTER_H
# pragma once
#include <xtensor/xarray.hpp>
#include <xtensor-blas/xlinalg.hpp>

xt::xarray<float> cholesky(xt::xarray<float> &arr);

class KalmanFilterOfVideo{
public:
    KalmanFilterOfVideo();
    void predict(xt::xarray<float> &mean,
                 xt::xarray<float> &covariance);
    std::pair<xt::xarray<float>, xt::xarray<float>> project(xt::xarray<float> mean,
                                                            xt::xarray<float> covariance);
    void update(xt::xarray<float> &mean,
                xt::xarray<float> & covariance,
                xt::xarray<float> &measurement);
    std::pair<xt::xarray<float>, xt::xarray<float>> initiate(xt::xarray<float> &measurement);

    xt::xarray<float> _motion_mat;
    xt::xarray<float> _update_mat;
    float _std_weight_position;
    float _std_weight_velocity;
};
// #endif //SORT_TRACKER_KALMAN_FILTER_H
