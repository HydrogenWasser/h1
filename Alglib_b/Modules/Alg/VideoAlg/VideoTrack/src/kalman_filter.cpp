//
// Created by root on 4/25/21.
//

#include "kalman_filter.h"

xt::xarray<float> cholesky(xt::xarray<float> &arr){
    xt::xarray<float> a = arr;
    for (int i = 0; i < arr.shape(0); ++i) {
        for (int j = 0; j <= i; ++j) {
            a(i, j) = arr(i, j);
            for (int k = 0; k < j; ++k) {
                a(i, j) -= a(i, k) * a(j, k);
            }
            if (i == j)
                a(i, j) = std::sqrt(a(i, j));
            else
                a(i, j) = a(i, j) / a(j, j);
        }
//        for (int x = i + 1; x < a.shape(0); ++x) {
//            a(i, x) = 0.0;
//        }
    }
    return a;
}

KalmanFilterOfVideo::KalmanFilterOfVideo() {
    int ndim = 4, dt = 1;
    _motion_mat = xt::eye<float>({static_cast<unsigned long>(2 * ndim), static_cast<unsigned long>(2 * ndim)});
    for (int i = 0; i < ndim; ++i)
        _motion_mat(i, ndim + i) = float (dt);
    _update_mat = xt::eye<float>({static_cast<unsigned long>(ndim), static_cast<unsigned long>(2 * ndim)});
    _std_weight_position = 1.0 / 20.0;
    _std_weight_velocity = 1.0 / 160.0;

}

void KalmanFilterOfVideo::predict(xt::xarray<float> &mean,
                           xt::xarray<float> &covariance) {
    xt::xarray<float> std_pos = {_std_weight_position * mean(3),
                                 _std_weight_position * mean(3),
                                 0.01,
                                 _std_weight_position * mean(3)};
    xt::xarray<float> std_val = {_std_weight_velocity * mean(3),
                                 _std_weight_velocity * mean(3),
                                 0.00001,
                                 _std_weight_velocity * mean(3)};

    xt::xarray<float> motion_cov = xt::diag(xt::square(xt::concatenate(xt::xtuple(std_pos, std_val), 0)));

    mean = xt::linalg::dot(_motion_mat, mean);
    covariance = xt::linalg::dot(xt::linalg::dot(_motion_mat, covariance),
                                 xt::transpose(_motion_mat)) + motion_cov;

//    return {mean, covariance};
}

std::pair<xt::xarray<float>, xt::xarray<float>> KalmanFilterOfVideo::project(xt::xarray<float> mean,
                                                                      xt::xarray<float> covariance) {
    xt::xarray<float> std = {_std_weight_position * mean(3),
                             _std_weight_position * mean(3),
                             0.1,
                             _std_weight_position * mean(3)};
    xt::xarray<float> innovation_cov = xt::diag(xt::square(std));
    mean = xt::linalg::dot(_update_mat, mean);
    covariance = xt::linalg::dot(xt::linalg::dot(_update_mat, covariance),
                                 xt::transpose(_update_mat));
    return {mean, covariance + innovation_cov};
}

void KalmanFilterOfVideo::update(xt::xarray<float> &mean,
                          xt::xarray<float> &covariance,
                          xt::xarray<float> &measurement) {
    auto project_res = project(mean, covariance);
    xt::xarray<float> projected_mean = project_res.first;
    xt::xarray<float> projected_cov = project_res.second;

    xt::xarray<float> kalman_gain = xt::linalg::dot(xt::linalg::dot(covariance, xt::transpose(_update_mat)),
                                                    xt::linalg::inv(projected_cov));
    xt::xarray<float> innovation = measurement - projected_mean;
    mean = mean + xt::linalg::dot(innovation, xt::transpose(kalman_gain));
    covariance = covariance - xt::linalg::dot(xt::linalg::dot(kalman_gain, projected_cov),
                                              xt::transpose(kalman_gain));
//    return {new_mean, new_covariance};
}

std::pair<xt::xarray<float>, xt::xarray<float>> KalmanFilterOfVideo::initiate(xt::xarray<float> &measurement) {
    xt::xarray<float> mean_pos = measurement;
    xt::xarray<float> mean_val = xt::zeros_like(mean_pos);
    xt::xarray<float> mean = xt::concatenate(xt::xtuple(mean_pos, mean_val), 0);
    xt::xarray<float> std = {2 * _std_weight_position * measurement(3),
                             2 * _std_weight_position * measurement(3),
                             0.0001,
                             2 * _std_weight_position * measurement(3),
                             10 * _std_weight_velocity * measurement(3),
                             10 * _std_weight_velocity * measurement(3),
                             0.00001,
                             10 * _std_weight_velocity * measurement(3)};
    xt::xarray<float> covariance = xt::diag(xt::square(std));
    return {mean, covariance};
}

