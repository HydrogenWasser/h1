//
// Created by root on 4/12/21.
//

#ifndef SECOND_DETECTOR_FUKALMANFILTERCROSSING_H
#define SECOND_DETECTOR_FUKALMANFILTERCROSSING_H

#include <iostream>
//#include <xtensor/xarray.hpp>
// #include <Eigen/Dense>
namespace helpers_crossing {
    xt::xarray<float> reshape_z(xt::xarray<float> z, int dim_z, int ndim);
}
//
//
class FUKalmanFilter_crossing{
public:
    FUKalmanFilter_crossing(int dim_x, int dim_z, int dim_u = 0);
    void predict(xt::xarray<float> u = xt::zeros<float>({0}),
                 xt::xarray<float> B = xt::zeros<float>({0}),
                 xt::xarray<float> F = xt::zeros<float>({1,1}),
                 xt::xarray<float> Q = xt::zeros<float>({0}));
    void update(xt::xarray<float> &z,
                xt::xarray<float> R = xt::zeros<float>({0}),
                xt::xarray<float> H = xt::zeros<float>({0}));
    FUKalmanFilter_crossing();
    ~FUKalmanFilter_crossing();

    int _dim_x;
    int _dim_z;
    int _dim_u;

    xt::xarray<float> _x;
    xt::xarray<float> _P;
    xt::xarray<float> _Q;
    xt::xarray<float> _B;
    xt::xarray<float> _F;
    xt::xarray<float> _H;
    xt::xarray<float> _R;
    xt::xarray<float> _alpha_sq = 1;
    xt::xarray<float> _M;
    xt::xarray<float> _z;

    xt::xarray<float> _K;
    xt::xarray<float> _y;
    xt::xarray<float> _S;
    xt::xarray<float> _SI;

    xt::xarray<float> _I;

    xt::xarray<float> _x_prior;
    xt::xarray<float> _P_prior;

    xt::xarray<float> _x_post;
    xt::xarray<float> _P_post;

    double _log_likelihood;
    double _likelihood;
    float _mahalanobis;

//    self.inv = np.linalg.inv

};

#endif //SECOND_DETECTOR_KALMANFILTER_H
