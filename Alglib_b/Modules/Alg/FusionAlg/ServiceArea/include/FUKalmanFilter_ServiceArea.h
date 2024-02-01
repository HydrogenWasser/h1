/********
 文件名：FUKalmanBoxFilter_ServiceArea.h
 作者：Huahuan
 描述：卡尔曼滤波器
 版本：v1.0
 日期：2024.01.10
 *******/

#ifndef SECOND_DETECTOR_FUKALMANFILTER_SERVICEAREA_H
#define SECOND_DETECTOR_FUKALMANFILTER_SERVICEAREA_H

#include <iostream>
#include <xtensor/xarray.hpp>

namespace helpers_ServiceArea {
    xt::xarray<float> reshape_z(xt::xarray<float> z,
                                int dim_z,
                                int ndim);
}


class FUKalmanFilter_ServiceArea{
public:
    FUKalmanFilter_ServiceArea(int dim_x,
                               int dim_z = 2,
                               int model_type = 0,
                               int dim_u = 0);
    
    void predict(float dt = 0.1,
                 int model_type = 0);

    void update(xt::xarray<float> &z,
                xt::xarray<float> R = xt::zeros<float>({0}),
                xt::xarray<float> H = xt::zeros<float>({0}));

    FUKalmanFilter_ServiceArea();
    ~FUKalmanFilter_ServiceArea();

public:
    int _dim_x;
    int _dim_z;
    int _dim_u;
    int _model_type = 0;

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
};



#endif //SECOND_DETECTOR_FUKALMANFILTER_SERVICEAREA_H