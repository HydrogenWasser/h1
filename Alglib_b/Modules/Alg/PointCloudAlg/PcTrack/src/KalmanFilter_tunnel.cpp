//
// Created by root on 2/11/22.
//

#include <iostream>
#include <cmath>
#include <xtensor/xarray.hpp>
#include "KalmanFilter_tunnel.h"
#include <xtensor/xadapt.hpp>
#include <xtensor-blas/xlinalg.hpp>
//
xt::xarray<float> helpers::reshape_z(xt::xarray<float> z, int dim_z, int ndim) {
    z = xt::atleast_2d(z);
    if (z.shape(1) == dim_z)
        z = xt::transpose(z);
    assert(z.shape(0) == dim_z and z.shape(1) == 1);

    if (ndim == 1)
        z = xt::view(z, xt::all(), 0);
    if (ndim == 0)
        z = z(0, 0);
    return z;

}
//
KalmanFilter::KalmanFilter(int dim_x, int dim_z, int dim_u):
        _dim_x(dim_x),
        _dim_z(dim_z),
        _dim_u(dim_u){
    assert(dim_x >= 1);
    assert(dim_z >= 1);
    assert(dim_u >= 0);

    _x = xt::zeros<float>({dim_x, 1});
    _P = xt::eye(dim_x);
    _Q = xt::eye(dim_x);
    _F = xt::eye<float>(dim_x);
    _H = xt::zeros<float>({dim_z, dim_x});
    _R = xt::eye(dim_z);
    _alpha_sq = 1.;
    _M = xt::zeros<float>({dim_z, dim_z});
    _z = xt::zeros<float>({dim_z, 1}); // transpose
    _K = xt::zeros<float>({dim_x, dim_z});
    _y = xt::zeros<float>({dim_z, 1});
    _S = xt::zeros<float>({dim_z, dim_z});
    _SI = xt::zeros<float>({dim_z, dim_z});
    _I = xt::eye(dim_x);
    _x_prior = _x;
    _P_prior = _P;
    _x_post = _x;
    _P_post = _P;

    _log_likelihood = -708.3964185322641;
    _likelihood = 2.2250738585072014e-308;
    _mahalanobis = 0;
}
//
KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}
//
void KalmanFilter::predict(xt::xarray<float> u, xt::xarray<float> B, xt::xarray<float> F, xt::xarray<float> Q) {
    /**
    Predict next state (prior) using the Kalman filter state propagation
    equations.

            Parameters
            ----------

    u : np.array
    Optional control vector. If not `None`, it is multiplied by B
    to create the control input into the system.

            B : np.array(dim_x, dim_z), or None
    Optional control transition matrix; a value of None
    will cause the filter to use `self.B`.

    F : np.array(dim_x, dim_x), or None
    Optional state transition matrix; a value of None
    will cause the filter to use `self.F`.

    Q : np.array(dim_x, dim_x), scalar, or None
    Optional process noise matrix; a value of None will cause the
    filter to use `self.Q`.                 */

    if (!B.size())
        B = _B;
    if (F.shape(1) != _dim_x)
        F = _F;
    if (!Q.size()) {
        Q = _Q;
    }
//    elif isscalar(Q):
//    Q = eye(self.dim_x) * Q
//    std::cout<<_x<<"\n";
//    std::cout<<F<<"\n";
    if (!B.size() and u.size())
        _x = xt::linalg::dot(F, _x) + xt::linalg::dot(B, u);
    else
        _x = xt::linalg::dot(F, _x);
//    std::cout<<_x<<"\n";

    _P = _alpha_sq * xt::linalg::dot(xt::linalg::dot(F, _P), xt::transpose(F)) + Q;
    _x_prior = _x;
    _P_prior = _P;
}
//
void KalmanFilter::update(xt::xarray<float> &z, int reduce, float delta_thresh,
                          xt::xarray<float> R, xt::xarray<float> H) {
    _log_likelihood = 0;
    _likelihood = 0;
    _mahalanobis = 0;

    if (!z.size()){
        _z = xt::transpose(xt::zeros<float>({1, _dim_z}));
        _x_post = _x;
        _P_post = _P;
        _y = xt::zeros<float>({_dim_z, 1});
        return;
    }
    xt::xarray<int> z_shape = xt::adapt(z.shape());
    z = helpers::reshape_z(z, _dim_z, z_shape.size());
//    std::cout<<z<<"\n";
    if (!R.size())
        R = _R;
    if (!H.size())
        H = _H;
    _y = z - xt::linalg::dot(H, _x);
//    std::cout<<"_y: " << _y<<"\n";
//    std::cout<<"_x: " << _x<<"\n";
//    std::cout<<"z: " << z<<"\n";
    xt::xarray<float> PHT = xt::linalg::dot(_P, xt::transpose(H));
//    std::cout<<"_P: " << _P<<"\n";
//    std::cout<<"H: " << H<<"\n";
//    std::cout<<"PHT: " << PHT<<"\n";
    float k_reduce = 1;
    if (reduce){
        if (std::abs(_y(0,0)) > delta_thresh)
        {
            k_reduce = 0.1;
        }
    }

//    std::cout<<PHT<<"\n";
    _S = xt::linalg::dot(H, PHT) + R;
//    std::cout<<_S<<"\n";
    _SI = xt::linalg::inv(_S);
//    std::cout<<_SI<<"\n";
    _K = xt::linalg::dot(PHT, _SI);
    _K *= k_reduce;
//    std::cout<<_K<<"\n";
    _x = _x + xt::linalg::dot(_K, _y);
//    std::cout<<_x<<"\n";
    xt::xarray<float> I_KH = _I - xt::linalg::dot(_K, H);
//    std::cout<<I_KH<<"\n";
    _P = xt::linalg::dot(xt::linalg::dot(I_KH, _P), xt::transpose(I_KH)) +
         xt::linalg::dot(xt::linalg::dot(_K, R), xt::transpose(_K));
//    std::cout<<_P<<"\n";
    _z = z;
    _x_post = _x;
    _P_post = _P;
}