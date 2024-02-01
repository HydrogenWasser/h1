/********
 文件名：fusion_object_ServiceArea.cpp
 作者：Huahuan
 描述：融合算法主函数
 版本：v1.0
 日期：2024.01.10
 *******/

#include <iostream>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xsort.hpp>

#include "fusion_object_ServiceArea.h"

TypeFusion_ServiceArea::TypeFusion_ServiceArea(xt::xarray<float> bbox)
{
    _framework = {
        {"car",         0},
        {"bicycle",     1},
        {"bus",         2},
        {"motorbike",   3},
        {"person",      4},
        {"semi",        5},
        {"truck",       6},
        {"jtruck",      7},
        {"Minibus",     8},
        {"road block",  9},
        {"constrution", 10},
        {"carton",      11},
        {"unknown",     12},
    };

    mass_A = xt::zeros<float>({int(_framework.size())});

    fusion_label = bbox[7];
    fusion_score = bbox[10];

    if (fusion_score >=0.98) fusion_score = 0.98;
    
    _bbox = bbox;
    mass_A = xt::full_like(mass_A, 1-fusion_score);
    mass_A[int(fusion_label)] = 1;
    mass_A[int(mass_A.size() - 1)] = 0;

    int src = 2;
    float dis_thresh = 120;
    
    // 十字路口需要， 服务区不需要这玩意儿吧
    float intersection_center[2] = {0, 0}; 
    _dis_thresh = dis_thresh;
    memcpy(&_intersection_center[0], &intersection_center[0], 2*sizeof(float));
}


void TypeFusion_ServiceArea::updateType(xt::xarray<float>   bbox, 
                                        int                 src)
{
    float sensor_reduction_ratio;
    float dis_thresh;
    float intersection_center[2];
    float detect_score;

    if (src == 2) // 点云？
        sensor_reduction_ratio = 0.5;
    else if (src == 1) // 图像？
        sensor_reduction_ratio = 1;
    else
        sensor_reduction_ratio = 0.5;

    dis_thresh = _dis_thresh;
    detect_score = bbox[10] * sensor_reduction_ratio;

    bbox[10] = detect_score;
    mass_B = xt::zeros<float>({int(_framework.size())});
    mass_B = xt::full_like(mass_B, 1 - bbox[10]);
    mass_B[int(bbox[7])] = 1;
    mass_B[int(mass_B.size() - 1)] = 0;

    xt::xarray<float> T;

    int new_label;

    float fenmu_A, 
          fenmu_B,
          total_k,
          new_score;

    xt::xarray<float> fenzi_A,
                      fenzi_B,
                      bayes_mass_A,
                      bayes_mass_B,
                      k,
                      new_total,
                      new_label_score;
    
    T = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 12};

    fenmu_A = xt::linalg::dot(mass_A, T)[0];
    fenmu_B = xt::linalg::dot(mass_B, T)[0];

    fenzi_A = mass_A*T;
    fenzi_B = mass_B*T;
    fenzi_A[int(fenzi_A.size())-1] = 0;
    fenzi_B[int(fenzi_B.size())-1] = 0;

    bayes_mass_A = (fenzi_A / fenmu_A);
    bayes_mass_B = (fenzi_B / fenmu_B);
    
    k = bayes_mass_A*bayes_mass_B;
    k[int(k.size())-1] = 0;
    total_k = xt::sum(k)[0];

    new_total = bayes_mass_A*bayes_mass_B;
    new_label_score = new_total / total_k;

    mass_A = new_label_score;
    
    new_score = xt::amax(new_label_score)[0];
    new_label = int(xt::argmax(new_label_score)[0]);

    if (new_score >= 1){
        new_score = 0.98;
        mass_A[new_label] = 0.98;
        mass_A[int(mass_A.size()-1)] = 0;

    }

    fusion_score = new_score;
    fusion_label = new_label;
    
    


    
}