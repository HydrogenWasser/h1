//
// Created by root on 10/24/22.
//

#ifndef TESTALGLIB_FUKALMANBOXTRACKER_H
#define TESTALGLIB_FUKALMANBOXTRACKER_H

#endif //TESTALGLIB_FUKALMANBOXTRACKER_H

#include <iostream>
#include <xtensor/xmath.hpp>
#include <cmath>
#include <string>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <utility>
#include <xtensor/xmath.hpp>
#include <xtensor/xcontainer.hpp>
#include <xtensor/xsort.hpp>
#include <xtensor/xbuilder.hpp>
#include <vector>
#include <xtensor/xcsv.hpp>


#include "FUKalmanFilter_crossing.h"

struct fustate_output_crossing{
    xt::xarray<float> x;
    xt::xarray<float> bbox;
    float speed;
    std::vector<xt::xarray<float>> angle_box;
    float lefttopx;
    float lefttopy;
    float rightbottomx;
    float rightbottomy;
    float camid;
    float VideoBoxid;
};

class FUKalmanBoxTracker_crossing{
public:

    FUKalmanBoxTracker_crossing(xt::xarray<float> bbox);

    FUKalmanBoxTracker_crossing();

    FUKalmanBoxTracker_crossing(const FUKalmanBoxTracker_crossing & rhs);

    ~FUKalmanBoxTracker_crossing();

    void update(xt::xarray<float> bbox);

    std::pair<xt::xarray<float>, xt::xarray<float>> predict();

    fustate_output_crossing get_state();

    std::vector<int> Get_label(){return _label_box;};

    void setLabel(int index, int label);

    static int _count;


public:
    FUKalmanFilter_crossing _kf;

    //info
    int _id;
    xt::xarray<float> _bbox;
    int _hits = 0;
    int _time_since_update = 0;

    std::vector<xt::xarray<float>> _state;

    //speed
    float _speed;
    bool _high_speed;
    bool _low_speed;
    int _speed_thresh;

    //angle
    std::vector<xt::xarray<float>> _angle_box;
    std::vector<float> _angle_list;
    float _detec_angle;
    float _lane_angle;
    float _track_angle;
    float _final_angle;
    std::vector<float> _head_angle;

    int _label = 0;
    int _ilaneIndexGlobal;

    //video
    float _lefttopx = 0;
    float _lefttopy = 0;
    float _rightbottomx = 0;
    float _rightbottomy = 0;
    int _camid = -1;
    int _VideoBoxid = -1;
private:

    int _use_acc_model = 0;


    bool _min_q;
    bool _max_q;


    std::vector<xt::xarray<float>> _history;
    std::map<int, xt::xarray<float>> _label_dict;

    bool _dynamic;
    bool _static;

    std::vector<xt::xarray<float>> _state_judge;


    std::vector<int> _label_box;
    std::vector<int> _label_box_central_area;
};
