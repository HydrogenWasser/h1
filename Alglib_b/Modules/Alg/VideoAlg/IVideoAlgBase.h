#pragma once

#include <iostream>
#include "xtensor/xarray.hpp"

struct IVideoAlgBase
{
    IVideoAlgBase() {};
    virtual ~IVideoAlgBase() {};
    virtual std::vector<xt::xarray<float>> RUN(std::vector<cv::Mat> & img_batch) = 0;    // detect

    virtual xt::xarray<float> RUN(xt::xarray<float> &input) = 0;                         // track
};
