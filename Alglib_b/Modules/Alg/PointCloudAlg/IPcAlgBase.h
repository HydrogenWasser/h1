#pragma once

#include <iostream>


struct IPcAlgBase
{
    IPcAlgBase() {};
    virtual ~IPcAlgBase() {};
    virtual xt::xarray<float> RUN(xt::xarray<float> &points_xt) = 0;
};
