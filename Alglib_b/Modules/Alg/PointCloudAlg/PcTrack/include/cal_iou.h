//
// Created by root on 2/22/22.
//

#ifndef SORT_TRACKER_CAL_IOU_H
#define SORT_TRACKER_CAL_IOU_H
#include <iostream>
#include <xtensor/xmath.hpp>
#include "SORT_tunnel.h"


namespace cal_iou{
    bool IsLineSegmentCross(xt::xarray<float> &pFirst1, xt::xarray<float> &pFirst2, xt::xarray<float> &pSecond1, xt::xarray<float> &pSecond2);
    bool IsRectCross(xt::xarray<float> &p1, xt::xarray<float> &p2, xt::xarray<float> &q1, xt::xarray<float> &q2);
    std::pair<bool, xt::xarray<float>> GetCrossPoint(xt::xarray<float> &p1, xt::xarray<float> &p2, xt::xarray<float> &q1, xt::xarray<float> &q2);
    bool IsPointInPolygon(xt::xarray<float> &poly, xt::xarray<float> &pt);
    bool PointCmp(xt::xarray<float> &vPoints_1, xt::xarray<float> &vPoints_2, float &center_x, float &center_y);
    void ClockwiseSortPoints(std::vector<xt::xarray<float>> &interpoly);
    bool PolygonClip(xt::xarray<float> &poly1, xt::xarray<float> &poly2, std::vector<xt::xarray<float>> &interpoly);
    float CalArea(std::vector<xt::xarray<float>> &interpoly);
}
#endif //SORT_TRACKER_CAL_IOU_H
