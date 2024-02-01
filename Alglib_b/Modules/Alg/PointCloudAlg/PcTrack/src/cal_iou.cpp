//
// Created by root on 2/22/22.
//

#include <utility>
#include <xtensor/xnpy.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xstrides.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xpad.hpp>
#include <xtensor/xoperation.hpp>

#include "cal_iou.h"

bool cal_iou::IsLineSegmentCross(xt::xarray<float> &pFirst1, xt::xarray<float> &pFirst2, xt::xarray<float> &pSecond1,
                                 xt::xarray<float> &pSecond2) {
    float line1 = float(pFirst1[0] * (pSecond1[1] - pFirst2[1]) + pFirst2[0] * (pFirst1[1] - pSecond1[1]) +
                        pSecond1[0] * (pFirst2[1] - pFirst1[1]));
    float line2 = float(pFirst1[0] * (pSecond2[1] - pFirst2[1]) + pFirst2[0] * (pFirst1[1] - pSecond2[1]) +
                        pSecond2[0] * (pFirst2[1] - pFirst1[1]));
    if ((((line1 * line2) >= 0) and (not(line1 == 0 and line2 == 0))) or (line1 == line2))
        return false;
    line1 = float(pSecond1[0] * (pFirst1[1] - pSecond2[1]) + pSecond2[0] * (pSecond1[1] - pFirst1[1]) +
                  pFirst1[0] * (pSecond2[1] - pSecond1[1]));
    line2 = float(pSecond1[0] * (pFirst2[1] - pSecond2[1]) + pSecond2[0] * (pSecond1[1] - pFirst2[1]) +
                  pFirst2[0] * (pSecond2[1] - pSecond1[1]));
    if ((((line1 * line2) >= 0) and (not(line1 == 0 and line2 == 0))) or (line1 == line2))
        return false;
    return true;
}


bool cal_iou::IsRectCross(xt::xarray<float> &p1, xt::xarray<float> &p2, xt::xarray<float> &q1, xt::xarray<float> &q2) {
    bool ret = std::min(p1[0], p2[0]) <= std::max(q1[0], q2[0]) and std::min(q1[0], q2[0]) <= std::max(p1[0], p2[0]) and
               std::min(p1[1], p2[1]) <= std::max(q1[1], q2[1]) and std::min(q1[1], q2[1]) <= std::max(p1[1], p2[1]);
    return ret;
}


std::pair<bool, xt::xarray<float>>
cal_iou::GetCrossPoint(xt::xarray<float> &p1, xt::xarray<float> &p2, xt::xarray<float> &q1, xt::xarray<float> &q2) {
    float x = 0.0, y = 0.0;
    xt::xarray<float> res_xy = {x, y};
    float x1, x2, x3, x4, y1, y2, y3, y4;
    if (IsRectCross(p1, p2, q1, q2)) {
        if (IsLineSegmentCross(p1, p2, q1, q2)) {
            x1 = p1(0);
            x2 = p2(0);
            x3 = q1(0);
            x4 = q2(0);
            y1 = p1(1);
            y2 = p2(1);
            y3 = q1(1);
            y4 = q2(1);
            x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) /
                ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
            y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) /
                ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
            res_xy = {x, y};
            return {true, res_xy};
        }
    }
    return {false, res_xy};
}


bool cal_iou::IsPointInPolygon(xt::xarray<float> &poly, xt::xarray<float> &pt) {
    float area = 0, area_tmp = 0;
    for (int i = 0; i < poly.shape(0); ++i) {
        int num = i % poly.shape(0);
        int num_next = (i + 1) % poly.shape(0);
        float vec_1 = poly(num, 0) - pt(0); //vec_1[0]
        float vec_2 = poly(num, 1) - pt(1);//vec_1[1]
        float vec_3 = poly(num_next, 0) - pt(0);//vec_2[0]
        float vec_4 = poly(num_next, 1) - pt(1);//vec_2[1]
        area += 0.5 * std::abs((vec_1 * vec_4 - vec_2 * vec_3));
    }
    xt::xarray<float> pt_new = xt::view(poly, poly.shape(0) - 1, xt::all());
    for (int i = 0; i < poly.shape(0) - 2; ++i) {
        int num = i % poly.shape(0);
        int num_next = (i + 1) % poly.shape(0);
        float vec_1 = poly(num, 0) - pt_new(0); //vec_1[0]
        float vec_2 = poly(num, 1) - pt_new(1);//vec_1[1]
        float vec_3 = poly(num_next, 0) - pt_new(0);//vec_2[0]
        float vec_4 = poly(num_next, 1) - pt_new(1);//vec_2[1]
        area_tmp += 0.5 * std::abs((vec_1 * vec_4 - vec_2 * vec_3));
    }
    return area == area_tmp;
}


bool cal_iou::PointCmp(xt::xarray<float> &a, xt::xarray<float> &b, float &center_x, float &center_y) {
    if (a(0) >= 0 and b(0) < 0)
        return true;
    if (a[0] == 0 and b[0] == 0)
        return a(1) > b(1);
    int det = int((a(1) - center_x) * (b(1) - center_y) - (b(0) - center_x) * (a(1) - center_y));
    if (det < 0)
        return true;
    if (det > 0)
        return false;
    int d1 = int((a[0] - center_x) * (a[0] - center_x) + (a[1] - center_y) * (a[1] - center_y));
    int d2 = int((b[0] - center_x) * (b[0] - center_y) + (b[1] - center_y) * (b[1] - center_y));
    return d1 > d2;
}


void cal_iou::ClockwiseSortPoints(std::vector<xt::xarray<float>> &interpoly) {
    float x = 0;
    float y = 0;
    int list_len = interpoly.size();
    for (int i = 0; i < list_len; ++i) {
        x += interpoly[i](0);
        y += interpoly[i](1);
    }
    float center_x = int(x / list_len);
    float center_y = int(y / list_len);
    for (int i = 0; i < list_len - 1; ++i) {
        for (int j = 0; j < list_len - i - 1; ++j) {
            if (cal_iou::PointCmp(interpoly[j], interpoly[j + 1], center_x, center_y)) {
                xt::xarray<float> point_tmp = interpoly[j];
                interpoly[j] = interpoly[j + 1];
                interpoly[j + 1] = point_tmp;
            }
        }
    }
}

//poly1 4X2  poly2 4X2
bool
cal_iou::PolygonClip(xt::xarray<float> &poly1, xt::xarray<float> &poly2, std::vector<xt::xarray<float>> &interpoly) {
    if (poly1.shape(0) < 3 or poly2.shape(0) < 3)
        return false;
    xt::xarray<float> p1 = poly1;
    p1.reshape({1, poly1.size()});
    xt::xarray<float> p2 = poly2;
    p2.reshape({1, poly2.size()});
    auto res_equal = xt::equal(p1, p2);
    bool res_bool;
    for (int i = 0; i < res_equal.size() - 1; ++i) {
        res_bool = res_equal(i) and res_equal(i + 1);
    }
    if (!res_bool) {
        return false;
    } else {
        xt::xarray<float> pol1y1_1;
        xt::xarray<float> pol1y1_2;
        xt::xarray<float> pol1y2_1;
        xt::xarray<float> pol1y2_2;
        for (int i = 0; i < poly1.shape(0); ++i) {
            int poly1_next_idx = (i + 1) % poly1.shape(0);
            for (int j = 0; j < poly2.shape(0); ++j) {
                int poly2_next_idx = (j + 1) % poly2.shape(0);
                pol1y1_1 = xt::view(poly1, i);
                pol1y1_2 = xt::view(poly1, poly1_next_idx);
                pol1y2_1 = xt::view(poly2, i);
                pol1y2_2 = xt::view(poly2, poly2_next_idx);
                auto get_cross_point = cal_iou::GetCrossPoint(pol1y1_1, pol1y1_2, pol1y2_1, pol1y2_2);
                if (get_cross_point.first) {
                    interpoly.push_back(get_cross_point.second);
                }
            }
        }
        for (int i = 0; i < poly1.shape(0); ++i) {
            pol1y1_1 = xt::view(poly1, i);
            if (cal_iou::IsPointInPolygon(poly2, pol1y1_1)) {
                interpoly.push_back(pol1y1_1);
            }
        }
        for (int i = 0; i < poly2.shape(0); ++i) {
            pol1y2_1 = xt::view(poly2, i);
            if (cal_iou::IsPointInPolygon(poly1, pol1y2_1)) {
                interpoly.push_back(pol1y2_1);
            }
        }
        if (interpoly.size() <= 0)
            return false;
//        cal_iou::ClockwiseSortPoints(interpoly);
        return true;
    }
}

float cal_iou::CalArea(std::vector<xt::xarray<float>> &interpoly) {
    int l = interpoly.size();
    xt::xarray<float> pt = interpoly[l - 1];
    float area = 0;
    for (int i = 0; i < l; ++i) {
        float line1 = interpoly[i](0) - pt(0);
        float line2 = interpoly[i](1) - pt(1);
        float line3 = interpoly[i + 1](0) - pt(0);
        float line4 = interpoly[i + 1](1) - pt(1);
        area += 0.5 * std::abs((line1 * line4 - line2 * line3));
    }
    return area;
}