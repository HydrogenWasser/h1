//
// Created by root on 5/11/21.
//

#include "box_ops_crossing.h"

/* generate relative box corners based on length per dim and
*   origin point.
*
*   Args:
*       dims(float array, shape = [N, ndim]) : array of length per dim
*       origin(list or array or float) : origin point relate to smallest point.
*
*   Returns :
*       float array, shape = [N, 2 * *ndim, ndim] : returned corners.
*       point layout example : (2d) x0y0, x0y1, x1y0, x1y1;
*           (3d) x0y0z0, x0y0z1, x0y1z0, x0y1z1, x1y0z0, x1y0z1, x1y1z0, x1y1z1
*           where x0 < x1, y0 < y1, z0 < z1
*/
xt::xarray<float> box_ops_crossing::corners_nd(xt::xarray<float> &dims, float origin) 
{
    int ndim = dims.shape(1);
    auto shape = xt::ones<int>({ndim}) * 2;
    xt::xarray<float> corners_norm;
    if (ndim == 2)
    {
        corners_norm = xt::zeros<float>({4,2});
        corners_norm(1, 1) = 1;
        corners_norm(2, 0) = 1;
        corners_norm(3, 0) = 1;
        corners_norm(2, 1) = 1;
    } else
    {
        corners_norm = xt::zeros<float>({8,3});
        corners_norm(4, 0) = 1;
        corners_norm(5, 0) = 1;
        corners_norm(6, 0) = 1;
        corners_norm(7, 0) = 1;
        corners_norm(2, 1) = 1;
        corners_norm(3, 1) = 1;
        corners_norm(6, 1) = 1;
        corners_norm(7, 1) = 1;
        corners_norm(1, 2) = 1;
        corners_norm(2, 2) = 1;
        corners_norm(5, 2) = 1;
        corners_norm(6, 2) = 1;
    }
    corners_norm = corners_norm - origin;
    dims = dims.reshape({int (dims.size() / ndim), 1, ndim});
    corners_norm = corners_norm.reshape({1, int(std::pow(2, ndim)), ndim});
    xt::xarray<float> res = xt::ones<float>({int (dims.shape(0)),int(std::pow(2, ndim)), ndim});
    for (int i = 0; i < res.shape(0); ++i) 
    {
        xt::view(res, i) = xt::view(dims, i) * xt::view(corners_norm, 0);
    }
    return res;
}

/*  rotation 2d points based on origin point clockwise when angle positive.
*
*   Args:
*       points(float array, shape = [N, point_size, 2]) : points to be rotated.
*       angles(float array, shape = [N]) : rotation angle.
*
*   Returns :
*       float array : same shape as points
*/
xt::xarray<float> box_ops_crossing::rotation_2d(xt::xarray<float> &corners, const xt::xarray<float> &angles) 
{
    xt::xarray<float> rot_sin = xt::sin(angles);
    xt::xarray<float> rot_cos = xt::cos(angles);
    rot_cos.reshape({1, rot_cos.size()});
    rot_sin.reshape({1, rot_sin.size()});

    xt::xarray<float> a1 = xt::concatenate(xt::xtuple(rot_cos, -1 * rot_sin), 0);
    a1.reshape({1, a1.shape(0), a1.shape(1)});
    xt::xarray<float> a2 = xt::concatenate(xt::xtuple(rot_sin, rot_cos), 0);
    a2.reshape({1, a2.shape(0), a2.shape(1)});
    xt::xarray<float> rot_mat_T = xt::concatenate(xt::xtuple(a1, a2), 0);
    for (int i = 0; i < corners.shape(0); ++i) {
        xt::xarray<float> a_ = xt::view(corners, i);
        xt::xarray<float> b_ = xt::view(rot_mat_T, xt::all(), xt::all(), i);
        xt::view(corners, i) = xt::linalg::dot(a_, b_);
    }
    return corners;
}


/*  convert kitti locations, dimensions and angles to corners.
*   format: center(xy), dims(xy), angles(clockwise when positive)
*
*   Args :
*       centers(float array, shape = [N, 2]) : locations in kitti label file.
*       dims(float array, shape = [N, 2]) : dimensions in kitti label file.
*       angles(float array, shape = [N]) : rotation_y in kitti label file.
*
*   Returns :
*       [type] : [description]
*/
xt::xarray<float> box_ops_crossing::center_to_corner_box2d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                               xt::xarray<float> &angles, float origin) 
{
    xt::xarray<float> corners = box_ops_crossing::corners_nd(dims, origin);

    if (angles.size())
        corners = box_ops_crossing::rotation_2d(corners, angles);

    centers.reshape({-1, 1, 2});
    corners += centers;
    return corners;
}

/*  convert kitti locations, dimensions and angles to corners
*   Args:
*       centers(float array, shape = [N, 3]) : locations in kitti label file.
*       dims(float array, shape = [N, 3])    : dimensions in kitti label file.
*       angles(float array, shape = [N])     : rotation_y in kitti label file.
*       origin(list or array or float)       : origin point relate to smallest point.
*       use[0.5, 1.0, 0.5] in camera and [0.5, 0.5, 0] in lidar.
*       axis(int) : rotation axis. 1 for camera and 2 for lidar.
*   Returns :
*       [type] : [description]
*/
xt::xarray<float> box_ops_crossing::center_to_corner_box3d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                                  float &angles, float origin, int axis) 
{
    angles = angles * M_PI / 180.0;
    xt::xarray<float> corners = box_ops_crossing::corners_nd(dims, origin);
    if (angles != float (None))
        box_ops_crossing::rotation_3d_in_axis(corners, angles, axis);//angles需要弧度
    angles = angles * 180.0 / M_PI;
    centers = centers.reshape({int (centers.size() / 3), 1, 3});
    return corners + centers;
}

xt::xarray<float> box_ops_crossing::corner_to_standup_nd(xt::xarray<float> &boxes_corner) 
{
    xt::xarray<int> box_shape = xt::adapt(boxes_corner.shape());
    assert(box_shape.size() == 3);
    xt::xarray<float> standup_boxes1 = xt::amin(boxes_corner, 1);;
    xt::xarray<float> standup_boxes2 = xt::amax(boxes_corner, 1);
    return xt::concatenate(xt::xtuple(standup_boxes1, standup_boxes2), 1);
}

std::pair<xt::xarray<float>, xt::xarray<float>> box_ops_crossing::rotate_nms_cc(xt::xarray<float> &dets,
                                                                    xt::xarray<float> &trackers) 
{
    xt::xarray<float> t1 = xt::view(trackers, xt::all(), xt::range(0, 2));
    xt::xarray<float> t2 = xt::view(trackers, xt::all(), xt::range(2, 4));
    xt::xarray<float> t3 = xt::view(trackers, xt::all(), 4);
    xt::xarray<float> trackers_corners = box_ops_crossing::center_to_corner_box2d(t1, t2, t3);
    xt::xarray<float> trackers_standup = box_ops_crossing::corner_to_standup_nd(trackers_corners);
    xt::xarray<float> d1 = xt::view(dets, xt::all(), xt::range(0, 2));
    xt::xarray<float> d2 = xt::view(dets, xt::all(), xt::range(2, 4));
    xt::xarray<float> d3 = xt::view(dets, xt::all(), 4);
    xt::xarray<float> dets_corners = box_ops_crossing::center_to_corner_box2d(d1, d2, d3);
    xt::xarray<float> dets_standup = box_ops_crossing::corner_to_standup_nd(dets_corners);
    return box_ops_crossing::iou_jit_new(dets_standup, trackers_standup, 0.0);
}

float box_ops_crossing::cal_angle(std::vector<xt::xarray<float>> &state_list, float &thresh) 
{
    auto dis_x = state_list[state_list.size() - 1](0, 0) - state_list[0](0, 0);
    auto dis_y = state_list[state_list.size() - 1](0, 1) - state_list[0](0, 1);
    auto dis_len = std::sqrt(dis_x * dis_x + dis_y * dis_y);
    float dis_angle;
    if (dis_len > thresh){
        dis_angle = std::acos(dis_x / dis_len) * 180 / M_PI - 180;
        if (dis_y > 0)
            dis_angle = 90 - dis_angle;
        else
            dis_angle = 90 - (360 - dis_angle);
        dis_angle = std::fmod(dis_angle, 360.0);
    } else{
        dis_angle = None;
    }
    return dis_angle;
}

std::pair<xt::xarray<float>, xt::xarray<float>> box_ops_crossing::iou_jit_new(xt::xarray<float> &boxes,
                                                                     xt::xarray<float> &query_boxes, float eps) 
{
    xt::xarray<double> d_1 = xt::expand_dims(query_boxes, 0); // shape1
    xt::xarray<double> d_0 = xt::expand_dims(boxes, 1);       // shape0

    xt::xarray<double> xx1 = xt::maximum(xt::view(d_1, xt::all(), xt::all(), 0),
                                         xt::view(d_0, xt::all(), xt::all(), 0));

    xt::xarray<double> yy1 = xt::maximum(xt::view(d_1, xt::all(), xt::all(), 1),
                                         xt::view(d_0, xt::all(), xt::all(), 1));

    xt::xarray<double> xx2 = xt::minimum(xt::view(d_1, xt::all(), xt::all(), 2),
                                         xt::view(d_0, xt::all(), xt::all(), 2));

    xt::xarray<double> yy2 = xt::minimum(xt::view(d_1, xt::all(), xt::all(), 3),
                                         xt::view(d_0, xt::all(), xt::all(), 3));

    xt::xarray<double> w = xt::maximum(0.0, xx2 - xx1);
    xt::xarray<double> h = xt::maximum(0.0, yy2 - yy1);
    xt::xarray<double> inter_area = w * h;

    xt::xarray<double> union_ =
            (xt::view(d_1, xt::all(), xt::all(), 2) - xt::view(d_1, xt::all(), xt::all(), 0)) *
            (xt::view(d_1, xt::all(), xt::all(), 3) - xt::view(d_1, xt::all(), xt::all(), 1)) +
            (xt::view(d_0, xt::all(), xt::all(), 2) - xt::view(d_0, xt::all(), xt::all(), 0)) *
            (xt::view(d_0, xt::all(), xt::all(), 3) - xt::view(d_0, xt::all(), xt::all(), 1)) - inter_area;

    xt::xarray<double> union_new =
            (xt::view(d_1, xt::all(), xt::all(), 2) - xt::view(d_1, xt::all(), xt::all(), 0)) *
            (xt::view(d_1, xt::all(), xt::all(), 3) - xt::view(d_1, xt::all(), xt::all(), 1)) ;

    xt::xarray<double> iou_mat = inter_area / union_;
    xt::xarray<double> iou_mat_new = inter_area / union_new;
    return {iou_mat, iou_mat_new};
}

void box_ops_crossing::rotation_3d_in_axis(xt::xarray<float> &points, float &angles, int axis) 
{
    float rot_sin = std::sin(angles);
    float rot_cos = std::cos(angles);
    float ones = 1.0;
    float zeros = 0.0;
    xt::xarray<float> rot_mat_T;
    if (axis == 1)
    {
        rot_mat_T = xt::xarray<float>{rot_cos, zeros, -1 * rot_sin,
                                      zeros, ones, zeros,
                                      rot_sin, zeros, rot_cos};
    }
    else if (axis == 2 or axis == -1)
    {
        rot_mat_T = xt::xarray<float>{rot_cos, -1 * rot_sin, zeros,
                                      rot_sin, rot_cos, zeros,
                                      zeros, zeros, ones};
    }
    else{
        throw "axis should in range";
    }

    rot_mat_T = rot_mat_T.reshape({3, 3, 1});

    for (int i = 0; i < points.shape(0); ++i) {
        xt::xarray<float> a_ = xt::view(points, i);
        xt::xarray<float> b_ = xt::view(rot_mat_T, xt::all(), xt::all(), i);
        xt::view(points, i) = xt::linalg::dot(a_, b_);
    }
}

