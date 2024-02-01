/*******************************************************
 文件名：FunctionHub_ServiceArea.cpp
 作者：HuaHuan
 描述：基本函数
 版本：v1.0
 日期：2024-01-11
 *******************************************************/
#include "FunctionHub_ServiceArea.h"


xt::xarray<float> FunctionHub_ServiceArea::corners_nd(xt::xarray<float> &dims,
                                                      float             origin)
{
    int ndim = dims.shape(1);
    //    std::cout<<dims<<"\n";

    auto shape = xt::ones<int>({ndim}) * 2;
    xt::xarray<float> corners_norm;
    if (ndim == 2)
    {
        corners_norm = xt::zeros<float>({4, 2});
        corners_norm(1, 1) = 1;
        corners_norm(2, 0) = 1;
        corners_norm(3, 0) = 1;
        corners_norm(2, 1) = 1;
    }
    else
    {
        corners_norm = xt::zeros<float>({8, 3});
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
    dims = dims.reshape({int(dims.size() / ndim), 1, ndim});
    corners_norm = corners_norm.reshape({1, int(std::pow(2, ndim)), ndim});
    xt::xarray<float> res = xt::ones<float>({int(dims.shape(0)), int(std::pow(2, ndim)), ndim});
    for (int i = 0; i < res.shape(0); ++i)
    {
        //        std::cout<<xt::view(dims, i) <<"\n";
        //        std::cout<<xt::view(corners_norm, 0)<<"\n";
        xt::view(res, i) = xt::view(dims, i) * xt::view(corners_norm, 0);
    }
    //    std::cout<<res<<"\n";
    return res;
}


xt::xarray<float> FunctionHub_ServiceArea::rotation_2d(xt::xarray<float>        &corners, 
                                                       const xt::xarray<float>  &angles)
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
    //    std::cout<<rot_mat_T<<"\n";

    //    xt::dump_npy("/data/second_cpp2/npy_test_corners.npy", corners);
    //    xt::dump_npy("/data/second_cpp2/npy_test_rot_math_t.npy", rot_mat_T);
    for (int i = 0; i < corners.shape(0); ++i)
    {
        xt::xarray<float> a_ = xt::view(corners, i);
        xt::xarray<float> b_ = xt::view(rot_mat_T, xt::all(), xt::all(), i);
        xt::view(corners, i) = xt::linalg::dot(a_, b_);
    }
    //    std::cout<<corners<<"\n";
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
xt::xarray<float> FunctionHub_ServiceArea::center_to_corner_box2d(xt::xarray<float> &centers, 
                                                                  xt::xarray<float> &dims,
                                                                  xt::xarray<float> &angles, 
                                                                  float             origin)
{
    //    std::cout<<dims<<"\n";
    xt::xarray<float> corners = FunctionHub_ServiceArea::corners_nd(dims, origin);

    if (angles.size())
        corners = FunctionHub_ServiceArea::rotation_2d(corners, angles);

    centers.reshape({-1, 1, 2});
    // centers = centers.reshape({int(centers.size()) / 2, 1, 2});
    corners += centers;
    //    std::cout<<corners<<"\n";
    return corners;
}

/*  convert kitti locations, dimensions and angles to corners
 *   Args:
 *       centers(float array, shape = [N, 3]) : locations in kitti label file.
 *       dims(float array, shape = [N, 3]) : dimensions in kitti label file.
 *       angles(float array, shape = [N]) : rotation_y in kitti label file.
 *       origin(list or array or float) : origin point relate to smallest point.
 *       use[0.5, 1.0, 0.5] in camera and [0.5, 0.5, 0] in lidar.
 *       axis(int) : rotation axis. 1 for camera and 2 for lidar.
 *   Returns :
 *       [type] : [description]
 */
xt::xarray<float> FunctionHub_ServiceArea::center_to_corner_box3d(xt::xarray<float> &centers, 
                                                                  xt::xarray<float> &dims,
                                                                  float             &angles, 
                                                                  float             origin, 
                                                                  int               axis)
{
    // angles = angles*M_PI / 180.0;
    xt::xarray<float> corners = FunctionHub_ServiceArea::corners_nd(dims, origin);
    if (angles != float(None))
        FunctionHub_ServiceArea::rotation_3d_in_axis(corners, angles, axis);
    //    std::cout<<corners<<"\n";
    // angles = angles*180/M_PI;
    centers = centers.reshape({int(centers.size() / 3), 1, 3});

    //    std::cout<<corners + centers<<"\n";
    return corners + centers;
}

xt::xarray<float> FunctionHub_ServiceArea::corner_to_standup_nd(xt::xarray<float> &boxes_corner)
{
    xt::xarray<int> box_shape = xt::adapt(boxes_corner.shape());
    assert(box_shape.size() == 3);
    xt::xarray<float> standup_boxes1 = xt::amin(boxes_corner, 1);
    //    std::cout<<standup_boxes1<<"\n";
    xt::xarray<float> standup_boxes2 = xt::amax(boxes_corner, 1);
    //    std::cout<<standup_boxes2<<"\n";
    //    std::cout<<xt::concatenate(xt::xtuple(standup_boxes1, standup_boxes2), 1)<<"\n";
    return xt::concatenate(xt::xtuple(standup_boxes1, standup_boxes2), 1);
}

std::pair<xt::xarray<float>, xt::xarray<float>> FunctionHub_ServiceArea::rotate_nms_cc(xt::xarray<float> &dets,
                                                                                       xt::xarray<float> &trackers)
{

    xt::xarray<float> t1 = xt::view(trackers, xt::all(), xt::range(0, 2));  // 所有tracker的x和y
    xt::xarray<float> t2 = xt::view(trackers, xt::all(), xt::range(2, 4));  // w，l
    xt::xarray<float> t3 = xt::view(trackers, xt::all(), 4);                // theta

    xt::xarray<float> trackers_corners = FunctionHub_ServiceArea::center_to_corner_box2d(t1, t2, t3); //转换

    xt::xarray<float> trackers_standup = FunctionHub_ServiceArea::corner_to_standup_nd(trackers_corners);

    //    std::cout<<trackers_standup<<"\n";
    xt::xarray<float> d1 = xt::view(dets, xt::all(), xt::range(0, 2));      // 同上
    xt::xarray<float> d2 = xt::view(dets, xt::all(), xt::range(2, 4));
    xt::xarray<float> d3 = xt::view(dets, xt::all(), 4);

    xt::xarray<float> dets_corners = FunctionHub_ServiceArea::center_to_corner_box2d(d1, d2, d3);

    xt::xarray<float> dets_standup = FunctionHub_ServiceArea::corner_to_standup_nd(dets_corners);

    //    std::cout<<dets_standup<<"\n";
    std::pair<xt::xarray<float>, xt::xarray<float>> result = FunctionHub_ServiceArea::iou_jit_new(dets_standup, trackers_standup, 0.000001);

    return result;
}

float FunctionHub_ServiceArea::cal_angle(std::vector<xt::xarray<float>> &state_list, 
                                         float                          &thresh)
{
    //    for (auto iii : state_list){
    //        std::cout<<iii<<"\n";
    //    }
    auto dis_x = state_list[state_list.size() - 1](0, 0) - state_list[0](0, 0);
    auto dis_y = state_list[state_list.size() - 1](0, 1) - state_list[0](0, 1);
    auto dis_len = std::sqrt(dis_x * dis_x + dis_y * dis_y);
    float dis_angle;
    if (dis_len > thresh)
    {
        dis_angle = std::acos(dis_x / dis_len) * 180 / M_PI - 180;
        if (dis_y > 0)
            dis_angle = 90 - dis_angle;
        else
            dis_angle = 90 - (360 - dis_angle);
        dis_angle = std::fmod(dis_angle, 360.0);
        //        dis_angle = dis_angle >= 0 ? dis_angle : 360.0 + dis_angle;
    }
    else
    {
        dis_angle = None;
    }
    return dis_angle;
}

std::pair<float, bool> FunctionHub_ServiceArea::cal_angle_new(FUKalmanBoxTracker_ServiceArea    *tracker, 
                                                              nlohmann::json                    *m_fusion_parameter)
{
    bool state_info_sufficient_flag = true;
    float object_speed = tracker->speed;
    std::vector<xt::xarray<float>> state_list;
    float dis_thresh_min;
    float dis_thresh_max;
    if (object_speed < 0.6)
    {
        if (tracker->state.size() >= 30)
        {
            // std::vector<xt::xarray<float>>::const_iterator slice_first = tracker->state.end()-30;
            // std::vector<xt::xarray<float>>::const_iterator slice_last  = tracker->state.end();
            // state_list.assign(slice_first,slice_last);
            state_list.assign(tracker->state.end() - 30, tracker->state.end());
            dis_thresh_min = 1.0;
            dis_thresh_max = 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    else if (object_speed < 1.0)
    {
        if (tracker->state.size() >= 25)
        {
            state_list.assign(tracker->state.end() - 25, tracker->state.end());
            dis_thresh_min = object_speed * 2.5 * 0.75;
            dis_thresh_max = object_speed * 2.5 * 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    else if (object_speed < 1.5)
    {
        if (tracker->state.size() >= 20)
        {
            state_list.assign(tracker->state.end() - 20, tracker->state.end());
            dis_thresh_min = object_speed * 2.0 * 0.5;
            dis_thresh_max = object_speed * 2.0 * 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    else if (object_speed < 3)
    {
        if (tracker->state.size() >= 15)
        {
            state_list.assign(tracker->state.end() - 15, tracker->state.end());
            dis_thresh_min = object_speed * 1.5 * 0.5;
            dis_thresh_max = object_speed * 1.5 * 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    else if (object_speed < 5)
    {
        if (tracker->state.size() >= 10)
        {
            state_list.assign(tracker->state.end() - 10, tracker->state.end());
            dis_thresh_min = object_speed * 1.0 * 0.5;
            dis_thresh_max = object_speed * 1.0 * 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    else
    { // speed>5
        if (tracker->state.size() >= 5)
        {
            state_list.assign(tracker->state.end() - 5, tracker->state.end());
            dis_thresh_min = object_speed * 0.5 * 0.5;
            dis_thresh_max = object_speed * 0.5 * 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    float dis_x, dis_y, dis_len;
    float dis_x_single_frame, dis_y_single_frame, dis_len_single_frame;
    dis_x = state_list[state_list.size() - 1](0) - state_list[0](0);
    dis_y = state_list[state_list.size() - 1](1) - state_list[0](1);
    dis_len = std::sqrt(dis_x * dis_x + dis_y * dis_y);
    if (state_list.size() >= 2)
    {
        dis_x_single_frame = state_list[state_list.size() - 1](0) - state_list[state_list.size() - 2](0);
        dis_y_single_frame = state_list[state_list.size() - 1](1) - state_list[state_list.size() - 2](1);
        dis_len_single_frame = std::sqrt(dis_x_single_frame * dis_x_single_frame + dis_y_single_frame * dis_y_single_frame);
    }
    else
    {
        dis_len_single_frame = dis_len;
    }
    float dis_angle;
    bool track_angle_flag;
    if (state_list.size() >= 2)
    {
        dis_angle = std::atan2(dis_y, dis_x) * 180 / M_PI;
        dis_angle = (-1 * dis_angle - 90);
        dis_angle = std::fmod(dis_angle, 360.0);
        if (dis_angle < 0)
        {
            dis_angle = dis_angle + 360.0;
        }
    }
    else
    {
        dis_angle = tracker->last_angle;
    }
    if (state_info_sufficient_flag && (dis_len > dis_thresh_min && dis_len < dis_thresh_max) && (dis_len_single_frame / dis_len < 0.5))
    {
        track_angle_flag = true;
    }
    else
    {
        track_angle_flag = false;
    }
    return {dis_angle, track_angle_flag};
}

std::pair<xt::xarray<float>, xt::xarray<float>> FunctionHub_ServiceArea::iou_jit_new(xt::xarray<float>  &boxes,
                                                                                     xt::xarray<float>  &query_boxes, 
                                                                                     float              eps)
{

    int N = boxes.shape(0);
    int K = query_boxes.shape(0);
    xt::xarray<float> overlaps = xt::zeros<float>({N, K});
    xt::xarray<float> overlaps_new = xt::zeros<float>({N, K});
    for (int k = 0; k < K; ++k)
    {
        float box_area = (query_boxes(k, 2) - query_boxes(k, 0) + eps) * (query_boxes(k, 3) - query_boxes(k, 1) + eps);

        for (int n = 0; n < N; ++n)
        {
            float iw = std::min(boxes(n, 2), query_boxes(k, 2)) - std::max(boxes(n, 0), query_boxes(k, 0)) + eps;

            if (iw > 0)
            {
                float ih = std::min(boxes(n, 3), query_boxes(k, 3)) - std::max(boxes(n, 1), query_boxes(k, 1)) + eps;

                if (ih > 0)
                {
                    float ua = (boxes(n, 2) - boxes(n, 0) + eps) * (boxes(n, 3) - boxes(n, 1) + eps) + box_area - iw * ih;

                    overlaps(n, k) = iw * ih / ua;

                    overlaps_new(n, k) = iw * ih / box_area;
                }
            }
        }
    }

    return {overlaps, overlaps_new};
}

void FunctionHub_ServiceArea::rotation_3d_in_axis(xt::xarray<float> &points, 
                                                  float             &angles, 
                                                  int               axis)
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
    else
    {
        throw "axis should in range";
    }

    rot_mat_T = rot_mat_T.reshape({3, 3, 1});

    //    std::cout<<points<<"\n";
    //    std::cout<<rot_mat_T<<"\n";

    for (int i = 0; i < points.shape(0); ++i)
    {
        xt::xarray<float> a_ = xt::view(points, i);
        xt::xarray<float> b_ = xt::view(rot_mat_T, xt::all(), xt::all(), i);
        xt::view(points, i) = xt::linalg::dot(a_, b_);
    }
}

/*
 * compute the iou of two boxes.
 * Args:
 *       box1, box2 : [xmin, ymin, xmax, ymax] (wh = False) or [x, y, w, h](wh = True) x, y is upper left corner
 *       wh : the format of coordinate.
 * Return :
 *       iou : iou of box1 and box2.
 */
float FunctionHub_ServiceArea::IOU_calc(xt::xarray<float>   box1, 
                                        xt::xarray<float>   box2, 
                                        bool                wh)
{
    float xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2;

    if (!wh)
    {
        xmin1 = box1(0, 0);
        ymin1 = box1(0, 1);
        xmax1 = box1(0, 2);
        ymax1 = box1(0, 3);
        xmin2 = box2(0, 0);
        ymin2 = box2(0, 1);
        xmax2 = box2(0, 2);
        ymax2 = box2(0, 3);
    }
    else
    {
        xmin1 = box1(0, 0);
        ymin1 = box1(0, 1);
        xmax1 = box1(0, 0) + box1(0, 2);
        ymax1 = box1(0, 1) + box1(0, 3);
        xmin2 = box2(0, 0);
        ymin2 = box2(0, 1);
        xmax2 = box2(0, 0) + box2(0, 2);
        ymax2 = box2(0, 3) + box2(0, 3);
    }

    float xx1 = xmin1 > xmin2 ? xmin1 : xmin2;
    float yy1 = ymin1 > ymin2 ? ymin1 : ymin2;
    float xx2 = xmax1 < xmax2 ? xmax1 : xmax2;
    float yy2 = ymax1 < ymax2 ? ymax1 : ymax2;

    float area1 = (xmax1 - xmin1) * (ymax1 - ymin1);
    float area2 = (xmax2 - xmin2) * (ymax2 - ymin2);
    //    float a = std::max(float (0.0), xx2 - xx1);
    float inter_area = std::max(0.0f, xx2 - xx1) * std::max(0.0f, yy2 - yy1);
    float iou = inter_area / (area1 + area2 - inter_area + 1e-6);
    return iou;
}

// 苏州写
int FunctionHub_ServiceArea::pcClass2label(const std::string    &p_strClass, 
                                           nlohmann::json       *m_fusion_parameter)
{
    for (int i = 0; i < 11; i++)
    {
        if (p_strClass == (*m_fusion_parameter)["fusion_param"]["m_strPcClass"][i])
        {
            return i;
        }
    }

    return 0;
}

int FunctionHub_ServiceArea::videoClass2label(const std::string     &p_strClass, 
                                              nlohmann::json        *m_fusion_parameter)
{
    for (int i = 0; i < 14; i++)
    {
        if (p_strClass == (*m_fusion_parameter)["fusion_param"]["m_strVideoClass"][i])
        {
            return i;
        }
    }

    return 0;
}

// 苏州写
//   cal_distance(xt::xarray<float> &detections, xt::xarray<float> &trackers, xt::xarray<float> &head_dis_new, xt::xarray<float> &ano_dis_new, xt::xarray<float> &square_dis, int flag)
void FunctionHub_ServiceArea::cal_distance(xt::xarray<float>    &detections, 
                                           xt::xarray<float>    &trackers, 
                                           xt::xarray<float>    &head_dis_new, 
                                           xt::xarray<float>    &ano_dis_new, 
                                           xt::xarray<float>    &square_dis, 
                                           int                  flag)
/*
    detections: x,y,w,l,theta(角度),z,h,label,speed, id, score, x_min,y_min,x_max,y_max, data_source, data_channel
    trackers:   x, y, z, w，   l,  h,   theta, speed
*/
{
    if (true)
    { // 雷达距离匹配
        for (int i = 0; i < detections.shape(0); i++)
        {
            for (int j = 0; j < trackers.shape(0); j++)
            {
                float theta = trackers(j, 6);
                float diff_x = detections(i, 0) - trackers(j, 0);
                float diff_y = detections(i, 1) - trackers(j, 1);
                float dis = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
                float head_dis = abs(diff_x * cos((-1 * theta - 90) * PI / 180) + diff_y * sin((-1 * theta - 90) * PI / 180));
                float ano_dis = abs(diff_x * sin((-1 * theta - 90) * PI / 180) - diff_y * cos((-1 * theta - 90) * PI / 180));

                square_dis(i, j) = dis;
                head_dis_new(i, j) = head_dis;
                ano_dis_new(i, j) = ano_dis;
            }
        }
    }
}


void FunctionHub_ServiceArea::result_nms(xt::xarray<float> &pc_ret_nms, 
                                         std::vector<int> &delete_index, 
                                         float nms_min_threshold, 
                                         float nms_max_threshold, 
                                         bool flag)
{
    if (pc_ret_nms.shape(0) > 0)
    {
        auto iou_res2 = FunctionHub_ServiceArea::rotate_nms_cc(pc_ret_nms, pc_ret_nms);
        xt::xarray<float> iou_matrix2 = iou_res2.first;
        for (int i = 0; i < iou_matrix2.shape(0); ++i)
        {
            for (int j = 0; j < iou_matrix2.shape(0); ++j)
            {
                if (i>j)
                {
                    if (iou_matrix2(i,j) >= nms_min_threshold && iou_matrix2(i,j) <= nms_max_threshold)
                    {
                        //                    std::cout<<"to_del3  "<<i<<" "<<j<<" "<<iou_matrix2(i,j)<<std::endl;
                        if (pc_ret_nms(i,10) > pc_ret_nms(j,10))
                            delete_index.push_back(j);
                        else
                            delete_index.push_back(i);
                    }
                }

            }
        }

        std::set<int> index(delete_index.begin(), delete_index.end());
        delete_index.assign(index.begin(), index.end());

        if (flag){
            if (delete_index.size() > 0){
                xt::view(pc_ret_nms, xt::drop(delete_index));
            }
        }
        // for (int i = 0; i < pc_ret_nms.shape(0); ++i)
        // {
        //     int flag_new = 0;
        //     for (int j = 0; j < to_del3.size(); ++j)
        //     {
        //         if (i == to_del3[j])
        //             flag_new =1;
        //     }
        //     if (flag_new == 1)  // need delete
        //     {
        //         xt::xarray<float> temp1 = xt::view(pc_ret_nms, i, xt::all());
        //         _pc_ret = xt::concatenate(xt::xtuple(_pc_ret, temp1.reshape({1,temp1.size()})), 0);
        //     }
        // }
    }
    // to_del3.clear();
}

std::pair<int, int> FunctionHub_ServiceArea::get_pixel_location_xc(float    x, 
                                                                   float    y, 
                                                                   int      width, 
                                                                   int      height, 
                                                                   float    x_limit[], 
                                                                   float    y_limit[])
{
    int dist2pixel = 10;
    float temp_x = floor((x - x_limit[0]) * dist2pixel);
    float temp_y = -1 * floor((y - y_limit[0]) * dist2pixel) + height;

    if (temp_x >= width || temp_y >= height || temp_x < 0 || temp_y < 0)
    {
        temp_x = -1000;
        temp_y = -1000;
    }

    return {int(temp_x), int(temp_y)};
}

std::pair<int, float> FunctionHub_ServiceArea::get_lane_info(float      x, 
                                                             float      y, 
                                                             int        width, 
                                                             int        height, 
                                                             float      x_limit[], 
                                                             float      y_limit[], 
                                                             std::unordered_map<int, std::pair<int, float>> *chedao)
{
    std::pair<int, int> temp_xy = FunctionHub_ServiceArea::get_pixel_location_xc(x, y, width, height, x_limit, y_limit);
    int temp_x = temp_xy.first;
    int temp_y = temp_xy.second;

    int haxi_indice;
    if (width > height)
    {
        haxi_indice = temp_x + width * temp_y;
    }
    else
    {
        haxi_indice = temp_y + height * temp_x;
    }
    int lane_name;
    float lane_angle;

    if (chedao->find(haxi_indice) != chedao->end())
    {
        lane_name = chedao->at(haxi_indice).first;
        lane_angle = chedao->at(haxi_indice).second;
    }
    else
    {
        lane_name = -1000;
        lane_angle = -1000.0;
    }

    return {lane_name, lane_angle};
}

void FunctionHub_ServiceArea::smooth_angle(FUKalmanBoxTracker_ServiceArea   *tracker, 
                                           std::vector<int>                 *motor_vehicle_labels)
{
    tracker->angle_flag = 0;
    float ang_thres;
    if (std::find(motor_vehicle_labels->begin(), motor_vehicle_labels->end(), tracker->_type_fusion.fusion_label) != motor_vehicle_labels->end())
    {
        if (tracker->speed < 0.5)
        {
            ang_thres = 0.01;
        }
        else if (tracker->speed >= 0.5 && tracker->speed < 1.2)
        {
            ang_thres = 5;
            if (tracker->direction == -1)
            {
                ang_thres = 0.1;
            }
        }
        else if (tracker->speed >= 1.2 && tracker->speed < 3)
        {
            ang_thres = 4.5;
            if (tracker->direction == -1)
            {
                ang_thres = 0.3;
            }
        }
        else if (tracker->speed >= 3 && tracker->speed < 5)
        {
            ang_thres = 4.5;
            if (tracker->direction == -1)
            {
                ang_thres = 0.3;
            }
        }
        else if (tracker->speed >= 5 && tracker->speed < 7)
        {
            ang_thres = 4;
        }
        else
        {
            ang_thres = 3;
        }
    }
    else
    {
        if (tracker->speed < 0.2)
        {
            ang_thres = 1;
        }
        else
        {
            ang_thres = 5;
        }
    }

    float last_angle = tracker->last_angle;
    float ang_diff = tracker->track_angle - tracker->last_angle;

    if (ang_diff > 180)
    {
        ang_diff = ang_diff - 360;
    }
    else if (ang_diff < -180)
    {
        ang_diff = ang_diff + 360;
    }

    if (abs(ang_diff) >= ang_thres)
    {
        if (ang_diff > 0)
        {
            tracker->track_angle = last_angle + (ang_thres + ang_thres * log10(ang_diff / ang_thres));
            tracker->angle_flag = 1;
        }
        if (ang_diff < 0)
        {
            tracker->track_angle = last_angle - (ang_thres + ang_thres * log10(abs(ang_diff) / ang_thres));
            tracker->angle_flag = 1;
        }
    }

    if (tracker->track_angle > 360)
    {
        tracker->track_angle = std::fmod(tracker->track_angle, 360.0);
    }

    if (tracker->track_angle < 0)
    {
        tracker->track_angle = tracker->track_angle + 360;
    }
}


std::map<int, float> FunctionHub_ServiceArea::neighbor_search(xt::xarray<float> &trackers, 
                                                              int               idx, 
                                                              float             search_radius, 
                                                              nlohmann::json    *m_fusion_parameter){
    std::map<int, float> occupy_quadrant;
    occupy_quadrant.insert(std::pair<int, float>(0, 1000000.0));
    occupy_quadrant.insert(std::pair<int, float>(1, 1000000.0));
    occupy_quadrant.insert(std::pair<int, float>(2, 1000000.0));
    occupy_quadrant.insert(std::pair<int, float>(3, 1000000.0));
    occupy_quadrant.insert(std::pair<int, float>(4, 1000000.0));
    occupy_quadrant.insert(std::pair<int, float>(5, 1000000.0));
    occupy_quadrant.insert(std::pair<int, float>(6, 1000000.0));
    occupy_quadrant.insert(std::pair<int, float>(7, 1000000.0));
    occupy_quadrant.insert(std::pair<int, float>(8, 1000000.0));
    if (search_radius == 0.0) return occupy_quadrant;
    
    for (int i = 0; i < trackers.shape(0); i++){
        if (i == idx) continue;
        if (int(trackers(i, 9)) >= int((*m_fusion_parameter)["fusion_param"]["sort_min_hits"])){
            float diff_x = trackers(i, 0) - trackers(idx, 0);
            float diff_y = trackers(i, 1) - trackers(idx, 1);
            float dis = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
            if (dis < search_radius){
                int quadrant = 0;
                if (diff_x > 0 && diff_y > 0 && abs(diff_x) < abs(diff_y))
                    quadrant = 1;
                else if (diff_x > 0 && diff_y > 0 && abs(diff_x) > abs(diff_y))
                    quadrant = 2;
                else if (diff_x > 0 && diff_y < 0 && abs(diff_x) > abs(diff_y))
                    quadrant = 3;
                else if (diff_x > 0 && diff_y < 0 && abs(diff_x) < abs(diff_y))
                    quadrant = 4;
                else if (diff_x < 0 && diff_y < 0 && abs(diff_x) < abs(diff_y))
                    quadrant = 5;
                else if (diff_x < 0 && diff_y < 0 && abs(diff_x) > abs(diff_y))
                    quadrant = 6;
                else if (diff_x < 0 && diff_y > 0 && abs(diff_x) > abs(diff_y))
                    quadrant = 7;
                else if (diff_x < 0 && diff_y > 0 && abs(diff_x) < abs(diff_y))
                    quadrant = 8;
                
                if (quadrant != 0){
                    dis *= 0.3;
                    occupy_quadrant[quadrant] = std::min(dis, occupy_quadrant[quadrant]);
                }
            }
        }
    }
    return occupy_quadrant;
}



