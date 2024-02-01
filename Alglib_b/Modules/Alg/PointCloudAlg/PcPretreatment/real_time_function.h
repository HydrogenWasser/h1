//
// Created by root on 5/7/21.
//

#ifndef PRE_PROCESS_REAL_TIME_FUNCTION_H
#define PRE_PROCESS_REAL_TIME_FUNCTION_H
#include "TSelfPcAlgParam.h"

#include <iostream>
#include <math.h>
#include <xtensor/xview.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xsort.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "yaml-cpp/yaml.h"
#include "box_ops2.h"

/*
 * 功能：点云旋转评议
 * 参数：
 *      angle:旋转角 弧度
 *      trans：评议量 米
 */
template<typename DType>
void rotate(xt::xarray<DType> &points,std::vector<DType> &angle, std::vector<DType> &trans){
    xt::xarray<DType> R_x = {{1.0, 0.0, 0.0},
                             {0.0, std::cos(angle[0]), -1 * std::sin(angle[0])},
                             {0.0, std::sin(angle[0]), std::cos(angle[0])}};
    xt::xarray<DType> R_y = {{std::cos(angle[1]), 0.0, std::sin(angle[1])},
                             {0.0, 1.0, 0.0},
                             {-1 * std::sin(angle[1]), 0.0, std::cos(angle[1])}};
    xt::xarray<DType> R_z = {{std::cos(angle[2]), -1 * std::sin(angle[2]), 0.0},
                             {std::sin(angle[2]), std::cos(angle[2]), 0.0},
                             {0.0, 0.0, 1.0}};
    xt::xarray<DType> rotate = xt::linalg::dot(R_z, R_y);
    rotate = xt::linalg::dot(rotate, R_x);
    xt::xarray<DType> point_ = xt::transpose(xt::view(points, xt::all(), xt::range(0, 3)));
    xt::xarray<DType> point_temp = xt::linalg::dot(rotate, point_);
    xt::view(points, xt::all(), xt::range(0, 3)) = xt::transpose(point_temp);
    xt::view(points, xt::all(), 0) += trans[0];
    xt::view(points, xt::all(), 1) += trans[1];
    xt::view(points, xt::all(), 2) += trans[2];
};

std::pair<xt::xarray<int>, xt::xarray<float>> points_count_rbbox2(xt::xarray<float>& points, xt::xarray<float> box3d);
class pc_process
{
public:
    pc_process();
    // pc_process(const std::string & bmp_path);
    pc_process(TSelfPcAlgParam * AlgParams);
    bool filter_points(xt::xarray<float> & points, const bool &crop=true, const bool & filter_ground=false);
    void rotate(xt::xarray<float> & points);
    bool empty(){return background_img.empty();}
    void set_background(const std::string & bmp_path);
    void set_rotate_params(const std::vector<float> & angles, const std::vector<float> & translations);
    void correct_angle(xt::xarray<float> & boxes_lidar);
    bool m_save_bmp(xt::xarray<float> & pc_data, const std::string & bmp_path, const int & im_size = 3000);
    std::vector<int> crop_bboxes2(xt::xarray<float> &bboxes);
    // /***************************************************************
    // * @file       real_time_function.h
    // * @brief      点云平移旋转
    // * @input      points: 输入点云, 维度为2; 单位为米。
    // * @author     陈向阳
    // * @date       2022.10.12
    // **************************************************************/
    // template<typename DType>
    // void rotate(xt::xarray<DType> &points, const bool & is_rotate=true){
    //     if (!points.shape(0) or !is_rotate) 
    //     {
    //         return;
    //     }
    //     xt::xarray<DType> R_x = {{1.0, 0.0, 0.0},
    //                             {0.0, std::cos(rotate_angle[0]), -1 * std::sin(rotate_angle[0])},
    //                             {0.0, std::sin(rotate_angle[0]), std::cos(rotate_angle[0])}};
    //     xt::xarray<DType> R_y = {{std::cos(rotate_angle[1]), 0.0, std::sin(rotate_angle[1])},
    //                             {0.0, 1.0, 0.0},
    //                             {-1 * std::sin(rotate_angle[1]), 0.0, std::cos(rotate_angle[1])}};
    //     xt::xarray<DType> R_z = {{std::cos(rotate_angle[2]), -1 * std::sin(rotate_angle[2]), 0.0},
    //                             {std::sin(rotate_angle[2]), std::cos(rotate_angle[2]), 0.0},
    //                             {0.0, 0.0, 1.0}};
    //     xt::xarray<DType> rotate = xt::linalg::dot(R_z, R_y);
    //     rotate = xt::linalg::dot(rotate, R_x);
    //     xt::xarray<DType> point_ = xt::transpose(xt::view(points, xt::all(), xt::range(0, 3)));
    //     xt::xarray<DType> point_temp = xt::linalg::dot(rotate, point_);
    //     xt::view(points, xt::all(), xt::range(0, 3)) = xt::transpose(point_temp);
    //     xt::view(points, xt::all(), 0) += rotate_trans[0];
    //     xt::view(points, xt::all(), 1) += rotate_trans[1];
    //     xt::view(points, xt::all(), 2) += rotate_trans[2];
    // }

    // rotate
    std::vector<float> rotate_angle;
    std::vector<float> rotate_trans;

private:
    void m_read_bmp(const std::string & bmp_path);
    cv::Mat background_img;

    xt::xarray<float> m_crop_pcdata(xt::xarray<float> &frame_data);
    void m_fused_back(xt::xarray<float> &backdata);
    xt::xarray<float> m_fused_crop_back(xt::xarray<float> &PC_data);
    void fix_angle(xt::xarray<float> &boxes_lidar, xt::xarray<float> &npAngle);
    xt::xarray<float> cal_trans(const float & x, const float & y, const float & z);


    xt::xarray<float> img_small;
    xt::xarray<float> img_big;
    
    // crop
    int grid_size_small;
    int grid_size_big;
    // std::string BackImgPath;

    // filter
    float thresh_small;
    float thresh_big;
    float h_thresh_x;
    float h_thresh_y;
    float dis_thresh;

    // fix
    std::vector<float> vecAngle;
    bool RotationTranslation;
};

                                                   
#endif //PRE_PROCESS_REAL_TIME_FUNCTION_H
