#ifndef CAMERA_REFLECT_H
#define CAMERA_REFLECT_H

#include <xtensor.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xmath.hpp>

#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>   

#include "TCameraParam.h"


// struct camera_param_result{
//     xt::xarray<float> in_parameter;
//     xt::xarray<float> rotate_matrix;
//     xt::xarray<float> translation_matrix;
//     xt::xarray<float> dist_matrix;
// };

class Camera_reflect
{

public:
    Camera_reflect(/* args */);
    ~Camera_reflect();

    Camera_reflect(const Camera_reflect& ob);

    void load_camera_param(int camera_index, TCameraParam *_camera_param);

    // void load_filter_image(std::string _file_path, cv::Mat *_filter_image_flag);
    void load_filter_image(std::string _file_path);

    void get_xyz(xt::xarray<float> &_orgin_boxs);

    void load_npy(std::string _file_path);  // 加载的反映射存放在这里

    void cal_rorate_matrix(float angle_x, float angle_y, float angle_z);  // 计算旋转矩阵

    void set_lidar_rotate_matrix(xt::xarray<float> _rotate);
    void set_lidar_trans_vec(xt::xarray<float> _trans);



    xt::xarray<float> get_reflect_points();


private:
    xt::xarray<float> m_in_param_matrix;  // 相机内参  load_camera_param
    xt::xarray<float> m_rotate_matrix;    // 相机旋转参数  load_camera_param
    xt::xarray<float> m_translation_matrix;   // 相机平移参数  load_camera_param
    xt::xarray<float> m_dist_matrix;     // 相机畸变系数  load_camera_param

    xt::xarray<float> m_lidar_rotate_matrix;  // 辅雷达向主雷达的旋转矩阵 load_csv
    xt::xarray<float> m_lidar_trans_vec;   // 辅雷达向主雷达的平移矩阵  load_csv

    cv::Mat m_filter_image_flag;  // 过滤图像标志位  load_filter_image

    xt::xarray<float> m_reflect_points = xt::empty<float>({0, 3});  // 反映射点 

    xt::xarray<float> m_pixel_xyz; // 加载的反映射存放在这里  load_npy



};
#endif