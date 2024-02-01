//
// Created by root on 5/8/21.
//

#ifndef FUSION_ALG_COORDINATE_CROSSING_MAP_H
#define FUSION_ALG_COORDINATE_CROSSING_MAP_H
#include <iostream>
#include <vector>

// #include "core.hpp"
// #include "calib3d.hpp"
// #include "highgui.hpp"
#include <map>
#include <math.h>
#include <algorithm>
#include <fstream>
#include <unordered_map>
#include <xtensor/xnpy.hpp>
#include <xtensor/xsort.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "TSelfFusionAlgParam.h"
// #include <opencv2/core.hpp>
// #include <opencv2/calib3d.hpp>
// #include <opencv2/highgui.hpp>
#include "yaml-cpp/yaml.h"



void Trim_crossing(std::string& str);

template <class Type>
Type stringToNum(const std::string& str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

template <class Type>
void vStrSplit(std::string &strSur, char cConChar, xt::xarray<Type> &video, int idx)
{
    std::string::size_type pos1, pos2;
    pos1 = 0;
    pos2 = strSur.find(cConChar, 0);
    int i = 0;
    while(std::string::npos != pos2)
    {
//        std::string key_string = strSur.substr(pos1, pos2 - pos1);
        video(idx, i) = stringToNum<Type>(strSur.substr(pos1, pos2 - pos1));
        i += 1;
        pos1 = pos2 + 1;
        pos2 = strSur.find(cConChar, pos1);
    }
    video(idx, i) = stringToNum<Type>(strSur.substr(pos1, strSur.size()));
}

template <class Type>
xt::xarray<Type> csv2Xtensor(std::string file_path, int max_row, int max_col){

    xt::xarray<Type> video = xt::zeros<Type>({max_row, max_col}); // max is 200
    int count = 0;
    std::cout<<"file_path:\n"<<file_path<<std::endl;
    std::ifstream csv_video;
    csv_video.open(file_path,std::ios::in);
    if (!csv_video.is_open())
    {
        std::cout<<"Open fusion Algconfig file error"<<std::endl;
    }
    
    std::string line;
    while (getline(csv_video, line))
    {
        Trim_crossing(line);
        char s = ',';
        vStrSplit(line, s, video, count);
        count += 1;
    }
    video = xt::view(video, xt::range(0, count));
    return video;
}


struct get_cam_param_res{
    xt::xarray<float> in_parameter;
    xt::xarray<float> rotate_matrix;
    xt::xarray<float> translation_matrix;
    xt::xarray<float> dist_matrix;
};



class Coordinate_Map_crossing{
public:
    Coordinate_Map_crossing( int camera_id = 1, bool debug_sign = false, TSelfFusionAlgParam* p_pAlgParam=NULL);
    Coordinate_Map_crossing();
    get_cam_param_res _get_cam_param(int cameraID, TSelfFusionAlgParam* p_pAlgParam);

    get_cam_param_res _get_cam_param(int cameraID);
    
    void _get_tf_lidar_to_cam();


    xt::xarray<float> lidar_to_cam(xt::xarray<float> xyz_lidar);


    xt::xarray<float> project_points(xt::xarray<float> point3d);

    cv::Mat xarray_to_mat_elementwise(xt::xarray<float> &xarr);


    xt::xarray<float> lidar_box_calculation(xt::xarray<float> img2d_list);

    int _input_frame_W;
    int _input_frame_H;
    int _project_frame_W;
    int _project_frame_H;


    xt::xarray<float> _in_param;
    xt::xarray<float> _rotate_vec;
    xt::xarray<float> _translation_vec;
    xt::xarray<float> _dist_vec;
    xt::xarray<float> _tf_lidar_to_cam;

};
#endif //FUSION_ALG_COORDINATE_MAP_H
