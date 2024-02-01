//
// Created by root on 2/18/22.
//

#ifndef CPP_UTILS_VIDEO_UTILS_H
#define CPP_UTILS_VIDEO_UTILS_H

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <xtensor/xarray.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xsort.hpp>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <xtensor/xcsv.hpp>
#include <xtensor/xnorm.hpp>
//opencv3 is different from opencv4
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "coordinate_map_tunnel.h"

#define MID_LINE_IMG_X 0
#define MID_LINE_IMG_Y 1
#define PC_X 2
#define PC_Y 3
#define LON 4
#define LAT 5
#define MAP_IMG_X 6
#define MAP_IMG_Y 7

enum class Object_Index {
    person = 0,
    tool_vehicle = 1,
    bicycle = 2,
    motorbike = 3,
    pedal_tricycle = 4,
    car = 5,
    passenger_car = 6,
    truck = 7,
    police_car = 8,
    ambulance = 9,
    bus = 10,
    dump_truck = 11,
    tanker = 12,
    roadblock = 13,
    fire_car = 14
};





// template<class Type>
// Type stringToNum(const std::string &str) {
//     std::istringstream iss(str);
//     Type num;
//     iss >> num;
//     return num;
// }

class WLH_Estimator {
public:
    WLH_Estimator();

    ~WLH_Estimator();

    void get_wlh(xt::xarray<int> &classes, xt::xarray<float> &wlh);

private:
    std::map<int, std::vector<float>> class_to_size;
};

class Box_Bottom_Lane_Judger {
public:
    Box_Bottom_Lane_Judger(std::string folder_path, int no, int border_sx, int border_sy, int border_ex, int border_ey);

    ~Box_Bottom_Lane_Judger();

    void __generate_configs();

    xt::xarray<int> get_lane_no(xt::xarray<float> &boxes, int from_to);

    void get_index(xt::xarray<float> &nums, xt::xarray<float> &unique_nums,
                                       std::vector<int> &index);
private:
    int border_sx;
    int border_sy;
    int border_ex;
    int border_ey;
    std::string lane_config_path;
    xt::xarray<int> lane_matrix;
};


class Match_Pairs_Coordinate_Estimator {
public:
    Match_Pairs_Coordinate_Estimator(std::string config_path, int cam_no, int border_y,
                                     int station_id, float ground_z, double lon, double lat, double angle);

    ~Match_Pairs_Coordinate_Estimator();
    xt::xarray<float> get_xyz(xt::xarray<float> &boxes, xt::xarray<int> &classes, xt::xarray<int> &lane_no);

private:
    void init_object_dict();
    std::map<std::string, std::vector<int>> object_dict;
    std::map<int, xt::xarray<float>> match_pairs;
    cv::Mat border_filter_img;

};

void Load_Param(xt::xarray<float> &in_param, xt::xarray<float> &rotate_mat, xt::xarray<float> &translation_mat,
                xt::xarray<float> &dist_mat, std::string path);

class Lidar_Camera_Transform {
public:
    Lidar_Camera_Transform();

    ~Lidar_Camera_Transform();

    Lidar_Camera_Transform(xt::xarray<float> &in_param, xt::xarray<float> &rotate_mat,
                           xt::xarray<float> &translation_mat,
                           xt::xarray<float> &dist_mat);

    void _get_tr_lidar_to_cam();

    void lidar_to_cam(xt::xarray<float> &xyz_lidar, xt::xarray<float> &xy_video);

    xt::xarray<float> project_points(xt::xarray<float> &point3d);

private:
    xt::xarray<float> in_param;
    xt::xarray<float> rotate_mat;
    xt::xarray<float> translation_mat;
    xt::xarray<float> dist_mat;

    xt::xarray<float> tr_lidar_to_cam;
};

class Generate_Match_Pairs {
public:
    Generate_Match_Pairs(int cam_no, std::string config_path, int station_id, float ground_z, double lon, double lat,
                         double angle);

private:
    int cam_no;
    std::string config_path;
    int pc_id;
    float ground_z;
    double lon;
    double lat;
    double angle;
    Lidar_Camera_Transform lidar_trans;
    int project_frame_W;
    int project_frame_H;
    std::map<int, xt::xarray<float>> mid_line_longlat;
    cv::Mat line_img;
    std::map<int, std::vector<int>> line_param;
    std::map<int, xt::xarray<float>> mid_line_img_xy;
};

class Camera_Map{
public:
    Camera_Map();
    ~Camera_Map();
    Camera_Map(xt::xarray<float> &in_param, xt::xarray<float> &rotate_vec, xt::xarray<float> &translation_vec, xt::xarray<float> &dist_vec);
    void _get_tf_lidar_to_cam();
    xt::xarray<float> lidar_to_cam(xt::xarray<float> xyz_lidar);
    xt::xarray<float> project_points(xt::xarray<float> &point3d);
    xt::xarray<bool> map(xt::xarray<float> &pc_det, std::vector<int> img_size = {1920, 1080});
//private:
    xt::xarray<float> _in_param;
    xt::xarray<float> _rotate_vec;
    xt::xarray<float> _translation_vec;
    xt::xarray<float> _dist_vec;
    xt::xarray<float> _tf_lidar_to_cam;
};

//
xt::xarray<float> StackDet(const xt::xarray<float>& p_refXarrBox,
                            const xt::xarray<float>& p_refXarrConfidence,
                            const xt::xarray<float>& p_refXarrClass, 
                            const xt::xarray<float>& p_refXarrId);

#endif //CPP_UTILS_VIDEO_UTILS_H
