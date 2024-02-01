//
// Created by root on 5/8/21.
//

#ifndef FUSION_ALG_COORDINATE_TUNNEL_MAP_H
#define FUSION_ALG_COORDINATE_TUNNEL_MAP_H
#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
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


enum class Fill_R_THETA_MODE{
    POINTS=1,
    GROUND=2,
    NONE= 91111
};

enum class FILL_Z_MODE{
    MAX=1,
    AVERAGE=2,
    MIN=3,
    NONE=91111
};

// ============== long_lat ================
#define __A 6378137
#define __B 6356725
#define __IterativeValue 10
#define R_a 6378137.00
#define R_b 6356752.3142
double FG_degree2rad(double degree);
double FG_rad2degree(double rad);

void LonLat2Mercator(double &B, double &L, double &x, double &y);
void Mercator2LonLat(double B, double L, double &X, double &Y, double &Object_Long, double &Object_Lat);
void lonlat_to_xyz_batch(xt::xarray<double> &box, double &lon0, double &lat0, double &angle_north);
xt::xarray<double> XYZ_To_BLH_batch(double &original_long, double &original_lat, xt::xarray<double> &box, double &rotaionangle);
xt::xarray<double> cal_trans(double x, double y, double z);


void Trim_tunnel(std::string& str);

template <class Type>
Type stringToNum_tunnel(const std::string& str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

template <class Type>
void vStrSplit_tunnel(std::string &strSur, char cConChar, xt::xarray<Type> &video, int idx)
{
    std::string::size_type pos1, pos2;
    pos1 = 0;
    pos2 = strSur.find(cConChar, 0);
    int i = 0;
    while(std::string::npos != pos2)
    {
//        std::string key_string = strSur.substr(pos1, pos2 - pos1);
        video(idx, i) = stringToNum_tunnel<Type>(strSur.substr(pos1, pos2 - pos1));
        i += 1;
        pos1 = pos2 + 1;
        pos2 = strSur.find(cConChar, pos1);
    }
    video(idx, i) = stringToNum_tunnel<Type>(strSur.substr(pos1, strSur.size()));
}

template <class Type>
xt::xarray<Type> csv2Xtensor_tunnel(std::string file_path, int max_row, int max_col){
    xt::xarray<Type> video = xt::zeros<Type>({max_row, max_col}); // max is 200
    int count = 0;
    int cam_id=2;

    std::ifstream csv_video;
    csv_video.open(file_path);

    std::string line;
    while (getline(csv_video, line))
    {
        Trim_tunnel(line);
        char s = ',';
        vStrSplit_tunnel(line, s, video, count);
        count += 1;
    }
    video = xt::view(video, xt::range(0, count));
    return video;
}

struct stCameraParam
{
    xt::xarray<float> m_xarrInParam;
    xt::xarray<float> m_xarrRotateMatrix;
    xt::xarray<float> m_xarrTranslationMatrix;
    xt::xarray<float> m_xarrDistMatrix;
};

class Coordinate_Map_tunnel{
public:
    Coordinate_Map_tunnel(std::string data_path, const stCameraParam& p_refCameraParam, int camera_id = 1, float ground_height = -4.7, int station_id = 0);
    Coordinate_Map_tunnel(std::string data_path, const stCameraParam& p_refCameraParam, int camera_id, bool debug_sign);
    Coordinate_Map_tunnel();
    void _get_cam_param(int cameraID);
    void _get_tf_lidar_to_cam();
    void _get_fill_lines(int cameraID);
    void _get_ground_z(int cameraID);
    void init(std::string data_path);

    void _get_format_data(std::string data_path, bool remain_torward_sign=true);
    void remain_toward_data_by_cam_coordinate(xt::xarray<float> &this_xyz);
    xt::xarray<float> lidar_to_cam(xt::xarray<float> xyz_lidar);
    void remove_err_point(xt::xarray<float> &xyz);

    xt::xarray<float> project_points(xt::xarray<float> point3d);

    cv::Mat xarray_to_mat_elementwise(xt::xarray<float> &xarr);

    std::pair<cv::Mat, cv::Mat> cartToPolar(xt::xarray<float> &xpt, xt::xarray<float> &ypt);
    std::pair<xt::xarray<float>, std::pair<xt::xarray<float>, xt::xarray<float>>> _fill_by_points_return_r_theta_z(std::map<std::string, xt::xarray<float>> &input_r, int no, int fill_no);


    void _remain_img_points_data(bool fill_remain_sign=true);
    void _remain_img_points_data_origin();
    void _remain_img_points_data_fill();
    void _fill_data();

    void _fit_circle();
    void _fit_circle_origin();
    void _fit_circle_fill();
    xt::xarray<float> _hyper_fit_circle(xt::xarray<float> & coords, int IterMax = 99, bool verbose = false);

    std::vector<float> map_2d_to_3d_by_points(xt::xarray<float> &box);

    std::pair<xt::xarray<float>, float> map_2dto3d_transform(xt::xarray<float> &point, int &cam_id, xt::xarray<float> &para_plane, xt::xarray<float> &size);
    xt::xarray<float> cal_chafen_diff2(xt::xarray<float>&cam_boxes_list, int &cam_id);

    std::pair<std::string, std::string> _get_clip_circle_idx(float x, float y, float l, float r);
    std::pair<int, int> _get_clip_circle_origin_idx(float x, float y, float l, float r);
    bool _is_point_in_circle(float x, float y, float x_0, float y_0, float r);
    std::pair<std::string, std::string> _get_clip_circle_detail_idx(float x, float y, int small_no);
    std::pair<int, int> get_idx_binary(float x, float y, float l, float r, std::unordered_map<int, std::string> idx_dic);

    std::pair<float, float> _intersection_of_fit_circle_and_line(float mx, xt::xarray<float> & param);
    xt::xarray<float> _find_optimal_match_point(float x, float y, std::string matched_idx);

    void config_chedao_bianxian(std::string config_path);
    xt::xarray<double> eulerAnglesToRotationMatrix(xt::xarray<double> &theta);

    int _input_frame_W;
    int _input_frame_H;
    int _project_frame_W;
    int _project_frame_H;
    int _start_line;
    int _end_line;

    std::map<std::string, xt::xarray<float>> _xyz;
    std::map<std::string, xt::xarray<float>> _img_xy;
    std::map<std::string, xt::xarray<float>> _r;
    std::map<std::string, xt::xarray<float>> _theta;
    std::map<std::string, float> _r_choice;
    std::map<std::string, xt::xarray<float>> _img_abr;

    bool _debug_for_match;


    xt::xarray<float> _in_param;
    xt::xarray<float> _rotate_vec;
    xt::xarray<float> _translation_vec;
    xt::xarray<float> _dist_vec;

    int _fill_r_theta_MODE;
    int _fill_z_MODE;
    bool _fit_fill_sign;
    bool _find_fill_sign;

    bool _debug_sign;
    int _plot_start_line;
    int _plot_end_line;
    xt::xarray<float> _tf_lidar_to_cam;
    xt::xarray<float> _fill_lines;
    float _ground_z;
//    if self.debug_sign:
//            self._init_plot()

    // new param: 2022.02.10
    std::vector<float > _lidarLongitude = {113.3295452925999, 113.3288191047671, 113.3278804022179, 113.3268057132405,
                                            113.3256247957487,
                                            113.3242191281648, 113.3228005937, 113.3213102846250, 113.3198098654866,
                                            113.3183278528113,
                                            113.3168669564, 113.3154122596798};
    std::vector<float> _lidarLatitude = {22.9768546291, 22.9759441589820, 22.9752029989194, 22.9746839336564, 22.9743325240624,
                                         22.9740019427648, 22.9736703894, 22.9733216358291,
                                         22.9729631680335, 22.9725601601250, 22.9725601601250, 22.9721048663, 22.9716423540572};
    std::vector<float> _angleNorthT = {300.82, 314.314, 321.377, 335.656, 342.9, 345.271, 343.25, 343.5, 342.25, 341.008, 338.408, 340.2};
    int _chedao_number = 3;
    int _station_id;
    int _ground_height;
    std::map<int, xt::xarray<float>> _middle_line;
    std::map<int, xt::xarray<float>> _middle_line_2d;
    std::map<int, xt::xarray<float>> _bianxian;
    std::map<int, xt::xarray<float>> _bianxian_2d;
    std::map<int, xt::xarray<float>> _middle_xyz_to_xy;

};
#endif //FUSION_ALG_COORDINATE_MAP_H
