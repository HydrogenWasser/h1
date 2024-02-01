//
// Created by root on 5/8/21.
//

#include "coordinate_map_tunnel.h"
#include <iostream>

void LonLat2Mercator(double &B, double &L, double &x, double &y) {
    double f, e, e_, NB0, E, dtemp = 0;
    E = double(std::exp(1));
    double __B0 = B;
    double __L0 = 0;
    if (L < -M_PI or L > M_PI or B < -M_PI / 2 or B > M_PI / 2)
        return;
//    if (__A <= 0 or __B <= 0):    // todo: the code of python is wrong!
//        return;
    f = double(__A - __B) / __A;
    dtemp = 1 - std::pow(double(__B) / double(__A), 2);
    if (dtemp < 0)
        return;

    e = std::sqrt(dtemp);
    dtemp = std::pow(double(__A) / double(__B), 2) - 1;
    if (dtemp < 0)
        return;
    e_ = std::sqrt(dtemp);

    NB0 = double(double(__A) / double(__B) * double(__A)) / std::sqrt(1 + e_ * e_ * std::cos(__B0) * std::cos(__B0));
    double K = NB0 * std::cos(__B0);
    x = K * (L - __L0);
    y = K * std::log(std::tan(M_PI / 4 + B / 2) * std::pow((1 - e * std::sin(B)) / (1 + e * std::sin(B)), e / 2));


//    printf("done");
}

void Mercator2LonLat(double B, double L, double &X, double &Y, double &Object_Long, double &Object_Lat) {
    double f, e, e_, NB0, E, dtemp, __L0 = 0;
    E = double(std::exp(1));
    double __B0 = B;
//    if (__A <= 0 or __B <= 0)   // todo: the code of python is wrong!
//        return;
    f = double(__A - __B) / __A;
    dtemp = 1 - std::pow(double(__B) / double(__A), 2);
    if (dtemp < 0)
        return;
    e = std::sqrt(dtemp);
    dtemp = std::pow(double(__A) / double(__B), 2) - 1;
    if (dtemp < 0)
        return;
    e_ = std::sqrt(dtemp);
    NB0 = double(double(__A) / double(__B) * double(__A)) / std::sqrt(1 + e_ * e_ * std::cos(__B0) * std::cos(__B0));
    double K = NB0 * std::cos(__B0);
    Object_Long = FG_rad2degree(Y / K + __L0);
    B = 0.0;
    for (int i = 0; i < __IterativeValue; ++i) {
        B = M_PI / 2 - 2 * std::atan(
                std::pow(E, (-X / K)) * std::pow(E, (e / 2) * std::log((1 - e * std::sin(B)) / (1 + e * std::sin(B)))));
    }
    Object_Lat = FG_rad2degree(B);

}

xt::xarray<double> cal_trans(double x, double y, double z) {
    xt::xarray<double> R_x = {{1.0, 0.0,         0.0},
                              {0.0, std::cos(x), -1 * std::sin(x)},
                              {0.0, std::sin(x), std::cos(x)}};
    xt::xarray<double> R_y = {{std::cos(y),      0.0, std::sin(y)},
                              {0.0,              1.0, 0.0},
                              {-1 * std::sin(y), 0.0, std::cos(y)}};
    xt::xarray<double> R_z = {{std::cos(z), -1 * std::sin(z), 0.0},
                              {std::sin(z), std::cos(z),      0.0},
                              {0.0,         0.0,              1.0}};
    xt::xarray<double> rotate = xt::linalg::dot(R_z, R_y);
    rotate = xt::linalg::dot(rotate, R_x);
    return rotate;
}

void lonlat_to_xyz_batch(xt::xarray<double> &box, double &lon0, double &lat0, double &angle_north) {
    for (int i = 0; i < box.shape(0); ++i) {
//        if(i == 2541){
//            printf("error");
//        }
        double lon = box(i, 0);
        double lat = box(i, 1);
//        std::cout<<box(i, 0)<<"\n";
//        int lon = int(std::floor(box(i, 0) * 1000 + 0.5));
//        int lat = int(std::floor(box(i, 1) * 1000 + 0.5));
//        int lon0_temp = int(std::floor(lon0 * 1000 + 0.5));
//        int lat0_temp = int(std::floor(lat0 * 1000 + 0.5));
        // todo:: diff connot be remove!!!
//        std::cout<<std::floor((lon - lon0) * M_PI / 180 * 1e7 + 0.5) / 1e7<<"\n";
        double x =
                std::floor((lon - lon0) * M_PI / 180 * 1e7 + 0.5) / 1e7 * double(R_a) * std::cos(lat0 * M_PI / 180.0);
        double y = std::floor((lat - lat0) * M_PI / 180 * 1e7 + 0.5) / 1e7 * R_b;
        xt::xarray<double> xyz = {{x, y, -4.2}};
        xt::xarray<double> R_bmp = cal_trans(0, 0, angle_north * M_PI / 180);
        xt::xarray<double> xyz_temp = xt::view(xyz, xt::all(), xt::range(0, 3));
        xyz_temp = xt::transpose(xyz_temp);
        xt::xarray<double> A = xt::linalg::dot(R_bmp, xyz_temp);
        xt::view(xyz, xt::all(), xt::range(0, 3)) = xt::transpose(A);
        box(i, 0) = xyz(0, 0);
        box(i, 1) = xyz(0, 1);
//        std::cout<<box(i, 0) << " " << box(i, 1)<<"\n";
    }
}


double FG_degree2rad(double degree) {
    return degree * M_PI / 180.0;
}

double FG_rad2degree(double rad) {
    return rad / M_PI * 180.0;
}


void Trim_tunnel(std::string &str) {
    str.erase(0, str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
}

Coordinate_Map_tunnel::Coordinate_Map_tunnel(std::string data_path, const stCameraParam& p_refCameraParam, int camera_id, float ground_height, int station_id) :
        _input_frame_W(640),
        _input_frame_H(640),
        _project_frame_W(1920),
        _project_frame_H(1080),
        _ground_height(ground_height),
        _station_id(station_id) 
{

    _in_param = p_refCameraParam.m_xarrInParam;
    _rotate_vec = p_refCameraParam.m_xarrRotateMatrix;
    _translation_vec = p_refCameraParam.m_xarrTranslationMatrix;
    _dist_vec = p_refCameraParam.m_xarrDistMatrix;

    _get_tf_lidar_to_cam();
    config_chedao_bianxian(data_path);

}

Coordinate_Map_tunnel::Coordinate_Map_tunnel(std::string data_path, const stCameraParam& p_refCameraParam, int camera_id, bool debug_sign) :
        _input_frame_W(640),
        _input_frame_H(640),
        _project_frame_W(1920),
        _project_frame_H(1080),
        _start_line(0),
        _end_line(31),
        _xyz({}),
        _img_xy({}),
        _r({}),
        _theta({}),
        _r_choice({}),
        _img_abr({}),
        _debug_for_match(false),
        _fit_fill_sign(true),
        _find_fill_sign(true),
        _debug_sign(debug_sign),
        _plot_start_line(0),
        _plot_end_line(17) 
{
    _in_param = p_refCameraParam.m_xarrInParam;
    _rotate_vec = p_refCameraParam.m_xarrRotateMatrix;
    _translation_vec = p_refCameraParam.m_xarrTranslationMatrix;
    _dist_vec = p_refCameraParam.m_xarrDistMatrix;

//    _in_param = cam_param_struct.in_parameter;
//    _rotate_vec = cam_param_struct.rotate_matrix;
//    _translation_vec = cam_param_struct.translation_matrix;
//    _dist_vec = cam_param_struct.dist_matrix;

//    std::cout<<_in_param<<"\n";
//    std::cout<<_rotate_vec<<"\n";
//    std::cout<<_translation_vec<<"\n";
//    std::cout<<_dist_vec<<"\n";

    _get_tf_lidar_to_cam();
    _get_fill_lines(camera_id);
    _get_ground_z(camera_id);

    _fill_r_theta_MODE = int(Fill_R_THETA_MODE::POINTS);
    _fill_z_MODE = int(FILL_Z_MODE::AVERAGE);

    init(data_path);
}

Coordinate_Map_tunnel::Coordinate_Map_tunnel() {}

void Coordinate_Map_tunnel::_get_cam_param(int cameraID) {

    switch (cameraID) {
        /*  suidao */
        case 1:
            _in_param = {{1181.166284, 0.,          951.870166},
                         {0.,          1174.906986, 528.353799},
                         {0,           0,           1}};
            _rotate_vec = {1.67545, -1.24024, 0.792044};
            _translation_vec = {-322.732144, 193.377473, -30.63385};
            _dist_vec = {-3.30144e-01, 9.87410e-02, 3.32000e-04, 1.90000e-04,
                         0.00000e+00};
            break;
            /*  suidao  */
        case 2:
            _in_param = {{1181.166284, 0.,          951.870166},
                         {0.,          1174.906986, 528.353799},
                         {0,           0,           1}};
            _rotate_vec = {1.655265, 1.043707, -0.780733};
            _translation_vec = {159.444094, 194.343406, -23.410967};
            _dist_vec = {-3.30144e-01, 9.87410e-02, 3.32000e-04, 1.90000e-04,
                         0.00000e+00};
            break;
            /*  suidao */
        case 3:
            _in_param = {{1148., 0.,     959.0876},
                         {0.,    1144.6, 549.3107},
                         {0,     0,      1}};
            _rotate_vec = {1.90183028, -0.28121915, 0.19092324};
            _translation_vec = {-114.127, 144.24, 13.037};
            _dist_vec = {-0.3195, 0.0919, -0.000867, -0.000868, 0.};
            break;
            /*  suidao */
        case 4:
            _in_param = {{1167.2, 0,      972.8049},
                         {0,      1162.3, 554.0905},
                         {0,      0,      1}};
            _rotate_vec = {0.75649687, 2.13602239, -1.5521844};
            _translation_vec = {-202.343, 262.929, -257.586};
            _dist_vec = {-0.3316, 0.1042, 0.0011, 0.001, 0.};
            break;
            /* 公司大门 */
//        case 0:
//            in_parameter = {{1561.81, 0, 951.44}, {0, 1561.3, 570.19}, {0, 0, 1}};
//            rotate_matrix = {1.3614502, -1.4791535, 1.1563997};
//            translation_matrix = {949.8266, -169.79738, 252.66444};
//            dist_matrix = {0.0188, 0.0511, 0.00086, -0.0013, 0};
//            break;
    }

}

void Coordinate_Map_tunnel::_get_tf_lidar_to_cam() {
    std::vector<float> rotate_vec(3);
    for (int i = 0; i < _rotate_vec.size(); ++i) {
        rotate_vec[i] = _rotate_vec(i);
    }
    cv::Mat rotate_mat;
    cv::Rodrigues(rotate_vec, rotate_mat);

    _tf_lidar_to_cam = xt::zeros<float>({4, 4});
    for (int x = 0; x < rotate_mat.rows; ++x) {
        for (int y = 0; y < rotate_mat.cols; ++y) {
            _tf_lidar_to_cam(x, y) = rotate_mat.at<float>(x, y);
        }
    }

    for (int i = 0; i < 3; ++i)
        _tf_lidar_to_cam(i, 3) = _translation_vec(i);

    _tf_lidar_to_cam(3, 3) = 1;
//    std::cout<<_tf_lidar_to_cam<<"\n";
//    printf(" ");
}

void Coordinate_Map_tunnel::_get_fill_lines(int cameraID) {
    switch (cameraID) {
        case 1:
            _fill_lines = {4, 4, 4, 4, 3,
                           3, 2, 3, 2, 2,
                           2, 2, 1, 1, 1,
                           1, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0};
            break;
        case 2:
            _fill_lines = {4, 4, 4, 4, 3,
                           3, 2, 3, 2, 2,
                           2, 2, 1, 1, 1,
                           1, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0};
            // PrintFun1D_I(fill_lines);
            break;
        case 3:
            _fill_lines = {4, 2, 3, 2, 2,
                           1, 1, 1, 1, 1,
                           1, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0};
            break;
        case 4:
            _fill_lines = {4, 2, 2, 2, 2,
                           2, 1, 1, 1, 0,
                           0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0,
                           0};
            break;
    }
//    std::cout<<_fill_lines<<"\n";
}


void Coordinate_Map_tunnel::_get_ground_z(int cameraID) {
    _ground_z = -4.5;
}

void Coordinate_Map_tunnel::init(std::string data_path) {
    _get_format_data(data_path);
    _fill_data();
    _remain_img_points_data();
    _fit_circle();
}

void Coordinate_Map_tunnel::_get_format_data(std::string data_path, bool remain_torward_sign) {
    xt::xarray<float> data = csv2Xtensor_tunnel<float>(data_path, 115200, 5);


    xt::xarray<float> temp1 = xt::view(data, xt::all(), 4);
    xt::xarray<float> temp2 = xt::view(data, xt::all(), xt::range(0, 3));
    temp1 = temp1.reshape({int(data.shape(0)), 1});
    temp2 = temp2.reshape({int(data.shape(0)), 3});
    data = xt::concatenate(xt::xtuple(temp1, temp2), 1);
    data = data.reshape({data.shape(0), 4});
    xt::view(data, xt::all(), 0) = xt::view(data, xt::all(), 0) * (-1) + 31;


    for (int i = _start_line; i < _end_line + 1; ++i) {
        xt::xarray<float> this_xyz = xt::zeros<float>({int(data.shape(0)), 4});
        int count = 0;
        for (int j = 0; j < data.shape(0); ++j) {
            if (data(j, 0) == i) {
                this_xyz(count, 0) = data(j, 0);
                this_xyz(count, 1) = data(j, 1);
                this_xyz(count, 2) = data(j, 2);
                this_xyz(count, 3) = data(j, 3);
                count += 1;
            }
        }
        this_xyz = xt::view(this_xyz, xt::range(0, count), xt::range(1, 4));

        if (remain_torward_sign) {
            remain_toward_data_by_cam_coordinate(this_xyz);
        }


        remove_err_point(this_xyz);


        if (this_xyz.shape(0) <= 1)
            continue;

        xt::xarray<float> this_xy = project_points(this_xyz);


        xt::xarray<float> this_xyz_x = xt::view(this_xyz, xt::all(), 0);
        xt::xarray<float> this_xyz_y = xt::view(this_xyz, xt::all(), 1);
        this_xyz_x.reshape({this_xyz.shape(0)});
        this_xyz_y.reshape({this_xyz.shape(0)});

        xt::xarray<float> this_r = xt::sqrt(xt::pow(this_xyz_x, 2) + xt::pow(this_xyz_y, 2));
//        std::cout<<this_r.shape(0)<<"\n";
//        std::cout<<this_r.shape(1)<<"\n";
//        std::cout<<this_r<<"\n";


        xt::xarray<float> this_theta = xt::atan2(this_xyz_y, this_xyz_x) + 2 * M_PI;


//        auto tt = xt::load_npy<float>("/data/WJ-ISFP-fusion/second/VideoAlg/theta.npy");
//        auto dif = xt::amax(xt::abs(tt - this_theta))(0);
//        std::cout<<dif<<"\n";

        std::string idx = std::to_string(i) + "_0";
//        std::cout<<xt::sort(this_r)(int (this_r.size() - 1))<<"\n";
//        xt::xarray<int> imgShape = xt::adapt(this_r.shape());
//        xt::xarray<int> imgShape1 = xt::adapt(this_theta.shape());
//        std::cout<<imgShape<<"\n"<<imgShape1<<"\n";
        _r[idx] = this_r;
        _theta[idx] = this_theta;
        _xyz[idx] = this_xyz;
        _img_xy[idx] = this_xy;
        _r_choice[idx] = xt::sort(this_r)(int(this_r.size() - 1));

    }

}


void Coordinate_Map_tunnel::remain_toward_data_by_cam_coordinate(xt::xarray<float> &this_xyz) {
    xt::xarray<float> xyz_cam = lidar_to_cam(this_xyz);
//    xt::xarray<float>  = xt::zeros<float>({int (xyz_cam.shape(0)), 3});
//    std::cout<<xyz_cam.shape(0)<<"\n";
//    std::cout<<xyz_cam.shape(1)<<"\n";
//    std::cout<<xyz_cam<<"\n";
    int count = 0;
    for (int i = 0; i < xyz_cam.shape(0); ++i) {
        if (xyz_cam(i, 2) > 0) {
            this_xyz(count, 0) = this_xyz(i, 0);
            this_xyz(count, 1) = this_xyz(i, 1);
            this_xyz(count, 2) = this_xyz(i, 2);
            count += 1;
        }
    }
    this_xyz = xt::view(this_xyz, xt::range(0, count));
//    return filter_xyz;
}

xt::xarray<float> Coordinate_Map_tunnel::lidar_to_cam(xt::xarray<float> xyz_lidar) {
    xyz_lidar = 1000 * xyz_lidar;
    xyz_lidar = xt::hstack(xt::xtuple(xyz_lidar, xt::ones<float>({int(xyz_lidar.shape(0)), 1})));

    xt::xarray<float> res;
//    std::cout<<_tf_lidar_to_cam<<"\n";
    res = xt::linalg::dot(_tf_lidar_to_cam, xt::transpose(xyz_lidar));
    res = xt::view(res, xt::range(0, 3));
    res = xt::transpose(res);
    return res;
}

void Coordinate_Map_tunnel::remove_err_point(xt::xarray<float> &xyz) {
    xt::xarray<float> r = xt::norm(xt::view(xyz, xt::all(), xt::range(0, 2)));
    r = xt::view(r, xt::all(), 0) + xt::view(r, xt::all(), 1);
    r = xt::sqrt(r);

    assert(xyz.shape(1) == 3);
    int count = 0;

    for (int i = 0; i < r.size(); ++i) {
        if (r(i) > 1.5) {
            xyz(count, 0) = xyz(i, 0);
            xyz(count, 1) = xyz(i, 1);
            xyz(count, 2) = xyz(i, 2);
            count += 1;
        }
    }
    xyz = xt::view(xyz, xt::range(0, count));
//    return filter_xyz;
}

cv::Mat xarray_to_mat_elementwise(xt::xarray<float> &xarr) {
    int ndims = xarr.dimension();
    assert(ndims == 2 && "can only convert 2d xarrays");
    int nrows = xarr.shape()[0];
    int ncols = xarr.shape()[1];
    cv::Mat mat(nrows, ncols, CV_32FC1);
    for (int rr = 0; rr < nrows; rr++) {
        for (int cc = 0; cc < ncols; cc++) {
            mat.at<float>(rr, cc) = xarr(rr, cc);
        }
    }
    return mat;
}

xt::xarray<float> Coordinate_Map_tunnel::project_points(xt::xarray<float> point3d) {
    point3d = 1000 * point3d;
    std::vector<cv::Point3f> object_points(point3d.shape(0));
    for (int i = 0; i < point3d.shape(0); ++i)
        object_points[i] = cv::Point3f(point3d(i, 0), point3d(i, 1), point3d(i, 2));

    cv::Mat rVec(3, 1, cv::DataType<float>::type); // Rotation vector
    rVec.at<float>(0) = _rotate_vec(0);
    rVec.at<float>(1) = _rotate_vec(1);
    rVec.at<float>(2) = _rotate_vec(2);
    // std::cout<<"_rotate_vec: "<<_rotate_vec<<"\n";
    cv::Mat tVec(3, 1, cv::DataType<float>::type); // Rotation vector
    tVec.at<float>(0) = _translation_vec(0);
    tVec.at<float>(1) = _translation_vec(1);
    tVec.at<float>(2) = _translation_vec(2);
    // std::cout<<"_translation_vec: "<<_translation_vec<<"\n";
    cv::Mat distCoeffs(5, 1, cv::DataType<float>::type);   // Distortion vector
    distCoeffs.at<float>(0) = _dist_vec(0);
    distCoeffs.at<float>(1) = _dist_vec(1);
    distCoeffs.at<float>(2) = _dist_vec(2);
    distCoeffs.at<float>(3) = _dist_vec(3);
    distCoeffs.at<float>(4) = _dist_vec(4);


    cv::Mat intrisicMat(3, 3, cv::DataType<float>::type); // Intrisic matrix
    intrisicMat.at<float>(0, 0) = _in_param(0, 0);
    intrisicMat.at<float>(1, 0) = _in_param(1, 0);
    intrisicMat.at<float>(2, 0) = _in_param(2, 0);

    intrisicMat.at<float>(0, 1) = _in_param(0, 1);
    intrisicMat.at<float>(1, 1) = _in_param(1, 1);
    intrisicMat.at<float>(2, 1) = _in_param(2, 1);

    intrisicMat.at<float>(0, 2) = _in_param(0, 2);
    intrisicMat.at<float>(1, 2) = _in_param(1, 2);
    intrisicMat.at<float>(2, 2) = _in_param(2, 2);
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(object_points, rVec, tVec, intrisicMat, distCoeffs, projectedPoints);

    xt::xarray<float> point2d = xt::zeros<float>({int(projectedPoints.size()), 2});
    for (int i = 0; i < projectedPoints.size(); ++i) {
        point2d(i, 0) = projectedPoints[i].x;
        point2d(i, 1) = projectedPoints[i].y;
    }
    point2d = xt::squeeze(point2d);
    return point2d;
}

std::pair<cv::Mat, cv::Mat> Coordinate_Map_tunnel::cartToPolar(xt::xarray<float> &xpt, xt::xarray<float> &ypt) {
    std::vector<cv::Point2f> sides(xpt.shape(0));
    for (int i = 0; i < xpt.shape(0); ++i)
        sides[i] = cv::Point2f(xpt(i, 0), ypt(i, 0));

    cv::Mat xpts(sides.size(), 1, CV_32F, &sides[0].x, 2 * sizeof(float));
    cv::Mat ypts(sides.size(), 1, CV_32F, &sides[0].y, 2 * sizeof(float));

    cv::Mat magnitude, angle;
    cv::cartToPolar(xpts, ypts, magnitude, angle);
    return {magnitude, angle};
}


void Coordinate_Map_tunnel::_fill_data() {
    if (_fill_r_theta_MODE == int(Fill_R_THETA_MODE::GROUND)) {
        std::function<int(int, int)> func;

    } else if (_fill_r_theta_MODE == int(Fill_R_THETA_MODE::POINTS)) {

    }

    for (int no = _start_line; no < _end_line; ++no) {
        for (int fill_no = 1; fill_no < _fill_lines(no) + 1; ++fill_no) {

            xt::xarray<float> this_x, this_y, this_z;
            std::string idx =
                    std::to_string(no) + "_" + std::to_string(int(_fill_lines(no))) + "_" + std::to_string(fill_no);
            if (_fill_r_theta_MODE == int(Fill_R_THETA_MODE::POINTS)) {
                auto fun_res = _fill_by_points_return_r_theta_z(_r, no, fill_no);
                _r[idx] = fun_res.first;
                _theta[idx] = fun_res.second.first;
                this_z = fun_res.second.second;
            } else {

            }
            this_x = _r[idx] * xt::cos(_theta[idx]);
            this_y = _r[idx] * xt::sin(_theta[idx]);

            if (this_z.shape(0) == 0) {
                float ground_z = xt::sort(xt::view(_xyz[std::to_string(no) + "_0"], xt::all(), 2))(0);
                this_z = xt::ones<float>({int(this_y.shape(0)), 1}) * ground_z;
            }
            this_x = this_x.reshape({int(this_x.shape(0)), 1});
            this_y = this_y.reshape({int(this_y.shape(0)), 1});
            this_z = this_z.reshape({int(this_z.shape(0)), 1});
            _xyz[idx] = xt::concatenate(xt::xtuple(this_x, this_y, this_z), 1);
            _img_xy[idx] = project_points(_xyz[idx]);
//            std::cout<<_xyz[idx]<<"\n";
//            std::cout<<_img_xy[idx]<<"\n";
        }
    }
}


std::pair<xt::xarray<float>, std::pair<xt::xarray<float>, xt::xarray<float>>>
Coordinate_Map_tunnel::_fill_by_points_return_r_theta_z(
        std::map<std::string, xt::xarray<float>> &input_r, int no, int fill_no) {
    std::string this_idx = std::to_string(no) + "_0";
    std::string next_idx = std::to_string(no + 1) + "_0";

    std::string less_idx, more_idx;
    if (input_r[this_idx].shape(0) > input_r[next_idx].shape(0)) {
        more_idx = this_idx;
        less_idx = next_idx;
    } else {
        more_idx = next_idx;
        less_idx = this_idx;
    }

    int delta = input_r[more_idx].shape(0) - input_r[less_idx].shape(0);

    int offset = delta / 2;
    int start_idx, end_idx;
    if (delta % 2 == 0) {
        start_idx = offset;
        end_idx = input_r[more_idx].shape(0) - offset;
    } else {
        start_idx = offset + 1;
        end_idx = input_r[more_idx].shape(0) - offset;
    }

    xt::xarray<float> this_r, next_r, fill_theta, this_z, next_z;

    if (input_r[this_idx].shape(0) >= input_r[next_idx].shape(0)) {

        this_r = xt::view(input_r[this_idx], xt::range(start_idx, end_idx));
        next_r = input_r[next_idx];
        fill_theta = _theta[next_idx];

        if (_fill_z_MODE != int(FILL_Z_MODE::NONE)) {
            this_z = xt::view(_xyz[this_idx], xt::range(start_idx, end_idx), 2);
            next_z = xt::view(_xyz[next_idx], xt::all(), 2);
        }
    } else {
        this_r = input_r[this_idx];
        next_r = xt::view(input_r[next_idx], xt::range(start_idx, end_idx));
        fill_theta = _theta[this_idx];

        if (_fill_z_MODE != int(FILL_Z_MODE::NONE)) {
            this_z = xt::view(_xyz[this_idx], xt::all(), 2);
            next_z = xt::view(_xyz[next_idx], xt::range(start_idx, end_idx), 2);
        }
    }
    xt::xarray<float> fill_r = this_r + fill_no / (1 + _fill_lines(no)) * (next_r - this_r);

    if (_fill_z_MODE == int(FILL_Z_MODE::NONE))
        return {fill_r, {fill_theta, xt::xarray<float>{float(FILL_Z_MODE::NONE)}}};
    xt::xarray<float> fill_z;
    if (_fill_z_MODE == int(FILL_Z_MODE::MAX))
        fill_z = xt::maximum(this_z, next_z);
    else if (_fill_z_MODE == int(FILL_Z_MODE::MIN))
        fill_z = xt::minimum(this_z, next_z);
    else if (_fill_z_MODE == int(FILL_Z_MODE::AVERAGE)) {
//        std::cout<<next_z.size()<<"\n";
//        std::cout<<this_z.size()<<"\n";
        fill_z = this_z + fill_no / (1 + _fill_lines(no)) * (next_z - this_z);
    }

    return {fill_r, {fill_theta, fill_z}};
}

void Coordinate_Map_tunnel::_remain_img_points_data_origin() {

    for (int no = _start_line; no < _end_line + 1; ++no) {
        std::string idx = std::to_string(no) + "_0";
//        std::cout<<_r[idx]<<"\n";
//        std::cout<<_r[idx].shape(0)<<"\n";
//        std::cout<<_r[idx].shape(1)<<"\n";
        int count = 0;
        for (int i = 0; i < _r[idx].shape(0); ++i) {
            if (_img_xy[idx](i, 0) < _project_frame_W &&
                _img_xy[idx](i, 0) >= 0 &&
                _img_xy[idx](i, 1) < _project_frame_H &&
                _img_xy[idx](i, 1) >= 0) {
//                if (i == 2007) {
//                    std::cout << "filter is " << i << "\n";
//                    std::cout<<_r[idx](i)<<"\n";
//                }
                _r[idx](count) = _r[idx](i);
                _theta[idx](count) = _theta[idx](i);
                _xyz[idx](count, 0) = _xyz[idx](i, 0);
                _xyz[idx](count, 1) = _xyz[idx](i, 1);
                _xyz[idx](count, 2) = _xyz[idx](i, 2);
                _img_xy[idx](count, 0) = _img_xy[idx](i, 0);
                _img_xy[idx](count, 1) = _img_xy[idx](i, 1);
                count += 1;
            }
        }
        _r[idx] = xt::view(_r[idx], xt::range(0, count));
        _theta[idx] = xt::view(_theta[idx], xt::range(0, count));
        _xyz[idx] = xt::view(_xyz[idx], xt::range(0, count));
        _img_xy[idx] = xt::view(_img_xy[idx], xt::range(0, count));
//        std::cout<<_r[idx]<<"\n";
//        std::cout<<_r[idx].shape(0)<<"\n";
//        std::cout<<_r[idx].shape(1)<<"\n";
//        std::cout<<_theta[idx]<<"\n";
//        std::cout<<_xyz[idx]<<"\n";
//        std::cout<<_img_xy[idx]<<"\n";
    }
}

void Coordinate_Map_tunnel::_remain_img_points_data_fill() {
    for (int no = _start_line; no < _end_line; ++no) {
        for (int fill_no = 1; fill_no < 1 + _fill_lines(no); ++fill_no) {
            std::string idx =
                    std::to_string(no) + "_" + std::to_string(int(_fill_lines(no))) + "_" + std::to_string(fill_no);
            int count = 0;

            for (int i = 0; i < _img_xy[idx].shape(0); ++i) {
                if (_img_xy[idx](i, 0) < _project_frame_W &&
                    _img_xy[idx](i, 0) >= 0 &&
                    _img_xy[idx](i, 1) < _project_frame_H &&
                    _img_xy[idx](i, 1) >= 0) {

                    _r[idx](count) = _r[idx](i);
                    _theta[idx](count) = _theta[idx](i);
                    _xyz[idx](count, 0) = _xyz[idx](i, 0);
                    _xyz[idx](count, 1) = _xyz[idx](i, 1);
                    _xyz[idx](count, 2) = _xyz[idx](i, 2);
                    _img_xy[idx](count, 0) = _img_xy[idx](i, 0);
                    _img_xy[idx](count, 1) = _img_xy[idx](i, 1);
                    count += 1;
                }
            }
            _r[idx] = xt::view(_r[idx], xt::range(0, count));
            _theta[idx] = xt::view(_theta[idx], xt::range(0, count));
            _xyz[idx] = xt::view(_xyz[idx], xt::range(0, count));
            _img_xy[idx] = xt::view(_img_xy[idx], xt::range(0, count));

        }
    }
}

void Coordinate_Map_tunnel::_remain_img_points_data(bool fill_remain_sign) {
    _remain_img_points_data_origin();
    if (fill_remain_sign)
        _remain_img_points_data_fill();
}


void Coordinate_Map_tunnel::_fit_circle() {
    _fit_circle_origin();
    if (_fit_fill_sign) {
        _fit_circle_fill();
    }
}

void Coordinate_Map_tunnel::_fit_circle_origin() {
    for (int no = _start_line; no < _end_line + 1; ++no) {
        std::string idx = std::to_string(no) + "_0";
        auto this_xy = _img_xy[idx];
        _img_abr[idx] = _hyper_fit_circle(this_xy, 99, false);
    }
}

void Coordinate_Map_tunnel::_fit_circle_fill() {
    for (int no = _start_line; no < _end_line; ++no) {
        for (int fill_no = 1; fill_no < _fill_lines(no) + 1; ++fill_no) {
            std::string idx =
                    std::to_string(no) + "_" + std::to_string(int(_fill_lines(no))) + "_" + std::to_string(fill_no);
            auto img_xy_temp = _img_xy[idx];
//            std::cout<<img_xy_temp<<"\n";
//            std::cout<<img_xy_temp.shape(0)<<"\n";
//            std::cout<<img_xy_temp.shape(1)<<"\n";
            _img_abr[idx] = _hyper_fit_circle(img_xy_temp, 99, false);

        }
    }
}


xt::xarray<float> Coordinate_Map_tunnel::_hyper_fit_circle(xt::xarray<float> &coords, int IterMax, bool verbose) {

    xt::xarray<float> X = xt::view(coords, xt::all(), 0);
    xt::xarray<float> Y = xt::view(coords, xt::all(), 1);
    X = X.reshape({X.shape(0)});
    Y = Y.reshape({Y.shape(0)});
    int n = X.shape(0);
//    float X_m = xt::mean(X);
    xt::xarray<float> Xi = X - xt::mean(X)(0);
    xt::xarray<float> Yi = Y - xt::mean(Y)(0);

    xt::xarray<float> Zi = Xi * Xi + Yi * Yi;

    float Mxy = xt::sum(Xi * Yi)(0) / n;
    float Mxx = xt::sum(Xi * Xi)(0) / n;
    float Myy = xt::sum(Yi * Yi)(0) / n;
    float Mxz = xt::sum(Xi * Zi)(0) / n;
    float Myz = xt::sum(Yi * Zi)(0) / n;
    float Mzz = xt::sum(Zi * Zi)(0) / n;

    float Mz = Mxx + Myy;
    float Cov_xy = Mxx * Myy - Mxy * Mxy;
    float Var_z = Mzz - Mz * Mz;

    float A2 = 4 * Cov_xy - 3 * Mz * Mz - Mzz;
    float A1 = Var_z * Mz + 4. * Cov_xy * Mz - Mxz * Mxz - Myz * Myz;
    float A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - Var_z * Cov_xy;
    float A22 = A2 + A2;

    float y = A0;
    float x = 0;
    int i;
    for (i = 0; i < IterMax; ++i) {
        float Dy = A1 + x * (A22 + 16. * x * x);
        float xnew = x - y / Dy;
        if (xnew == x or not std::isfinite(xnew))
            break;
        float ynew = A0 + xnew * (A1 + xnew * (A2 + 4. * xnew * xnew));
        if (std::abs(ynew) >= std::abs(y))
            break;
        x = xnew;
        y = ynew;
    }
    float det = x * x - x * Mz + Cov_xy;
    float Xcenter = (Mxz * (Myy - x) - Myz * Mxy) / det / 2.;
    float Ycenter = (Myz * (Mxx - x) - Mxz * Mxy) / det / 2.;

    x = Xcenter + xt::mean(X)(0);
    y = Ycenter + xt::mean(Y)(0);
    float r = std::sqrt(std::abs(std::pow(Xcenter, 2) + std::pow(Ycenter, 2) + Mz));
    int iter_ = i;
    if (verbose) {
        float err = xt::sqrt(
                xt::abs(xt::mean(xt::pow(xt::view(coords, 0) - x, 2) + xt::pow(xt::view(coords, 1) - y, 2) - r * r)))(
                0);
        printf("Regression complete in %d iterations.\n", iter_);
        printf("Fit error: %f \n", err);
    }
    xt::xarray<float> res = {x, y, r};
//    std::cout<<res<<"\n";
    return res;
}


std::vector<float> Coordinate_Map_tunnel::map_2d_to_3d_by_points(xt::xarray<float> &box) {

    std::vector<float> out_put = {};

    float sx = box(0, 0) * _project_frame_W / _input_frame_W;
    float sy = box(0, 1) * _project_frame_H / _input_frame_H;
    float ex = box(0, 2) * _project_frame_W / _input_frame_W;
    float ey = box(0, 3) * _project_frame_H / _input_frame_H;
    float mx = (sx + ex) / 2.0;

    auto temp = _get_clip_circle_idx(mx, ey, _start_line, _end_line + 1);
    std::string smaller_circle_idx = temp.first;
    std::string bigger_circle_idx = temp.second;

    if (smaller_circle_idx == std::to_string(int(FILL_Z_MODE::NONE)))
        return out_put;
    xt::xarray<float> smaller_circle_abr = _img_abr[smaller_circle_idx];
    xt::xarray<float> bigger_circle_abr = _img_abr[bigger_circle_idx];

    auto temp1 = _intersection_of_fit_circle_and_line(mx, smaller_circle_abr);
    float smaller_circle_y = temp1.second;

    if (smaller_circle_y < 0 or smaller_circle_y > _project_frame_H)
        return out_put;

    auto temp2 = _intersection_of_fit_circle_and_line(mx, bigger_circle_abr);
    float bigger_circle_y = temp2.second;
    if (bigger_circle_y < 0 or bigger_circle_y > _project_frame_H)
        return out_put;

    float L1 = smaller_circle_y - ey;
    float L2 = ey - bigger_circle_y;

    auto temp3 = _find_optimal_match_point(mx, smaller_circle_y, smaller_circle_idx);
    auto temp4 = _find_optimal_match_point(mx, bigger_circle_y, bigger_circle_idx);

    float X1 = temp3(0);
    float Y1 = temp3(1);
    float Z1 = temp3(2);

    float X2 = temp4(0);
    float Y2 = temp4(1);
    float Z2 = temp4(2);

    float X = L1 / (L1 + L2) * X1 + L2 / (L1 + L2) * X2;
    float Y = L1 / (L1 + L2) * Y1 + L2 / (L1 + L2) * Y2;
    float Z = L1 / (L1 + L2) * Z1 + L2 / (L1 + L2) * Z2;

    out_put.push_back(X);
    out_put.push_back(Y);
    out_put.push_back(Z);
    return out_put;
}

std::pair<std::string, std::string> Coordinate_Map_tunnel::_get_clip_circle_idx(float x, float y, float l, float r) {
    auto temp = _get_clip_circle_origin_idx(x, y, l, r);
    int small_no = temp.first;
    int bigger_no = temp.second;

    if (small_no == int(FILL_Z_MODE::NONE))
        return {std::to_string(int(FILL_Z_MODE::NONE)),
                std::to_string(int(FILL_Z_MODE::NONE))};
    std::string small_idx = std::to_string(small_no) + "_0";
    std::string bigger_idx = std::to_string(bigger_no) + "_0";

    if (_find_fill_sign) {
        auto temp2 = _get_clip_circle_detail_idx(x, y, small_no);
        small_idx = temp2.first;
        bigger_idx = temp2.second;
    }
    return {small_idx, bigger_idx};

}

std::pair<int, int> Coordinate_Map_tunnel::_get_clip_circle_origin_idx(float x, float y, float l, float r) {

    while (l < r) {
        int mid = (l + r) / 2;
        xt::xarray<float> this_0 = _img_abr[std::to_string(mid) + "_0"];
        xt::xarray<float> this_1 = _img_abr[std::to_string(mid - 1) + "_0"];

        bool this_0_flag = _is_point_in_circle(x, y, this_0(0), this_0(1), this_0(2));
        bool this_1_flag = _is_point_in_circle(x, y, this_1(0), this_1(1), this_1(2));

        if (this_1_flag)
            r = mid;
        else if (this_0_flag && !this_1_flag)
            return {mid - 1, mid};
        else if (!this_0_flag)
            l = mid;
        else if (!this_0_flag && this_1_flag)
            throw "error!";
        if (mid == 1 or mid == r - 1)
            return {int(FILL_Z_MODE::NONE), int(FILL_Z_MODE::NONE)};
    }
}

bool Coordinate_Map_tunnel::_is_point_in_circle(float x, float y, float x_0, float y_0, float r) {
    if (std::pow(x - x_0, 2) + std::pow(y - y_0, 2) < r * r)
        return true;
    else
        return false;
}

std::pair<std::string, std::string> Coordinate_Map_tunnel::_get_clip_circle_detail_idx(float x, float y, int small_no) {
    std::unordered_map<int, std::string> idx_dic;
    for (int i = 0; i < 2 + int(_fill_lines(small_no)); ++i) {
        if (i == 0)
            idx_dic[i] = std::to_string(small_no) + "_0";
        else if (i == 1 + int(_fill_lines(small_no)))
            idx_dic[i] = std::to_string(small_no + 1) + "_0";
        else
            idx_dic[i] = std::to_string(small_no) + "_" + std::to_string(int(_fill_lines(small_no))) + "_" +
                         std::to_string(i);
    }

    auto temp = get_idx_binary(x, y, 0, idx_dic.size() + 1, idx_dic);
    int small_idx_dic_dix = temp.first;
    int bigger_idx_dic_dix = temp.second;
    if (small_idx_dic_dix == int(FILL_Z_MODE::NONE))
        return {std::to_string(small_no) + "_0",
                std::to_string(small_no + 1) + "_0"};
    return {idx_dic[small_idx_dic_dix], idx_dic[bigger_idx_dic_dix]};
}

std::pair<int, int> Coordinate_Map_tunnel::get_idx_binary(float x, float y, float l, float r,
                                                   std::unordered_map<int, std::string> idx_dic) {
    while (l < r) {
        int mid = (l + r) / 2;
        xt::xarray<float> this_0 = _img_abr[idx_dic[mid]];
        xt::xarray<float> this_1 = _img_abr[idx_dic[mid - 1]];

        bool this_0_flag = _is_point_in_circle(x, y, this_0(0), this_0(1), this_0(2));
        bool this_1_flag = _is_point_in_circle(x, y, this_1(0), this_1(1), this_1(2));

        if (this_1_flag)
            r = mid;
        else if (this_0_flag && !this_1_flag)
            return {mid - 1, mid};
        else if (!this_0_flag)
            l = mid;
        else if (!this_0_flag && this_1_flag)
            throw "error!";
        if (mid == 1 or mid == r - 1)
            return {int(FILL_Z_MODE::NONE), int(FILL_Z_MODE::NONE)};
    }
}

std::pair<float, float> Coordinate_Map_tunnel::_intersection_of_fit_circle_and_line(float mx, xt::xarray<float> &param) {
    float x_0 = param(0);
    float y_0 = param(1);
    float r = param(2);

    return {mx, y_0 - std::sqrt(r * r - std::pow(mx - x_0, 2))};
}

xt::xarray<float> Coordinate_Map_tunnel::_find_optimal_match_point(float x, float y, std::string matched_idx) {
    xt::xarray<float> temp = xt::xarray<float>({x, y}) - _img_xy[matched_idx];
    xt::xarray<float> temp1 = xt::pow(temp, 2);
    xt::xarray<float> temp2 = xt::sum(temp1, 1);
    int idx = xt::argmin(temp2)(0);
    return xt::view(_xyz[matched_idx], idx);
}

void Coordinate_Map_tunnel::config_chedao_bianxian(std::string config_path) {

    for (int i = 0; i < 4; ++i) {
        xt::xarray<double> temp = xt::load_npy<double>(
                config_path + "/chedao/bianxian_" +
                std::to_string(i) + ".npy");
        _bianxian[i] = temp;
    }
    for (int i = 0; i < 4; ++i) {
        xt::xarray<double> temp = xt::load_npy<double>(
                config_path + "/chedao/bianxian_2d_" +
                std::to_string(i) + ".npy");
        _bianxian_2d[i] = temp;
    }
    for (int i = 0; i < 3; ++i) {
        xt::xarray<double> temp = xt::load_npy<double>(
                config_path + "/chedao/mid_line_" +
                std::to_string(i) + ".npy");
        _middle_line[i] = temp;
    }
    for (int i = 0; i < 3; ++i) {
        xt::xarray<double> temp = xt::load_npy<double>(
                config_path + "/chedao/middle_line_2d_" +
                std::to_string(i) + ".npy");
        _middle_line_2d[i] = temp;
    }
    for (int i = 0; i < 3; ++i) {
        xt::xarray<double> temp = xt::load_npy<double>(
                config_path + "/chedao/middle_xyz_to_xy_" +
                std::to_string(i) + ".npy");
        _middle_xyz_to_xy[i] = temp;
    }

    /**
    double original_long = _lidarLongitude[0];
    double original_lat = _lidarLatitude[0];
    double original_north = _angleNorthT[0];
    double reference_long = _lidarLongitude[_station_id - 1];
    double reference_lat = _lidarLatitude[_station_id - 1];
    double reference_north = _angleNorthT[_station_id - 1];
    std::vector<int> index(_chedao_number);
    std::vector<double> y(_chedao_number);
    for (int i = 0; i < _chedao_number; ++i) {
        xt::xarray<double> chedao_middle = xt::load_npy<double>(config_path + "chedao" + std::to_string(i+1) + ".npy");
        xt::xarray<double> boxes_for_sort = XYZ_To_BLH_batch(original_long, original_lat, chedao_middle, original_north);
        xt::dump_npy("/root/lonlat_to_xyz_batch.npy", boxes_for_sort);
        lonlat_to_xyz_batch(boxes_for_sort, reference_long, reference_lat, reference_north);
        xt::dump_npy("/root/lonlat_to_xyz_batch.npy", boxes_for_sort);
        xt::xarray<float> x_value = xt::view(boxes_for_sort, xt::all(), 0);
        auto x_range = xt::where(x_value > -100 && x_value < 100, true, false);

        xt::xarray<double> boxes_for_sort_temp = xt::zeros_like(boxes_for_sort);
        int row_temp = 0;
        for (int j = 0; j < x_range.shape(0); ++j) {
            if (x_range(j)){
                xt::view(boxes_for_sort_temp, row_temp) = xt::view(boxes_for_sort, j);
                row_temp++;
            }
        }
        boxes_for_sort_temp = xt::view(boxes_for_sort_temp, xt::range(0, row_temp));
        boxes_for_sort_temp = xt::sort(boxes_for_sort_temp, 0);
        _middle_line[i] = boxes_for_sort_temp;
        for (int j = 0; j < boxes_for_sort_temp.shape(0); ++j) {
            if (boxes_for_sort_temp(j, 0) > 0 & boxes_for_sort_temp(j, 0) < 1){
                index[i] = j;
                break;
            }
        }
        y[i] = boxes_for_sort_temp(index[i], 1);
    }
    printf("hah");
    double diff_y = (y[2] - y[0]) / 2;
    for (int i = 0; i < _chedao_number; ++i) {
        _bianxian[i] = xt::zeros_like(_middle_line[i]);
        xt::view(_bianxian[i], xt::all(), 1) = xt::view(_middle_line[i], xt::all(), 1) - diff_y / 2;
        xt::view(_bianxian[i], xt::all(), 0) = xt::view(_middle_line[i], xt::all(), 0);
    }
    _bianxian[_chedao_number] = xt::zeros_like(_middle_line[_chedao_number - 1]);
    xt::view(_bianxian[_chedao_number], xt::all(), 1) = xt::view(_middle_line[_chedao_number - 1], xt::all(), 1) + diff_y / 2;
    xt::view(_bianxian[_chedao_number], xt::all(), 0) = xt::view(_middle_line[_chedao_number - 1], xt::all(), 0);
*/
}

xt::xarray<double> Coordinate_Map_tunnel::eulerAnglesToRotationMatrix(xt::xarray<double> &theta) {
    xt::xarray<double> R_x = {{1, 0,                  0},
                              {0, std::cos(theta(0)), -1 * std::sin(theta(0))},
                              {0, std::sin(theta(0)), std::cos(theta(0))}};

    xt::xarray<double> R_y = {{std::cos(theta(1)),      0, std::sin(theta(1))},
                              {0,                       1, 0},
                              {-1 * std::sin(theta(1)), 0, std::cos(theta(1))}};

    xt::xarray<double> R_z = {{std::cos(theta(2)), -1 * std::sin(theta(2)), 0},
                              {std::sin(theta(2)), std::cos(theta(2)),      0},
                              {0,                  0,                       1}};
    xt::xarray<float> R = xt::linalg::dot(R_z, xt::linalg::dot(R_y, R_x));
    return R;
}

xt::xarray<double>
XYZ_To_BLH_batch(double &original_long, double &original_lat, xt::xarray<double> &box, double &rotaionangle) {
    double RadAngle = FG_degree2rad(rotaionangle);
    double mer_x, mer_y;
    double original_lat_temp = FG_degree2rad(original_lat);
    double original_long_temp = FG_degree2rad(original_long);
    LonLat2Mercator(original_lat_temp, original_long_temp, mer_x, mer_y);
    for (int i = 0; i < box.shape(0); ++i) {
        double move_x = box(i, 0);
        double move_y = box(i, 1);
        double mer_move_x = move_x * std::cos(RadAngle) + move_y * std::sin(RadAngle) + mer_x;
        double mer_move_y = move_y * std::cos(RadAngle) - move_x * std::sin(RadAngle) + mer_y;
        double Object_Long, Object_Lat;
        Mercator2LonLat(original_lat_temp, original_long_temp, mer_move_y, mer_move_x, Object_Long, Object_Lat);
        box(i, 0) = Object_Long;
        box(i, 1) = Object_Lat;
    }
    return box;
}

std::pair<xt::xarray<float>, float> Coordinate_Map_tunnel::map_2dto3d_transform(xt::xarray<float> &point, int &cam_id,
                                                                         xt::xarray<float> &para_plane,
                                                                         xt::xarray<float> &size) {
    float plane_normal_a = para_plane(0, 0);
    float plane_normal_b = para_plane(0, 1);
    float plane_normal_c = para_plane(0, 2);
    xt::xarray<float> rotate_matix = xt::view(_tf_lidar_to_cam, xt::range(0, 3), xt::range(0, 3));
    xt::xarray<float> trans_matrix = xt::view(_tf_lidar_to_cam, xt::range(0, 3), 3) / 1000.0;
    xt::xarray<float> distort_matix = _dist_vec;
    xt::xarray<float> in_param = _in_param;

    float a1 = (point(0) * _project_frame_W / _input_frame_W) / in_param(0, 0);
    float a2 = in_param(0, 2) / in_param(0, 0);
    float a3 = (point(1) * _project_frame_H / _input_frame_H) / in_param(1, 1);
    float a4 = in_param(1, 2) / in_param(1, 1);

    float R11_tmp = rotate_matix(0, 0) - (a1 - a2) * rotate_matix(2, 0);
    float R12_tmp = rotate_matix(0, 1) - (a1 - a2) * rotate_matix(2, 1);
    float R21_tmp = rotate_matix(1, 0) - (a3 - a4) * rotate_matix(2, 0);
    float R22_tmp = rotate_matix(1, 1) - (a3 - a4) * rotate_matix(2, 1);

    float T11 = rotate_matix(0, 2) - (a1 - a2) * rotate_matix(2, 2);
    float T12 = trans_matrix(0) - (a1 - a2) * trans_matrix(2);
    float T21 = rotate_matix(1, 2) - (a3 - a4) * rotate_matix(2, 2);
    float T22 = trans_matrix(1) - (a3 - a4) * trans_matrix(2);

    float R11 = R11_tmp + T11 * (-1 * plane_normal_a / plane_normal_c);
    float R12 = R12_tmp + T11 * (-1 * plane_normal_b / plane_normal_c);
    float R21 = R21_tmp + T21 * (-1 * plane_normal_a / plane_normal_c);
    float R22 = R22_tmp + T21 * (-1 * plane_normal_b / plane_normal_c);

    float X0 = T11 / plane_normal_c - T12;
    float Y0 = T21 / plane_normal_c - T22;

    xt::xarray<float> R = {{R11, R12},
                           {R21, R22}};
    xt::xarray<float> T = {X0, Y0};
    T = T.reshape({T.size(), 1});
    xt::xarray<float> mat_inv = xt::linalg::inv(R);
//    std::cout<<mat_inv<<"\n";
//    std::cout<<T<<"\n";
    xt::xarray<float> res = xt::linalg::dot(mat_inv, T);
    float z = -1 * (1 + plane_normal_a * res(0) + plane_normal_b * res(1)) / plane_normal_c;
    if (cam_id == 1) {
        res(0) = res(0) - size(1) / 2;
    } else if (cam_id == 2) {
        res(0) = res(0) + size(1) / 2;
    } else {
        res(0) = res(0);
    }
    return {res, z};
}

xt::xarray<float> Coordinate_Map_tunnel::cal_chafen_diff2(xt::xarray<float> &cam_boxes_list, int &cam_id) {
    cam_boxes_list(0) = cam_boxes_list(0) * 1920 / 640;
    cam_boxes_list(1) = cam_boxes_list(1) * 1080 / 640;
    cam_boxes_list(2) = cam_boxes_list(2) * 1920 / 640;
    cam_boxes_list(3) = cam_boxes_list(3) * 1080 / 640;
//    std::cout<<cam_boxes_list<<"\n";
    std::map<int, xt::xarray<float>> chedao_x, chedao_y, point;
    std::map<int, xt::xarray<float>>::iterator iter;
    for (iter = _bianxian_2d.begin(); iter != _bianxian_2d.end(); iter++) {
        chedao_x[iter->first] = xt::view(iter->second, xt::all(), 0);
        chedao_y[iter->first] = xt::view(iter->second, xt::all(), 1);
    }
    int chedao_num = iter->first - 1;
    if (cam_id == 1) {
        point[0] = xt::xarray<float>({cam_boxes_list(0), cam_boxes_list(3)});
    } else {
        point[0] = xt::xarray<float>({cam_boxes_list(2), cam_boxes_list(3)});
    }

    xt::xarray<float> point_central = {float((cam_boxes_list(0) + cam_boxes_list(2)) / 2.),
                                       float((cam_boxes_list(1) + cam_boxes_list(3)) / 2.)};
    float box_central_y = float((cam_boxes_list(1) + cam_boxes_list(3)) / 2.);
    float box_dibian_y = cam_boxes_list(3);

    std::map<int, int> indics_y;
    std::vector<float> x_near;
    int flag;
    if ((cam_boxes_list(2) > 1800 and cam_id == 2) or (cam_boxes_list(2) < 100 and cam_id == 1) or
        box_central_y > 900) {
        flag = 2;
        xt::xarray<float> dibian_point = {0, 0, float(flag), -1};
        return dibian_point;
    }

    for (iter = _bianxian_2d.begin(); iter != _bianxian_2d.end(); ++iter) {
        auto indics_temp = xt::where(chedao_y[iter->first] > box_dibian_y)[0];

        if (indics_temp.size()) {
            indics_y[iter->first] = indics_temp[0];
        } else {
            indics_y[iter->first] = -1;
        }
        x_near.push_back(chedao_x[iter->first][indics_y[iter->first]]);
    }
    int chedao_id;
    if ((std::abs(point[0](0) - x_near[1]) < std::abs(point[0](0) - x_near[0])) and
        (std::abs(point[0](0) - x_near[1]) < std::abs(point[0](0) - x_near[2]))) {
        chedao_id = 0;
    }
    else if ((std::abs(point[0](0) - x_near[2]) < std::abs(point[0](0) - x_near[1])) and
             (std::abs(point[0](0) - x_near[2]) < std::abs(point[0](0) - x_near[3]))){
        chedao_id = 1;
    }
    else {
        chedao_id = 2;
    }

    std::map<int, xt::xarray<float>> point1, point2;
    for (iter = _middle_line_2d.begin(); iter != _middle_line_2d.end(); ++iter) {
        auto filter_all = xt::where(chedao_x[iter->first] >= cam_boxes_list[0] &&
                                    chedao_x[iter->first] <= cam_boxes_list[2] &&
                                    chedao_y[iter->first] >= cam_boxes_list[1] &&
                                    chedao_y[iter->first] <= cam_boxes_list[3])[0];
        xt::xarray<float> chedao_index = xt::zeros_like(_bianxian_2d[iter->first]);
        int count = 0;
        for (int i = 0; i < filter_all.size(); ++i) {
            xt::view(chedao_index, count) = xt::view(_bianxian_2d[iter->first], filter_all[i]);
            count++;
        }
        chedao_index = xt::view(chedao_index, xt::range(0, count));
        std::cout<<chedao_index.shape(0)<<"\n";
        if (!chedao_index.shape(0)){
            chedao_num = chedao_num - 1;
        }
        else{
            point1[iter->first] = {chedao_index(0, 0), chedao_index(0, 1)};
            point2[iter->first] = {chedao_index(chedao_index.shape(0) - 1, 0),chedao_index(chedao_index.shape(0) - 1, 1)};
        }
    }
    if (chedao_num == 0){
        flag = 0;
        xt::xarray<float> dibian_point = {point_central(0), point_central(1), float (flag), float (chedao_id)};
        return dibian_point;
    }
    else {
        if (point1.count(chedao_id) == 0 and point1.count(chedao_id + 1) != 0){
            flag = 1;
            xt::xarray<float> dibian_point = {(point1[chedao_id + 1](0) + point2[chedao_id + 1](0)) / 2,
                                              (point1[chedao_id + 1](1) + point2[chedao_id + 1](1)) / 2,
                                              float (flag), float (chedao_id)};
            return dibian_point;
        }
        else if (point1.count(chedao_id + 1) == 0 and point1.count(chedao_id) == 0){
            flag = 0;
            xt::xarray<float> dibian_point = {point_central(0), point_central(1), float (flag), float (chedao_id)};
            return dibian_point;
        }
        else{
            flag = 1;
            xt::xarray<float> dibian_point = {(point1[chedao_id](0) + point2[chedao_id](0)) / 2,
                                              (point1[chedao_id](1) + point2[chedao_id](1)) / 2,
                                              float (flag), float (chedao_id)};
            return dibian_point;
        }
    }

}