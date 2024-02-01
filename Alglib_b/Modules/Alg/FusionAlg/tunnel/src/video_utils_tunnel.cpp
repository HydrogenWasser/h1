//
// Created by root on 2/18/22.
//

#include "video_utils_tunnel.h"
#include "xtensor/xbuilder.hpp"
#include <xtensor/xarray.hpp>
#include <xtensor/xcontainer.hpp>
#include <iostream>



WLH_Estimator::WLH_Estimator() {
    class_to_size[0] = {0.66344886, 0.7256437, 1.75748069};
    class_to_size[1] = {2.4560939, 6.73778078, 2.73004906};
    class_to_size[2] = {0.60058911, 1.68452161, 1.27192197};
    class_to_size[3] = {0.76279481, 2.09973778, 1.44403034};
    class_to_size[4] = {0.76279481, 2.09973778, 1.44403034};
    class_to_size[5] = {1.95017717, 4.60718145, 1.72270761};
    class_to_size[6] = {1.95017717, 4.60718145, 1.72270761};
    class_to_size[7] = {2.4560939, 6.73778078, 2.73004906};
    class_to_size[8] = {1.95017717, 4.60718145, 1.72270761};
    class_to_size[9] = {1.95017717, 4.60718145, 1.72270761};
    class_to_size[10] = {2.94046906, 11.1885991, 3.47030982};
    class_to_size[11] = {2.4560939, 6.73778078, 2.73004906};
    class_to_size[12] = {2.4560939, 6.73778078, 2.73004906};
    class_to_size[13] = {0.66344886, 0.7256437, 1.75748069};
    class_to_size[14] = {2.4560939, 6.73778078, 2.73004906};
}

WLH_Estimator::~WLH_Estimator() {}

void WLH_Estimator::get_wlh(xt::xarray<int> &classes, xt::xarray<float> &wlh) {
    wlh = xt::zeros<float>({int(classes.size()), 3});
    for (int i = 0; i < classes.size(); ++i) {
        wlh(i, 0) = class_to_size[classes(i)][0];
        wlh(i, 1) = class_to_size[classes(i)][1];
        wlh(i, 2) = class_to_size[classes(i)][2];
    }
}

Box_Bottom_Lane_Judger::Box_Bottom_Lane_Judger(std::string folder_path, int no, int border_sx, int border_sy,
                                               int border_ex, int border_ey) :
        border_ex(border_ex),
        border_ey(border_ey),
        border_sx(border_sx),
        border_sy(border_sy) {
    lane_config_path = folder_path + "/ch" + std::to_string(no);
    if (access((lane_config_path + "/lane_matrix.npy").c_str(), 0) == -1) {
        __generate_configs();
    } else {
        lane_matrix = xt::load_npy<int>(lane_config_path + "/lane_matrix.npy");  // if error, try int8/int16 ...
    }
}

Box_Bottom_Lane_Judger::~Box_Bottom_Lane_Judger() {}

void Box_Bottom_Lane_Judger::__generate_configs() {
    std::ifstream inFile(lane_config_path + "/lane.csv", std::ios::in);
    std::string lineStr;
    std::vector<std::vector<int>> color_lane;
    int k = -1;
    while (getline(inFile, lineStr)) {
        k++;
        if (k == 0)
            continue;
        std::stringstream ss(lineStr);
        std::string str;
        std::vector<int> lineArray;

        while (getline(ss, str, ','))
            lineArray.push_back(stringToNum_tunnel<int>(str));
        color_lane.push_back(lineArray);
    }
//    getchar();

    cv::Mat img = cv::imread(lane_config_path + "/lane.png", cv::COLOR_BGR2RGB);;
    int h = img.rows;
    int w = img.cols;
    int c = img.channels();


    lane_matrix = xt::zeros<int>({w, h});
    lane_matrix += -1;


    for (int i = 0; i < w; ++i) {
        for (int j = 0; j < h; ++j) {
            int r = img.at<cv::Vec3b>(j, i)[2];
            int g = img.at<cv::Vec3b>(j, i)[1];
            int b = img.at<cv::Vec3b>(j, i)[0];

            for (int l = 0; l < color_lane.size(); ++l) {
                if (r == color_lane[l][1] and g == color_lane[l][2] and b == color_lane[l][3]) {
                    lane_matrix(i, j) = color_lane[l][0];
                }
            }
        }
    }
    xt::dump_npy(lane_config_path + "/lane_matrix.npy", lane_matrix);
}


xt::xarray<int> Box_Bottom_Lane_Judger::get_lane_no(xt::xarray<float> &boxes, int from_to) {
    if (boxes.shape(0) == 0) {
//        return;
    }
    xt::xarray<float> points = xt::empty<float>({0, 2});
    std::vector<float> percents = {0, 0.125, 0.25};
    for (auto &percent : percents) {
        if (from_to) {
            xt::xarray<float> this_point = xt::zeros<float>({int(boxes.shape(0)), 2});
            xt::view(this_point, xt::all(), 0) = xt::view(boxes, xt::all(), 2) * (1 - percent) +
                                                 xt::view(boxes, xt::all(), 0) * percent;
            xt::view(this_point, xt::all(), 1) = xt::view(boxes, xt::all(), 3);
            // this_point = xt::transpose(this_point);

            // std::cout << "points=" << points << "this_point=" << this_point << std::endl;
            if (points.shape(0) > 0) {
                points = xt::hstack(xt::xtuple(points, this_point));
            } else {
                points = this_point;
            }
        } else {
            xt::xarray<float> this_point = xt::zeros<float>({int(boxes.shape(0)), 2});
            for (int i = 0; i < this_point.shape(0); ++i) {
                this_point(i, 0) = boxes(i, 0) + (boxes(i, 2) - boxes(i, 0)) * percent;
                this_point(i, 1) = boxes(i, 3);
            }

            if (points.shape(0) > 0) {
                points = xt::hstack(xt::xtuple(points, this_point));
            } else {
                points = this_point;
            }
        }

    }
//    std::cout<<points<<"\n";
    xt::xarray<int> points_int = xt::cast<int>(points);
    xt::xarray<int> lane_nos = xt::zeros<int>({boxes.shape(0), percents.size()});

    for (int i = 0; i < percents.size(); ++i) {
        for (int j = 0; j < lane_nos.shape(0); ++j) {
            lane_nos(j, i) = lane_matrix(int(points_int(j, 2 * i)), int(points_int(j, 2 * i + 1)));
        }
    }

    // xt::xarray<int> output_lane_no(boxes.shape(0), -1);
    xt::xarray<int> output_lane_no = xt::zeros<int>({boxes.shape(0)});
    for (int i = 0; i < lane_nos.shape(0); ++i) {
        xt::xarray<float> this_lane_nos = xt::view(lane_nos, i);
        xt::xarray<float> unique_lane_no = xt::unique(this_lane_nos);
        std::vector<int> index;
        get_index(this_lane_nos, unique_lane_no, index);
        int max_index = std::max_element(index.begin(), index.end()) - index.begin();

//        std::cout<<unique_lane_no<<"\n";
        output_lane_no(i) = unique_lane_no(max_index);
    }

    for (int i = 0; i < boxes.shape(0); ++i) {
        int sx, sy, ex, ey;
        sx = boxes(i, 0);
        sy = boxes(i, 1);
        ex = boxes(i, 2);
        ey = boxes(i, 3);

        if (sx < border_sx or sy < border_sy or ex > lane_matrix.shape(0) - border_ex or
            ey > lane_matrix.shape(1) - border_ey) {
            output_lane_no(i) = -1;
        }
    }
    return output_lane_no;
}

void Box_Bottom_Lane_Judger::get_index(xt::xarray<float> &nums, xt::xarray<float> &unique_nums,
                                       std::vector<int> &index) {
    for (int i = 0; i < unique_nums.shape(0); ++i) {
        for (int j = 0; j < nums.shape(0); ++j) {
            if (nums(j) == unique_nums(i)){
                index.emplace_back(j);
                break;
            }
        }
    }
}

Match_Pairs_Coordinate_Estimator::Match_Pairs_Coordinate_Estimator(std::string config_path, int cam_no,
                                                                   int border_y, int station_id, float ground_z,
                                                                   double lon, double lat, double angle){
    init_object_dict();
    if (access((config_path + "/ch" + std::to_string(cam_no) + "/match_pairs/0.npy").c_str(), 0) == -1) {
        std::cout<<"error! ---> connot find: match_pairs/*.npy\n";
        // Generate_Match_Pairs *gmp = new Generate_Match_Pairs(cam_no, config_path, station_id, ground_z, lon, lat,
        //                                                      angle);
        // gmp->run();
        // delete gmp;
    }
    match_pairs[0] = xt::load_npy<double>(config_path + "/ch" + std::to_string(cam_no) + "/match_pairs/0.npy");
    match_pairs[1] = xt::load_npy<double>(config_path + "/ch" + std::to_string(cam_no) + "/match_pairs/1.npy");
    match_pairs[2] = xt::load_npy<double>(config_path + "/ch" + std::to_string(cam_no) + "/match_pairs/2.npy");
    border_filter_img = cv::imread(config_path + "/ch" + std::to_string(cam_no) + "/border_filter.png");

    std::map<int, xt::xarray<float>>::iterator iter;
    for (iter = match_pairs.begin(); iter != match_pairs.end(); ++iter) {
        int count = 0;
        std::vector<int> color(3, 0);
        xt::xarray<float> match_pairs_temp = xt::zeros_like(match_pairs[iter->first]);
        for (int i = 0; i < match_pairs[iter->first].shape(0); ++i) {
            int index_0 = int(match_pairs[iter->first](i, MAP_IMG_Y));
            int index_1 = int(match_pairs[iter->first](i, MAP_IMG_X));

            color[0] = int(border_filter_img.at<cv::Vec3b>(int(match_pairs[iter->first](i, MAP_IMG_Y)),
                                                           int(match_pairs[iter->first](i, MAP_IMG_X)))[0]);
            color[1] = int(border_filter_img.at<cv::Vec3b>(int(match_pairs[iter->first](i, MAP_IMG_Y)),
                                                           int(match_pairs[iter->first](i, MAP_IMG_X)))[1]);
            color[2] = int(border_filter_img.at<cv::Vec3b>(int(match_pairs[iter->first](i, MAP_IMG_Y)),
                                                           int(match_pairs[iter->first](i, MAP_IMG_X)))[2]);

            if (color[0] == 255 and color[1] == 255 and color[2] == 255) {

            } else {
                xt::view(match_pairs_temp, count) = xt::view(match_pairs[iter->first], i);
                count++;
            }
        }
        match_pairs[iter->first] = xt::view(match_pairs_temp, xt::range(0, count), xt::all());
    }
}
xt::xarray<float> Match_Pairs_Coordinate_Estimator::get_xyz(xt::xarray<float> &boxes, xt::xarray<int> &classes,
                                                            xt::xarray<int> &lane_no) {
    xt::xarray<float> xyz = xt::zeros<float>({int (boxes.shape(0)), 3});
    for (int i = 0; i < boxes.shape(0); ++i) {
        int det_class = classes(i);
        if (lane_no(i) == -1){
            continue;
        }

        float sx = boxes(i, 0);
        float sy = boxes(i, 1);
        float ex = boxes(i, 2);
        float ey = boxes(i, 3);

        std::vector<float> map_img_xy(2, 0);
        map_img_xy[0] = int (sx + (ex - sx) / 2);
        if (std::find(object_dict["little"].begin(), object_dict["little"].end(), det_class) != object_dict["little"].end()){
            map_img_xy[1] = ey - int ((ey - sy) / 2);
        } else{
            map_img_xy[1] = ey - int ((ey - sy) / 4);
        }
        if (border_filter_img.at<cv::Vec3b>(int (map_img_xy[1]), int (map_img_xy[0]))[0] == 255 and
            border_filter_img.at<cv::Vec3b>(int (map_img_xy[1]), int (map_img_xy[0]))[1] == 255 and
            border_filter_img.at<cv::Vec3b>(int (map_img_xy[1]), int (map_img_xy[0]))[2] == 255){
            continue;
        }

        xt::xarray<float> this_match_pair = match_pairs[lane_no(i)];
        xt::xarray<float> match_img_xy = xt::view(match_pairs[lane_no(i)], xt::all(), xt::range(MID_LINE_IMG_X, MID_LINE_IMG_X + 2));
        xt::view(match_img_xy, xt::all(), 0) -= map_img_xy[0];
        xt::view(match_img_xy, xt::all(), 1) -= map_img_xy[1];
        match_img_xy *= -1;


        match_img_xy = xt::norm(match_img_xy);
        match_img_xy = xt::sum(match_img_xy, 1);
        match_img_xy = xt::sqrt(match_img_xy);

        int index = xt::argmin(match_img_xy)[0];

        xyz(i, 0) = this_match_pair(index, PC_X);
        xyz(i, 1) = this_match_pair(index, PC_Y);
        xyz(i, 2) = -4.9;
    }
    // std::cout<<xyz<<"\n";
    return xyz;
}

void Match_Pairs_Coordinate_Estimator::init_object_dict() {
    object_dict["all"] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    object_dict["used"] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    object_dict["little"] = {int (Object_Index::person),
                             int (Object_Index::bicycle),
                             int (Object_Index::motorbike),
                             int (Object_Index::pedal_tricycle),
                             int (Object_Index::car),
                             int (Object_Index::passenger_car),
                             int (Object_Index::police_car),
                             int (Object_Index::roadblock)};
    object_dict["big"] = {int (Object_Index::tool_vehicle),
                          int (Object_Index::truck),
                          int (Object_Index::ambulance),
                          int (Object_Index::bus),
                          int (Object_Index::dump_truck),
                          int (Object_Index::tanker),
                          int (Object_Index::fire_car)};
}
Match_Pairs_Coordinate_Estimator::~Match_Pairs_Coordinate_Estimator() {}


Generate_Match_Pairs::Generate_Match_Pairs(int cam_no, std::string config_path, int station_id, float ground_z,
                                           double lon, double lat, double angle) :
        cam_no(cam_no),
        config_path(config_path),
        pc_id(station_id),
        ground_z(ground_z),
        lon(lon),
        lat(lat),
        angle(angle) {
    xt::xarray<float> in_param;
    xt::xarray<float> rotate_mat;
    xt::xarray<float> translation_mat;
    xt::xarray<float> dist_mat;
    Load_Param(in_param, rotate_mat, translation_mat, dist_mat,
               config_path + "/ch" + std::to_string(cam_no) + "/camera_param_to_ground2.csv");

    lidar_trans = Lidar_Camera_Transform(in_param, rotate_mat, translation_mat, dist_mat);
    mid_line_longlat[0] = xt::load_npy<double>(
            config_path + "/ch" + std::to_string(cam_no) + "/mid_line_lonlat_dense/0.npy");
    mid_line_longlat[1] = xt::load_npy<double>(
            config_path + "/ch" + std::to_string(cam_no) + "/mid_line_lonlat_dense/1.npy");
    mid_line_longlat[2] = xt::load_npy<double>(
            config_path + "/ch" + std::to_string(cam_no) + "/mid_line_lonlat_dense/2.npy");

    project_frame_H = 1080;
    project_frame_W = 1920;

    line_img = cv::imread(config_path + "/ch" + std::to_string(cam_no) + "/mid_line.png", cv::COLOR_BGR2RGB);

    // read csv
    std::ifstream inFile(config_path + "/ch" + std::to_string(cam_no) + "/mid_line.csv", std::ios::in);
    std::string lineStr;

    int k = -1;
    while (getline(inFile, lineStr)) {
        k++;
        if (k == 0)
            continue;
        std::stringstream ss(lineStr);
        std::string str;
        std::vector<int> lineArray;

        while (getline(ss, str, ','))
            lineArray.push_back(stringToNum_tunnel<int>(str));
        line_param[lineArray[0]] = {lineArray[1], lineArray[2], lineArray[3]};

    }

    mid_line_img_xy[0] = xt::empty<int>({0, 2});
    mid_line_img_xy[1] = xt::empty<int>({0, 2});
    mid_line_img_xy[2] = xt::empty<int>({0, 2});
    for (int i = 0; i < line_img.size[0]; ++i) {
        for (int j = 0; j < line_img.size[1]; ++j) {
            for (int l = 0; l < 3; ++l) {
                if (line_img.at<cv::Vec3b>(i, j)[0] == line_param[l][0] and
                    line_img.at<cv::Vec3b>(i, j)[1] == line_param[l][1] and
                    line_img.at<cv::Vec3b>(i, j)[2] == line_param[l][2]){
                    xt::xarray<int> temp = {{j, i}};
                    mid_line_img_xy[l] = xt::concatenate(xt::xtuple(mid_line_img_xy[l], temp), 0);
                }
            }
        }
    }

//    xt::dump_npy("/data/Documents/project_cpp/video_utils/Configs/Video/ch1/000.npy", mid_line_img_xy[0]);
//    xt::dump_npy("/data/Documents/project_cpp/video_utils/Configs/Video/ch1/001.npy", mid_line_img_xy[1]);
//    xt::dump_npy("/data/Documents/project_cpp/video_utils/Configs/Video/ch1/002.npy", mid_line_img_xy[2]);
}


void Load_Param(xt::xarray<float> &in_param, xt::xarray<float> &rotate_mat, xt::xarray<float> &translation_mat,
                xt::xarray<float> &dist_mat, std::string path) {

    std::ifstream inFile(path, std::ios::in);
    xt::xarray<float> data = xt::load_csv<float>(inFile);
    in_param = xt::view(data, 0);
//    std::cout << in_param << "\n";
    in_param.reshape({3, 3});

    rotate_mat = xt::view(data, 1, xt::range(0, 3));

    translation_mat = xt::view(data, 2, xt::range(0, 3));

    dist_mat = xt::view(data, 3, xt::range(0, 5));
}


Lidar_Camera_Transform::Lidar_Camera_Transform(xt::xarray<float> &in_param, xt::xarray<float> &rotate_mat,
                                               xt::xarray<float> &translation_mat, xt::xarray<float> &dist_mat) :
        in_param(in_param),
        rotate_mat(rotate_mat),
        translation_mat(translation_mat),
        dist_mat(dist_mat) {
    _get_tr_lidar_to_cam();
}

Lidar_Camera_Transform::Lidar_Camera_Transform() {}

Lidar_Camera_Transform::~Lidar_Camera_Transform() {}

void Lidar_Camera_Transform::_get_tr_lidar_to_cam() {
    tr_lidar_to_cam = xt::zeros<float>({4, 4});

    std::vector<float> rotate_vec(3);
    for (int i = 0; i < rotate_mat.size(); ++i) {
        rotate_vec[i] = rotate_mat(i);
    }
    cv::Mat rotate_mat_cv;
    cv::Rodrigues(rotate_vec, rotate_mat_cv);
    for (int x = 0; x < rotate_mat_cv.rows; ++x) {
        for (int y = 0; y < rotate_mat_cv.cols; ++y) {
            tr_lidar_to_cam(x, y) = rotate_mat_cv.at<float>(x, y);
        }
    }
    for (int i = 0; i < 3; ++i)
        tr_lidar_to_cam(i, 3) = translation_mat(i);
    tr_lidar_to_cam(3, 3) = 1;
    // std::cout << tr_lidar_to_cam << "\n";
}

Camera_Map::Camera_Map() {}
Camera_Map::~Camera_Map() {}

Camera_Map::Camera_Map(xt::xarray<float> &in_param, xt::xarray<float> &rotate_vec, xt::xarray<float> &translation_vec,
                       xt::xarray<float> &dist_vec):
                       _in_param(in_param),
                       _rotate_vec(rotate_vec),
                       _translation_vec(translation_vec),
                       _dist_vec(dist_vec){
    _get_tf_lidar_to_cam();
}

void Camera_Map::_get_tf_lidar_to_cam() {
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
}

xt::xarray<float> Camera_Map::lidar_to_cam(xt::xarray<float> xyz_lidar) {
    xyz_lidar = 1000 * xyz_lidar;
    xyz_lidar = xt::hstack(xt::xtuple(xyz_lidar, xt::ones<float>({int(xyz_lidar.shape(0)), 1})));
    xt::xarray<float> res;
    res = xt::linalg::dot(_tf_lidar_to_cam, xt::transpose(xyz_lidar));
    res = xt::view(res, xt::range(0, 3));
    res = xt::transpose(res);
    return res;
}

xt::xarray<float> Camera_Map::project_points(xt::xarray<float> &point3d) {
    point3d = 1000 * point3d;
    std::vector<cv::Point3f> object_points(point3d.shape(0));
    for (int i = 0; i < point3d.shape(0); ++i)
        object_points[i] = cv::Point3f(point3d(i, 0), point3d(i, 1), point3d(i, 2));

    cv::Mat rVec(3, 1, cv::DataType<float>::type); // Rotation vector
    rVec.at<float>(0) = _rotate_vec(0);
    rVec.at<float>(1) = _rotate_vec(1);
    rVec.at<float>(2) = _rotate_vec(2);
//    std::cout<<rVec<<"\n";
    cv::Mat tVec(3, 1, cv::DataType<float>::type); // Rotation vector
    tVec.at<float>(0) = _translation_vec(0);
    tVec.at<float>(1) = _translation_vec(1);
    tVec.at<float>(2) = _translation_vec(2);
//    std::cout<<_translation_vec(0)<<"\n";
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

xt::xarray<bool> Camera_Map::map(xt::xarray<float> &pc_det, std::vector<int> img_size) {


    xt::xarray<bool> indics = xt::zeros<bool>({pc_det.shape(0)});
    xt::xarray<float> points = pc_det.reshape({int (pc_det.size() / 3), 3});
    xt::xarray<float> xyz_cam = lidar_to_cam(points);
    auto filter_cam = xt::where(xt::view(xyz_cam, xt::all(), 2) > 0, true, false);
    xt::xarray<float> r = xt::view(points, xt::all(), xt::range(0, 2));
    r = xt::norm(r);
    r = xt::sum(r, 1);
    r = xt::sqrt(r);

    auto filter_r = xt::where(r > 4, true, false);
    xt::xarray<bool> filter_ = filter_cam && filter_r;
    xt::xarray<float> img2d = project_points(points);
    auto filter_img = xt::where(xt::view(img2d, xt::all(), 0) > 0 and
                                        xt::view(img2d, xt::all(), 0) < img_size[0] and
                                        xt::view(img2d, xt::all(), 1) > 0 and
                                        xt::view(img2d, xt::all(), 1) < img_size[1], true, false);
//    std::cout<<filter_img<<"\n";
    for (int i = 0; i < int (img2d.shape(0) / 8); ++i) {
        xt::xarray<bool> temp = xt::view(filter_, xt::range(8 * i, 8 * i + 8));
        temp.reshape({8, 1});
        temp = xt::where(xt::view(temp, xt::all(), 0) < 1, true, false);
        if (xt::all(temp)){
            continue;
        }
        temp = xt::view(filter_img, xt::range(8 * i, 8 * i + 8));
        temp.reshape({8, 1});
        temp = xt::where(xt::view(temp, xt::all(), 0) < 1, true, false);
        if (xt::all(temp)){
            continue;
        }
        indics(i) = true;
    }

    // std::cout<<indics<<"\n";
    // std::cout<<""<<"\n";
    return indics;
}

xt::xarray<float> StackDet(const xt::xarray<float>& p_refXarrBox,
                            const xt::xarray<float>& p_refXarrClass, 
                            const xt::xarray<float>& p_refXarrConfidence,
                            const xt::xarray<float>& p_refXarrId)
{
    // std::cout << "p_refXarrBox = (" << p_refXarrBox.shape(0) << ", " << p_refXarrBox.shape(1) << ")" << std::endl;
    // std::cout << "p_refXarrConfidence = (" << p_refXarrConfidence.shape(0) << ")" << std::endl;
    // std::cout << "p_refXarrClass = (" << p_refXarrClass.shape(0) << ")" << std::endl;
    // std::cout << "p_refXarrId = (" << p_refXarrId.shape(0) << ")" << std::endl;

    if (p_refXarrBox.shape(0) == 0 || p_refXarrBox.shape(1) != 4)
    {
        return xt::empty<float>({0, 7});
    }
    
    xt::xarray<float> l_xarrBox = p_refXarrBox;
    xt::xarray<float> l_xarrConfidence = p_refXarrConfidence;
    l_xarrConfidence = l_xarrConfidence.reshape({int (l_xarrConfidence.size()), 1});
    xt::xarray<float> l_xarrClass = p_refXarrClass;
    l_xarrClass = l_xarrClass.reshape({int (l_xarrClass.size()), 1});
    xt::xarray<float> l_xarrId = p_refXarrId;
    l_xarrId = l_xarrId.reshape({int (l_xarrId.size()), 1});

    // std::cout << "l_xarrBox = (" << l_xarrBox.shape(0) << ", " << l_xarrBox.shape(1) << ")" << std::endl;
    // std::cout << "l_xarrConfidence = (" << l_xarrConfidence.shape(0) << ", " << l_xarrConfidence.shape(1) << ")" << std::endl;
    // std::cout << "l_xarrClass = (" << l_xarrClass.shape(0) << ", " << l_xarrClass.shape(1) << ")" << std::endl;
    // std::cout << "l_xarrId = (" << l_xarrId.shape(0) << ", " << l_xarrId.shape(1) << ")" << std::endl;

    xt::xarray<float> l_xarrRet = xt::hstack(xt::xtuple(l_xarrBox, l_xarrClass, l_xarrConfidence, l_xarrId));
    return l_xarrRet;
}