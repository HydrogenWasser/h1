//
// Created by root on 5/8/21.
//

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <utility>
#include <xtensor/xmath.hpp>
#include <xtensor/xsort.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xarray.hpp>
#include "fusion_algorithm_crossing.h"
#include "hungarian.h"
#define pi_zsc 3.1415926
// #define USETILES

int FUKalmanBoxTracker_crossing::_count = 0;
Fusion_Algorithm_crossing::~Fusion_Algorithm_crossing(){
    delete _sort;
}

/*
*Parameters
*    config_path : 配置文件路径
*    camID : 智慧基站的第几路摄像头传来的视频
*/
// Fusion_Algorithm::Fusion_Algorithm(TSelfFusionAlgParam *m_stAlgParam) :
//     _config_path(m_stAlgParam->m_pFusionAlgParam.m_strFusionConfPath),
//     _GroundHeight(m_stAlgParam->m_pFusionAlgParam.m_GroundHeight),
//     _iou_threshold(m_stAlgParam->m_pFusionAlgParam.m_iouThreshold),
//     _cam_num(int(m_stAlgParam->m_pCameraParam.m_vecCameraDev.size())),
//     _input_frame_W(m_stAlgParam->m_pFusionAlgParam.m_compressImg_W),
//     _input_frame_H(m_stAlgParam->m_pFusionAlgParam.m_compressImg_H),
//     _project_frame_W(m_stAlgParam->m_pFusionAlgParam.m_originalImg_W),
//     _project_frame_H(m_stAlgParam->m_pFusionAlgParam.m_originalImg_H),
//     _CountPeopleflag(m_stAlgParam->m_pFusionAlgParam.m_CountPeopleflag),
//     _PeopleCoorflag(m_stAlgParam->m_pFusionAlgParam.m_PeopleCoorflag)

Fusion_Algorithm_crossing::Fusion_Algorithm_crossing(TSelfFusionAlgParam *m_stAlgParam){
    // 读取融合算法配置文件
    YAML::Node fusion_cfg = YAML::LoadFile(m_stAlgParam->m_strRootPath + m_stAlgParam->m_strFusionCfgPath);

    m_stAlgParam->m_stFusionAlgParam.m_vecPcClass = {fusion_cfg["CROSSING_FUSION"]["PC_CLASS_NAMES"][0].as<std::string>(),
                                                     fusion_cfg["CROSSING_FUSION"]["PC_CLASS_NAMES"][1].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["PC_CLASS_NAMES"][2].as<std::string>(),
                                                     fusion_cfg["CROSSING_FUSION"]["PC_CLASS_NAMES"][3].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["PC_CLASS_NAMES"][4].as<std::string>(),
                                                     fusion_cfg["CROSSING_FUSION"]["PC_CLASS_NAMES"][5].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["PC_CLASS_NAMES"][6].as<std::string>()};

    m_stAlgParam->m_stFusionAlgParam.m_vecVideoClass = {fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][0].as<std::string>(),
                                                        fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][1].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][2].as<std::string>(),
                                                        fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][3].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][4].as<std::string>(),
                                                        fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][5].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][6].as<std::string>(),
                                                        fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][7].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][8].as<std::string>(),
                                                        fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][9].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][10].as<std::string>(),
                                                        fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][11].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][12].as<std::string>(),
                                                        fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][13].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][14].as<std::string>(),
                                                        fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][15].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][16].as<std::string>(),
                                                        fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][17].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["VIDEO_CLASS_NAMES"][18].as<std::string>()};

    m_stAlgParam->m_stFusionAlgParam.m_vecFusionClass = {fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][0].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][1].as<std::string>(),
                                                         fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][2].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][3].as<std::string>(),
                                                         fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][4].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][5].as<std::string>(),
                                                         fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][6].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][7].as<std::string>(),
                                                         fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][8].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][9].as<std::string>(),
                                                         fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][10].as<std::string>(), fusion_cfg["CROSSING_FUSION"]["FUSION_CLASS_NAMES"][11].as<std::string>()};
    // 融合算法参数
    _config_path     = m_stAlgParam->m_strRootPath + fusion_cfg["CROSSING_FUSION_CONF_PATH"].as<std::string>();
    _GroundHeight    = fusion_cfg["CROSSING_FUSION"]["GROUND_HEIGHT"].as<float>();
    _iou_threshold   = fusion_cfg["CROSSING_FUSION"]["IOU_THRESHOLD"].as<float>();

    _input_frame_W   = fusion_cfg["CROSSING_FUSION"]["COMPRESS_IMG_W"].as<int>();
    _input_frame_H   = fusion_cfg["CROSSING_FUSION"]["COMPRESS_IMG_H"].as<int>();
    _project_frame_W = fusion_cfg["CROSSING_FUSION"]["ORIGINAL_IMG_W"].as<int>();
    _project_frame_H = fusion_cfg["CROSSING_FUSION"]["ORIGINAL_IMG_H"].as<int>();
    _CountPeopleflag = fusion_cfg["CROSSING_FUSION"]["COUNT_PEOPLE_FLAG"].as<int>();
    _PeopleCoorflag  = fusion_cfg["CROSSING_FUSION"]["PEOPLE_COOR_FLAG"].as<int>();
    _cam_num = int(m_stAlgParam->m_stCameraParam.m_vecCameraDev.size()); //TODO 读配置文件？
    auto class_size_path = _config_path + "/class_size.csv";
    class_size = csv2Xtensor<float>(class_size_path, 12, 3);
    for(int i = 0; i < _cam_num; i++)
    {
        auto tiles_path = _config_path + "/test_tiles" + std::to_string(i) + ".npy";
        xt::xarray<float> tiles = xt::load_npy<float>(tiles_path);
        tiles_vec.push_back(tiles);
    }

    //读取行人统计图片 TODO 理解
    for (int i = 0; i < _cam_num; ++i)
    {
        std::string count_file = _config_path + cv::format("/image/_video%d",i) + ".bmp";
        cv::Mat img = cv::imread(count_file,cv::IMREAD_COLOR);
        _img_people_count.push_back(img);

        std::string coor_file = _config_path + cv::format("/image/%d",i) + ".png";
        cv::Mat img_ = cv::imread(coor_file,cv::IMREAD_COLOR);
        _img_people_coor.push_back(img_);
    }

    //initial sort
    _sort = new FSort_crossing(m_stAlgParam);

    //初始帧数
    _frame_count = 0;

    for (int i = 0; i < _cam_num; ++i) {
        _coordinate_map.push_back(Coordinate_Map_crossing(i + 1, false, m_stAlgParam));
    }

    //加载配置文件,用于尺寸模型和投射变换
    xt::xarray<float> ClassConf,WarpMatrixConf;
    load_config(_config_path, ClassConf, WarpMatrixConf);
    //用于小目标孪生，目前没用
    _class_size["car"]      = xt::view(ClassConf,0,xt::all());
    _class_size["bicycle"]  = xt::view(ClassConf,1,xt::all());
    _class_size["bus"]      = xt::view(ClassConf,2,xt::all());
    _class_size["tricycle"] = xt::view(ClassConf,3,xt::all());
    _class_size["person"]   = xt::view(ClassConf,4,xt::all());
    _class_size["truck"]    = xt::view(ClassConf,5,xt::all());

    //加载图像投射变换孪生参数
    xt::xarray<float> corner_2d_sum = xt::view(WarpMatrixConf,xt::all(),xt::all(),xt::range(0,2));
    xt::xarray<float> corner_3d_sum = xt::view(WarpMatrixConf,xt::all(),xt::all(),xt::range(2,5));

    for (int i = 0; i < corner_2d_sum.shape(0); ++i){
        xt::xarray<float> warpMatrix;
        xt::xarray<float> corner_2d=xt::view(corner_2d_sum,i,xt::all(),xt::all());
        xt::xarray<float> corner_3d=xt::view(corner_3d_sum,i,xt::all(),xt::all());
        WarPersepctiveMatrix(corner_2d,corner_3d,warpMatrix);
        warpMatrix_vec.push_back(warpMatrix);
    }
}

//融合配置文件
void Fusion_Algorithm_crossing::load_config(std::string config_path,xt::xarray<float> &ClassConf,xt::xarray<float> &WarpMatrixConf)
{
    //类别参数和斑马线孪生方法参数
    std::string class_size_path = config_path+"/class_init.csv";
    std::string warpMatrix_path = config_path+"/warpMatrix_init.csv";
    //行数列数配置
    ClassConf = csv2Xtensor<float>(class_size_path, 6, 3);
    WarpMatrixConf = csv2Xtensor<float>(warpMatrix_path, 3, 20);
    WarpMatrixConf = WarpMatrixConf.reshape({3, 4, 5});
}

//图像孪生加载
void Fusion_Algorithm_crossing::WarPersepctiveMatrix(xt::xarray<float> &corner_2d, xt::xarray<float> &corner_3d,
                                            xt::xarray<float> &warpMatrix) {
    //4*2 2d_corner, 4*3 3d_corner
    int nums = corner_2d.shape(0);
    xt::xarray<float> A = xt::zeros<float> ({2 * nums, 8});
    xt::xarray<float> B = xt::zeros<float> ({2 * nums, 1});
    xt::xarray<float> corner3d_reshape = xt::zeros<float> ({nums, 2});

    for (int i = 0; i < nums; ++i) {
        corner3d_reshape(i, 0) = corner_3d(i, 0) / corner_3d(i, 2);
        corner3d_reshape(i, 1) = corner_3d(i, 1) / corner_3d(i, 2);
        xt::xarray<float> A_i = xt::view(corner_2d, i, xt::all());
        xt::xarray<float> B_i = xt::view(corner3d_reshape, i, xt::all());
        xt::view(A, 2 * i, xt::all()) = xt::xarray<float> ({A_i(0), A_i(1), 1,
                                                            0, 0, 0,
                                                            -A_i(0)*B_i(0), -A_i(1)*B_i(0)});
        xt::view(B, 2 * i, xt::all()) = B_i(0);
        xt::view(A, 2 * i + 1, xt::all()) = xt::xarray<float> ({0, 0, 0,
                                                                A_i(0), A_i(1), 1,
                                                                -A_i(0)*B_i(1), -A_i(1)*B_i(1)});
        xt::view(B, 2 * i + 1,xt::all()) = B_i(1);
    }

    warpMatrix = xt::linalg::solve(A,B);

    xt::xarray<float> c3_temp = xt::xarray<float>({1});
    c3_temp = c3_temp.reshape({1,1});
    warpMatrix = xt::concatenate(xt::xtuple(warpMatrix,c3_temp),0);
    warpMatrix = warpMatrix.reshape({3,3});
}

/*
*Parameters
*    video : 保存"视频检测框左上角x,视频检测框左上角y,视频检测框右下角x,视频检测框右下角y,
*            检测框类别, 检测框置信度"的列表
*            没有单位，均为像素值
*    data :  保存"点云检测框中心点x,点云检测框中心点y,点云检测框中心点z,点云检测框宽,
*            点云检测框长, 点云检测框高, 航向角, 类别, 速度, 点云跟踪ID号, 置信度，角点1的x, 角点1的y, #idx : 11
*            角点1的z, 角点2的x, 角点2的y, 角点2的z, 角点3的x, 角点3的y, 角点3的z, 角点4的x, #idx : 19
*            角点4的y, 角点4的z, 角点5的x, 角点5的y, 角点5的z, 角点6的x, 角点6的y, 角点6的z, #idx : 27
*            角点7的x, 角点7的y, 角点7的z, 角点8的x, 角点8的y, 角点8的z"的列表#idx:33
*            单位为米
*   _trk:x y w l angle(rad) z h label speed(-1) blanket conf blanket ...
*    mapSign : 是否使用二维点反映射到激光雷达坐标系功能
*/

Fusion_Res Fusion_Algorithm_crossing::RUN(std::vector <xt::xarray<float>> &video,
                              xt::xarray<float> &data,
                              int cam_num) {
    //clear history data
    _match_flag_lidar.clear();
    _match_flag_video.clear();
    _matches_res.clear();
    _matches_res_untrk.clear();
    _boxes_list.clear();        // video_boxes
    _score.clear();             // video_score
    _labels_list.clear();       // video_label
    _cam_box_id.clear();        // video_boxes_id  0~boxes.size-1
    _video_ret.clear();
    _lidar_box_vector.clear();
    /*    video: leftx, lefty, rightx, righty, label, conf, id
          data : x, y, z, w, l, h, angle, label, speed, id, conf, 24*corners    */

    _people_count = xt::zeros<int>({4,1});
    _video_coordinate = xt::empty<float>({0,21});
    xt::xarray<float> tempVideoBox = xt::zeros<float> ({int(data.shape(0)), 6});//add video 6 cows to lidar detection (leftx, lefty, rightx, righty, cam_id, cambox_id)
    data = xt::concatenate(xt::xtuple(data,tempVideoBox), 1);

    for (int i = 0; i < data.shape(0); ++i)
    {
        data(i, 40) = -1;
    }
    _match_flag_video.resize(cam_num);
    _matches_res.resize(cam_num);
    for (int i = 0; i < data.shape(0); ++i) {
        _match_flag_lidar.push_back(0);
    }
    _data = data;

    /******************* fusion part *******************/
    for (int l = 0; l < cam_num; l++)
    {
        _cam_id = l;
        xt::xarray<int> matches;
        std::vector<int> indicss;
        xt::xarray<float> cost;
        //图像处理
        VideoSrcdataHandle(video[l], l);

        //图像功能  行人计数
        VideoFunction(l);//目前未用-2023/03/21

        //点云映射
        /*_lidar_box_list 返回值  点云检测结果转至图像二维像素检测结果 0～3 TLX TLY BRX BRY
          _camlidar       返回值  成功转换的点云结果（原始的）
          _indicss        返回值  成功转换的索引flag，若成功转换，索引对应值_indicss(i)=1
        */
        #ifdef USETILES
        // 使用瓦片图进行融合
        data_fusion(data, _cam_id, 0.0, matches, cost);

        #else

        data_analysis(_lidar_box_list, _camlidar, _indicss, _cam_id);

        //iou计算
        DealFusionKM(_lidar_box_list, 0.0, matches, indicss, cost, l);

        //融合处理
        FusionResHandle(matches, cost, data, indicss, l);

        #endif //

        //图像输出
        xt::xarray<float> temp1 = _score[l].reshape({_score[l].size(), 1});
        xt::xarray<float> temp2 = _labels_list[l].reshape({_labels_list[l].size(), 1});
        xt::xarray<float> temp_video = xt::concatenate(xt::xtuple(_boxes_list[l], _score[l], _labels_list[l]), 1);
        xt::xarray<float> videobox_id = xt::zeros<int>({(int)_score[l].size(), 1});
        temp_video = xt::concatenate(xt::xtuple(temp_video,_match_flag_video[l],videobox_id), 1);
        //添加点云映射框
        // xt::xarray<float> temp_lidar_box_list = xt::empty<float>({(int)_lidar_box_list.shape(0),8});
        // for(int t = 0; t < _lidar_box_list.shape(0); t++)
        // {
        //     temp_lidar_box_list(t, 0) = _lidar_box_list(t, 0);
        //     temp_lidar_box_list(t, 1) = _lidar_box_list(t, 1);
        //     temp_lidar_box_list(t, 2) = _lidar_box_list(t, 2);
        //     temp_lidar_box_list(t, 3) = _lidar_box_list(t, 3);
        //     temp_lidar_box_list(t, 4) = 3;
        //     temp_lidar_box_list(t, 5) = 1;
        //     temp_lidar_box_list(t, 6) = 1;
        //     temp_lidar_box_list(t, 7) = 101;
        // }
        // temp_video = xt::concatenate(xt::xtuple(temp_video,temp_lidar_box_list), 0);
        _video_ret.push_back(temp_video);
        _lidar_box_vector.push_back(_lidar_box_list);
    }

    /******************* track part *******************/
    //det 0:x 1:y 2:w 3:l 4:alpha 5:z 6:h 7:label 8:score 9-12:video_box 13:camID 14:video_boxid
    //跟踪算法及输出
    _sort->sort(data, _video_ret, _frame_count);
    _pc_ret = _sort->_pc_ret;
    _frame_count += 1;

    _output.pc_res = _pc_ret;
    _output.video_res = _video_ret;
    _output.PeopleCount = _people_count;
    _output.lidar_box_vector = _lidar_box_vector;
    return _output;
}

//图像检测预处理
void Fusion_Algorithm_crossing::VideoSrcdataHandle(xt::xarray<float> VideoData,int l){
    xt::xarray<float> l_arrVideoSortInput = xt::ones<float>({int(VideoData.shape(0)), 6});
    for (int j = 0; j < VideoData.shape(0); j++) {
        l_arrVideoSortInput(j, 0) = int(VideoData(j, 0));   // x0
        l_arrVideoSortInput(j, 1) = int(VideoData(j, 1));   // y0
        l_arrVideoSortInput(j, 2) = int(VideoData(j, 2));   // x1
        l_arrVideoSortInput(j, 3) = int(VideoData(j, 3));   // y1
        l_arrVideoSortInput(j, 4) = VideoData(j, 5);        // score
        l_arrVideoSortInput(j, 5) = (int) (VideoData(j, 4));// label
    }
    _boxes_list.emplace_back(xt::view(l_arrVideoSortInput, xt::all(), xt::range(0, 4)));
    xt::xarray<float> score_temp = xt::view(l_arrVideoSortInput, xt::all(), 4);
    _score.emplace_back(score_temp.reshape({score_temp.size()}));
    xt::xarray<float> label_temp = xt::view(l_arrVideoSortInput, xt::all(), 5);
    _labels_list.emplace_back(label_temp.reshape({label_temp.size()}));
    _cam_box_id.emplace_back(xt::arange<int>(0, _boxes_list[l].shape(0), 1));

    //post_nms
    post_nms_crossing::postNms(_boxes_list[l], _score[l], _labels_list[l]);
    _match_flag_video[l]= xt::zeros<int>({(int)_boxes_list[l].shape(0), 1});
}

//图像功能
//目前都是020232
void Fusion_Algorithm_crossing::VideoFunction(int l){
    if (_CountPeopleflag) {
        cv::Mat img_people_count = _img_people_count[l];
        for (int i = 0; i < _boxes_list[l].shape(0); ++i)
        {
//            PeopleWait
            if (_labels_list[l](i) == 4) {
                xt::xarray<int> bottom_point = xt::cast<int>(xt::xarray<float>(
                        {(_boxes_list[l](i, 0) + _boxes_list[l](i, 2)) / 2 / _input_frame_W * _project_frame_W, _boxes_list[l](i, 3) / _input_frame_H * _project_frame_H,1}));
                int tempx = bottom_point(0),tempy = bottom_point(1);
                if (tempx < _project_frame_W && tempx > 0 && tempy < _project_frame_H && tempy > 0)
                {
                    int color0 = img_people_count.at<cv::Vec3b>(tempy, tempx)[0];
                    int color1 = img_people_count.at<cv::Vec3b>(tempy, tempx)[1];
                    int color2 = img_people_count.at<cv::Vec3b>(tempy, tempx)[2];

                    if (color0 == 50)//white
                        _people_count(0,0) += 1;
                    if (color0 == 100)//red
                        _people_count(1,0) += 1;
                    if (color0 == 150)//yellow
                        _people_count(2,0) += 1;
                    if (color0 == 200)//blue
                        _people_count(3,0) += 1;
                }
            }
        }
    }

    if (_PeopleCoorflag)
    {
        cv::Mat ImgZebraCrossing = _img_people_coor[l];

        for (int i = 0; i < _boxes_list[l].shape(0); ++i)
        {
            xt::xarray<int> bottom_point = xt::cast<int>(xt::xarray<float>(
                    {(_boxes_list[l](i, 0) + _boxes_list[l](i, 2)) / 2 / _input_frame_W * _project_frame_W, _boxes_list[l](i, 3) / _input_frame_H * _project_frame_H, 1}));
            int tempx = bottom_point(0), tempy = bottom_point(1);
            if (tempx < _project_frame_W && tempx > 0 && tempy < _project_frame_H && tempy > 0)
            {
                int color = ImgZebraCrossing.at<cv::Vec3b>(tempy,tempx )[0];
                bottom_point = bottom_point.reshape({3,1});
                if (_cam_id == 1)
                {
                    if (color ==150 && (_labels_list[l](i) == 1 || _labels_list[l](i) == 2 || _labels_list[l](i) == 4))
                    {
                        xt::xarray<float> dst = xt::linalg::dot(warpMatrix_vec[0],bottom_point);
                        dst(0,0) = dst(0,0) / dst(2,0) * _GroundHeight;
                        dst(1,0) = dst(1,0) / dst(2,0) * _GroundHeight;
                        dst(2,0) = dst(2,0) / dst(2,0) * _GroundHeight;
                        std::string label_str = label_crossing::class2size(_labels_list[l](i));
                        xt::xarray<float> video_temp = xt::xarray<float>{dst(0,0),
                                                                         dst(1,0),
                                                                         _class_size[label_str](0,0),
                                                                         _class_size[label_str](0,1),
                                                                         180,
                                                                         dst(2,0),
                                                                         _class_size[label_str](0,2),
                                                                         static_cast<float>(_labels_list[l](i)),
                                                                         0,
                                                                         0,
                                                                         _score[l](i),
                                                                         0,
                                                                         0,
                                                                         0,
                                                                         2.0,
                                                                         static_cast<float>(_frame_count),
                                                                         _boxes_list[l](i, 0)/_input_frame_W*_project_frame_W,
                                                                         _boxes_list[l](i, 1)/_input_frame_H*_project_frame_H,
                                                                         _boxes_list[l](i, 2)/_input_frame_W*_project_frame_W,
                                                                         _boxes_list[l](i, 3)/_input_frame_H*_project_frame_H,
                                                                         float(_cam_id)};
                        video_temp = video_temp.reshape({1,video_temp.size()});
                        _video_coordinate = xt::concatenate(xt::xtuple(_video_coordinate,video_temp),0);

                    }
                    else if (color ==200 && (_labels_list[l](i) == 1 || _labels_list[l](i) == 2 || _labels_list[l](i) == 4))
                    {
                        xt::xarray<float> dst = xt::linalg::dot(warpMatrix_vec[1],bottom_point);
                        dst(0,0) = dst(0,0) / dst(2,0) * _GroundHeight;
                        dst(1,0) = dst(1,0) / dst(2,0) * _GroundHeight;
                        dst(2,0) = dst(2,0) / dst(2,0) * _GroundHeight;
                        std::string label_str = label_crossing::class2size(_labels_list[l](i));
                        xt::xarray<float> video_temp = xt::xarray<float>{dst(0,0),
                                                                         dst(1,0),
                                                                         _class_size[label_str](0,0),
                                                                         _class_size[label_str](0,1),
                                                                         90,
                                                                         dst(2,0),
                                                                         _class_size[label_str](0,2),
                                                                         static_cast<float>(_labels_list[l](i)),
                                                                         0,
                                                                         0,
                                                                         _score[l](i),
                                                                         0,
                                                                         0,
                                                                         0,
                                                                         2.0,
                                                                         static_cast<float>(_frame_count),
                                                                         _boxes_list[l](i, 0)/_input_frame_W*_project_frame_W,
                                                                         _boxes_list[l](i, 1)/_input_frame_H*_project_frame_H,
                                                                         _boxes_list[l](i, 2)/_input_frame_W*_project_frame_W,
                                                                         _boxes_list[l](i, 3)/_input_frame_H*_project_frame_H,
                                                                         float(_cam_id)};
                        video_temp = video_temp.reshape({1,video_temp.size()});
                        _video_coordinate = xt::concatenate(xt::xtuple(_video_coordinate,video_temp),0);
                    }
                }
                else if (_cam_id == 2)
                {
                    if (color ==100 && (_labels_list[l](i) == 1 || _labels_list[l](i) == 2 || _labels_list[l](i) == 4))
                    {
                        xt::xarray<float> dst = xt::linalg::dot(warpMatrix_vec[2],bottom_point);
                        dst(0,0) = dst(0,0) / dst(2,0) * _GroundHeight;
                        dst(1,0) = dst(1,0) / dst(2,0) * _GroundHeight;
                        dst(2,0) = dst(2,0) / dst(2,0) * _GroundHeight;
                        std::string label_str = label_crossing::class2size(_labels_list[l](i));
                        xt::xarray<float> video_temp = xt::xarray<float>{dst(0,0),
                                                                         dst(1,0),
                                                                         _class_size[label_str](0,0),
                                                                         _class_size[label_str](0,1),
                                                                         0,
                                                                         dst(2,0),
                                                                         _class_size[label_str](0,2),
                                                                         static_cast<float>(_labels_list[l](i)),
                                                                         0,
                                                                         0,
                                                                         _score[l](i),
                                                                         0,
                                                                         0,
                                                                         0,
                                                                         2.0,
                                                                         static_cast<float>(_frame_count),
                                                                         _boxes_list[l](i, 0)/_input_frame_W*_project_frame_W,
                                                                         _boxes_list[l](i, 1)/_input_frame_H*_project_frame_H,
                                                                         _boxes_list[l](i, 2)/_input_frame_W*_project_frame_W,
                                                                         _boxes_list[l](i, 3)/_input_frame_H*_project_frame_H,
                                                                         float(_cam_id)};
                        video_temp = video_temp.reshape({1,video_temp.size()});
                        _video_coordinate = xt::concatenate(xt::xtuple(_video_coordinate,video_temp),0);

                    }
                }
            }
        }
    }
}

/*
* 参数:
*       data : 点云检测
*       camID:  智慧基站的第几路摄像头传来的视频

* 返回 :
*       matches:点云检测与视频检测匹配结果
*       cost : iou计算结果cost矩阵
*/
void Fusion_Algorithm_crossing::data_fusion(xt::xarray<float> &data, int camID, float eps, xt::xarray<int> &matches, xt::xarray<float> &cost) 
{
    xt::xarray<float> tiles = tiles_vec[camID];
    xt::xarray<float> video_point = xt::zeros<float>({(int)_boxes_list[camID].shape(0), 4});
    xt::xarray<float> video_boxes = _boxes_list[camID];
    for(int i = 0;i < _boxes_list[camID].shape(0); i++)
    {
        int a = (int)(0.5 * (video_boxes(i, 0) + video_boxes(i, 2))) / _input_frame_W * _project_frame_W;
        int b = (int)(0.5 * (video_boxes(i, 1) + video_boxes(i, 3))) / _input_frame_H * _project_frame_H;
        video_point(i, 0) = tiles(a, b, 0);
        video_point(i, 1) = tiles(a, b, 1);
        video_point(i, 2) = class_size(int(_labels_list[camID](i)), 0);
        video_point(i, 3) = class_size(int(_labels_list[camID](i)), 1);
    }
    xt::xarray<float> video_xyxy = xt::zeros<float>({(int)_boxes_list[camID].shape(0), 4});

    xt::view(video_xyxy, xt::all(), 0) = xt::view(video_point, xt::all(), 0) - xt::view(video_point, xt::all(), 3) / 2;
    xt::view(video_xyxy, xt::all(), 1) = xt::view(video_point, xt::all(), 1) - xt::view(video_point, xt::all(), 2) / 2;
    xt::view(video_xyxy, xt::all(), 2) = xt::view(video_point, xt::all(), 0) + xt::view(video_point, xt::all(), 3) / 2;
    xt::view(video_xyxy, xt::all(), 3) = xt::view(video_point, xt::all(), 1) + xt::view(video_point, xt::all(), 2) / 2;
    
    xt::xarray<float> pc_xyxy = xt::zeros<float>({(int)_data.shape(0), 4});
    xt::view(pc_xyxy, xt::all(), 0) = xt::view(_data, xt::all(), 0) - xt::view(_data, xt::all(), 3) / 2;
    xt::view(pc_xyxy, xt::all(), 1) = xt::view(_data, xt::all(), 1) - xt::view(_data, xt::all(), 2) / 2;
    xt::view(pc_xyxy, xt::all(), 2) = xt::view(_data, xt::all(), 0) + xt::view(_data, xt::all(), 3) / 2;
    xt::view(pc_xyxy, xt::all(), 3) = xt::view(_data, xt::all(), 1) + xt::view(_data, xt::all(), 2) / 2;
    
    auto res = box_ops_crossing::iou_jit_new(pc_xyxy, video_xyxy, eps);
    cost = res.first;
    xt::xarray<float> cost_matrix = -cost;
    auto matches_unorder = run_hungarian_match1(cost_matrix);
    auto sort_index = xt::argsort(xt::view(matches_unorder, xt::all(), 0));
    matches = matches_unorder;
    for (int i = 0; i < sort_index.size(); ++i) {
        xt::view(matches, i) = xt::view(matches_unorder, sort_index(i));
    }
    for (int k = 0; k < matches.shape(0); ++k) 
    {
        int i = matches(k, 0);
        int j = matches(k, 1);

        if (cost(i, j) > _iou_threshold) 
        {
            if (_match_flag_lidar[i] == 0)
                _match_flag_lidar[i] = 1;
            int final_class = label_crossing::fusion_class(int(data(i, 7)), int(_labels_list[camID](j)));
            data(i, 7) = float(final_class);
            _labels_list[camID](j) = float(final_class);
            _match_flag_video[camID](j, 0) = 1;
            _matches_res[camID].push_back({i, j});
            data(i,35) = _boxes_list[camID](j,0) / _input_frame_W * _project_frame_W;
            data(i,36) = _boxes_list[camID](j,1) / _input_frame_H * _project_frame_H;
            data(i,37) = _boxes_list[camID](j,2) / _input_frame_W * _project_frame_W;
            data(i,38) = _boxes_list[camID](j,3) / _input_frame_H * _project_frame_H;
            data(i,39) = camID + 1;
            data(i,40) = j;
        }
    }
}

/*
* 参数:
*       lidar_box_list : 雷达映射框
*       camlidar ： 相机朝向内的点云数据
*       indicss ： 相机朝向内的点云框索引
*       direct : 是只否保留相机朝向的点云点
*       camID:  智慧基站的第几路摄像头传来的视频
*       cvSign: True:采用opencv自带的三维投影到二维的函数进行映射
*               False:采用编写的numpy形式计算将三维点映射到二维图像中
* 返回 :
*       lidar_boxes:点云检测的8个三维点映射到图像中的二维框
*       camlidar : 在相机视场里的点云检测结果
*       indicss : camlidar在所有雷达检测结果里的索引
*/
void Fusion_Algorithm_crossing::data_analysis(xt::xarray<float> &lidar_box_list, xt::xarray<float> &camlidar,
                                     xt::xarray<int> &indicss, int camID, bool direct, bool cvSign) 
{
    xt::xarray<float> DirLidar;
    if (direct) 
    {

    } else 
    {
        DirLidar = _data;
    }
    std::vector<int> img_size = {_project_frame_W, _project_frame_H};
    xt::xarray<float> dirLidar_input = xt::view(DirLidar, xt::all(), xt::range(11, 35));
    auto lidar_box_indices = cv_map(dirLidar_input, camID, img_size, _coordinate_map[camID]);
    lidar_box_list = lidar_box_indices.first;
    indicss = lidar_box_indices.second;
    int count = 0;
    for (int i = 0; i < DirLidar.shape(0); ++i) 
    {
        if (indicss(i) == 1) 
        {
            xt::view(DirLidar, count) = xt::view(DirLidar, i);
            count += 1;
        }
    }

    camlidar = xt::view(DirLidar, xt::range(0, count));
}

/*
* 参数：
*       param points : 三维点云框角点
*       cameraID ： 相机ID
*       param img_size : 图像分辨率
*       param coordinate_map : coordinate_map类对象
*return :
*       indimg_lists : 三维映射到二维角点坐标
*       indics : 映射目标在所有点云目标的索引
*/
std::pair <xt::xarray<float>, xt::xarray<int>> Fusion_Algorithm_crossing::cv_map(xt::xarray<float> &points,
                                                                        int cameraID,
                                                                        std::vector<int> img_size,
                                                                        Coordinate_Map_crossing coordinate_map) 
{
    xt::xarray<float> indimg_lists = xt::empty<float>({0, 4});
    xt::xarray<int> indices = xt::zeros<int>({(int)points.shape(0)});
    points = points.reshape({int(points.size() / 3), 3});
    xt::xarray<float> xyz_cam = coordinate_map.lidar_to_cam(points);
    xt::xarray<int> filter_cam = xt::zeros<int>({(int)xyz_cam.shape(0)});
    for (int i = 0; i < xyz_cam.shape(0); ++i) 
    {
        if (xyz_cam(i, 2) > 0)
            filter_cam(i) = 1;
        else
            filter_cam(i) = 0;
    }

    xt::xarray<float> r = xt::norm(xt::view(points, xt::all(), xt::range(0, 2)));
    r = xt::view(r, xt::all(), 0) + xt::view(r, xt::all(), 1);
    r = xt::sqrt(r);
    r = r.reshape({r.size()});

    xt::xarray<int> filter_r = xt::zeros<int>({(int)xyz_cam.shape(0)});
    for (int i = 0; i < r.shape(0); ++i) {
        if (r(i) > 4)        //盲区？
            filter_r(i) = 1;
        else
            filter_r(i) = 0;
    }
    xt::xarray<int> filter_ = filter_r * filter_cam;
    xt::xarray<float> img2d = coordinate_map.project_points(points);//3d->2d
    xt::xarray<int> filter_img = xt::zeros<int>({(int)img2d.shape(0)});
    for (int i = 0; i < img2d.shape(0); ++i) 
    {
        if (img2d(i, 0) > 0 && img2d(i, 0) < img_size[0] &&
        img2d(i, 1) > 0 && img2d(i, 1) < img_size[1])
            filter_img(i) = 1;
        else
            filter_img(i) = 0;
    }

    xt::xarray<float> zero_tensor = {{0, 0, float(_input_frame_W - 1), float(_input_frame_H - 1)}};
    for (int i = 0; i < int(img2d.shape(0) / 8); ++i) 
    {
        if (xt::sum(xt::view(filter_, xt::range(8 * i, 8 * i + 8)))(0) == 0)
            continue;
        if (xt::sum(xt::view(filter_img, xt::range(8 * i, 8 * i + 8)))(0) == 0)
            continue;

        indices(i) = 1;
        xt::xarray<float> indimg_list = xt::view(img2d, xt::range(8 * i, 8 * i + 8));
        auto lidar_box = coordinate_map.lidar_box_calculation(indimg_list);//project_size -> input_size
        if (xt::amax(xt::abs(zero_tensor - lidar_box))(0) > 0)
            indimg_lists = xt::concatenate(xt::xtuple(indimg_lists, lidar_box), 0);
    }
    return {indimg_lists, indices};
}

//图像与点云融合KM
void Fusion_Algorithm_crossing::DealFusionKM(xt::xarray<float> _lidar_box_list,
        float eps,
        xt::xarray<int> &matches,
        std::vector<int> &indicss,
        xt::xarray<float> &cost,
        int l)
{
    auto res = box_ops_crossing::iou_jit_new(_lidar_box_list, _boxes_list[l], eps);
    cost = res.first;
    xt::xarray<float> cost_matrix = -cost;
    // auto matches_unorder = run_hungarian_match(cost_matrix);
    auto matches_unorder = run_hungarian_match1(cost_matrix);

    //这里用了排序是因为KM与匈牙利匹配的结果存在顺序上的差异，为了与python结果做对比，因此加了排序
    auto sort_index = xt::argsort(xt::view(matches_unorder, xt::all(), 0));
    matches = matches_unorder;
    for (int i = 0; i < sort_index.size(); ++i) {
        xt::view(matches, i) = xt::view(matches_unorder, sort_index(i));
    }
    //判断_indicss为true的点云检测目标
    for (int i = 0; i < _indicss.size(); ++i) {
        if (_indicss(i) == 1)
            indicss.push_back(i);
    }
}

//融合结果按图像协议格式输出，融合结果中加入图像信息
void Fusion_Algorithm_crossing::FusionResHandle(xt::xarray<int> matches, xt::xarray<float> cost, xt::xarray<float> &data, std::vector<int> indicss, int l)
{
    for (int k = 0; k < matches.shape(0); ++k) 
    {
        int i = matches(k, 0);
        int j = matches(k, 1);

        if (cost(i, j) > _iou_threshold) 
        {
            //匹配成功，标志位置1
            if (_match_flag_lidar[indicss[i]] == 0)
                _match_flag_lidar[indicss[i]] = 1;
            int final_class = label_crossing::fusion_class(int(_camlidar(i, 7)), int(_labels_list[l](j)));
            data(indicss[i], 7) = float(final_class);
            _labels_list[l](j) = float(final_class);

            _match_flag_video[l](j, 0) = 1;
            _matches_res[l].push_back({i, j});
            data(indicss[i],35) = _boxes_list[l](j,0)/_input_frame_W*_project_frame_W;
            data(indicss[i],36) = _boxes_list[l](j,1)/_input_frame_H*_project_frame_H;
            data(indicss[i],37) = _boxes_list[l](j,2)/_input_frame_W*_project_frame_W;
            data(indicss[i],38) = _boxes_list[l](j,3)/_input_frame_H*_project_frame_H;
            data(indicss[i],39) = l + 1;
            data(indicss[i],40) = j;
        }
    }
}

/*
* 返回:
*    camlidar: 在相机视场范围内的雷达检测结果3维
*    indicss : camlidar在所有雷达检测结果里的索引
*    infor_list : 图像检测到，雷达没检测到的图像反映射结果
*    output_index : 融合成功的雷达信息(在camlidar里的索引)
*    lidtovid_inf : 雷达检测到，图像没检测到的3维 + 2维信息
*    output_infor : 融合成功的图像信息
*/
// Fusion_Res Fusion_Algorithm::output_result() {
//     Fusion_Res output;
//     output.pc_res = _pc_ret;
//     output.video_res = _video_ret;
//     output.PeopleCount = _people_count;
//     output.lidar_box_vector = _lidar_box_vector;
//     return output;
// }

//小目标图像映射及结果过滤
xt::xarray<float> Fusion_Algorithm_crossing::littleTargetMap(xt::xarray<float> &ret)
{
    xt::xarray<float> video_coordinate_temp = xt::empty<float>({0,(int)ret.shape(1)});
    xt::xarray<float> LittleTarget_ret = xt::empty<float>({0,(int)ret.shape(1)});
    for (int k = 0; k < ret.shape(0); ++k)
    {
        if (ret(k,7) == 1 || ret(k,7) == 2 || ret(k,7) == 4 || ret(k,7) == 7 || ret(k,7) == 8)
        {
            xt::xarray<float> temp1 = xt::view(ret,k,xt::all());
            temp1.reshape({1,temp1.size()});
            LittleTarget_ret = xt::concatenate(xt::xtuple(LittleTarget_ret,temp1),0);
        }
    }

    xt::xarray<float> dis_matrix = xt::zeros<float>({(int)_video_coordinate.shape(0),(int)LittleTarget_ret.shape(0)});
    for (int i = 0; i < _video_coordinate.shape(0); ++i)
    {
        for (int j = 0; j < LittleTarget_ret.shape(0); ++j)
        {
            float dis_x = _video_coordinate(i,0)-LittleTarget_ret(j,0);
            float dis_y = _video_coordinate(i,1)-LittleTarget_ret(j,1);
            float dis_temp = sqrt(pow(dis_x,2)+pow(dis_y,2));
            xt::view(dis_matrix,i,j) = dis_temp;
        }
    }
    // auto matches_distance = run_hungarian_match(dis_matrix);
    auto matches_distance = run_hungarian_match1(dis_matrix);

    for (int k = 0; k < matches_distance.shape(0); ++k) {
        int i = matches_distance(k, 0);
        int j = matches_distance(k, 1);
        if (dis_matrix(i, j) < -4.0)
        {
            xt::xarray<float> list_temp = xt::view(_video_coordinate,i,xt::all());
            list_temp = list_temp.reshape({1, list_temp.size()});
            video_coordinate_temp = xt::concatenate(xt::xtuple(video_coordinate_temp, list_temp), 0);
        }
    }
    return video_coordinate_temp;
}
