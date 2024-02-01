//
// Created by root on 5/8/21.
//

#include "coordinate_map_crossing.h"
//#include "CommonDef.h"
// #include "C"


void Trim_crossing(std::string& str) 
{
    str.erase(0, str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
}


// Coordinate_Map::Coordinate_Map(int camera_id, bool debug_sign, TSelfFusionAlgParam* p_pAlgParam):
//  _input_frame_W(p_pAlgParam->m_pFusionAlgParam.m_compressImg_W),
//  _input_frame_H(p_pAlgParam->m_pFusionAlgParam.m_compressImg_H),
//  _project_frame_W(p_pAlgParam->m_pFusionAlgParam.m_originalImg_W),
//  _project_frame_H(p_pAlgParam->m_pFusionAlgParam.m_originalImg_H)
Coordinate_Map_crossing::Coordinate_Map_crossing(int camera_id, bool debug_sign, TSelfFusionAlgParam* p_pAlgParam)
{
    YAML::Node fusion_cfg = YAML::LoadFile(p_pAlgParam->m_strRootPath + p_pAlgParam->m_strFusionCfgPath);
    _input_frame_W = fusion_cfg["CROSSING_FUSION"]["COMPRESS_IMG_W"].as<int>();
    _input_frame_H = fusion_cfg["CROSSING_FUSION"]["COMPRESS_IMG_H"].as<int>();
    _project_frame_W = fusion_cfg["CROSSING_FUSION"]["ORIGINAL_IMG_W"].as<int>();
    _project_frame_H = fusion_cfg["CROSSING_FUSION"]["ORIGINAL_IMG_H"].as<int>();
    auto cam_param_struct = _get_cam_param(camera_id, p_pAlgParam); //TODO
    _in_param = cam_param_struct.in_parameter;
    _rotate_vec = cam_param_struct.rotate_matrix;
    _translation_vec = cam_param_struct.translation_matrix;
    _dist_vec = cam_param_struct.dist_matrix;

    _get_tf_lidar_to_cam();
}

Coordinate_Map_crossing::Coordinate_Map_crossing() {}

/* 
*  3d投影到2d的转换参数 相机内外参：
*  in_parameter       ：相机内参
*  rotate_matrix      ：旋转参数  弧度
*  translation_matrix ：平移参数  毫米
*  dist_matrix        ：畸变系数
*/
get_cam_param_res Coordinate_Map_crossing::_get_cam_param(int cameraID, TSelfFusionAlgParam* p_pAlgParam) 
{
    get_cam_param_res out_put;
    xt::xarray<float> in_parameter;
    xt::xarray<float> rotate_matrix;
    xt::xarray<float> translation_matrix;
    xt::xarray<float> dist_matrix;
    // p_pAlgParam->m_pCameraParam->m_vecCameraDev()[]
    if( p_pAlgParam->m_stCameraParam.m_vecCameraDev.size() >= cameraID)
    {
        in_parameter = {{p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecInParameter[0],
                         p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecInParameter[1],
                         p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecInParameter[2]}, 
                        {p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecInParameter[3],
                         p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecInParameter[4],
                         p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecInParameter[5]}, 
                        {p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecInParameter[6],
                         p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecInParameter[7],
                         p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecInParameter[8]}};

        rotate_matrix = {p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecRotateMatrix[0],
                         p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecRotateMatrix[1] ,
                         p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecRotateMatrix[2]};

        translation_matrix = {p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecTranslationMatrix[0],
                              p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecTranslationMatrix[1],
                              p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecTranslationMatrix[2]};

        dist_matrix = {p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecDistMatrix[0],
                       p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecDistMatrix[1],
                       p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecDistMatrix[2],
                       p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecDistMatrix[3],
                       p_pAlgParam->m_stCameraParam.m_vecCameraDev[cameraID-1].m_vecDistMatrix[4]};
    }
    out_put.in_parameter = in_parameter;
    out_put.rotate_matrix = rotate_matrix;
    out_put.translation_matrix = translation_matrix;
    out_put.dist_matrix = dist_matrix;

    return out_put;
}

// 获取激光雷达到相机的变换矩阵
void Coordinate_Map_crossing::_get_tf_lidar_to_cam() 
{
    std::vector<float> rotate_vec(3);
    for (int i = 0; i < _rotate_vec.size(); ++i) 
    {
        rotate_vec[i] = _rotate_vec(i);
    }
    cv::Mat rotate_mat;
    cv::Rodrigues(rotate_vec, rotate_mat);

    _tf_lidar_to_cam = xt::zeros<float>({4, 4});
    for (int x = 0; x < rotate_mat.rows; ++x) 
    {
        for (int y = 0; y < rotate_mat.cols; ++y) 
        {
            _tf_lidar_to_cam(x, y) = rotate_mat.at<float>(x, y);
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        _tf_lidar_to_cam(i, 3) = _translation_vec(i);
    }
    _tf_lidar_to_cam(3, 3) = 1;
}



//将激光雷达坐标转换到相机坐标系下
xt::xarray<float> Coordinate_Map_crossing::lidar_to_cam(xt::xarray<float> xyz_lidar) {
    xyz_lidar = 1000 * xyz_lidar;  //1000是固定的吗
    xyz_lidar = xt::hstack(xt::xtuple(xyz_lidar, xt::ones<float>({int (xyz_lidar.shape(0)), 1})));
    xt::xarray<float> res;
    res = xt::linalg::dot(_tf_lidar_to_cam, xt::transpose(xyz_lidar));
    res = xt::view(res, xt::range(0, 3));
    res = xt::transpose(res);
    return res;
}

// xtensor 转换至 cv::mat
cv::Mat Coordinate_Map_crossing::xarray_to_mat_elementwise(xt::xarray<float> &xarr)
{
    int ndims = xarr.dimension();
    assert(ndims == 2  && "can only convert 2d xarrays");
    int nrows = xarr.shape()[0];
    int ncols = xarr.shape()[1];
    cv::Mat mat(nrows, ncols, CV_32FC1);
    for (int rr=0; rr<nrows; rr++)
    {
        for (int cc=0; cc<ncols; cc++)
        {
            mat.at<float>(rr, cc) = xarr(rr, cc);
        }
    }
    return mat;
}

// 3d点云投影到2d
xt::xarray<float> Coordinate_Map_crossing::project_points(xt::xarray<float> point3d) {
    point3d = 1000 * point3d;                       //m(pccloud)->mm(camera)
    std::vector<cv::Point3f> object_points(point3d.shape(0));
    for (int i = 0; i < point3d.shape(0); ++i)
        object_points[i] = cv::Point3f(point3d(i, 0), point3d(i, 1), point3d(i, 2));

    cv::Mat rVec(3, 1, cv::DataType<float>::type); // Rotation vector
    rVec.at<float>(0) = _rotate_vec(0);
    rVec.at<float>(1) = _rotate_vec(1);
    rVec.at<float>(2) = _rotate_vec(2);

    cv::Mat tVec(3, 1, cv::DataType<float>::type); // Translation vector
    tVec.at<float>(0) = _translation_vec(0);
    tVec.at<float>(1) = _translation_vec(1);
    tVec.at<float>(2) = _translation_vec(2);

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

    xt::xarray<float> point2d = xt::zeros<float>({int (projectedPoints.size()), 2});
    for (int i = 0; i < projectedPoints.size(); ++i) {
        point2d(i, 0) = projectedPoints[i].x;
        point2d(i, 1) = projectedPoints[i].y;
    }
    point2d = xt::squeeze(point2d);
    return point2d;
}


/*
* param:
*       img2d_list : 点云映射框八个角点
* return：
*       矩形检测框(左上右下点二维坐标)
*/
xt::xarray<float> Coordinate_Map_crossing::lidar_box_calculation(xt::xarray<float> img2d_list) {
    int64_t xmin = int64_t(xt::amin(xt::view(img2d_list, xt::all(), 0))(0) / _project_frame_W * _input_frame_W);
    int64_t xmax = int64_t(xt::amax(xt::view(img2d_list, xt::all(), 0))(0) / _project_frame_W * _input_frame_H);
    int64_t ymin = int64_t(xt::amin(xt::view(img2d_list, xt::all(), 1))(0) / _project_frame_H * _input_frame_H);
    int64_t ymax = int64_t(xt::amax(xt::view(img2d_list, xt::all(), 1))(0) / _project_frame_H * _input_frame_H);

    xmin = xmin < 0 ? 0 : xmin;
    ymin = ymin < 0 ? 0 : ymin;
    xmax = xmax > (_input_frame_W-1) ? (_input_frame_W-1) : xmax;
    ymax = ymax > (_input_frame_H-1) ? (_input_frame_H-1) : ymax;
    xt::xarray <int64_t> lidar_box = {{xmin, ymin, xmax, ymax}};
    return xt::cast<float>(lidar_box);
}