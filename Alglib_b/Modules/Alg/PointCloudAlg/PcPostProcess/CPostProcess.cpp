//
// Created by root on 5/7/21.
//
#include <iostream>
#include <math.h>
#include "CPostProcess.h"
/***************************************************************
 * @file       pc_preprocess.cpp
 * @brief      点云处理对象的构造函数
 * @input      AlgParams: 参数结构体指针
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
CPostProcess::~CPostProcess(){;}
CPostProcess::CPostProcess(TSelfPcAlgParam *p_pAlgParams, int p_nId)
{
    m_usType = 0x4d42;
    m_tAlgParams = *p_pAlgParams;
    m_nLidarId = p_nId;
    m_bPcProType = p_pAlgParams->m_stPcAlgParam.m_bPcProType;
    // std::cout<< " bmp read start"<<std::endl;
    // std::cout << "--zqj-debug-alg-pc_config_yaml:" << p_pAlgParams->m_stPcAlgParam.m_strCfgPath << std::endl;
 
    // if (!m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_strBmpPath1.empty())   
    // {
    //     m_egBmpMat1 = init_bmp(m_tAlgParams.m_stPcAlgParam.m_strBmpPath + m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_strBmpPath1);
    //     std::cout<< "--zqj-debug-alg-bmp1 read success " << m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_strBmpPath1 << std::endl;
    // }
    // else
    // {
    //     std::cout<< "--zqj-debug-alg-bmp1 read failed"<<std::endl;
    // }
    

    // if (m_tAlgParams.m_stPcAlgParam.m_bPcProType && !m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_strBmpPath2.empty())   
    // {
    //     m_egBmpMat2 = init_bmp(m_tAlgParams.m_stPcAlgParam.m_strBmpPath + m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_strBmpPath2);
    //     std::cout<< "--zqj-debug-alg-bmp2 read success "<< m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_strBmpPath2 << std::endl;
    // }
    // else
    // {
    //     std::cout<< "--zqj-debug-alg-bmp2 read failed"<<std::endl;
    // }

    if (m_tAlgParams.m_stPcAlgParam.m_bPcProType && !m_tAlgParams.m_stPcAlgParam.m_strSplitBmpPath.empty())   
    {
        m_egSplitBmpMat = init_bmp(m_tAlgParams.m_stPcAlgParam.m_strBmpPath + m_tAlgParams.m_stPcAlgParam.m_strSplitBmpPath);
        std::cout<< "--zqj-debug-alg-split bmp read success "<< m_tAlgParams.m_stPcAlgParam.m_strBmpPath + m_tAlgParams.m_stPcAlgParam.m_strSplitBmpPath << std::endl;
    }
    else
    {
        std::cout<< "--zqj-debug-alg split bmp read failed"<<std::endl;
    }
    // mosaic_r
    m_xaSplitRotMat1 = init_rotate(m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans1, true);
    m_xaSplitRotMat2 = init_rotate(m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans2, true);

}

/***************************************************************
 * @brief      初始化bmp图
 * @input      p_strBmpPath: BMP圖路徑 
 * @author     陈星谕
 * @date       2022.10.12
 **************************************************************/
Eigen::Tensor<uint8_t, 3> CPostProcess::init_bmp(const std::string &p_strBmpPath)
{
    Eigen::Tensor<uint8_t, 3> l_egBmpMat;
    FILE * fpBMP;
    if ((fpBMP = fopen(p_strBmpPath.c_str(), "rb")) == NULL) {
        printf("open the image failed : %s\n", p_strBmpPath.c_str());
        exit(0);
    }
    //从bmp文件中读取数据块给文件信息头和图片信息头
    bmp_file_header fileheader={0}; 
    fread(&fileheader.type,2,1,fpBMP); 
    fread(&fileheader.size,4,1,fpBMP); 
    fread(&fileheader.reserved1,2,1,fpBMP); 
    fread(&fileheader.reserved2,2,1,fpBMP); 
    fread(&fileheader.offset,4,1,fpBMP); 

    if(fileheader.type!=0x4D42) 
    {
        fclose(fpBMP); 
        return l_egBmpMat; 
    }
 
    fread(&infoHeader,sizeof(bmp_info_header),1,fpBMP);  
    long bmpWidth = infoHeader.biWidth; 
    long bmpHeight = infoHeader.biHeight; 
    if(infoHeader.biBitCount != 24) 
    {
        fclose(fpBMP); 
        return l_egBmpMat; 
    }

    int totalSize = (bmpWidth * infoHeader.biBitCount / 8 + 3) / 4 * 4 * bmpHeight; 
    char *pBmpBuf = new char[totalSize]; 
    size_t size = 0; 
    while(true) 
    {
        int iret = fread(&pBmpBuf[size],1,1,fpBMP); 
        if(iret == 0) 
            break; 
        size = size + iret; 
    }
    fclose(fpBMP);

    l_egBmpMat = Eigen::Tensor<uint8_t, 3>(infoHeader.biHeight, infoHeader.biWidth, 3);
    
    int pitch = bmpWidth % 4; 
    for(int i = 0; i < infoHeader.biHeight; i++) 
    { 
        int realPitch = i * pitch; 
        for(int j = 0; j < infoHeader.biWidth; j++) 
        {
            l_egBmpMat(i, j, 0) = pBmpBuf[(i*bmpWidth+j)*3+2+realPitch];
            l_egBmpMat(i, j, 1) = pBmpBuf[(i*bmpWidth+j)*3+1+realPitch];
            l_egBmpMat(i, j, 2) = pBmpBuf[(i*bmpWidth+j)*3+realPitch];
        }
    }
    delete [] pBmpBuf; pBmpBuf = NULL;
    return l_egBmpMat;
}

/***************************************************************
 * @brief      初始化旋轉矩陣
 * @input      
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
xt::xarray<float> CPostProcess::init_rotate(float p_fRotTrans[6], bool p_bConvert)
{
    xt::xarray<float> l_xaRotMat;
    double pi = std::acos(-1);
    float rotate_x;
    float rotate_y;
    float rotate_z;
    if (p_bConvert)
    {
        rotate_x = -p_fRotTrans[0] * pi / 180.0;
        rotate_y = -p_fRotTrans[1] * pi / 180.0;
        rotate_z = -p_fRotTrans[2] * pi / 180.0;
    }
    else
    {
        rotate_x = p_fRotTrans[0] * pi / 180.0;
        rotate_y = p_fRotTrans[1] * pi / 180.0;
        rotate_z = p_fRotTrans[2] * pi / 180.0;
    }

    xt::xarray<float> R_x = {{1.0, 0.0, 0.0},
                             {0.0, std::cos(rotate_x), -1 * std::sin(rotate_x)},
                             {0.0, std::sin(rotate_x), std::cos(rotate_x)}};
    xt::xarray<float> R_y = {{std::cos(rotate_y), 0.0, std::sin(rotate_y)},
                             {0.0, 1.0, 0.0},
                             {-1 * std::sin(rotate_y), 0.0, std::cos(rotate_y)}};
    xt::xarray<float> R_z = {{std::cos(rotate_z), -1 * std::sin(rotate_z), 0.0},
                             {std::sin(rotate_z), std::cos(rotate_z), 0.0},
                             {0.0, 0.0, 1.0}};
    l_xaRotMat = xt::linalg::dot(R_z, R_y);
    l_xaRotMat = xt::linalg::dot(l_xaRotMat, R_x);

    return l_xaRotMat;
}


void CPostProcess::BoxSplit(float& x, float& y, float& z, float& angle)
{
    xt::xarray<float> l_xaBoxAsix = xt::xarray<float>({x, y, z});
    // xt::xarray<float> l_xaBoxAsix = xt::xarray<float>({-34.990505,   8.899934,  10.}); // zqj 测试点{10,10,10}的反旋转平移
    bool l_bInFirst = true;
    int l_nImgSize = m_egSplitBmpMat.dimension(0);

    int img_corner = l_nImgSize / 2;
    int dist2pixel = 5; // edit by zqj20230727 for 400m
    int xindex = std::ceil(l_xaBoxAsix(0, 0) * dist2pixel) + img_corner - 1;
    int line_index = std::ceil(l_xaBoxAsix(0, 1) * dist2pixel) + img_corner - 1;
    if (xindex >= l_nImgSize || line_index >= l_nImgSize || xindex <= 0 || line_index <= 0){  // zqj 点不在bmp图内

    }
    else if (m_egSplitBmpMat(line_index, xindex, 2) == 255)
    {
        l_bInFirst = false;  // 对应的是bmp2
    }
    if (l_bInFirst)
    {
        xt::view(l_xaBoxAsix, xt::all(), 0) -= m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans1[3];
        xt::view(l_xaBoxAsix, xt::all(), 1) -= m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans1[4];
        xt::view(l_xaBoxAsix, xt::all(), 2) -= m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans1[5];
        xt::xarray<float> point_ = xt::transpose(xt::view(l_xaBoxAsix, xt::all(), xt::range(0, 3)));
        xt::xarray<float> point_temp = xt::linalg::dot(m_xaSplitRotMat1, point_);
        // x = point_temp(0, 0);  // zqj for x y
        // y = point_temp(1, 0);
        // z = point_temp(2, 0);
        // std::cout<< "--zqj-debug-alg-point_xyz1:"  << point_temp << std::endl;
        x = point_temp(0, 0);
        y = point_temp(0, 1);
        z = point_temp(0, 2);
        // std::cout<< "--zqj-debug-alg-point_r_old:"  << angle << std::endl;
        angle -= m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans1[2];  // m_fSelfRotTrans  22 m_fRangeRotTrans1
        // std::cout<< "--zqj-debug-alg-point_xyz1:"  << m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans1[2] << std::endl;
        // std::cout<< "--zqj-debug-alg-point_r_new:"  << angle << std::endl;
    }
    else
    {
        xt::view(l_xaBoxAsix, xt::all(), 0) -= m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans2[3];
        xt::view(l_xaBoxAsix, xt::all(), 1) -= m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans2[4];
        xt::view(l_xaBoxAsix, xt::all(), 2) -= m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans2[5];
        xt::xarray<float> point_ = xt::transpose(xt::view(l_xaBoxAsix, xt::all(), xt::range(0, 3)));
        xt::xarray<float> point_temp = xt::linalg::dot(m_xaSplitRotMat2, point_);
        // x = point_temp(0, 0);
        // y = point_temp(1, 0);
        // z = point_temp(2, 0);
        // std::cout<< "--zqj-debug-alg-point_xyz2:"  << point_temp << std::endl;
        x = point_temp(0, 0);
        y = point_temp(0, 1);
        z = point_temp(0, 2);
        // std::cout<< "--zqj-debug-alg-point_r_old2:"  << angle << std::endl;
        angle -= m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans2[2];  // m_fSelfRotTrans  22 m_fRangeRotTrans1
        // std::cout<< "--zqj-debug-alg-point_xyz2:"  << m_tAlgParams.m_stLidarParam.m_vecLidarDev[m_nLidarId].m_fRangeRotTrans2[2] << std::endl;
        // std::cout<< "--zqj-debug-alg-point_r_new2:"  << angle << std::endl;
    }
}

// ***************************************** nms for mosaic end *************************************************************************
xt::xarray<float> box_ops_nms::corners_nd(xt::xarray<float> &dims, float origin) 
{
    int ndim = dims.shape(1);
    auto shape = xt::ones<int>({ndim}) * 2;
    xt::xarray<float> corners_norm;
    if (ndim == 2)
    {
        corners_norm = xt::zeros<float>({4,2});
        corners_norm(1, 1) = 1;
        corners_norm(2, 0) = 1;
        corners_norm(3, 0) = 1;
        corners_norm(2, 1) = 1;
    } else
    {
        corners_norm = xt::zeros<float>({8,3});
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
    dims = dims.reshape({int (dims.size() / ndim), 1, ndim});
    corners_norm = corners_norm.reshape({1, int(std::pow(2, ndim)), ndim});
    xt::xarray<float> res = xt::ones<float>({int (dims.shape(0)),int(std::pow(2, ndim)), ndim});
    for (int i = 0; i < res.shape(0); ++i) 
    {
        xt::view(res, i) = xt::view(dims, i) * xt::view(corners_norm, 0);
    }
    return res;
}

/*  rotation 2d points based on origin point clockwise when angle positive.
*
*   Args:
*       points(float array, shape = [N, point_size, 2]) : points to be rotated.
*       angles(float array, shape = [N]) : rotation angle.
*
*   Returns :
*       float array : same shape as points
*/
xt::xarray<float> box_ops_nms::rotation_2d(xt::xarray<float> &corners, const xt::xarray<float> &angles) 
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
    for (int i = 0; i < corners.shape(0); ++i) {
        xt::xarray<float> a_ = xt::view(corners, i);
        xt::xarray<float> b_ = xt::view(rot_mat_T, xt::all(), xt::all(), i);
        xt::view(corners, i) = xt::linalg::dot(a_, b_);
    }
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
xt::xarray<float> box_ops_nms::center_to_corner_box2d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                               xt::xarray<float> &angles, float origin) 
{
    xt::xarray<float> corners = box_ops_nms::corners_nd(dims, origin);

    if (angles.size())
        corners = box_ops_nms::rotation_2d(corners, angles);

    centers.reshape({-1, 1, 2});
    corners += centers;
    return corners;
}

/*  convert kitti locations, dimensions and angles to corners
*   Args:
*       centers(float array, shape = [N, 3]) : locations in kitti label file.
*       dims(float array, shape = [N, 3])    : dimensions in kitti label file.
*       angles(float array, shape = [N])     : rotation_y in kitti label file.
*       origin(list or array or float)       : origin point relate to smallest point.
*       use[0.5, 1.0, 0.5] in camera and [0.5, 0.5, 0] in lidar.
*       axis(int) : rotation axis. 1 for camera and 2 for lidar.
*   Returns :
*       [type] : [description]
*/
xt::xarray<float> box_ops_nms::center_to_corner_box3d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                                  float &angles, float origin, int axis) 
{
    angles = angles * M_PI / 180.0;
    xt::xarray<float> corners = box_ops_nms::corners_nd(dims, origin);
    if (angles != float (None))
        box_ops_nms::rotation_3d_in_axis(corners, angles, axis);//angles需要弧度
    angles = angles * 180.0 / M_PI;
    centers = centers.reshape({int (centers.size() / 3), 1, 3});
    return corners + centers;
}

xt::xarray<float> box_ops_nms::corner_to_standup_nd(xt::xarray<float> &boxes_corner) 
{
    xt::xarray<int> box_shape = xt::adapt(boxes_corner.shape());
    assert(box_shape.size() == 3);
    xt::xarray<float> standup_boxes1 = xt::amin(boxes_corner, 1);;
    xt::xarray<float> standup_boxes2 = xt::amax(boxes_corner, 1);
    return xt::concatenate(xt::xtuple(standup_boxes1, standup_boxes2), 1);
}

std::pair<xt::xarray<float>, xt::xarray<float>> box_ops_nms::rotate_nms_cc(xt::xarray<float> &dets,
                                                                    xt::xarray<float> &trackers) 
{
    xt::xarray<float> t1 = xt::view(trackers, xt::all(), xt::range(0, 2));  // zqj x y
    xt::xarray<float> t2 = xt::view(trackers, xt::all(), xt::range(3, 5));  // w h
    xt::xarray<float> t3 = xt::view(trackers, xt::all(), 6);                // 航向角
    xt::xarray<float> trackers_corners = box_ops_nms::center_to_corner_box2d(t1, t2, t3);
    xt::xarray<float> trackers_standup = box_ops_nms::corner_to_standup_nd(trackers_corners);
    xt::xarray<float> d1 = xt::view(dets, xt::all(), xt::range(0, 2));
    xt::xarray<float> d2 = xt::view(dets, xt::all(), xt::range(3, 5));
    xt::xarray<float> d3 = xt::view(dets, xt::all(), 6);

    xt::xarray<float> dets_corners = box_ops_nms::center_to_corner_box2d(d1, d2, d3);
    xt::xarray<float> dets_standup = box_ops_nms::corner_to_standup_nd(dets_corners);
    return box_ops_nms::iou_jit_new(dets_standup, trackers_standup, 0.0);
}

float box_ops_nms::cal_angle(std::vector<xt::xarray<float>> &state_list, float &thresh) 
{
    auto dis_x = state_list[state_list.size() - 1](0, 0) - state_list[0](0, 0);
    auto dis_y = state_list[state_list.size() - 1](0, 1) - state_list[0](0, 1);
    auto dis_len = std::sqrt(dis_x * dis_x + dis_y * dis_y);
    float dis_angle;
    if (dis_len > thresh){
        dis_angle = std::acos(dis_x / dis_len) * 180 / M_PI - 180;
        if (dis_y > 0)
            dis_angle = 90 - dis_angle;
        else
            dis_angle = 90 - (360 - dis_angle);
        dis_angle = std::fmod(dis_angle, 360.0);
    } else{
        dis_angle = None;
    }
    return dis_angle;
}

std::pair<xt::xarray<float>, xt::xarray<float>> box_ops_nms::iou_jit_new(xt::xarray<float> &boxes,
                                                                     xt::xarray<float> &query_boxes, float eps) 
{
    xt::xarray<double> d_1 = xt::expand_dims(query_boxes, 0); // shape1
    xt::xarray<double> d_0 = xt::expand_dims(boxes, 1);       // shape0

    xt::xarray<double> xx1 = xt::maximum(xt::view(d_1, xt::all(), xt::all(), 0),
                                         xt::view(d_0, xt::all(), xt::all(), 0));

    xt::xarray<double> yy1 = xt::maximum(xt::view(d_1, xt::all(), xt::all(), 1),
                                         xt::view(d_0, xt::all(), xt::all(), 1));

    xt::xarray<double> xx2 = xt::minimum(xt::view(d_1, xt::all(), xt::all(), 2),
                                         xt::view(d_0, xt::all(), xt::all(), 2));

    xt::xarray<double> yy2 = xt::minimum(xt::view(d_1, xt::all(), xt::all(), 3),
                                         xt::view(d_0, xt::all(), xt::all(), 3));

    xt::xarray<double> w = xt::maximum(0.0, xx2 - xx1);
    xt::xarray<double> h = xt::maximum(0.0, yy2 - yy1);
    xt::xarray<double> inter_area = w * h;

    xt::xarray<double> union_ =     // zqj 并集即等于各自面积减重叠
            (xt::view(d_1, xt::all(), xt::all(), 2) - xt::view(d_1, xt::all(), xt::all(), 0)) *
            (xt::view(d_1, xt::all(), xt::all(), 3) - xt::view(d_1, xt::all(), xt::all(), 1)) +
            (xt::view(d_0, xt::all(), xt::all(), 2) - xt::view(d_0, xt::all(), xt::all(), 0)) *
            (xt::view(d_0, xt::all(), xt::all(), 3) - xt::view(d_0, xt::all(), xt::all(), 1)) - inter_area;

    xt::xarray<double> union_new =  // zqj 求d_1的面积是什么作用
            (xt::view(d_1, xt::all(), xt::all(), 2) - xt::view(d_1, xt::all(), xt::all(), 0)) *
            (xt::view(d_1, xt::all(), xt::all(), 3) - xt::view(d_1, xt::all(), xt::all(), 1)) ;

    xt::xarray<double> iou_mat = inter_area / union_;
    xt::xarray<double> iou_mat_new = inter_area / union_new;
    return {iou_mat, iou_mat_new};
}

void box_ops_nms::rotation_3d_in_axis(xt::xarray<float> &points, float &angles, int axis) 
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
    else{
        throw "axis should in range";
    }

    rot_mat_T = rot_mat_T.reshape({3, 3, 1});

    for (int i = 0; i < points.shape(0); ++i) {
        xt::xarray<float> a_ = xt::view(points, i);
        xt::xarray<float> b_ = xt::view(rot_mat_T, xt::all(), xt::all(), i);
        xt::view(points, i) = xt::linalg::dot(a_, b_);
    }
}
// ********************************** nms for mosaic end *********************************************************************


// ********************************** filter boxes start *********************************************************************
/***************************************************************
 * @brief      filter box by some keys
 * @input      labels 
 * @author     ***
 * @date       ***
 * @cite       isfp  (note:it)
 **************************************************************/
// 过滤检测结果 nuscense
// 过滤检测结果 nuscense
std::vector<int> filter_label_forNuScenes(std::vector<int> &labels,xt::xarray<float> &boxes_lidar, std::vector<float> &scores,xt::xarray<int> &num_point_per_boxes, xt::xarray<float> &height_area,  int mode,std::vector<float> point_cloud_range){     //, cv::Mat &img
    std::vector<int> taked_indics(labels.size(), 0);
    int count_idx = 0;
    int num_labels = labels.size();

    //int im_size = img.size[0];
    // int img_corner = im_size / 2;
    // int dist2pixel = im_size / 400;

    //    ##0:car,1:bike,2:bus,3:tricycle,4:pedestrian,5:semitrailer, 6:truck
    if (mode == 0) {
        for (int idx = 0; idx < num_labels; ++idx) {
            // std::cout<<"================="<<std::endl;
            // std::cout<<height_area(idx, 1)<<":"<<num_point_per_boxes[idx]<<std::endl;
            // std::cout<<height_area(idx, 1)<<":"<<point_cloud_range[2]<<std::endl;
            float size_box = boxes_lidar(idx, 3) * boxes_lidar(idx, 4);
            // std::cout<<"==========="<<labels[idx]<<"=============="<<std::endl;
            if((height_area(idx, 1) - point_cloud_range[2])<5 and (num_point_per_boxes[idx]>=3))
            {
                if (labels[idx] == 0 and ((scores[idx] >= 0.22 and num_point_per_boxes[idx] >=8) || (scores[idx] >= 0.5))){
                    if (size_box > 6.6){
                        taked_indics[count_idx] = idx;
                        count_idx += 1;
                    //}
                        if (boxes_lidar(idx, 3)>6 || boxes_lidar(idx, 4)>6){
                        labels[idx] = 6;
                    }
                    }
                    // int xindex = std::ceil(boxes_lidar(idx, 0) * dist2pixel) + img_corner - 1;
                    // int line_index = std::ceil(boxes_lidar(idx, 1) * dist2pixel) + img_corner - 1;
                    // if (xindex >= im_size || line_index >= im_size || xindex <= 0 || line_index <= 0){
                    //     continue;
                    // }
                    // else if (img.ptr<cv::Vec3b>(line_index)[xindex][2] == 255){
       
                    // }
                    // else{


                } else if (labels[idx] == 1 and scores[idx] >= 0.12 and num_point_per_boxes[idx] >=4) {
                    if (size_box< 2 || size_box>0.5){
                    if((abs(boxes_lidar(idx,0))>50 or abs(boxes_lidar(idx,1))>50) and num_point_per_boxes[idx] >=8){
                    taked_indics[count_idx] = idx;
                    count_idx += 1; 
                    }
                    else if (scores[idx] >= 0.2 ){
                    taked_indics[count_idx] = idx;
                    count_idx += 1;
                    }
                    if (size_box < 0.75){
                        labels[idx] = 4;
                    }

                    }

                    
                } else if (labels[idx] == 2 and scores[idx] >= 0.2 and num_point_per_boxes[idx] >=10) {
                    if (size_box > 8 or scores[idx]>0.4)
                    {
                    taked_indics[count_idx] = idx;
                    count_idx += 1;

                    }
                    
                } else if (labels[idx] == 3 and scores[idx] >= 0.15 and num_point_per_boxes[idx] >=4) {
                    if (size_box>1.2 or scores[idx]>0.3)
                    {
                    if((abs(boxes_lidar(idx,0))>50 or abs(boxes_lidar(idx,1))>50) and num_point_per_boxes[idx] >=8){
                    taked_indics[count_idx] = idx;
                    count_idx += 1; 
                    }
                    else if (scores[idx] >= 0.2){
                    taked_indics[count_idx] = idx;
                    count_idx += 1;
                    }  
                    }
                    
                } else if (labels[idx] == 4 and scores[idx] >= 0.12 and num_point_per_boxes[idx] >=3 ) {    //and boxes_lidar(idx,2)<-4.5
                    if((abs(boxes_lidar(idx,0))>50 or abs(boxes_lidar(idx,1))>50) and (scores[idx] <= 0.25 or num_point_per_boxes[idx]<=5)){
                        continue;
                    }    
                    else if (num_point_per_boxes[idx] >=5 and size_box<1.5){
                        taked_indics[count_idx] = idx;
                        count_idx += 1;
                    }
                } else if (labels[idx] == 5 and scores[idx] >= 0.2 and num_point_per_boxes[idx] >=8) {
                    taked_indics[count_idx] = idx;
                    count_idx += 1;
                } else if (labels[idx] == 6 and scores[idx] >= 0.2 and num_point_per_boxes[idx] >=8) {
                    if (size_box>5 || scores[idx] >= 0.3)
                    {
                    taked_indics[count_idx] = idx;
                    count_idx += 1;
                    }
                    

                }
            }

        }
    }


    taked_indics.erase(taked_indics.begin() + count_idx, taked_indics.end());
    return taked_indics;
}





std::vector<int> filter_label_for_WJ(std::vector<int> &labels, xt::xarray<float> &pc_boxes, std::vector<float> &scores, std::vector<int> &point_num,
                                     std::vector<float> &heights, xt::xarray<float> &area)
{
    // code  paras: label point_box score point_nem hegit area
    // 后处理参数 ['car:0', 'bicycle:1', 'bus:2', 'tricycle:3', 'pedestrian:4', 'semitrailer:5', 'truck:6']
    float class_point_count_thresh[7] = {8, 6, 18, 6, 6, 15, 20};
    // float box_avg_height_thresh[8] = {-6.0, -6.0, -6.0, -6.0, -6.0, -6.0, -4.2, 0.8};
    float box_avg_height_thresh[8] = {-8.0, -8.0, -8.0, -8.0, -8.0, -8.0, -8.2, 0.8};
    float class_high_score_thresh[7] = {0.5, 0.4, 0.5, 0.4, 0.4, 0.5, 0.5};
    // float class_low_score_thresh[7] = {0.22, 0.15, 0.3, 0.15, 0.15, 0.2, 0.23};
    float class_low_score_thresh[7] = {0.22, 0.15, 0.2, 0.15, 0.15, 0.2, 0.2};
    float class_box_len[14] = {3.5, 5.8, 2.0, 2.8, 6.0, 15.0, 1.8, 3.5, 0.5, 1.5, 12.0, 16.0, 6.0, 15.0};
    float box_point_heights[14] = {1.2, 2.5, 2.3, 2.8, 2.5, 4.0, 1.2, 2.8, 0.8, 2.0, 2.0, 4.0, 2.8, 4.0};
    int num_labels = labels.size();
    std::vector<int> taked_indics(num_labels, 0);
    int count_idx = 0;
    // **** according label and  score filter box ****
    for (int i = 0; i < num_labels; ++i) 
    {
        float box_len = pc_boxes(i,4);
        float box_heigh = pc_boxes(i,5);
        float box_heigh_diff_max = area(i,1)-area(i,0);
        // std::cout << "--zqj-debug-alg-point_num and heigh_diff:"<< i << "--" << point_num[i] << "--" << box_heigh_diff_max << std::endl;
        // std::cout << "--zqj-debug:"<< pc_boxes(i,0) << "--" << pc_boxes(i,1) << "--" <<pc_boxes(i,2) << "--" <<pc_boxes(i,3) << "--" <<pc_boxes(i,4)<< "--" << pc_boxes(i,5)<< std::endl;
        // **** stage1: according points_num and box_height filter box ****   对所有的框统一过滤  减少计算代价
        if(point_num[i] > 4 and (box_heigh_diff_max > box_avg_height_thresh[7]))
        {
            // // test code 
            // taked_indics[count_idx] = i;
            // count_idx += 1;

            float height_avg = heights[i]/ point_num[i];
            // **** stage2: according label and  score and point_num and heigh and  filter box ****
            if (labels[i] == 0 and scores[i] >= class_low_score_thresh[0] and point_num[i] > class_point_count_thresh[0] and height_avg>box_avg_height_thresh[0])
            {
                if (box_len < class_box_len[0*2+1])   // adjust label
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                }
                else
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                    labels[i] = 6;  // car2truck
                }
                
            }
            else if (labels[i] == 1 and scores[i] >= class_low_score_thresh[1] and point_num[i] > class_point_count_thresh[1] and height_avg>box_avg_height_thresh[1])
            {
                if(scores[i] > class_high_score_thresh[1] or box_len < class_box_len[1*2+1])
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                }
                else if (box_len < class_box_len[3*2+1] and (box_heigh_diff_max > box_point_heights[0*2+0]))  // to do
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                    labels[i] = 3;  // bicycle2tricycle
                }
                else if (box_len > class_box_len[0*2+0] and (box_heigh_diff_max > box_point_heights[0*2+0]))  // to do
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                    labels[i] = 0;  // bicycle2car
                }
                else
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                    labels[i] = 4;  // bicycle2pedestrian
                }
                
            }
            else if (labels[i] == 2 and scores[i] >= class_low_score_thresh[2] and point_num[i] > class_point_count_thresh[2] and height_avg>box_avg_height_thresh[2])
            {
                // std::cout << "-zqj-debug-2-bug:" << i <<std::endl;
                taked_indics[count_idx] = i;
                count_idx += 1;
                if (box_len < class_box_len[0*2+1] or box_heigh < box_point_heights[0*2+1])
                {
                    labels[i] = 0;  // bus2car
                }
            }
            else if (labels[i] == 3 and scores[i] >= class_low_score_thresh[3] and point_num[i] > class_point_count_thresh[3] and height_avg>box_avg_height_thresh[3])
            {
                if (box_len < class_box_len[3*2+1] and (box_heigh_diff_max<box_point_heights[6*2+1] and box_heigh_diff_max>box_point_heights[3*2+1]))
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                }
                else if((box_len < class_box_len[0*2+1] and box_len > class_box_len[0*2+0]) and box_heigh_diff_max<box_point_heights[3*2+1]) // to do
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                    labels[i] = 0;  // bus2car
                }
                else if (box_len > class_box_len[6*2+0] and box_heigh_diff_max>box_point_heights[6*2+0])  // to do
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                    labels[i] = 6;  // bus2truck
                }  
            }
            else if (labels[i] == 4 and scores[i] >= class_low_score_thresh[4] and point_num[i] > class_point_count_thresh[4] and height_avg > box_avg_height_thresh[4])
            {
                if (box_len < class_box_len[4*2+1])
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                }
                else if(box_len < class_box_len[1*2+1] and box_len > class_box_len[1*2+0])
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                    labels[i] = 1;  // pedestrian2bicycle
                }
            }
            else if (labels[i] == 5 and scores[i] >= class_low_score_thresh[5] and point_num[i] > class_point_count_thresh[5] and height_avg>box_avg_height_thresh[4])
            {
                taked_indics[count_idx] = i;
                count_idx += 1;
            }
            else if (labels[i] == 6 and point_num[i] > class_point_count_thresh[6] and height_avg>box_avg_height_thresh[0])
            {
                if (box_len > class_box_len[6*2+0])
                {
                    if (scores[i] >= class_high_score_thresh[6] or (box_heigh_diff_max>box_point_heights[6*2+0] and height_avg>box_avg_height_thresh[6]))
                    {
                        taked_indics[count_idx] = i;
                        count_idx += 1;
                    }
                }
                else if (scores[i] >= class_low_score_thresh[6] and (box_len < class_box_len[0*2+1] and box_len > class_box_len[0*2+0]))
                {
                    taked_indics[count_idx] = i;
                    count_idx += 1;
                    labels[i] = 0;  // truck2bus
                }
            }
            else if (labels[i] == 7 and scores[i] >= class_low_score_thresh[6] and point_num[i] > class_point_count_thresh[6] and height_avg>box_avg_height_thresh[6])  // in python
            {
                taked_indics[count_idx] = i;
                count_idx += 1;
            }
        }
    }
    taked_indics.erase(taked_indics.begin() + count_idx, taked_indics.end());
    return taked_indics;
}

std::vector<int> filter_label_for_KT(std::vector<int> &labels, std::vector<float> &scores, int mode)
{
    // code
    int num_labels = labels.size();
    std::vector<int> taked_indics(num_labels, 0);
    int count_idx = 0;
    // **** according label and  score filter box ****
    if (mode == 0)
    {
        for (int i = 0; i < num_labels; ++i) 
        {
            if (labels[i] == 0 and scores[i] > 0.1){
                taked_indics[count_idx] = i;
                count_idx += 1;
            }
            else if (labels[i] == 1 and scores[i] >= 0.05){
                taked_indics[count_idx] = i;
                count_idx += 1;
            }
            else if (labels[i] == 2 and scores[i] >= 0.05){
                taked_indics[count_idx] = i;
                count_idx += 1;
            }
            else if (labels[i] == 3 and scores[i] >= 0.05){
                taked_indics[count_idx] = i;
                count_idx += 1;
            }
        }
    }
    // **** according label filter box ****
    else if (mode == 1)
    {
        // continued
        for (int i = 0; i < num_labels; ++i) 
        {
            if (labels[i] == 0){
                taked_indics[count_idx]=i;
                count_idx += 1;
            }
            else if (labels[i]==1) {
                taked_indics[count_idx] = i;
                count_idx += 1;
            }
            else if (labels[i] == 2){
                taked_indics[count_idx] = i;
                count_idx+=1;
            }
            else if (labels[i] == 3){
                taked_indics[count_idx]=i;
                count_idx+=1;
            }
        }
    }
    else
    {
        // error
    }
    taked_indics.erase(taked_indics.begin() + count_idx, taked_indics.end());
    return taked_indics;
}


/***************************************************************
 * @brief      count points in box
 * @input      arg1:points
               arg2:boxes
 * @author     ***
 * @date       ***
 * @cite       isfp  (note:it's return is different python  )
 **************************************************************/
std::shared_ptr<Count_Box_Result>  points_count_rbbox(xt::xarray<float> & points, xt::xarray<float> box3d)
{
    // xt::xarray<float> box3d= {{10,10,10,10,10,10,0}};  // zqj  验证边界计算的准确
    xt::xarray<float> out_box = xt::zeros<float>({(int)box3d.shape(0),8, 3});

    for (int i=0; i<box3d.shape(0); i++)
    {
        xt::xarray<float> center = xt::view(box3d,i, xt::range(0,3));
        xt::xarray<float> dims = xt::view(box3d,i, xt::range(3,6));
        dims = dims.reshape({1,3});
        float angles = box3d(i,6);
        xt::view(out_box,i,xt::all(),xt::all()) =xt::view(box_ops_post::center_to_corner_box3d(center,dims,angles),0, xt::all(), xt::all());
    }
    xt::xarray<float> out_box_edge = cal_box_edge(out_box);
    // std::cout << "--zqj-debug-alg-cal_box_edge:" << out_box_edge << "--" << out_box_edge.shape(0) << std::endl;
    // xt::xarray<float> surfaces = corner_to_surfaces_3d(out_box);
    // return points_count_convex_polygon_3d_jit(xt::view(points, xt::all(), xt::range(0,3)), surfaces, out_box_edge);  // in python use edge filter
    return _points_count_convex_polygon_3d_jit_edge(xt::view(points, xt::all(), xt::range(0,3)), box3d.shape(0), out_box_edge);  // added by zqj20230619

}

xt::xarray<float> cal_box_edge(xt::xarray<float> out_box)
{
    // code zqj20230526  计算俯视图最小外接矩形的范围 用于保留框内的点
    xt::xarray<float> out_box_edge;

    xt::xarray<float> box_x_min = xt::amin(xt::view(out_box, xt::all(), xt::all(), 0), -1);   // 1*n   .T?:no member named ‘T’
    xt::xarray<float> box_x_min_keepaxis = box_x_min.reshape({int(box_x_min.shape(0)), 1});   // change diamons to n*1

    xt::xarray<float> box_x_max = xt::amax(xt::view(out_box, xt::all(), xt::all(), 0), -1);
    xt::xarray<float> box_x_max_keepaxis = box_x_max.reshape({int(box_x_max.shape(0)), 1});

    xt::xarray<float> box_y_min = xt::amin(xt::view(out_box, xt::all(),xt::all(), 1), -1);
    xt::xarray<float> box_y_min_keepaxis = box_y_min.reshape({int(box_y_min.shape(0)), 1});
    
    xt::xarray<float> box_y_max = xt::amax(xt::view(out_box, xt::all(), xt::all(), 1), -1);
    xt::xarray<float> box_y_max_keepaxis = box_y_max.reshape({int(box_y_max.shape(0)), 1});

    out_box_edge = xt::concatenate(xt::xtuple(box_x_min_keepaxis,box_x_max_keepaxis, box_y_min_keepaxis, box_y_max_keepaxis), 1);

    return out_box_edge;
}

xt::xarray<float> corner_to_surfaces_3d(xt::xarray<float> corners){
    xt::xarray<float> surfaces = xt::zeros<float>({6, 4, (int)corners.shape(0), 3});
    xt::xarray<float> a= {{1.0,2.0},{2.0,3.0}};
    xt::xarray<int> index = {{0,1,2,3},{7,6,5,4},{0,3,7,4},{1,5,6,2},{0,4,5,1},{3,2,6,7}};
    for(int i=0; i<6;i++)
    {
        for(int j=0; j<4; j++)
        {
            xt::view(surfaces,i,j,xt::all(),xt::all()) = xt::view(corners,xt::all(), index(i,j), xt::all());
        }
    }
//stack speed is 1/8 of for circle
//    xt::xarray<float> surfaces = xt::stack(xt::xtuple(xt::stack(xt::xtuple(xt::view(corners,xt::all(), 0, xt::all()),xt::view(corners,xt::all(), 1, xt::all()),
//                                                      xt::view(corners,xt::all(), 2, xt::all()),xt::view(corners,xt::all(), 3, xt::all()))),
//                                                      xt::stack(xt::xtuple(xt::view(corners,xt::all(), 7, xt::all()),xt::view(corners,xt::all(), 6, xt::all()),
//                                                      xt::view(corners,xt::all(), 5, xt::all()),xt::view(corners,xt::all(), 4, xt::all()))),
//                                                      xt::stack(xt::xtuple(xt::view(corners,xt::all(), 0, xt::all()),xt::view(corners,xt::all(), 3, xt::all()),
//                                                      xt::view(corners,xt::all(), 7, xt::all()),xt::view(corners,xt::all(), 4, xt::all()))),
//                                                      xt::stack(xt::xtuple(xt::view(corners,xt::all(), 1, xt::all()),xt::view(corners,xt::all(), 5, xt::all()),
//                                                      xt::view(corners,xt::all(), 6, xt::all()),xt::view(corners,xt::all(), 2, xt::all()))),
//                                                      xt::stack(xt::xtuple(xt::view(corners,xt::all(), 0, xt::all()),xt::view(corners,xt::all(), 4, xt::all()),
//                                                      xt::view(corners,xt::all(), 5, xt::all()),xt::view(corners,xt::all(), 1, xt::all()))),
//                                                      xt::stack(xt::xtuple(xt::view(corners,xt::all(), 3, xt::all()),xt::view(corners,xt::all(), 2, xt::all()),
//                                                      xt::view(corners,xt::all(), 6, xt::all()),xt::view(corners,xt::all(), 7, xt::all())))));
    surfaces = xt::transpose(surfaces,{2,0,1,3});
    return surfaces;
}

std::shared_ptr<Count_Box_Result>  points_count_convex_polygon_3d_jit(xt::xarray<float> points, xt::xarray<float> polygon_surfaces, xt::xarray<float> out_box_edge)
{
//    int max_num_surfaces = polygon_surfaces.shape(1);
//    int max_num_points_of_surfaces = polygon_surfaces.shape(2);
//    int num_points = points.shape(0);
    int num_polygons = polygon_surfaces.shape(0);
    xt::xarray<int> num_surfaces = xt::zeros<int>({num_polygons});
    for(int i=0; i<num_polygons; i++)
    {
        num_surfaces(i) =  (int)99999;
    }
    xt::xarray<float> normal_vec = xt::zeros<float>({(int)polygon_surfaces.shape(0), (int)polygon_surfaces.shape(1), 3});
    xt::xarray<float> d = xt::zeros<float>({(int)polygon_surfaces.shape(0), (int)polygon_surfaces.shape(1)});
    surface_equ_3d_jitv2(xt::view(polygon_surfaces,xt::all(), xt::all(), xt::range(0,3), xt::all()), normal_vec, d);
    // std::pair<std::vector<int>, xt::xarray<float>> num_point_per_boxes_height_area;
    // xt::xarray<int> num_point_per_boxes = _points_count_convex_polygon_3d_jit(points, polygon_surfaces, normal_vec, d, num_surfaces);
    return _points_count_convex_polygon_3d_jit(points, polygon_surfaces, normal_vec, d, num_surfaces, out_box_edge);

}
void surface_equ_3d_jitv2(xt::xarray<float> surfaces, xt::xarray<float>& normal_vec, xt::xarray<float>& d)
{
    int num_polygon = surfaces.shape(0);
    int max_num_surfaces = surfaces.shape(1);
    xt::xarray<float> sv0 = xt::zeros<float>({3});
    xt::xarray<float> sv1 = xt::zeros<float>({3});
    for(int  i=0; i<num_polygon; i++){
        for(int j=0; j<max_num_surfaces; j++ ){
            sv0(0)  = surfaces(i, j, 0, 0) -surfaces(i, j, 1, 0);
            sv0(1)  = surfaces(i, j, 0, 1) -surfaces(i, j, 1, 1);
            sv0(2)  = surfaces(i, j, 0, 2) -surfaces(i, j, 1, 2);
            sv1(0)  = surfaces(i, j, 1, 0) -surfaces(i, j, 2, 0);
            sv1(1)  = surfaces(i, j, 1, 1) -surfaces(i, j, 2, 1);
            sv1(2)  = surfaces(i, j, 1, 2) -surfaces(i, j, 2, 2);
            normal_vec(i, j, 0) = sv0(1) * sv1(2) - sv0(2) * sv1(1);
            normal_vec(i, j, 1) = sv0(2) * sv1(0) - sv0(0) * sv1(2);
            normal_vec(i, j, 2) = sv0(0) * sv1(1) - sv0(1) * sv1(0);
            d(i, j) = -surfaces(i, j, 0, 0) * normal_vec(i, j, 0) - \
            surfaces(i, j, 0, 1) * normal_vec(i, j, 1) - \
            surfaces(i, j, 0, 2) * normal_vec(i, j, 2);
        }

    }
}

std::shared_ptr<Count_Box_Result>  _points_count_convex_polygon_3d_jit(xt::xarray<float> points,xt::xarray<float> polygon_surfaces,
                                         xt::xarray<float> normal_vec, xt::xarray<float> d,
                                         xt::xarray<float> num_surfaces, xt::xarray<float> out_box_edge){
    auto t_start = std::chrono::high_resolution_clock::now();
    int max_num_surfaces = polygon_surfaces.shape(1);
    int num_points = points.shape(0);
    int num_polygons = polygon_surfaces.shape(0);
    // Count_Box_Result Count_Box_Result_new;
    std::shared_ptr<Count_Box_Result>  Count_Box_Result_new(new Count_Box_Result);
    // Count_Box_Result_new.reset(new Count_Box_Result);

    // std::pair<std::vector<int>, xt::xarray<float>> number_height;
    
    // xt::xarray<int> ret = xt::zeros<int>({num_polygons});
    // for (int i=0; i<num_polygons; i++){
    //     ret(i) = num_points;
    // }
    std::vector<int> ret1(num_polygons,0);  // zqj instead of ret->xtensor
    std::vector<float> heights_box(num_polygons,0);

    xt::xarray<float> h_area = xt::zeros<float>({num_polygons, 2});  // to do 需要修改为vector
    std::pair<std::vector<float>, std::vector<float>> vec_h_area;
    // std::vector<float> vec_h_area_min(num_polygons, 0);
    // std::vector<float> vec_h_area_max(num_polygons, -100);
    for(int i=0; i<num_polygons; i++){
        // h_area(i, 0) = 0;
        h_area(i, 1) = -100;
    }
    // bool flag =true;

    // std::string save_idx_new = "000000"+std::to_string(save_id_zqj_post++);
    // std::string l_strzqjDataPath = "/data/suyan_9720/Alglib/data/pc_res_qlg1002/" + save_idx_new.substr(save_idx_new.length()-6, 6) + ".txt";
    // // string l_strzqjDataPath = "/data/Alglib/data/pc_res_qlg/" + ToStr(save_id_zqj++) + ".txt";  //int save_id_zqj = 0;
    // std::cout << "--zqj-debug-save_pc_path:"  << l_strzqjDataPath << std::endl;
    // ofstream write(l_strzqjDataPath, ios::out);
    // std::cout << "--zqj-debug-alg-num_points:" <<num_points <<"--" << "num_polygons:" << num_polygons <<"--" << "max_num_surfaces:" << max_num_surfaces <<std::endl;
    // for(int i=0; i<num_points; i++){
    for(int j=0; j<num_polygons; j++){   // wai jie juxing
        // std::cout<<"--zqj-debug-alg-waijiejuxing filter0000:" << points(i,0) << "--" << points(i,1) << "--"<< points(i,2) <<std::endl;        // write << to_string(points(i, 0)) << " " 
        //             << to_string(points(i, 1)) << " " 
        //             << to_string(points(i, 2)) << endl;
        
        for(int i=0; i<num_points; i++){
            if ((out_box_edge(j,0)<points(i,0) and points(i,0 )< out_box_edge(j,1)) and (out_box_edge(j,2)<points(i,1) and points(i,1)<out_box_edge(j,3)))
            {
                // std::cout << "--zqj-debug-info_of_points:" << points(i,0) << "--" << points(i,1) << "--"<< points(i,2) << std::endl;
                // // code zqj
                // // ret(j) += 1;
                ret1[j] += 1;
                heights_box[j] += points(i, 2);
                h_area(j, 0) = std::min(h_area(j, 0), points(i, 2));
                h_area(j, 1) = std::max(h_area(j, 1), points(i, 2));  // init -100
                // vec_h_area_min[j] = std::min(vec_h_area_min[j], points(i, 2));
                // vec_h_area_max[j] = std::max(vec_h_area_max[j], points(i, 2));  // init -100
            }
            // for(int k=0;k<max_num_surfaces; k++){
            //     if(k>num_surfaces(j))
            //         break;
            //     float sign = points(i, 0) * normal_vec(j, k, 0) + \
            //     points(i, 1) * normal_vec(j, k, 1) + \
            //     points(i, 2) * normal_vec(j, k, 2) + d(j, k);
            //     // if (sign >= 0){
            //     //     ret(j) -= 1;
            //     //     flag = false;
            //     //     break;
            //     // }
            //      if (sign >= 0){
            //         flag = false;
            //         break;
            //     }
            // if(flag)
            // {
            //     // flag=false;
            //     ret(j) += 1;  // zqj move here
            //     h_area(j, 0) = std::min(h_area(j, 0), points(i, 2));
            //     h_area(j, 1) = std::max(h_area(j, 1), points(i, 2));
            // }
            // else
            // {
            //     flag=true;
            // }
            // }
        }
    }
    // write.close();
    auto t_end = std::chrono::high_resolution_clock::now();
    auto time_spend = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    std::cout<<"--zqj-debug-alg-time_of_count_point_in_box:  "<<time_spend<<" ms"<<std::endl;
    // for (int i =0; i <num_polygons;i++)
    // {
    //     std::cout << "ret1:" << vec_h_area_max[i] <<std::endl;
    // }
    Count_Box_Result_new->vec_result_point = ret1;
    Count_Box_Result_new->vec_result_hight = heights_box;
    Count_Box_Result_new->arr_result_area = h_area;
    
    // number_height.first=ret1;    // std::pair
    // number_height.second=h_area;
    // return number_height;
    return Count_Box_Result_new;
}


std::shared_ptr<Count_Box_Result>  _points_count_convex_polygon_3d_jit_edge(xt::xarray<float> points, int num_polygons, xt::xarray<float> out_box_edge){
    auto t_start = std::chrono::high_resolution_clock::now();
    int num_points = points.shape(0);
    // Count_Box_Result Count_Box_Result_new;
    std::shared_ptr<Count_Box_Result>  Count_Box_Result_new(new Count_Box_Result);
    std::vector<int> ret1(num_polygons,0);  // zqj instead of ret->xtensor
    std::vector<float> heights_box(num_polygons,0);

    xt::xarray<float> h_area = xt::zeros<float>({num_polygons, 2});  // to do 需要修改为vector
    std::pair<std::vector<float>, std::vector<float>> vec_h_area;
    // std::vector<float> vec_h_area_min(num_polygons, 0);
    // std::vector<float> vec_h_area_max(num_polygons, -100);
    for(int i=0; i<num_polygons; i++){
        // h_area(i, 0) = 0;
        h_area(i, 1) = -100;
    }

    std::cout << "--zqj-debug-alg-num_points:" <<num_points <<"--" << "num_polygons:" << num_polygons <<std::endl;
    // for(int i=0; i<num_points; i++){
    for(int j=0; j<num_polygons; j++){   // wai jie juxing
        for(int i=0; i<num_points; i++){
            if ((out_box_edge(j,0)<points(i,0) and points(i,0 )< out_box_edge(j,1)) and (out_box_edge(j,2)<points(i,1) and points(i,1)<out_box_edge(j,3)))
            {
                ret1[j] += 1;
                heights_box[j] += points(i, 2);
                h_area(j, 0) = std::min(h_area(j, 0), points(i, 2));
                h_area(j, 1) = std::max(h_area(j, 1), points(i, 2));  // init -100
                // vec_h_area_min[j] = std::min(vec_h_area_min[j], points(i, 2));
                // vec_h_area_max[j] = std::max(vec_h_area_max[j], points(i, 2));  // init -100
            }
        }
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    auto time_spend = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    std::cout<<"--zqj-debug-alg-time_of_count_point_in_box:  "<<time_spend<<" ms"<<std::endl;

    Count_Box_Result_new->vec_result_point = ret1;
    Count_Box_Result_new->vec_result_hight = heights_box;
    Count_Box_Result_new->arr_result_area = h_area;
    
    // number_height.first=ret1;    // std::pair
    // number_height.second=h_area;
    // return number_height;
    return Count_Box_Result_new;
}
// ********************************** filter boxes end *********************************************************************
