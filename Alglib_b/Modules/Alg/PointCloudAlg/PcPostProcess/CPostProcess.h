#ifndef PC_POSTPROCESS_H
#define PC_POSTPROCESS_H
#include "TSelfPcAlgParam.h"

#include <xtensor/xview.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xsort.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor-blas/xlinalg.hpp>

#include "eigen3/unsupported/Eigen/CXX11/Tensor"
#include "box_ops_post.h"
//位图文件头定义;
//其中不包含文件类型信息（由于结构体的内存结构决定，
//要是加了的话将不能正确读取文件信息）
// #pragma pack(1)
typedef struct { 
    unsigned short int type; /* 文件类型 常见BM(0x42,0x4d) */ 
    unsigned int size;/* 文件大小 */ 
    unsigned short int reserved1, reserved2; /* 保留位 */ 
    unsigned int offset;/* 数据区在文件中的位置偏移量byte */ 
} bmp_file_header;
#pragma pack()
// #pragma pack(1)
typedef struct { 
	unsigned int biSize; /* 位图信息头的大小 */ 
	int biWidth; /* 位图的宽度，单位是像素 */
	int biHeight; /* 位图的高度，单位是像素, 如果该值是一个负数表示倒向位图 */
	unsigned short int biPlanes; /* 为目标设备说明位面数，其值将总是被设为1 */
	unsigned short int biBitCount; /* 说明bit数/像素，其值为1、4、8、16、24、或32 */
	unsigned int biCompression; /* 压缩的类型，常见是0表示非压缩，见下表 */
	unsigned int biSizeImage; /* 图像的大小，当用BI_RGB格式-非压缩时，可设置为0 */
	unsigned int biXPelsPerMeter; /* 水平分辨率，用像素/米表示 */
	unsigned int biYPelsPerMeter; /* 垂直分辨率，用像素/米表示 */
	unsigned int biClrUsed; /* 使用的彩色表中的颜色索引数（设为0的话，则说明使用所有调色板项） */
	unsigned int biClrImportant; /* 对图像显示有重要影响的颜色索引的数目，如果是0表示都重要 */
} bmp_info_header;
#pragma pack()

// class CPcProcess: public IPcPreProcess
class CPostProcess
{
public:
    //构造函数
    CPostProcess(TSelfPcAlgParam * p_pAlgParams, int p_nId);

    ~CPostProcess();

    //目标分割
    void BoxSplit(float& x, float& y, float& z, float& angle);
    
private:
    //初始化bmp图
    Eigen::Tensor<uint8_t, 3> init_bmp(const std::string &m_strBmpPath);
    //初始化旋转矩阵
    xt::xarray<float> init_rotate(float p_fRotTrans[6], bool p_bConvert = false);
    //点云裁剪——扩充范围方式裁剪
  

private:
    uint16_t m_usType;
    bmp_file_header fileHeader;
    bmp_info_header infoHeader;
    //点云处理方式
    bool m_bPcProType;
    //雷达id
    int m_nLidarId;
    //栅格大小
    int m_nImgSizeFilter;
    //栅格边界X
    double m_dMinX;
    //栅格边界Y
    double m_dMinY;
    //栅格倒數
    double m_fGridFilterInv;
    //过滤栅格矩阵
    xt::xarray<float> m_xaImgFilter;
    //旋转矩阵
    xt::xarray<float> m_xaSelfRotMat;
    xt::xarray<float> m_xaRangeRotMat1;
    xt::xarray<float> m_xaRangeRotMat2;

    //旋转矩阵
    xt::xarray<float> m_xaSplitRotMat1;
    xt::xarray<float> m_xaSplitRotMat2;
    //BMP图
    Eigen::Tensor<uint8_t, 3> m_egBmpMat1;
    Eigen::Tensor<uint8_t, 3> m_egBmpMat2;
    Eigen::Tensor<uint8_t, 3> m_egSplitBmpMat;
    
    //算法参数
    TSelfPcAlgParam m_tAlgParams;
};

#define None -10000.1
namespace box_ops_nms
{
    xt::xarray<float> corners_nd(xt::xarray<float> &dims, float origin = 0.5);
    xt::xarray<float> rotation_2d(xt::xarray<float> &corners, const xt::xarray<float> &angles);
    xt::xarray<float> center_to_corner_box2d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                                   xt::xarray<float> &angles, float origin = 0.5);

    xt::xarray<float> center_to_corner_box3d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                             float &angles, float origin = 0.5, int axis = 2);
    void rotation_3d_in_axis(xt::xarray<float> &points, float &angles, int axis = 0);


    xt::xarray<float> corner_to_standup_nd(xt::xarray<float> &boxes_corner);

    std::pair<xt::xarray<float>, xt::xarray<float>> rotate_nms_cc(xt::xarray<float> &dets,
                                                                xt::xarray<float> &trackers);

    float cal_angle(std::vector<xt::xarray<float>> &state_list, float &thresh);


    std::pair<xt::xarray<float>, xt::xarray<float>> iou_jit_new(xt::xarray<float> &boxes,
                                                                xt::xarray<float> &query_boxes,
                                                                float eps = 0.0);
}

// filter
struct Count_Box_Result
{
    std::vector<int> vec_result_point;
    std::vector<float> vec_result_hight;
    xt::xarray<float> arr_result_area;
};
std::vector<int> filter_label_for_KT(std::vector<int> &labels, std::vector<float> &scores, int mode);
std::vector<int> filter_label_for_WJ(std::vector<int> &labels, xt::xarray<float> &pc_boxes, std::vector<float> &scores, std::vector<int> &point_num,
                                     std::vector<float> &heights, xt::xarray<float> &area);
std::vector<int> filter_label_forNuScenes(std::vector<int> &labels,xt::xarray<float> &boxes_lidar, std::vector<float> &scores,xt::xarray<int> &num_point_per_boxes, xt::xarray<float> &height_area,  int mode,std::vector<float> point_cloud_range);

void center_to_corner_box3d(xt::xarray<float> &centers, xt::xarray<float> &dims, xt::xarray<float> angle,
                            xt::xarray<float> orgin={0.5,0.5,0.5}, int axis=2);

void corners_nd(xt::xarray<float> dims, float orgin=0.5f);
xt::xarray<float> cal_box_edge(xt::xarray<float> out_box);
void surface_equ_3d_jitv2(xt::xarray<float> surfaces, xt::xarray<float>& normal_vec, xt::xarray<float>& d);
xt::xarray<float> corner_to_surfaces_3d(xt::xarray<float> corners);
std::shared_ptr<Count_Box_Result>  points_count_rbbox(xt::xarray<float>& points, xt::xarray<float> box3d);
std::shared_ptr<Count_Box_Result>  points_count_convex_polygon_3d_jit(xt::xarray<float> points, xt::xarray<float> polygon_surfaces, xt::xarray<float> out_box_edge);
std::shared_ptr<Count_Box_Result>  _points_count_convex_polygon_3d_jit(xt::xarray<float> points,xt::xarray<float> polygon_surfaces,
                                         xt::xarray<float> normal_vec, xt::xarray<float> d,
                                         xt::xarray<float> num_surfaces, xt::xarray<float> out_box_edge);
std::shared_ptr<Count_Box_Result> _points_count_convex_polygon_3d_jit_edge(xt::xarray<float> points, int num_polygons, xt::xarray<float> out_box_edge);
#endif
