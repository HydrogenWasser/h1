#ifndef PROCESS_REFLECT_H
#define PROCESS_REFLECT_H

#include "Camera_reflect.h"

#include <xtensor.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <fstream>
#include <sstream>
#include <json.hpp>

#define PI 3.1415926

class Process_reflect
{

public:
    Process_reflect(nlohmann::json *_parameter);
    ~Process_reflect();

    // 1. 初始化反映射
    void init_reflect(std::string _rorate_file_path, std::string _trans_file_path, TCameraParam *_camera_param, int _camera_dev_num, std::string _filter_image_dir, std::string _pixel_xyz_dir);

    // 2. 加载json参数，装载到Process_reflect类属性中
    void load_fusion_param();

    // 3.图像推理得到n路的origin_box结果，调用此方法，进行反映射操作
    void get_tracker_videoBoxInfo(std::vector<xt::xarray<float>> &origin_box, std::vector<xt::xarray<float>> &new_box);

private:
    // 加载csv文件，得到辅雷达到主雷达转换的旋转向量和平移向量
    void load_csv(std::string _file_path, xt::xarray<float> &_matrix);

    // 1. 加载csv文件，得到m_rorate
    void load_rorate(std::string _file_path, xt::xarray<float> &_matrix);

    // 2. 加载csv文件，得到m_trans
    void load_trans(std::string _file_path, xt::xarray<float> &_matrix);

    // 3. 生成反映射对象
    void load_reflect(TCameraParam *_camera_param, int camera_dev_num, std::string _filter_image_dir, std::string _pixel_xyz_dir);

    void video_class_transform(xt::xarray<int> &_labels, int index);

private:
    xt::xarray<float> m_rorate;
    xt::xarray<float> m_trans;

    nlohmann::json *m_parameter;

    std::vector<int> m_camera_raw_size;
    int IMG_size = 640;
    std::map<std::string, std::vector<float>> m_class_to_size_dict;
    std::map<std::string, int> m_video_class_to_lidar_class;

    std::vector<std::shared_ptr<Camera_reflect>> m_coordinate_estimators;
};
#endif