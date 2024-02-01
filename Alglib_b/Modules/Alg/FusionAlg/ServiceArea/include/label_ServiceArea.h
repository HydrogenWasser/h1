/*******************************************************
 文件名：label_ServiceArea.h
 作者：HuaHuan
 描述：标签内容
 版本：v1.0
 日期：2024-01-11
 *******************************************************/
#ifndef FUSION_ALG_LABEL_SERVICEAREA_H
#define FUSION_ALG_LABEL_SERVICEAREA_H

#include <xtensor/xarray.hpp>




namespace label_ServiceArea{

    int video_class_transform(int video_id);

    int lidar_class_transform(int lidar_id);

    void lidar_class_judge(xt::xarray<float> &data_temp,float &lidar_class);

    std::string class2size(int class_id);

    int fusion_class(int lidar_id, int video_id);

    bool judge_large_target(int target_class);

}

#endif // FUSION_ALG_LABEL_SERVICEAREA_H
