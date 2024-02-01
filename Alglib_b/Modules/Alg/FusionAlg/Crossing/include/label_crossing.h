//
// Created by root on 1/6/22.
//

#ifndef FUSION_ALG_LABEL_H
#define FUSION_ALG_LABEL_H

#include <xtensor/xarray.hpp>




namespace label_crossing{

    int video_class_transform(int video_id);

    int lidar_class_transform(int lidar_id);

    void lidar_class_judge(xt::xarray<float> &data_temp,float &lidar_class);

    std::string class2size(int class_id);

    int fusion_class(int lidar_id, int video_id);

    bool judge_large_target(int target_class);

}

#endif //FUSION_ALG_LABEL_H
