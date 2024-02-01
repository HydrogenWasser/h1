//
// Created by root on 3/1/21.
//
#ifndef SPARSECONVOLUTIONPLUGIN_SECOND_H
#define SPARSECONVOLUTIONPLUGIN_SECOND_H
#include <iostream>
#include <string>
#include <torch/script.h>
#include "TSelfPcAlgParam.h"
#include "voxel_encoder.h"
#include "middle.h"
#include "rpn.h"
#include "target_assigner.h"
#include "xtensor/xtensor_forward.hpp"
#include "yaml-cpp/yaml.h"
#include "IPcAlgBase.h"
#include "real_time_function.h"

class Params
{
public:
    Params(){};
    Params(const std::string &cfg_path);
    bool getNetType(std::string &layers_path);
    std::vector<std::string> voxelnet_class_names;
    std::vector<float> voxel_size;
    std::vector<float> point_cloud_range;
    std::vector<int> vfe_num_filters;
    bool with_distance;
    std::vector<int64_t> output_shape;
    std::vector<float> grid_size;
    int num_class;
    std::vector<int> nms_pre_max_sizes_all_class;
    std::vector<int> nms_post_max_sizes_all_class;
    std::vector<float> nms_iou_thresholds_all_class;
    std::vector<float> nms_score_thresholds;
    int nms_pre_max_sizes;
    int nms_post_max_sizes;
    float nms_iou_thresholds;
    int64_t num_input_features;
    int32_t middle_num_input_features;
    int64_t num_anchor_per_loc;
    int64_t rpn_num_input_features;
    std::vector<int64_t> rpn_layer_nums;
    std::vector<int64_t> rpn_layer_strides;
    std::vector<int64_t> rpn_num_filters;
    std::vector<float> rpn_upsample_strides;
    std::vector<int64_t> rpn_num_upsample_filters;
    bool use_groupnorm;
    int64_t num_groups;
    int64_t box_code_size;
    bool use_direction_classifier;
    bool encode_background_as_zeros;
    int64_t num_direction_bins;
    bool use_norm;
    std::vector<float> post_center_range;
    std::vector<int> feature_map_size;
    std::vector<std::vector<int>> feature_map_sizes;
    float direction_limit_offset;
    float direction_offset;
    std::vector<Anchor_generators> anchor_generator;
    std::string checkpoints_path;
    std::string layers_path;
    bool kd;
};


class VoxelNet : public IPcAlgBase{
public:
    VoxelNet(){}
    ~VoxelNet();

    VoxelNet(TSelfPcAlgParam * AlgParams);
    
    // xt::xarray<float> forward(xt::xarray<float> &points_xt);
    xt::xarray<float> RUN(xt::xarray<float> &points_xt);

    void predict(torch::Tensor & example, std::map<std::string, torch::Tensor> & preds_dict);

private:
    std::vector<float> _voxel_size;
    std::vector<float> _point_cloud_range;
    int _num_class;
    bool _use_rotate_nms; // pre
    bool _multiclass_nms; // pre
    std::vector<float> _nms_score_thresholds;  // pre
    int _nms_pre_max_sizes;  //pre
    int _nms_post_max_sizes;  //pre
    float _nms_iou_thresholds; //pre
    bool _use_sigmoid_score; // pre
    bool _encode_background_as_zeros;  // pre
    bool _use_direction_classifier;  //pre
    bool _box_coder;
    TargetAssigner *_target_assigner;  // TODO : add in the future
    float _dir_offset; // pre
    std::vector<float> _post_center_range; // pre
    bool _nms_class_agnostic; // pre
    int _num_direction_bins; //pre
    float _dir_limit_offset; // pre
    
    int _pc_postfilter;  // add by zqj20230527 for postfilter

    SpMiddleFHD *_middle_feature_extractor;
    RPNV2 *_rpn;
    std::map<std::string, torch::Tensor> _weights_middle;
    std::map<std::string, torch::Tensor> _weights_rpn;
    std::map<std::string, torch::Tensor> _weights_head;

    torch::Tensor _anchors;

    DefaultHead *_small_head;
    DefaultHead *_twowheel_head;
    DefaultHead *_large_head;
    DefaultHead *_fourwheel_head;
};


// ======================  predict function ==============================
torch::Tensor rotate_nms( const torch::Tensor & boxes_for_nms,
                          const torch::Tensor & top_scores,
                          int pre_max_sizes,
                          int post_max_sizes,
                          float iou_thresholds);

torch::Tensor rotate_nms_cc(const torch::Tensor & dets_np, float thresh);
torch::Tensor rotate_nms_cc_new(const torch::Tensor & dets_np, float thresh);
xt::xarray<float> iou_jit( const torch::Tensor & boxes,
                           const torch::Tensor & query_boxes,
                           float eps=1.0);

torch::Tensor corner_to_standup_nd(const torch::Tensor & dets_corners);

torch::Tensor center_to_corner_box2d(torch::Tensor & centers,
                                     torch::Tensor & dims,
                                     torch::Tensor & angles,
                                     float origin=0.5);

torch::Tensor rotation_2d(const torch::Tensor & corners,
                          const torch::Tensor & angles);

torch::Tensor corners_nd(torch::Tensor & dims, float origin=0.5 );

torch::Tensor decode_torch(const torch::Tensor& box_encodings,
                           const torch::Tensor& anchors);

torch::Tensor second_box_decode(const torch::Tensor& box_encodings,
                                const torch::Tensor& anchors,
                                bool encode_angle_to_vector,
                                bool smooth_dim);

torch::Tensor rotate_non_max_suppression_cpu(const torch::Tensor & t_box_corners,
                                             const torch::Tensor & t_order,
                                             const xt::xarray<float> & t_standup_iou,
                                             float thresh);

template <typename DType>
xt::xarray<DType> tensorToxTensor(const torch::Tensor & boxes);
// =============================================================================

#endif //SPARSECONVOLUTIONPLUGIN_SECOND_H