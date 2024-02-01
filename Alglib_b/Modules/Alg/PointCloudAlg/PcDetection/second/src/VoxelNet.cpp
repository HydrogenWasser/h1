//
// Created by chenxiangyang on 3/2/21.
//
#include <iostream>
#include "VoxelNet.h"
#include <memory>
#include <vector>
#include <string>
#include <torch/script.h>
#include "voxel_encoder.h"
#include "voxelize.h"
#include "middle.h"
#include "xtensor/xbuilder.hpp"
#include "xtensor/xtensor_forward.hpp"
#include <ctime>
#include <boost/geometry.hpp>
#include <unordered_map>
#include "CPostProcess.h"

template <class Type>
Type stringToNum(const std::string& str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

template <class Type>
void vStrSplit(std::string &strSur, char cConChar, std::vector<std::string> &srt_vector)
{
    srt_vector.clear();
    std::string::size_type pos1, pos2;
    pos1 = 0;
    pos2 = strSur.find(cConChar, 0);
    while(std::string::npos != pos2)
    {
        auto t = strSur.substr(pos1, pos2 - pos1);
        srt_vector.push_back(t);
        pos1 = pos2 + 1;
        pos2 = strSur.find(cConChar, pos1);
    }
    srt_vector.push_back(strSur.substr(pos1, strSur.size()));
}

std::map<std::string, std::map<std::string, torch::Tensor>> load_state(std::string &weight_csv, std::string &weight_data){
    std::ifstream inFile(weight_csv, std::ios::in);
    std::string lineStr;
    std::vector<std::string> str_vector;
    float *pos = new float[6000000];

    torch::Tensor bias = torch::zeros({0}).to(torch::kCUDA);
    std::map<std::string, torch::Tensor> weights_mid;
    std::map<std::string, torch::Tensor> weight_rpn;
    std::map<std::string, torch::Tensor> weight_head;
    std::map<std::string, std::map<std::string, torch::Tensor>> weights;

    std::ifstream g(weight_data, std::ios::binary);
    if (!g.is_open()) 
    {
        std::cout << "ERROR!!! weight file [" + weight_data + "] is not exist!" << std::endl;
        abort();
    }
    g.read((char*)pos,6000000*sizeof(float));
    int index_weight = 0;


    while (getline(inFile, lineStr))
    {
        vStrSplit<float>(lineStr, ',', str_vector);

        // net type
        std::string net_type = str_vector[0];
        if (net_type == "net")
            continue;

        // layer type
        std::string layer_type = str_vector[1];

        // num size
        int size_num = stringToNum<int>(str_vector[2]);
        std::vector<std::string> shape_str;
        vStrSplit<int>(str_vector[3], '.', shape_str);

        // dim_shape
        std::vector<int64_t> shape;
        for(auto &it : shape_str)
            shape.push_back(stringToNum<int64_t>(it));
        if (size_num == 0){
            if (net_type == "middle")
                weights_mid[layer_type] = bias;
            else if (net_type == "rpn")
                weight_rpn[layer_type] = bias;
            else if(net_type == "head")
                weight_head[layer_type] = bias;
            continue;
        }

        // load data
        torch::Tensor weight_tensor = torch::zeros({size_num}, torch::TensorOptions().dtype(torch::kFloat32));
        for (int i = index_weight; i < size_num + index_weight; ++i) {
            weight_tensor[i - index_weight] = pos[i];
        }
        index_weight += size_num;

        // chage device && type && dim
        c10::ArrayRef<int64_t> shape_TENSOR = c10::ArrayRef<int64_t>(shape);
        weight_tensor = weight_tensor.reshape(shape_TENSOR);
        weight_tensor = weight_tensor.to(torch::Device(torch::kCUDA));

        if (net_type == "middle")
            weights_mid[layer_type] = weight_tensor;
        else if (net_type == "rpn")
            weight_rpn[layer_type] = weight_tensor;
        else if(net_type == "head")
            weight_head[layer_type] = weight_tensor;

    }
    delete[] pos;
    weights["middle2"] = weights_mid;
    weights["rpn"] = weight_rpn;
    weights["head"] = weight_head;
    return weights;
}


Params::Params(const std::string &cfg_path)
{
    YAML::Node cfg = YAML::LoadFile(cfg_path);
    voxelnet_class_names = {cfg["CLASS_NAMES"][0].as<std::string>(),
                            cfg["CLASS_NAMES"][1].as<std::string>(), cfg["CLASS_NAMES"][2].as<std::string>(),
                            cfg["CLASS_NAMES"][3].as<std::string>(), cfg["CLASS_NAMES"][4].as<std::string>(),
                            cfg["CLASS_NAMES"][5].as<std::string>(), cfg["CLASS_NAMES"][6].as<std::string>()};
    voxel_size = {cfg["SECOND_MODEL"]["VOXEL_SIZE"][0].as<float>(),
                  cfg["SECOND_MODEL"]["VOXEL_SIZE"][1].as<float>(),
                  cfg["SECOND_MODEL"]["VOXEL_SIZE"][2].as<float>()};
    point_cloud_range = {cfg["SECOND_MODEL"]["POINT_CLOUD_RANGE"][0].as<float>(),
                         cfg["SECOND_MODEL"]["POINT_CLOUD_RANGE"][1].as<float>(),
                         cfg["SECOND_MODEL"]["POINT_CLOUD_RANGE"][2].as<float>(),
                         cfg["SECOND_MODEL"]["POINT_CLOUD_RANGE"][3].as<float>(),
                         cfg["SECOND_MODEL"]["POINT_CLOUD_RANGE"][4].as<float>(),
                         cfg["SECOND_MODEL"]["POINT_CLOUD_RANGE"][5].as<float>()};
    vfe_num_filters = {cfg["SECOND_MODEL"]["VFE"]["VFE_NUM_FILTERS"].as<int>()};
    with_distance = cfg["SECOND_MODEL"]["VFE"]["WITH_DISTANCE"].as<bool>();
    output_shape.resize(5);
    grid_size.resize(3);
    for (int i = 0; i < 3; ++i)
        grid_size[i] = (point_cloud_range[i + 3] - point_cloud_range[i]) / voxel_size[i];
    output_shape[0] = 1;
    for (int i = 0; i < 3; ++i)
        output_shape[i + 1] = grid_size[2 - i];
    output_shape[4] = vfe_num_filters[0];
    num_class = cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"].size();

    nms_pre_max_sizes_all_class.resize(num_class);
    nms_post_max_sizes_all_class.resize(num_class);
    nms_iou_thresholds_all_class.resize(num_class);
    anchor_generator.resize(7);
    nms_score_thresholds.resize(num_class);
    for (int i = 0; i < num_class; ++i)
    {
        nms_pre_max_sizes_all_class[i] = cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["nms_pre_max_size"].as<int>();
        nms_post_max_sizes_all_class[i] = cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["nms_post_max_size"].as<int>();
        nms_iou_thresholds_all_class[i] = cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["nms_iou_threshold"].as<float>();
        nms_score_thresholds[i] = cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["nms_score_threshold"].as<float>();

        anchor_generator[i]._anchor_ranges = {
            cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_ranges"][0].as<float>(),
            cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_ranges"][1].as<float>(),
            cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_ranges"][2].as<float>(),
            cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_ranges"][3].as<float>(),
            cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_ranges"][4].as<float>(),
            cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_ranges"][5].as<float>()};
        anchor_generator[i]._anchor_ranges = anchor_generator[i]._anchor_ranges.reshape({1, 6});
        anchor_generator[i]._class_name = cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["class_name"].as<std::string>();
        anchor_generator[i]._custom_values = 0;
        anchor_generator[i]._rotations = {
            cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_rotations"][0].as<float>(),
            cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_rotations"][1].as<float>()};
        anchor_generator[i]._sizes = {
            cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_sizes"][0].as<float>(),
            cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_sizes"][1].as<float>(),
            cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_sizes"][2].as<float>()};
        anchor_generator[i].num_anchors_per_localization = 2;
    }
    nms_pre_max_sizes = *std::max_element(nms_pre_max_sizes_all_class.begin(), nms_pre_max_sizes_all_class.end());
    nms_post_max_sizes = *std::max_element(nms_post_max_sizes_all_class.begin(),
                                           nms_post_max_sizes_all_class.end());
    nms_iou_thresholds = *std::max_element(nms_iou_thresholds_all_class.begin(),
                                           nms_iou_thresholds_all_class.end());
    num_input_features = cfg["SECOND_MODEL"]["NUM_INPUT_FEATURE"].as<int>();
    middle_num_input_features = cfg["SECOND_MODEL"]["BACKBONE_3D"]["MIDDLE_NUM_INPUT_FEATURE"].as<int>();
    num_anchor_per_loc = num_class * cfg["SECOND_MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][0]["anchor_rotations"].size();
    rpn_num_input_features = cfg["SECOND_MODEL"]["RPN"]["RPN_NUM_INPUT_FEATURE"].as<int>();
    rpn_layer_nums = {cfg["SECOND_MODEL"]["RPN"]["RPN_LAYER_NUMS"][0].as<int>(),
                      cfg["SECOND_MODEL"]["RPN"]["RPN_LAYER_NUMS"][1].as<int>()};
    rpn_layer_strides = {cfg["SECOND_MODEL"]["RPN"]["RPN_LAYER_STRIDES"][0].as<int>(),
                         cfg["SECOND_MODEL"]["RPN"]["RPN_LAYER_STRIDES"][1].as<int>()};
    rpn_num_filters = {cfg["SECOND_MODEL"]["RPN"]["RPN_NUM_FILTERS"][0].as<int>(),
                       cfg["SECOND_MODEL"]["RPN"]["RPN_NUM_FILTERS"][1].as<int>()};
    rpn_upsample_strides = {cfg["SECOND_MODEL"]["RPN"]["RPN_UPSAMPLE_STRIDES"][0].as<float>(),
                            cfg["SECOND_MODEL"]["RPN"]["RPN_UPSAMPLE_STRIDES"][1].as<float>()};
    rpn_num_upsample_filters = {cfg["SECOND_MODEL"]["RPN"]["RPN_NUM_UPSAMPLE_FILTERS"][0].as<int>(),
                                cfg["SECOND_MODEL"]["RPN"]["RPN_NUM_UPSAMPLE_FILTERS"][1].as<int>()};
    use_groupnorm = cfg["SECOND_MODEL"]["RPN"]["USE_GROPNORM"].as<bool>();
    num_groups = cfg["SECOND_MODEL"]["RPN"]["NUM_GROPS"].as<int>();
    box_code_size = cfg["SECOND_MODEL"]["DENSE_HEAD"]["BOX_CODE_SIZE"].as<int>();
    use_direction_classifier = cfg["SECOND_MODEL"]["DENSE_HEAD"]["USE_DIRECTION_CLASSIFIER"].as<bool>();
    encode_background_as_zeros = cfg["SECOND_MODEL"]["DENSE_HEAD"]["ENCODE_BACKGROUND_AS_ZEROS"].as<bool>();
    num_direction_bins = cfg["SECOND_MODEL"]["DENSE_HEAD"]["NUM_DIR_BINS"].as<int>();
    use_norm = cfg["SECOND_MODEL"]["DENSE_HEAD"]["USE_NORM"].as<bool>();

    post_center_range.resize(cfg["SECOND_MODEL"]["DENSE_HEAD"]["POST_CENTER_LIMIT_RANGE"].size());
    for (int i = 0; i < cfg["SECOND_MODEL"]["DENSE_HEAD"]["POST_CENTER_LIMIT_RANGE"].size(); ++i)
    {
        post_center_range[i] = cfg["SECOND_MODEL"]["DENSE_HEAD"]["POST_CENTER_LIMIT_RANGE"][i].as<float>();
    }

    direction_limit_offset = cfg["SECOND_MODEL"]["DENSE_HEAD"]["DIR_LIMIT_OFFSET"].as<float>();
    direction_offset = cfg["SECOND_MODEL"]["DENSE_HEAD"]["DIR_OFFSET"].as<float>();

    feature_map_size.resize(3);
    int downsample_factor = 1;
    for (int i = 0; i < rpn_layer_strides.size(); ++i)
    {
        downsample_factor *= rpn_layer_strides[i];
    }
    downsample_factor /= rpn_upsample_strides[rpn_upsample_strides.size() - 1];
    downsample_factor *= cfg["SECOND_MODEL"]["BACKBONE_3D"]["MIDDLE_DOWNSAMPLE_FACTOR"].as<int>();
    feature_map_size[0] = 1;
    feature_map_size[1] = int(grid_size[0]) / downsample_factor;
    feature_map_size[2] = int(grid_size[1]) / downsample_factor;
    for (int i = 0; i < num_class; ++i)
    {
        feature_map_sizes.push_back(feature_map_size);
    }
    checkpoints_path = cfg["SECOND_CHECKPOINTS"].as<std::string>();
    layers_path = cfg["SECOND_NETWORK_LAYER"].as<std::string>();
    kd = getNetType(layers_path);
    rpn_num_input_features = kd ? 192 : 128;
}
bool Params::getNetType(std::string &layers_path)
{
    std::ifstream file(layers_path);
    std::string line;
    int row = 0;
    while (std::getline(file, line))
    {
        row = row + 1;
    }
    if (row == 133)
    {
        return true;
    }
    else
    {
        return false;
    }
}


VoxelNet::~VoxelNet()
{
    delete _target_assigner;
    delete _middle_feature_extractor;
    delete _rpn;
    delete _small_head;
    delete _twowheel_head;
    delete _large_head;
    delete _fourwheel_head;
}

/***************************************************************
 * @file       VoxelNet.cpp
 * @brief      点云检测对象的构造函数
 * @input      AlgParams: 参数结构体指针
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
VoxelNet::VoxelNet(TSelfPcAlgParam * AlgParams)
{
    std::shared_ptr<Params> params = std::make_shared<Params>(AlgParams->m_strRootPath + AlgParams->m_stPcAlgParam.m_strCfgPath);
    AlgParams->m_stPcAlgParam.m_vecPcClass = params->voxelnet_class_names;

    // Params params(AlgParams->cfg_path);
    params->layers_path = AlgParams->m_strRootPath + params->layers_path;
    params->checkpoints_path = AlgParams->m_strRootPath + params->checkpoints_path;
    std::map<std::string, std::map<std::string, torch::Tensor>> weights = load_state(params->layers_path, params->checkpoints_path);
    bool checkpoint_type = weights["middle2"].count("b13") ? false : true;
    if (checkpoint_type != params->kd)
    {
        // TODO:
        // spdlog::critical("Checkpoints doesn't match Layers. Please check your layers_path and checkpoints_path in yaml file.");
    }

    _num_class = params->num_class;
    _box_coder = params->box_code_size;
    _num_direction_bins = params->num_direction_bins;
    _encode_background_as_zeros = params->encode_background_as_zeros;
    _use_direction_classifier = params->use_direction_classifier;
    _weights_middle = weights["middle2"];
    _weights_rpn = weights["rpn"];
    _weights_head = weights["head"]; 
    _nms_score_thresholds = params->nms_score_thresholds;
    _nms_pre_max_sizes = params->nms_pre_max_sizes;
    _nms_post_max_sizes = params->nms_post_max_sizes;
    _nms_iou_thresholds = params->nms_iou_thresholds;
    _post_center_range = params->post_center_range;
    _dir_offset = params->direction_offset;
    _dir_limit_offset = params->direction_limit_offset;
    _voxel_size = params->voxel_size;
    _point_cloud_range = params->point_cloud_range;

    _pc_postfilter = AlgParams->m_stPcAlgParam.m_nGridSizeSmall;  // add by zqj20230527 是否进行点云postfilter

    _target_assigner = new TargetAssigner(params->anchor_generator, params->feature_map_sizes);
    _anchors = _target_assigner->generate_anchors(params->feature_map_size);
    _middle_feature_extractor = new SpMiddleFHD(params->output_shape, params->use_norm, params->middle_num_input_features, _weights_middle, params->kd);
    _rpn = new RPNV2(params->use_norm, params->num_class, params->rpn_layer_nums, params->rpn_layer_strides, params->rpn_num_filters, params->rpn_upsample_strides,
                 params->rpn_num_upsample_filters,
                 params->rpn_num_input_features, params->num_anchor_per_loc, params->encode_background_as_zeros, params->use_direction_classifier,
                 params->use_groupnorm,
                 params->num_groups, params->box_code_size, _num_direction_bins, false);
    _rpn->load_weight(_weights_rpn, true, false);
    int small_num_anchor_loc = 2;     // pedestrian;
    int twowheel_num_anchor_loc = 4;  // bicycle, tricycle
    int large_num_anchor_loc = 2;     // car
    int fourwheel_num_anchor_loc = 6; // truck bus semitrailer
    std::vector<int64_t> rpn_num_upsample = _rpn->get_num_upsample_filters();
    int64_t rpn_num_upsample_filters_sum = std::accumulate(rpn_num_upsample.begin(), rpn_num_upsample.end(), 0);

    _small_head = new DefaultHead(rpn_num_upsample_filters_sum,
                              _num_class,
                              small_num_anchor_loc,
                              params->box_code_size,
                              params->num_direction_bins,
                              params->use_direction_classifier,
                              params->encode_background_as_zeros,
                              "small_head",
                              _weights_head);

    _twowheel_head = new DefaultHead(rpn_num_upsample_filters_sum,
                                 _num_class,
                                 twowheel_num_anchor_loc,
                                 params->box_code_size,
                                 params->num_direction_bins,
                                 params->use_direction_classifier,
                                 params->encode_background_as_zeros,
                                 "twowheel_head",
                                 _weights_head);
    _large_head = new DefaultHead(rpn_num_upsample_filters_sum,
                              _num_class,
                              large_num_anchor_loc,
                              params->box_code_size,
                              params->num_direction_bins,
                              params->use_direction_classifier,
                              params->encode_background_as_zeros,
                              "large_head",
                              _weights_head);
    _fourwheel_head = new DefaultHead(rpn_num_upsample_filters_sum,
                                  _num_class,
                                  fourwheel_num_anchor_loc,
                                  params->box_code_size,
                                  params->num_direction_bins,
                                  params->use_direction_classifier,
                                  params->encode_background_as_zeros,
                                  "fourwheel_head",
                                  _weights_head);
}

template <typename DType>
xt::xarray<DType> tensorToxTensor(const torch::Tensor & boxes)
{
    int n = boxes.dim();
    if (n == 1)
    {
        size_t size = (size_t) boxes.size(0) ;
        std::vector<DType> tensorshape = {DType (boxes.size(0))};
        xt::xarray<DType> out = xt::adapt(boxes.data_ptr<DType>(), size, xt::no_ownership(), tensorshape);
        return out;
    }else if (n == 2)
    {
        size_t size = (size_t) boxes.size(0) * (size_t) boxes.size(1);
        std::vector<DType> tensorshape = {DType (boxes.size(0)),DType (boxes.size(1))};
        xt::xarray<DType> out = xt::adapt(boxes.data_ptr<DType>(), size, xt::no_ownership(), tensorshape);
        return out;
    }else if(n == 3)
    {
        size_t size = (size_t) boxes.size(0) * (size_t) boxes.size(1)* (size_t) boxes.size(2);
        std::vector<DType> tensorshape = {DType (boxes.size(0)),DType (boxes.size(1)),DType (boxes.size(2))};
        xt::xarray<DType> out = xt::adapt(boxes.data_ptr<DType>(), size, xt::no_ownership(), tensorshape);
        return out;
    }
    else
    {
        std::cout << "wrong dim" << std::endl;
    }
    return -1;

}

xt::xarray<float> iou_jit( const torch::Tensor & boxes,
                           const torch::Tensor & query_boxes,
                           float eps){

    xt::xarray<float> xt_boxes = tensorToxTensor<float> (boxes);
    xt::xarray<float> xt_query_boxes = tensorToxTensor<float> (query_boxes);
    int N = xt_boxes.shape(0);
    int K = xt_query_boxes.shape(0);
    xt::xarray<float> overlaps = xt::zeros<float>({N,K});

    float box_area;
    float iw;
    float ih;
    float ua;

    for (int k = 0; k < K; ++k)
    {
        box_area = ((xt_query_boxes(k, 2) - xt_query_boxes(k, 0) + eps) * (xt_query_boxes(k, 3) - xt_query_boxes(k ,1) + eps));
        for (int n = 0; n < N; ++n)
        {
            iw = std::min(xt_boxes(n, 2), xt_query_boxes(k, 2)) - std::max(xt_boxes(n, 0), xt_query_boxes(k, 0) + eps);
            if (iw > 0)
            {
                ih = std::min(xt_boxes(n, 3), xt_query_boxes(k, 3)) - std::max(xt_boxes(n, 1), xt_query_boxes(k, 1) + eps);
                if (ih > 0)
                {
                    ua = ((xt_boxes(n, 2) - xt_boxes(n, 0) + eps) * (xt_boxes(n, 3) - xt_boxes(n, 1) + eps) + box_area - iw * ih);
                    overlaps(n, k) = iw * ih / ua;
                }
            }
        }

    }

    return overlaps;
}

torch::Tensor corner_to_standup_nd(const torch::Tensor & dets_corners){

    assert(dets_corners.sizes().size() == 3);
    std::tuple<torch::Tensor,torch::Tensor> max_standup_boxes;
    std::tuple<torch::Tensor,torch::Tensor> min_standup_boxes;
    torch::Tensor x_standup_boxes;
    torch::Tensor n_standup_boxes;
    std::vector<torch::Tensor> standup_boxes;

    min_standup_boxes = torch::min({dets_corners},1);
    max_standup_boxes = torch::max({dets_corners},1);

    n_standup_boxes = std::get<0>(min_standup_boxes);
    x_standup_boxes = std::get<0>(max_standup_boxes);

    standup_boxes.push_back(n_standup_boxes);
    standup_boxes.push_back(x_standup_boxes);

    torch::Tensor res4 = torch::cat({standup_boxes}, -1);

    return res4;
}


torch::Tensor rotation_2d(const torch::Tensor & corners, const torch::Tensor & angles){
    torch::Tensor rot_sin = torch::sin(angles);
    torch::Tensor rot_cos = torch::cos(angles);
    torch::Tensor a = torch::cat({rot_cos,-rot_sin},1).t();
    torch::Tensor b = torch::cat({rot_sin,rot_cos},1).t();
    torch::Tensor rot_mat_T = torch::stack({a, b},0);
    //std::cout<<corners.sizes()<<std::endl;
    //std::cout<< rot_mat_T.sizes()<<std::endl;
    torch::Tensor res3 = torch::einsum("aij,jka->aik",{corners , rot_mat_T});
    //std::cout<<res3.sizes()<<std::endl;
    return res3;
}

torch::Tensor corners_nd(torch::Tensor & dims, float origin){

    int ndim = int(dims.size(1));
    torch::Tensor corners_norm;
    if(ndim == 2)
    {
        corners_norm = torch::zeros({4,2});
        corners_norm[1][1] = 1;
        corners_norm[2][0] = 1;
        corners_norm[3][0] = 1;
        corners_norm[2][1] = 1;
        corners_norm.to(torch::kFloat32);
    }
    else if(ndim == 3)
    {
        corners_norm = torch::zeros({8,3});
        corners_norm[4][0] = 1;
        corners_norm[5][0] = 1;
        corners_norm[6][0] = 1;
        corners_norm[7][0] = 1;
        corners_norm[2][1] = 1;
        corners_norm[3][1] = 1;
        corners_norm[6][1] = 1;
        corners_norm[7][1] = 1;
        corners_norm[1][2] = 1;
        corners_norm[2][2] = 1;
        corners_norm[5][2] = 1;
        corners_norm[6][2] = 1;
        corners_norm.to(torch::kFloat32);
    }
    corners_norm = corners_norm - origin;
    torch::Tensor corners;
    int x = pow(2,ndim);
    corners = dims.reshape({-1,1,ndim})*corners_norm.reshape({1,x,ndim});
    return corners;
}

torch::Tensor center_to_corner_box2d(torch::Tensor & centers,
                                     torch::Tensor & dims,
                                     torch::Tensor & angles,
                                     float origin){
    torch::Tensor corners = corners_nd(dims, origin);  // corners: [N, 4, 2]
    if (angles.size(0) != 0)
    {
        corners = rotation_2d(corners, angles);
    }
    corners += centers.reshape({-1, 1, 2});
    return corners;
}



torch::Tensor second_box_decode(const torch::Tensor& box_encodings,
                                const torch::Tensor& anchors,
                                bool encode_angle_to_vector,
                                bool smooth_dim)
{


    int box_ndim = anchors.sizes()[2];
    //boxes ([N, 7] Tensor): normal boxes: x, y, z, w, l, h, r
    //note: for encode_angle_to_vector is true: r==rx ry ==ry
    // for encode_angle_to_vector is false: r == r, ry not use
    enum anchor_type{x__ = 0, y__, z__, w__, l__, h__, r__, ry__};

    std::vector<torch::Tensor> vec_anchors = torch::split(anchors, 1, -1);
    std::vector<torch::Tensor> vec_box_encodings = torch::split(box_encodings, 1, -1);

    torch::Tensor diagonal = torch::sqrt(vec_anchors.at(l__) * vec_anchors.at(l__)
                                         + vec_anchors.at(w__) * vec_anchors.at(w__));

    torch::Tensor xg = vec_box_encodings.at(x__) * diagonal + vec_anchors.at(x__);
    torch::Tensor yg = vec_box_encodings.at(y__) * diagonal + vec_anchors.at(y__);
    torch::Tensor zg = vec_box_encodings.at(z__) * vec_anchors.at(h__) + vec_anchors.at(z__);

    torch::Tensor lg;
    torch::Tensor wg;
    torch::Tensor hg;
    if (smooth_dim)
    {
        lg = (vec_box_encodings.at(l__) + 1) * vec_anchors.at(l__);
        wg = (vec_box_encodings.at(w__) + 1) * vec_anchors.at(w__);
        hg = (vec_box_encodings.at(h__) + 1) * vec_anchors.at(h__);
    }
    else
    {
        lg = torch::exp(vec_box_encodings.at(l__)) * vec_anchors.at(l__);
        wg = torch::exp(vec_box_encodings.at(w__)) * vec_anchors.at(w__);
        hg = torch::exp(vec_box_encodings.at(h__)) * vec_anchors.at(h__);
    }

    torch::Tensor rg;
    if (encode_angle_to_vector)
    {
        torch::Tensor rgx = vec_box_encodings.at(r__) + torch::cos(vec_anchors.at(r__));
        torch::Tensor rgy = vec_box_encodings.at(ry__) + torch::sin(vec_anchors.at(r__));
        rg = torch::atan2(rgy, rgx);
    }
    else
    {
        rg = vec_box_encodings.at(r__) + vec_anchors.at(r__);
    }

    //get cgs
    std::vector<torch::Tensor> cgs;
    if (box_ndim > 7)
    {
        std::vector<torch::Tensor>::iterator it_anchors = vec_anchors.begin();
        std::vector<torch::Tensor>::iterator it_box_encoding = vec_box_encodings.begin();
        int minimum_size = std::min(vec_anchors.size(), vec_box_encodings.size());

        it_anchors += r__ + 1;
        if (encode_angle_to_vector)
        {
            it_box_encoding += ry__ + 1;
        }
        else
        {
            it_box_encoding += r__ + 1;
        }
        while (it_anchors < vec_anchors.end() && it_box_encoding < vec_box_encodings.end())
        {
            cgs.emplace_back(*it_anchors + *it_box_encoding);
            it_box_encoding++;
            it_anchors++;
        }
    }
    else
    {
        cgs.clear();
    }

    std::vector<torch::Tensor> temp;
    temp.push_back(std::move(xg));
    temp.push_back(std::move(yg));
    temp.push_back(std::move(zg));
    temp.push_back(std::move(wg));
    temp.push_back(std::move(lg));
    temp.push_back(std::move(hg));
    temp.push_back(std::move(rg));

    std::vector<torch::Tensor>::iterator it;
    for (it = cgs.begin(); it < cgs.end(); it++)
    {
        temp.push_back(std::move(*it));
    }

    torch::Tensor res = torch::cat(temp, -1);
    temp.clear();
    return res;
}

torch::Tensor decode_torch(const torch::Tensor& box_encodings, const torch::Tensor& anchors){
    auto res = second_box_decode(box_encodings, anchors, false, false);
    return res;
}



torch::Tensor rotate_non_max_suppression_cpu( const torch::Tensor & t_box_corners,
                                              const torch::Tensor & t_order,
                                              const xt::xarray<float> & t_standup_iou,
                                             float thresh) {

    xt::xarray<float> box_corners = tensorToxTensor<float>(t_box_corners);
    xt::xarray<int> order1 = tensorToxTensor<int>(t_order.cpu());
    order1 = order1.reshape({-1});



    int ndets = t_box_corners.size(0);
    xt::xarray<float> box_corners_r = box_corners;
    xt::xarray<float> order_r = order1;
    xt::xarray<int> suppressed = xt::zeros<int>({ndets});
    xt::xarray<int> suppressed_rw = suppressed;
    xt::xarray<float> standup_iou_r = t_standup_iou;
    std::vector<int> keep;
    int i, j;
    namespace bg = boost::geometry;
    typedef bg::model::point<float, 2, bg::cs::cartesian> point_t;
    typedef bg::model::polygon<point_t> polygon_t;
    polygon_t poly, qpoly;
    std::vector<polygon_t> poly_inter, poly_union;
    float inter_area, union_area, overlap;

    for (int _i = 0; _i < ndets; ++_i) {
        i = order_r(_i);
        if (suppressed_rw(i) == 1)
            continue;
        keep.push_back(i);
        for (int _j = _i + 1; _j < ndets; ++_j) {
            j = order_r(_j);
            if (suppressed_rw(j) == 1)
                continue;
            if (standup_iou_r(i, j) <= 0.0)
                continue;
            try {
                bg::append(poly,
                           point_t(box_corners_r(i, 0, 0), box_corners_r(i, 0, 1)));
                bg::append(poly,
                           point_t(box_corners_r(i, 1, 0), box_corners_r(i, 1, 1)));
                bg::append(poly,
                           point_t(box_corners_r(i, 2, 0), box_corners_r(i, 2, 1)));
                bg::append(poly,
                           point_t(box_corners_r(i, 3, 0), box_corners_r(i, 3, 1)));
                bg::append(poly,
                           point_t(box_corners_r(i, 0, 0), box_corners_r(i, 0, 1)));
                bg::append(qpoly,
                           point_t(box_corners_r(j, 0, 0), box_corners_r(j, 0, 1)));
                bg::append(qpoly,
                           point_t(box_corners_r(j, 1, 0), box_corners_r(j, 1, 1)));
                bg::append(qpoly,
                           point_t(box_corners_r(j, 2, 0), box_corners_r(j, 2, 1)));
                bg::append(qpoly,
                           point_t(box_corners_r(j, 3, 0), box_corners_r(j, 3, 1)));
                bg::append(qpoly,
                           point_t(box_corners_r(j, 0, 0), box_corners_r(j, 0, 1)));

                bg::intersection(poly, qpoly, poly_inter);  //slow  jiaoji
            }catch (const std::exception &e) {
                std::cout << "box i corners:" << std::endl;
                for (int k = 0; k < 4; ++k) {
                    std::cout << box_corners_r(i, k, 0) << " " << box_corners_r(i, k, 1)
                              << std::endl;
                }
                std::cout << "box j corners:" << std::endl;
                for (int k = 0; k < 4; ++k) {
                    std::cout << box_corners_r(j, k, 0) << " " << box_corners_r(j, k, 1)
                              << std::endl;
                }
                continue;
            }
            if (!poly_inter.empty()) {
                inter_area = bg::area(poly_inter.front());
                bg::union_(poly, qpoly, poly_union);   //slow  bingji
                if (!poly_union.empty()) { // ignore invalid box
                    union_area = bg::area(poly_union.front());
                    overlap = inter_area / union_area;
                    if (overlap >= thresh)
                        suppressed_rw(j) = 1;
                    poly_union.clear();
                }
            }
            poly.clear();
            qpoly.clear();
            poly_inter.clear();
        }
    }
    int t = keep.size();
    torch::Tensor result = torch::tensor(keep).to(torch::kInt32).to(torch::kCUDA);
//    for (int k = 0; j < keep.size(); ++k) {
//        result[k] = keep[k];
//    }
//    std::cout << "keeps= \n"<<keep << std::endl;
//    std::cout << "result= \n"<<result << std::endl;
    return result;
}


torch::Tensor rotate_nms_cc_new(const torch::Tensor & dets_np, float thresh){


//    torch::Tensor c_scores = dets_np.index({"...", torch::tensor({5})}).to(torch::kCUDA);
//    torch::Tensor c_order = c_scores.argsort(0,true).to(torch::kInt32).to(torch::kCPU);

//    torch::Tensor c_scores = .to(torch::kCUDA);
    torch::Tensor c_order = dets_np.index({"...", torch::tensor({8})}).argsort(0,true).to(torch::kInt32);


    torch::Tensor centers =  dets_np.index({"...", torch::tensor({0,1})});
    torch::Tensor dims =  dets_np.index({"...", torch::tensor({2,3})});
    torch::Tensor angles = dets_np.index({"...", torch::tensor({6})});


    auto t1 = std::chrono::steady_clock::now();

    torch::Tensor dets_corners = center_to_corner_box2d(centers, dims, angles );

    auto t2 = std::chrono::steady_clock::now();
    double dr = std::chrono::duration<double, std::milli>(t2 - t1).count();
//    printf("center_to_corner_box2d cost  %f\n",dr);

    auto t3 = std::chrono::steady_clock::now();
    torch::Tensor dets_standup = corner_to_standup_nd(dets_corners);
    auto t4 = std::chrono::steady_clock::now();
    double dq = std::chrono::duration<double, std::milli>(t4 - t3).count();
//    printf("corner_to_standup_nd cost  %f\n",dq);

    auto t5 = std::chrono::steady_clock::now();
    xt::xarray<float> standup_iou = iou_jit(dets_standup, dets_standup, 0.0);
    auto t6 = std::chrono::steady_clock::now();
    double dw = std::chrono::duration<double, std::milli>(t6 - t5).count();
//    printf("iou_jit cost  %f\n",dw);

    auto t7 = std::chrono::steady_clock::now();

    torch::Tensor res5 = rotate_non_max_suppression_cpu(dets_corners,c_order,standup_iou, thresh);
//    std::cout << "res5 = \n"<< res5 << std::endl;
    auto t8 = std::chrono::steady_clock::now();
    double di = std::chrono::duration<double, std::milli>(t8 - t7).count();
//    printf("rotate_non_max_suppression_cpu cost  %f\n",di);

    return res5;
}


torch::Tensor rotate_nms_cc(const torch::Tensor & dets_np, float thresh){


//    torch::Tensor c_scores = dets_np.index({"...", torch::tensor({5})}).to(torch::kCUDA);
//    torch::Tensor c_order = c_scores.argsort(0,true).to(torch::kInt32).to(torch::kCPU);

//    torch::Tensor c_scores = .to(torch::kCUDA);
    torch::Tensor c_order = dets_np.index({"...", torch::tensor({5})}).argsort(0,true).to(torch::kInt32);


    torch::Tensor centers =  dets_np.index({"...", torch::tensor({0,1})});
    torch::Tensor dims =  dets_np.index({"...", torch::tensor({2,3})});
    torch::Tensor angles = dets_np.index({"...", torch::tensor({4})});


    auto t1 = std::chrono::steady_clock::now();

    torch::Tensor dets_corners = center_to_corner_box2d(centers, dims, angles );

    auto t2 = std::chrono::steady_clock::now();
    double dr = std::chrono::duration<double, std::milli>(t2 - t1).count();
//    printf("center_to_corner_box2d cost  %f\n",dr);

    auto t3 = std::chrono::steady_clock::now();
    torch::Tensor dets_standup = corner_to_standup_nd(dets_corners);
    auto t4 = std::chrono::steady_clock::now();
    double dq = std::chrono::duration<double, std::milli>(t4 - t3).count();
//    printf("corner_to_standup_nd cost  %f\n",dq);

    auto t5 = std::chrono::steady_clock::now();
    xt::xarray<float> standup_iou = iou_jit(dets_standup, dets_standup, 0.0);
    auto t6 = std::chrono::steady_clock::now();
    double dw = std::chrono::duration<double, std::milli>(t6 - t5).count();
//    printf("iou_jit cost  %f\n",dw);

    auto t7 = std::chrono::steady_clock::now();

    torch::Tensor res5 = rotate_non_max_suppression_cpu(dets_corners,c_order,standup_iou, thresh);
//    std::cout << "res5 = \n"<< res5 << std::endl;
    auto t8 = std::chrono::steady_clock::now();
    double di = std::chrono::duration<double, std::milli>(t8 - t7).count();
//    printf("rotate_non_max_suppression_cpu cost  %f\n",di);

    return res5;
}

torch::Tensor rotate_nms( const torch::Tensor & boxes_for_nms,
                          const torch::Tensor & top_scores,
                          int pre_max_sizes,
                          int post_max_sizes,
                          float iou_thresholds)
{

    torch::Tensor rbboxes = boxes_for_nms;
    torch::Tensor scores;
    std::tuple<torch::Tensor,torch::Tensor> topk;
    int num_keeped_scores = top_scores.size(0);
    pre_max_sizes = std::min(num_keeped_scores, pre_max_sizes);
    topk = torch::topk(top_scores, pre_max_sizes);
    scores = std::get<0>(topk);
    torch::Tensor indices = std::get<1>(topk);
    rbboxes = rbboxes.index({indices.to(torch::kLong)});
    torch::Tensor dets;
    dets = torch::cat({rbboxes, scores.unsqueeze(-1)},1);

    torch::Tensor dets_np =  dets.data().cpu();



    torch::Tensor ret = rotate_nms_cc(dets_np, iou_thresholds);


    torch::Tensor res2 = indices.index({ret.to(torch::kLong)}).to(torch::kInt32);

    return res2;

}

void VoxelNet::predict(torch::Tensor & example, std::map<std::string, torch::Tensor> & preds_dict)
{

    int batch_size = example.size(0);
    example = example.view({batch_size,-1,example.size(-1)});

    preds_dict["box_preds"] = preds_dict["box_preds"].view({batch_size,-1, 7});  //  7  =  m_box_coder.code_size


    int num_class_with_pg = _num_class;
    if (!_encode_background_as_zeros)
    {
        num_class_with_pg = _num_class + 1;
    }
    preds_dict["cls_preds"] = preds_dict["cls_preds"].view({batch_size,-1,num_class_with_pg});

    preds_dict["box_preds"]  = decode_torch(preds_dict["box_preds"],example);

    if (_use_direction_classifier)
    {
        preds_dict["dir_cls_preds"] = preds_dict["dir_cls_preds"].view({batch_size,-1,_num_direction_bins});
    }




    torch::Tensor post_center_range = torch::tensor(_post_center_range).to(preds_dict["box_preds"].dtype()).to(preds_dict["box_preds"].device());

    preds_dict["box_preds"] = preds_dict["box_preds"][0];
    preds_dict["cls_preds"] = preds_dict["cls_preds"][0];
    preds_dict["dir_cls_preds"] = preds_dict["dir_cls_preds"][0];


    torch::Tensor boxes_for_nms; //use
    torch::Tensor top_scores_keep; //use
    torch::Tensor selected; //use


    torch::Tensor dir_labels = std::get<1>(torch::max(preds_dict["dir_cls_preds"], -1));

    // int feature_map_size_prod = 21120;       //    feature_map_size_prod = batch_box_preds.size(1) / _target_assigner.num_anchors_per_location;
    std::tuple<torch::Tensor,torch::Tensor> top = torch::max(torch::sigmoid(preds_dict["cls_preds"]),-1);     //   std::get<0>(top):top_scores      std::get<1>(top):top_labels
    torch::Tensor top_scores = std::get<0>(top);
    torch::Tensor top_labels = std::get<1>(top);

    if (_nms_score_thresholds[0] > 0.0)
    {
        top_scores_keep = top_scores >= _nms_score_thresholds[0];
        top_scores = top_scores.masked_select(top_scores_keep);
    }
//    std::cout<<top_scores<<"\n";
    if (top_scores.size(0) != 0)
    {
        if (_nms_score_thresholds[0] > 0.0 )
        {
            preds_dict["box_preds"] = preds_dict["box_preds"].index({top_scores_keep.to(torch::kBool)});
//            if (_use_direction_classifier)
//            {
//                preds_dict["dir_cls_preds"] = preds_dict["dir_cls_preds"].index({top_scores_keep.to(torch::kBool)});
//            }
            dir_labels = dir_labels.index({top_scores_keep.to(torch::kBool)});
            top_labels = top_labels.index({top_scores_keep.to(torch::kBool)});
        }

        boxes_for_nms = preds_dict["box_preds"].index({"...", torch::tensor({0, 1, 3, 4, 6})});

        auto t1 = std::chrono::steady_clock::now();

        selected = rotate_nms(boxes_for_nms, top_scores, _nms_pre_max_sizes, _nms_post_max_sizes, _nms_iou_thresholds);
        auto t2 = std::chrono::steady_clock::now();
        double dr = std::chrono::duration<double, std::milli>(t2 - t1).count();

    }
    else{
        selected = torch::tensor({});
    }

//    std::cout<<selected<<"\n";

    preds_dict["box_preds"] =  preds_dict["box_preds"].index({selected.to(torch::kLong)});

    // finally generate predictions.
    if (preds_dict["box_preds"].size(0) != 0)
    {
        dir_labels = dir_labels.index({selected.to(torch::kLong)});
        double period = M_PI * 2 / _num_direction_bins;
        torch::Tensor dir_rot = (preds_dict["box_preds"].index({"...", torch::tensor({6})}) - _dir_offset) -
                                torch::floor(
                                        (preds_dict["box_preds"].index({"...", torch::tensor({6})}) - _dir_offset) /
                                        period + _dir_limit_offset) * period;
        torch::Tensor temp = dir_rot + _dir_offset + period * dir_labels.to(preds_dict["box_preds"].dtype()).reshape(
                {preds_dict["box_preds"].size(0), 1});

        preds_dict["box_preds"].slice(1, 6, 7) = dir_rot + _dir_offset + period * dir_labels.to(
                preds_dict["box_preds"].dtype()).reshape({preds_dict["box_preds"].size(0), 1});
        torch::Tensor mask = (preds_dict["box_preds"].index({"...", torch::tensor({0, 1, 2})})
                              >= post_center_range.slice(0, 0, 3)).to(torch::kBool).all(1);
        mask &= (preds_dict["box_preds"].index({"...", torch::tensor({0, 1, 2})})
                 <= post_center_range.slice(0, 3, preds_dict["box_preds"].size(1) - 1)).to(torch::kBool).all(1);


        preds_dict["box_preds"] = preds_dict["box_preds"].index({mask.to(torch::kBool)});
        preds_dict["cls_preds"] = top_labels.index({selected.to(torch::kLong)}).index({mask.to(torch::kBool)});
        preds_dict["dir_cls_preds"] = top_scores.index({selected.to(torch::kLong)}).index({mask.to(torch::kBool)});

    }else{
        preds_dict["cls_preds"] = preds_dict["cls_preds"].index({selected.to(torch::kLong)});
        preds_dict["dir_cls_preds"] = preds_dict["dir_cls_preds"].index({selected.to(torch::kLong)});
    }

}

void saveNpy(std::string path, torch::Tensor tens, int n){
    size_t inference_size = 1;
    std::vector<int> inference_shape = {};
    for(int i = 0; i < n; ++i){
        inference_size *= (size_t) tens.size(i);
        inference_shape.push_back((int) tens.size(i));
    }
    xt::xarray<float> outXtensor = xt::adapt(tens.to(torch::kCPU).data_ptr<float>(),
                                             inference_size,
                                             xt::no_ownership(),
                                             inference_shape);
    xt::dump_npy(path, outXtensor);

}

/***************************************************************
 * @file       VoxelNet.cpp
 * @brief      点云检测主函数
 * @input      points_xt:   输入点云，尺寸N * 4; 单位: 米
 *             batch_size:  输入batch，默认为1，当前版本不支持修改。
 * @return     点云检测结果， 尺寸N * 9 (x, y, z, w, l, h, yaw, cls, conf)， N可为0.单位: 米、弧度
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
xt::xarray<float> VoxelNet::RUN(xt::xarray<float> &points_xt)
{

    torch::Tensor points = torch::from_blob(points_xt.data(), {(int) points_xt.shape(0), (int) points_xt.shape(1)}, torch::kFloat32);
    points = points.to(torch::kCUDA);

    xt::xarray<float> output = xt::empty<float>({0, 9});

    /** ========================= Voxelize ====================*/
    torch::Tensor coors = generate_voxel(points, _voxel_size, _point_cloud_range);

    /** ========================= VFE-simple ====================*/
    std::vector<torch::Tensor> voxel_feature = DynamicSimpleVFE(points, coors);
    coors = torch::nn::functional::pad(voxel_feature[1],
                                       torch::nn::functional::PadFuncOptions({1, 0}).mode(torch::kConstant).value(0));

    if (!voxel_feature[0].size(0))
    {
        std::cout << "--zqj-debug-alg-vfe-faild"<< std::endl;
        return output;
    }

    /** ========================= MIDDLE ====================*/
    auto spatial_features = _middle_feature_extractor->forward(voxel_feature[0], coors);

    /** ========================= RPNV2 ====================*/
    auto preds_dict = _rpn->forward(spatial_features);

    /** ========================= MultiHead ====================*/
    auto small = _small_head->forward(preds_dict["out"]);
    auto twowheel = _twowheel_head->forward(preds_dict["out"]);
    auto large = _large_head->forward(preds_dict["out"]);
    auto fourwheel = _fourwheel_head->forward(preds_dict["out"]);

    preds_dict["box_preds"] = torch::cat(
        {fourwheel["box_preds"], large["box_preds"], twowheel["box_preds"], small["box_preds"]}, 1);
    preds_dict["cls_preds"] = torch::cat(
        {fourwheel["cls_preds"], large["cls_preds"], twowheel["cls_preds"], small["cls_preds"]}, 1);
    preds_dict["dir_cls_preds"] = torch::cat(
        {fourwheel["dir_cls_preds"], large["dir_cls_preds"], twowheel["dir_cls_preds"], small["dir_cls_preds"]}, 1);
    /** ======================== Predict ====================*/
    predict(_anchors, preds_dict);


    if (preds_dict["box_preds"].size(0) != 0) 
    {
        int is_postfilter = _pc_postfilter;
        if (is_postfilter)
        {
            // code zqj
            auto t_start = std::chrono::high_resolution_clock::now();
            std::cout << "--zqj-debug-alg-Voxel_filter_and_conut_num" << std::endl;
            auto cls_preds_cpu = preds_dict["cls_preds"].to(at::kInt).clone().to(at::kCPU);
            auto scores_cpu = preds_dict["dir_cls_preds"].to(at::kFloat).clone().to(at::kCPU);
            size_t box3D_size = (size_t) preds_dict["box_preds"].size(0) * preds_dict["box_preds"].size(1);
            std::vector<int> box3D_shape = {(int) preds_dict["box_preds"].size(0), (int) preds_dict["box_preds"].size(1)};
            xt::xarray<float> xtensor_3Dbox = xt::adapt(preds_dict["box_preds"].to(torch::kCPU).data_ptr<float>(), box3D_size, xt::no_ownership(), box3D_shape);
            xt::xarray<float> point_arr = tensorToxTensor<float>(points.to(torch::kCPU));
            std::vector<int> vector_cls_preds(cls_preds_cpu.data_ptr<int>(), cls_preds_cpu.data_ptr<int>() + cls_preds_cpu.numel());
            std::vector<float> vector_scores(scores_cpu.data_ptr<float>(), scores_cpu.data_ptr<float>() + scores_cpu.numel());
            //std::shared_ptr<Count_Box_Result> num_point_per_boxes_height_area = points_count_rbbox(point_arr, xtensor_3Dbox);
            // // num_point_per_boxes_height_area.first, num_point_per_boxes_height_area.second
            // std::cout << "--zqj-debug-alg-Voxel_count_num:" << num_point_per_boxes_height_area.first << std::endl;       // point_num in box
            // 
            // std::cout << "--zqj-debug-alg-Voxel_count_range_z:" << num_point_per_boxes_height_area->arr_result_area << std::endl;  // rang_z_array of area
            
            // int mode = 0;
            // std::vector<int> taked_indics =filter_label_for_KT(vector_cls_preds, vector_scores, mode);
            // std::cout << "--zqj-debug-alg-Voxel_filter_end:" << taked_indics.size()  << std::endl;
            
            // paras: label point score point_nem hegit area
          //   std::vector<int> taked_indics = filter_label_for_WJ(vector_cls_preds, xtensor_3Dbox, vector_scores, num_point_per_boxes_height_area->vec_result_point, num_point_per_boxes_height_area->vec_result_hight, num_point_per_boxes_height_area->arr_result_area);
            // std::cout << "--zqj-debug-alg-Voxel_filter_wj_iput&&output_num:" << vector_cls_preds.size()<< " && " << taked_indics.size() << std::endl;

           
           std::pair<xt::xarray<int>, xt::xarray<float>> num_point_per_boxes = points_count_rbbox2(point_arr, xtensor_3Dbox);
           
           std::vector<int> taked_indics = filter_label_forNuScenes(vector_cls_preds, xtensor_3Dbox, vector_scores, num_point_per_boxes.first, num_point_per_boxes.second, 0,_point_cloud_range);
            torch::Tensor taked_select = torch::from_blob(taked_indics.data(), taked_indics.size(), torch::TensorOptions().dtype(torch::kInt)).clone();
            preds_dict["box_preds"] = preds_dict["box_preds"].index({taked_select.to(torch::kLong)});

            // code zqj20230605  for vector2tensor
            torch::Tensor vector_cls_preds_tensor = torch::from_blob(vector_cls_preds.data(), vector_cls_preds.size(), torch::TensorOptions().dtype(torch::kInt)).clone();
            // std::cout<< "--zqj-debug-alg-new_label:" << vector_cls_preds_tensor << std::endl;
            preds_dict["cls_preds"] = vector_cls_preds_tensor.to(torch::kCUDA).index({taked_select.to(torch::kLong)});
            // preds_dict["cls_preds"] = preds_dict["cls_preds"].index({taked_select.to(torch::kLong)});

            preds_dict["dir_cls_preds"] = preds_dict["dir_cls_preds"].index({taked_select.to(torch::kLong)});

            auto t_end = std::chrono::high_resolution_clock::now();
            auto time_spend = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            std::cout<<"--zqj-debug-time all spend:  "<<time_spend<<" ms"<<std::endl;
        }
        torch::Tensor inference_res = torch::cat({preds_dict["box_preds"],
                                                preds_dict["cls_preds"].reshape({preds_dict["cls_preds"].size(0), 1}),
                                                preds_dict["dir_cls_preds"].reshape({preds_dict["dir_cls_preds"].size(0), 1})},
                                                    1);
        size_t inference_size = (size_t) inference_res.size(0) * inference_res.size(1);
        std::vector<int> inference_shape = {(int) inference_res.size(0), (int) inference_res.size(1)};
        output = xt::adapt(inference_res.to(torch::kCPU).data_ptr<float>(),
                                inference_size,
                                xt::no_ownership(),
                                inference_shape);
    }
    return output;
}