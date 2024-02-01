//
// Created by root on 3/10/21.
//

#include <iostream>
#include <vector>
#include <map>
#include <torch/script.h>
#include <torch/torch.h>
#include "rpn.h"
#include <numeric>

torch::nn::Conv2dOptions conv_options(int64_t in_planes, int64_t out_planes, int64_t kerner_size,
                                      int64_t stride = 1, int64_t padding = 0, int64_t groups = 1, bool with_bias=false){
    torch::nn::Conv2dOptions conv_options = torch::nn::Conv2dOptions(in_planes, out_planes, kerner_size);
    conv_options.stride(stride);
    conv_options.padding(padding);
    conv_options.groups(groups);
    conv_options.bias(with_bias);
    return conv_options;
}
torch::nn::ConvTranspose2dOptions convtran_option(int64_t in_planes, int64_t out_planes, int64_t kerner_size,
                                                  int64_t stride = 1, int64_t padding = 0, int64_t groups = 1, bool with_bias=false){
    torch::nn::ConvTranspose2dOptions convtran_option = torch::nn::ConvTranspose2dOptions(in_planes, out_planes, kerner_size) ;
    convtran_option.stride(stride);
    convtran_option.padding(padding);
    convtran_option.groups(groups);
    convtran_option.bias(with_bias);
    return convtran_option;
}


RPNV2::RPNV2(bool use_norm,
             int num_class,
             std::vector<int64_t> layer_nums,
             std::vector<int64_t> layer_strides,
             std::vector<int64_t> num_filters,
             std::vector<float> upsample_strides,
             std::vector<int64_t> num_upsample_filters,
             int64_t num_input_features,
             int64_t num_anchor_per_loc,
             bool encode_background_as_zeros,
             bool use_direction_classifier,
             bool use_groupnorm,
             int64_t num_groups,
             int64_t box_code_size,
             int64_t num_direction_bins,
             bool fuse_bn):
             _layer_strides(layer_strides),
             _num_filters(num_filters),
             _layer_nums(layer_nums),
             _upsample_strides(upsample_strides),
             _num_upsample_filters(num_upsample_filters),
             _num_input_features(num_input_features),
             _use_norm(use_norm),
             _use_groupnorm(use_groupnorm),
             _num_groups(num_groups),
             _num_anchor_per_loc(num_anchor_per_loc),
             _num_direction_bins(num_direction_bins),
             _num_class(num_class),
             _use_direction_classifier(use_direction_classifier),
             _box_code_size(box_code_size),
             _fuse_bn(fuse_bn){
    assert(layer_strides.size() == layer_nums.size());
    assert(num_filters.size() == layer_nums.size());
    assert(num_upsample_filters.size() == upsample_strides.size());
    _upsample_start_idx = layer_nums.size() - upsample_strides.size();
    std::vector<float> must_equal_list(2); // len(upsample_strides)
    must_equal_list[0] = upsample_strides[0] / layer_strides[0];
    must_equal_list[1] = upsample_strides[1] / (layer_strides[0] * layer_strides[1]);
    int num_cls;
    if (encode_background_as_zeros)
        num_cls = num_anchor_per_loc * num_class;
    else
        num_cls = num_anchor_per_loc * (num_class + 1);
    int final_num_filters;
    if (num_upsample_filters.size() == 0)
        final_num_filters = _num_out_filters;
    else
        final_num_filters = std::accumulate(num_upsample_filters.begin(), num_upsample_filters.end(), 0);

    /*_con_box = torch::nn::Conv2d(torch::nn::Conv2dOptions(256, 56, 1).stride(1).padding(0).groups(1).bias(
            true));
    _conv_dir_cls = torch::nn::Conv2d(torch::nn::Conv2dOptions(256, 16, 1).stride(1).padding(0).groups(1).bias(
            true));
    _con_cls = torch::nn::Conv2d(torch::nn::Conv2dOptions(256, 32, 1).stride(1).padding(0).groups(1).bias(
            true));*/



    /** ====================== block & deblock ===========================*/
    for (int i = 0; i < must_equal_list.size(); ++i) {
        assert(must_equal_list[i] == must_equal_list[0]);
    }
//    torch::nn::BatchNorm2d BatchNorm2d = torch::nn::BatchNorm2d()
    std::vector<int64_t> in_filters = {num_input_features, num_filters[0]};
//    std::vector<int64_t> num_out_filters = {64, 128}; /** kitti **/
    std::vector<int64_t> num_out_filters = {128, 256}; /** 32line **/
    if (!fuse_bn) {
        for (int i = 0; i < layer_nums.size(); ++i) {
            if (i == 0) {
                _block0 = make_layer(in_filters[i], num_filters[i], layer_nums[i], layer_strides[0], _fuse_bn);
            } else {
                _block1 = make_layer(in_filters[i], num_filters[i], layer_nums[i], layer_strides[1], _fuse_bn);
            }

            if (i - _upsample_start_idx >= 0) {
                float stride = upsample_strides[i - _upsample_start_idx];

                if (stride >= 1) {
                    int stride_ = int (stride);
                    torch::nn::Sequential deblock;
                    deblock->push_back(torch::nn::ConvTranspose2d(convtran_option(num_out_filters[i],
                                                                                  num_upsample_filters[i -
                                                                                                       _upsample_start_idx],
                                                                                  stride_, stride_)));
                    if (!_fuse_bn)
                        deblock->push_back(torch::nn::BatchNorm2d(
                                torch::nn::BatchNorm2dOptions(num_upsample_filters[i - _upsample_start_idx]).eps(
                                        0.001).momentum(0.01)));
                    deblock->push_back(torch::nn::ReLU());
                    if (i == 0) {
                        _deblock0 = deblock;
                    } else {
                        _deblock1 = deblock;
                    }

                } else {
                    // TODO : add BN fuse alg!!! 2021.04.07
                    int stride_ = std::round(1 / stride);
                    torch::nn::Sequential deblock;
                    deblock->push_back(torch::nn::Conv2d(conv_options(num_out_filters[i],
                                                                      num_upsample_filters[i - _upsample_start_idx],
                                                                      stride_, stride_)));
                    deblock->push_back(torch::nn::BatchNorm2d(
                            torch::nn::BatchNorm2dOptions(num_upsample_filters[i - _upsample_start_idx]).eps(
                                    0.001).momentum(0.01)));
                    deblock->push_back(torch::nn::ReLU());

                    if (i == 0) {
                        _deblock0 = deblock;
                    } else {
                        _deblock1 = deblock;
                    }
                }
            }
        }
    } else {
//        conv_options(int64_t in_planes, int64_t out_planes, int64_t kerner_size,
//                int64_t stride = 1, int64_t padding = 0, int64_t groups = 1, bool with_bias=false)
        _block0->push_back(torch::nn::ZeroPad2d(torch::nn::ZeroPad2dOptions(1)));
        // 1-3 -> 1 2
        _block0->push_back(torch::nn::Conv2d(conv_options(128, 64, 3, 1, 0, 1, true)));
        _block0->push_back(torch::nn::ReLU());
        // 4-6 -> 3 4
        _block0->push_back(torch::nn::Conv2d(conv_options(64, 64, 3, 1, 1, 1, true)));
        _block0->push_back(torch::nn::ReLU());
        // 7-9 -> 5 6
        _block0->push_back(torch::nn::Conv2d(conv_options(64, 64,3,1, 1, 1, true)));
        _block0->push_back(torch::nn::ReLU());
        // 10-12 -> 7 8
        _block0->push_back(torch::nn::Conv2d(conv_options(64,64,3,1,1,1,true)));
        _block0->push_back(torch::nn::ReLU());
        // 13-15 -> 9 10
        _block0->push_back(torch::nn::Conv2d(conv_options(64,64,3,1,1,1, true)));
        _block0->push_back(torch::nn::ReLU());
        // 16-18 -> 11 12
        _block0->push_back(torch::nn::Conv2d(conv_options(64,64,3,1,1,1, true)));
        _block0->push_back(torch::nn::ReLU());

        _block1->push_back(torch::nn::ZeroPad2d(torch::nn::ZeroPad2dOptions(1)));
        // 1-3 -> 1 2
        _block1->push_back(torch::nn::Conv2d(conv_options(64, 128, 3, 2, 0, 1, true)));
        _block1->push_back(torch::nn::ReLU());
        // 4-6 -> 3 4
        _block1->push_back(torch::nn::Conv2d(conv_options(128, 128, 3, 1, 1, 1, true)));
        _block1->push_back(torch::nn::ReLU());
        // 7-9 -> 5 6
        _block1->push_back(torch::nn::Conv2d(conv_options(128, 128, 3, 1, 1, 1, true)));
        _block1->push_back(torch::nn::ReLU());
        // 10-12 -> 7 8
        _block1->push_back(torch::nn::Conv2d(conv_options(128, 128, 3, 1, 1, 1, true)));
        _block1->push_back(torch::nn::ReLU());
        // 13-15 -> 9 10
        _block1->push_back(torch::nn::Conv2d(conv_options(128, 128, 3, 1, 1, 1, true)));
        _block1->push_back(torch::nn::ReLU());
        //16-18 -> 11 12
        _block1->push_back(torch::nn::Conv2d(conv_options(128, 128, 3, 1, 1, 1, true)));
        _block1->push_back(torch::nn::ReLU());

        _deblock0->push_back(torch::nn::ConvTranspose2d(convtran_option(64, 128, 1, 1,0, 1, true)));
        _deblock0->push_back(torch::nn::ReLU());

        _deblock1->push_back(torch::nn::ConvTranspose2d(convtran_option(128, 128, 2, 2, 0, 1, true)));
        _deblock1->push_back(torch::nn::ReLU());
    }
    _num_out_filters = num_out_filters[-1];
//    std::cout<<_block0<<"\n";
//    std::cout<<_block1<<"\n";
//    std::cout<<_deblock0<<"\n";
//    std::cout<<_deblock1<<"\n";
}
RPNV2::RPNV2() {}
torch::nn::Sequential RPNV2::make_layer(int64_t inplanes, int64_t planes, int64_t num_blocks, int stride, bool fuse_bn) {
    torch::nn::Sequential block;
    block->push_back(torch::nn::ZeroPad2d(torch::nn::ZeroPad2dOptions(1)));

//    block->push_back()
    if (_use_norm){
        block->push_back(torch::nn::Conv2d(conv_options(inplanes, planes, 3, stride)));
        if (!fuse_bn)
            block->push_back(torch::nn::BatchNorm2d(torch::nn::BatchNorm2dOptions(planes).eps(0.001).momentum(0.01)));
        block->push_back(torch::nn::ReLU());
        for (int i = 0; i < num_blocks; ++i) {
            block->push_back(torch::nn::Conv2d(conv_options(planes, planes, 3, 1, 1)));
            if (!fuse_bn)
                block->push_back(torch::nn::BatchNorm2d(torch::nn::BatchNorm2dOptions(planes).eps(0.001).momentum(0.01)));
            block->push_back(torch::nn::ReLU());
        }
    }
    return block;
}


void RPNV2::load_weight_fuse(std::map<std::string, torch::Tensor> &weight, bool eval) {

    for (int i = 1; i < 13; i=i+2) {
        // block0
        torch::nn::Conv2dImpl *conv_imp_0 = dynamic_cast<torch::nn::Conv2dImpl *>(_block0.ptr()->ptr(i).get());
        auto conv_w_0 = weight["b_0_" + std::to_string((i / 2) * 3 + 1) + "_w"];  // origin conv weight, no bias!
        auto bn_w_0 = weight["bn_0_" + std::to_string((i / 2) * 3 + 2) + "_w"]; // origin BN weight
        auto bn_b_0 = weight["bn_0_" + std::to_string((i / 2) * 3 + 2) + "_b"]; // origim BN bias
        auto bn_m_0 = weight["bn_0_" + std::to_string((i / 2) * 3 + 2) + "_m"]; // origin BN mean
        auto bn_v_0 = weight["bn_0_" + std::to_string((i / 2) * 3 + 2) + "_v"]; // origin BN var
        // fuse conv & BN weight bias!

        auto b_v_f0 = (bn_w_0 / torch::sqrt(bn_v_0 + 0.001)).to(torch::kCUDA);
        auto bias_f0 = torch::zeros(bn_m_0.sizes()).to(torch::kCUDA); // new bias
        bias_f0 = (bias_f0 - bn_m_0) / torch::sqrt(bn_v_0 + 0.001) * bn_w_0 + bn_b_0;
        for (int j = 0; j < conv_w_0.size(0); ++j) conv_w_0[j] *= b_v_f0[j];
        // new weight -> conv
        conv_imp_0->weight.set_data(conv_w_0.view_as(conv_imp_0->weight));
        conv_imp_0->bias.set_data(bias_f0.view_as(conv_imp_0->bias));

        // block1
        torch::nn::Conv2dImpl *conv_imp_1 = dynamic_cast<torch::nn::Conv2dImpl *>(_block1.ptr()->ptr(i).get());
        auto conv_w_1 = weight["b_1_" + std::to_string((i / 2) * 3 + 1) + "_w"];  // origin conv weight, no bias!
        auto bn_w_1 = weight["bn_1_" + std::to_string((i / 2) * 3 + 2) + "_w"]; // origin BN weight
        auto bn_b_1 = weight["bn_1_" + std::to_string((i / 2) * 3 + 2) + "_b"]; // origim BN bias
        auto bn_m_1 = weight["bn_1_" + std::to_string((i / 2) * 3 + 2) + "_m"]; // origin BN mean
        auto bn_v_1 = weight["bn_1_" + std::to_string((i / 2) * 3 + 2) + "_v"]; // origin BN var
        // fuse conv & BN weight bias!
//        std::cout<<"origin conv weight shape : "<<conv_w_0.sizes()<<std::endl;
//        std::cout<<"origin BN weight shape : "<<bn_w_0.sizes()<<std::endl;
        auto b_v_f1 = (bn_w_1 / torch::sqrt(bn_v_1 + 0.001)).to(torch::kCUDA);
        auto bias_f1 = torch::zeros(bn_m_1.sizes()).to(torch::kCUDA);
        bias_f1 = (bias_f1 - bn_m_1) / torch::sqrt(bn_v_1 + 0.001) * bn_w_1 + bn_b_1;
        for (int j = 0; j < conv_w_1.size(0); ++j) conv_w_1[j] *= b_v_f1[j];
        // new weight -> conv
        conv_imp_1->weight.set_data(conv_w_1.view_as(conv_imp_1->weight));
        conv_imp_1->bias.set_data(bias_f1.view_as(conv_imp_1->bias));
    }

    // deblock0 & deblock1
    torch::nn::ConvTranspose2dImpl *dconv_imp_0 = dynamic_cast<torch::nn::ConvTranspose2dImpl *>(_deblock0.ptr()->ptr(0).get());
    auto dconv_w_0 = weight["db_0_0_w"];  // origin conv weight, no bias!
    auto dbn_w_0 = weight["dbn_0_1_w"]; // origin BN weight
    auto dbn_b_0 = weight["dbn_0_1_b"]; // origim BN bias
    auto dbn_m_0 = weight["dbn_0_1_m"]; // origin BN mean
    auto dbn_v_0 = weight["dbn_0_1_v"]; // origin BN var
    // fuse conv & BN weight bias!
    auto db_v_f0 = (dbn_w_0 / torch::sqrt(dbn_v_0 + 0.001)).to(torch::kCUDA);
    auto dbias_f0 = torch::zeros(dbn_m_0.sizes()).to(torch::kCUDA); // new bias
    dbias_f0 = (dbias_f0 - dbn_m_0) / torch::sqrt(dbn_v_0 + 0.001) * dbn_w_0 + dbn_b_0;
    for (int j = 0; j < dconv_w_0.size(1); ++j) dconv_w_0.slice(1, j, j+1) *= db_v_f0[j]; // devonv is different from conv
    // new weight -> conv
    dconv_imp_0->weight.set_data(dconv_w_0.view_as(dconv_imp_0->weight));
    dconv_imp_0->bias.set_data(dbias_f0.view_as(dconv_imp_0->bias));
    // ==================
    torch::nn::ConvTranspose2dImpl *dconv_imp_1 = dynamic_cast<torch::nn::ConvTranspose2dImpl *>(_deblock1.ptr()->ptr(0).get());
    auto dconv_w_1 = weight["db_1_0_w"];  // origin conv weight, no bias!
    auto dbn_w_1 = weight["dbn_1_1_w"]; // origin BN weight
    auto dbn_b_1 = weight["dbn_1_1_b"]; // origim BN bias
    auto dbn_m_1 = weight["dbn_1_1_m"]; // origin BN mean
    auto dbn_v_1 = weight["dbn_1_1_v"]; // origin BN var
    // fuse conv & BN weight bias!
//    std::cout<<"origin conv weight shape : "<<dconv_w_0.sizes()<<std::endl;
//    std::cout<<"origin BN weight shape : "<<dbn_w_0.sizes()<<std::endl;
    auto db_v_f1 = (dbn_w_1 / torch::sqrt(dbn_v_1 + 0.001)).to(torch::kCUDA);
    auto dbias_f1 = torch::zeros(dbn_m_1.sizes()).to(torch::kCUDA); // new bias
    dbias_f1 = (dbias_f1 - dbn_m_1) / torch::sqrt(dbn_v_1 + 0.001) * dbn_w_1 + dbn_b_1;
    for (int j = 0; j < dconv_w_1.size(0); ++j) dconv_w_1.slice(1, j, j+1) *= db_v_f1[j];
    // new weight -> conv
    dconv_imp_1->weight.set_data(dconv_w_1.view_as(dconv_imp_1->weight));
    dconv_imp_1->bias.set_data(dbias_f1.view_as(dconv_imp_1->bias));

    // ====================== cls & box & dir
    _con_cls->weight.set_data(weight["cls_w"].view_as(_con_cls->weight));
    _con_cls->bias.set_data(weight["cls_b"].view_as(_con_cls->bias));
    _con_box->weight.set_data(weight["box_w"].view_as(_con_box->weight));
    _con_box->bias.set_data(weight["box_b"].view_as(_con_box->bias));
    _conv_dir_cls->weight.set_data(weight["dir_w"].view_as(_conv_dir_cls->weight));
    _conv_dir_cls->bias.set_data(weight["dir_b"].view_as(_conv_dir_cls->bias));
    if (eval){
        _block0->eval();
        _block1->eval();
        _deblock0->eval();
        _deblock1->eval();
        _con_cls->eval();
        _con_box->eval();
        _conv_dir_cls->eval();
    }

}



void RPNV2::load_weight(std::map<std::string, torch::Tensor> &weight, bool eval, bool fuse_bn) {
//    for(std::map<std::string, torch::Tensor>::iterator it = weight.begin(); it != weight.end(); ++it){
//        std::cout<< "key is : " << it->first<<std::endl;
//    }
    if (!fuse_bn) {
//        std::cout<< " ================== Don't fuse Conv & BN ==================" << std::endl;
//        std::cout<<_block0<<"\n";
//        std::cout<<_block1<<"\n";
        for (int j = 0; j < 2; ++j) {
            for (int i = 1; i < _block0->size(); i = i + 3) {

                // ================  conv  =================
                torch::nn::Conv2dImpl *conv_imp;
                if (j == 0)
                    conv_imp = dynamic_cast<torch::nn::Conv2dImpl *>(_block0.ptr()->ptr(i).get());
                else
                    conv_imp = dynamic_cast<torch::nn::Conv2dImpl *>(_block1.ptr()->ptr(i).get());


                conv_imp->weight.set_data(
                        weight["b_" + std::to_string(j) + "_" + std::to_string(i) + "_w"].view_as(conv_imp->weight));
                // =================  bn  =================
                torch::nn::BatchNorm2dImpl *bn_imp;
                if (j == 0)
                    bn_imp = dynamic_cast<torch::nn::BatchNorm2dImpl *>(_block0.ptr()->ptr(i + 1).get());
                else
                    bn_imp = dynamic_cast<torch::nn::BatchNorm2dImpl *>(_block1.ptr()->ptr(i + 1).get());

                bn_imp->weight.set_data(
                        weight["bn_" + std::to_string(j) + "_" + std::to_string(i + 1) + "_w"].view_as(bn_imp->weight));
                bn_imp->bias.set_data(
                        weight["bn_" + std::to_string(j) + "_" + std::to_string(i + 1) + "_b"].view_as(bn_imp->bias));
                bn_imp->running_mean.set_data(
                        weight["bn_" + std::to_string(j) + "_" + std::to_string(i + 1) + "_m"].view_as(
                                bn_imp->running_mean));
                bn_imp->running_var.set_data(
                        weight["bn_" + std::to_string(j) + "_" + std::to_string(i + 1) + "_v"].view_as(
                                bn_imp->running_var));
            }
        }
        /**
         take care here!!!
         if you use 32line data and config file,
         the deblock0 will be Conv block but not deConv!
         */

        // kitti rpn deblock
        /*for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; j = j + 2) {
//                std::cout<<_deblock0<<"\n";
//                std::cout<<_deblock1<<"\n";
                // ================ deconv ====================
                torch::nn::ConvTranspose2dImpl *deconv_imp;
                if (i == 0)
                    deconv_imp = dynamic_cast<torch::nn::ConvTranspose2dImpl *>(_deblock0.ptr()->ptr(j).get());
                else
                    deconv_imp = dynamic_cast<torch::nn::ConvTranspose2dImpl *>(_deblock1.ptr()->ptr(j).get());
                deconv_imp->weight.set_data(
                        weight["db_" + std::to_string(i) + "_" + std::to_string(j) + "_w"].view_as(deconv_imp->weight));

                // ================== deconv bn ==================
                torch::nn::BatchNorm2dImpl *dbn_imp;
                if (i == 0)
                    dbn_imp = dynamic_cast<torch::nn::BatchNorm2dImpl *>(_deblock0.ptr()->ptr(j + 1).get());
                else
                    dbn_imp = dynamic_cast<torch::nn::BatchNorm2dImpl *>(_deblock1.ptr()->ptr(j + 1).get());
                dbn_imp->weight.set_data(
                        weight["dbn_" + std::to_string(i) + "_" + std::to_string(j + 1) + "_w"].view_as(
                                dbn_imp->weight));
                dbn_imp->bias.set_data(
                        weight["dbn_" + std::to_string(i) + "_" + std::to_string(j + 1) + "_b"].view_as(dbn_imp->bias));
                dbn_imp->running_mean.set_data(
                        weight["dbn_" + std::to_string(i) + "_" + std::to_string(j + 1) + "_m"].view_as(
                                dbn_imp->running_mean));
                dbn_imp->running_var.set_data(
                        weight["dbn_" + std::to_string(i) + "_" + std::to_string(j + 1) + "_v"].view_as(
                                dbn_imp->running_var));
            }
        }*/

        // 32line rpn deblock
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; j = j + 2) {
//                std::cout<<_deblock0<<"\n";
//                std::cout<<_deblock1<<"\n";
                // ================ deconv ====================

                if (i == 0) {
                    torch::nn::Conv2dImpl *conv_imp;
                    conv_imp = dynamic_cast<torch::nn::Conv2dImpl *>(_deblock0.ptr()->ptr(0).get());
                    conv_imp->weight.set_data(
                            weight["db_" + std::to_string(i) + "_" + std::to_string(j) + "_w"].view_as(
                                    conv_imp->weight));
                } else{
                    torch::nn::ConvTranspose2dImpl *deconv_imp;
                    deconv_imp = dynamic_cast<torch::nn::ConvTranspose2dImpl *>(_deblock1.ptr()->ptr(j).get());
                    deconv_imp->weight.set_data(
                        weight["db_" + std::to_string(i) + "_" + std::to_string(j) + "_w"].view_as(deconv_imp->weight));
                }
//                else
//
//                    deconv_imp = dynamic_cast<torch::nn::ConvTranspose2dImpl *>(_deblock1.ptr()->ptr(j).get());
//                deconv_imp->weight.set_data(
//                        weight["db_" + std::to_string(i) + "_" + std::to_string(j) + "_w"].view_as(deconv_imp->weight));
//
                // ================== deconv bn ==================
                torch::nn::BatchNorm2dImpl *dbn_imp;
                if (i == 0)
                    dbn_imp = dynamic_cast<torch::nn::BatchNorm2dImpl *>(_deblock0.ptr()->ptr(j + 1).get());
                else
                    dbn_imp = dynamic_cast<torch::nn::BatchNorm2dImpl *>(_deblock1.ptr()->ptr(j + 1).get());
                dbn_imp->weight.set_data(
                        weight["dbn_" + std::to_string(i) + "_" + std::to_string(j + 1) + "_w"].view_as(
                                dbn_imp->weight));
                dbn_imp->bias.set_data(
                        weight["dbn_" + std::to_string(i) + "_" + std::to_string(j + 1) + "_b"].view_as(dbn_imp->bias));
                dbn_imp->running_mean.set_data(
                        weight["dbn_" + std::to_string(i) + "_" + std::to_string(j + 1) + "_m"].view_as(
                                dbn_imp->running_mean));
                dbn_imp->running_var.set_data(
                        weight["dbn_" + std::to_string(i) + "_" + std::to_string(j + 1) + "_v"].view_as(
                                dbn_imp->running_var));
            }
        }
    } else{
//        std::cout<< " ================== fuse Conv & BN ==================" << std::endl;
        for (int i = 1; i < 13; i=i+2) {
            // block0
            torch::nn::Conv2dImpl *conv_imp_0 = dynamic_cast<torch::nn::Conv2dImpl *>(_block0.ptr()->ptr(i).get());
            auto conv_w_0 = weight["b_0_" + std::to_string((i / 2) * 3 + 1) + "_w"];  // origin conv weight, no bias!
            auto bn_w_0 = weight["bn_0_" + std::to_string((i / 2) * 3 + 2) + "_w"]; // origin BN weight
            auto bn_b_0 = weight["bn_0_" + std::to_string((i / 2) * 3 + 2) + "_b"]; // origim BN bias
            auto bn_m_0 = weight["bn_0_" + std::to_string((i / 2) * 3 + 2) + "_m"]; // origin BN mean
            auto bn_v_0 = weight["bn_0_" + std::to_string((i / 2) * 3 + 2) + "_v"]; // origin BN var
            // fuse conv & BN weight bias!

            auto b_v_f0 = (bn_w_0 / torch::sqrt(bn_v_0 + 0.001)).to(torch::kCUDA);
            auto bias_f0 = torch::zeros(bn_m_0.sizes()).to(torch::kCUDA); // new bias
            bias_f0 = (bias_f0 - bn_m_0) / torch::sqrt(bn_v_0 + 0.001) * bn_w_0 + bn_b_0;
            for (int j = 0; j < conv_w_0.size(0); ++j) conv_w_0[j] *= b_v_f0[j];
            // new weight -> conv
            conv_imp_0->weight.set_data(conv_w_0.view_as(conv_imp_0->weight));
            conv_imp_0->bias.set_data(bias_f0.view_as(conv_imp_0->bias));

            // block1
            torch::nn::Conv2dImpl *conv_imp_1 = dynamic_cast<torch::nn::Conv2dImpl *>(_block1.ptr()->ptr(i).get());
            auto conv_w_1 = weight["b_1_" + std::to_string((i / 2) * 3 + 1) + "_w"];  // origin conv weight, no bias!
            auto bn_w_1 = weight["bn_1_" + std::to_string((i / 2) * 3 + 2) + "_w"]; // origin BN weight
            auto bn_b_1 = weight["bn_1_" + std::to_string((i / 2) * 3 + 2) + "_b"]; // origim BN bias
            auto bn_m_1 = weight["bn_1_" + std::to_string((i / 2) * 3 + 2) + "_m"]; // origin BN mean
            auto bn_v_1 = weight["bn_1_" + std::to_string((i / 2) * 3 + 2) + "_v"]; // origin BN var
            // fuse conv & BN weight bias!
//        std::cout<<"origin conv weight shape : "<<conv_w_0.sizes()<<std::endl;
//        std::cout<<"origin BN weight shape : "<<bn_w_0.sizes()<<std::endl;
            auto b_v_f1 = (bn_w_1 / torch::sqrt(bn_v_1 + 0.001)).to(torch::kCUDA);
            auto bias_f1 = torch::zeros(bn_m_1.sizes()).to(torch::kCUDA);
            bias_f1 = (bias_f1 - bn_m_1) / torch::sqrt(bn_v_1 + 0.001) * bn_w_1 + bn_b_1;
            for (int j = 0; j < conv_w_1.size(0); ++j) conv_w_1[j] *= b_v_f1[j];
            // new weight -> conv
            conv_imp_1->weight.set_data(conv_w_1.view_as(conv_imp_1->weight));
            conv_imp_1->bias.set_data(bias_f1.view_as(conv_imp_1->bias));
        }

        // deblock0 & deblock1
        torch::nn::ConvTranspose2dImpl *dconv_imp_0 = dynamic_cast<torch::nn::ConvTranspose2dImpl *>(_deblock0.ptr()->ptr(0).get());
        auto dconv_w_0 = weight["db_0_0_w"];  // origin conv weight, no bias!
        auto dbn_w_0 = weight["dbn_0_1_w"]; // origin BN weight
        auto dbn_b_0 = weight["dbn_0_1_b"]; // origim BN bias
        auto dbn_m_0 = weight["dbn_0_1_m"]; // origin BN mean
        auto dbn_v_0 = weight["dbn_0_1_v"]; // origin BN var
        // fuse conv & BN weight bias!
        auto db_v_f0 = (dbn_w_0 / torch::sqrt(dbn_v_0 + 0.001)).to(torch::kCUDA);
        auto dbias_f0 = torch::zeros(dbn_m_0.sizes()).to(torch::kCUDA); // new bias
        dbias_f0 = (dbias_f0 - dbn_m_0) / torch::sqrt(dbn_v_0 + 0.001) * dbn_w_0 + dbn_b_0;
        for (int j = 0; j < dconv_w_0.size(1); ++j) dconv_w_0.slice(1, j, j+1) *= db_v_f0[j]; // devonv is different from conv
        // new weight -> conv
        dconv_imp_0->weight.set_data(dconv_w_0.view_as(dconv_imp_0->weight));
        dconv_imp_0->bias.set_data(dbias_f0.view_as(dconv_imp_0->bias));
        // ==================
        torch::nn::ConvTranspose2dImpl *dconv_imp_1 = dynamic_cast<torch::nn::ConvTranspose2dImpl *>(_deblock1.ptr()->ptr(0).get());
        auto dconv_w_1 = weight["db_1_0_w"];  // origin conv weight, no bias!
        auto dbn_w_1 = weight["dbn_1_1_w"]; // origin BN weight
        auto dbn_b_1 = weight["dbn_1_1_b"]; // origim BN bias
        auto dbn_m_1 = weight["dbn_1_1_m"]; // origin BN mean
        auto dbn_v_1 = weight["dbn_1_1_v"]; // origin BN var
        // fuse conv & BN weight bias!
//        std::cout<<"origin conv weight shape : "<<dconv_w_0.sizes()<<std::endl;
//        std::cout<<"origin BN weight shape : "<<dbn_w_0.sizes()<<std::endl;
        auto db_v_f1 = (dbn_w_1 / torch::sqrt(dbn_v_1 + 0.001)).to(torch::kCUDA);
        auto dbias_f1 = torch::zeros(dbn_m_1.sizes()).to(torch::kCUDA); // new bias
        dbias_f1 = (dbias_f1 - dbn_m_1) / torch::sqrt(dbn_v_1 + 0.001) * dbn_w_1 + dbn_b_1;
        for (int j = 0; j < dconv_w_1.size(0); ++j) dconv_w_1.slice(1, j, j+1) *= db_v_f1[j];
        // new weight -> conv
        dconv_imp_1->weight.set_data(dconv_w_1.view_as(dconv_imp_1->weight));
        dconv_imp_1->bias.set_data(dbias_f1.view_as(dconv_imp_1->bias));
    }


    /**
     * 多检测头模块中，没有以下几个模块，因此将其注释
     * */
//    _con_cls->weight.set_data(weight["cls_w"].view_as(_con_cls->weight));
//    _con_cls->bias.set_data(weight["cls_b"].view_as(_con_cls->bias));
//    _con_box->weight.set_data(weight["box_w"].view_as(_con_box->weight));
//    _con_box->bias.set_data(weight["box_b"].view_as(_con_box->bias));
//    _conv_dir_cls->weight.set_data(weight["dir_w"].view_as(_conv_dir_cls->weight));
//    _conv_dir_cls->bias.set_data(weight["dir_b"].view_as(_conv_dir_cls->bias));

    if (eval){
        _block0->eval();
        _block1->eval();
        _deblock0->eval();
        _deblock1->eval();
//        _con_cls->eval();
//        _con_box->eval();
//        _conv_dir_cls->eval();
    }
}


std::map<std::string, torch::Tensor> RPNV2::forward(torch::Tensor &x) {
    std::map<std::string, torch::Tensor> res;
    std::map<int, torch::Tensor> ups, stage_outputs;

    x = _block0->forward(x);
    stage_outputs[0] = x;
    ups[0] = _deblock0->forward(x);
//    std::cout<<ups[0]<<std::endl;

    x = _block1->forward(x);
    stage_outputs[1] = x;
    ups[1] = _deblock1->forward(x);
    x = torch::cat({ups[0], ups[1]}, 1);

    res["out"] = x;

    /**===================== multi_head ======================*/
    return res;


    /*
     * multihead模块替换了以下内容，如果不使用multihead模块的话，
     *      1：注释掉前一个return
     *      2：打开以下注释
     *      3：在VoxelNet中注释掉multihead
     * */
    /**
    auto box_preds = _con_box->forward(x);
    auto cls_preds = _con_cls->forward(x);

    auto C = box_preds.size(1);
    auto H = box_preds.size(2);
    auto W = box_preds.size(3);



    box_preds = box_preds.view({-1, _num_anchor_per_loc, _box_code_size, H, W}).permute({0, 1, 3, 4, 2}).contiguous();
    cls_preds = cls_preds.view({-1, _num_anchor_per_loc, _num_class, H, W}).permute({0, 1, 3, 4, 2}).contiguous();
    res["box_preds"] = box_preds;
    res["cls_preds"] = cls_preds;
    if (_use_direction_classifier){
        auto dir_cls_preds = _conv_dir_cls(x);
        dir_cls_preds = dir_cls_preds.view({-1, _num_anchor_per_loc, _num_direction_bins, H, W}).permute({0, 1, 3, 4, 2}).contiguous();
//        std::cout<<dir_cls_preds<<std::endl;
        res["dir_cls_preds"] = dir_cls_preds;
    }

    return res;*/
}

void RPNV2::fuse_conv_bn(std::map<std::string, torch::Tensor> &weight, bool eval) {
}


DefaultHead::DefaultHead(int64_t num_filters, int num_class, int num_anchor_per_loc, int64_t box_code_size,
                         int64_t num_direction_bins, bool use_direction_classifier, bool encode_background_as_zeros,
                         std::string head_type, std::map<std::string, torch::Tensor> &weight_head):
                         _num_anchor_per_loc(num_anchor_per_loc),
                         _num_direction_bins(num_direction_bins),
                         _num_class(num_class),
                         _use_direction_classifier(use_direction_classifier),
                         _box_code_size(box_code_size),
                         _head_type(head_type){

    int64_t num_cls = encode_background_as_zeros ? num_anchor_per_loc * num_class : num_anchor_per_loc * (num_class + 1);
    int64_t final_num_filters = num_filters;
//    torch::nn::Conv2dOptions conv_options(int64_t in_planes, int64_t out_planes, int64_t kerner_size,
//                                          int64_t stride = 1, int64_t padding = 0, int64_t groups = 1, bool with_bias=false)
    _conv_cls->push_back(torch::nn::Conv2d(conv_options(final_num_filters, num_cls, 1, 1, 0, 1, true)));
    _conv_box->push_back(torch::nn::Conv2d(conv_options(final_num_filters, num_anchor_per_loc * box_code_size, 1, 1, 0, 1, true)));
    _conv_dir_cls->push_back(torch::nn::Conv2d(conv_options(final_num_filters, num_anchor_per_loc * num_direction_bins, 1, 1, 0, 1, true)));

//    std::cout<<_conv_box<<"\n"<<_conv_cls<<"\n"<<_conv_dir_cls<<"\n";
    load_weight(_head_type, weight_head);
}

DefaultHead::DefaultHead() {}
DefaultHead::~DefaultHead() {}

std::map<std::string, torch::Tensor> DefaultHead::forward(torch::Tensor &x) {
    int batch_size = x.size(0);
    torch::Tensor box_pred = _conv_box->forward(x);
    torch::Tensor cls_pred = _conv_cls->forward(x);
    torch::Tensor dir_cls_preds = _conv_dir_cls->forward(x);
    auto C = box_pred.size(1);
    auto H = box_pred.size(2);
    auto W = box_pred.size(3);

    box_pred = box_pred.view({-1, _num_anchor_per_loc, _box_code_size, H, W}).permute({0,1,3,4,2}).contiguous();
    cls_pred = cls_pred.view({-1, _num_anchor_per_loc, _num_class, H, W}).permute({0,1,3,4,2}).contiguous();
    dir_cls_preds = dir_cls_preds.view({-1, _num_anchor_per_loc, _num_direction_bins, H, W}).permute({0,1,3,4,2}).contiguous();
    std::map<std::string, torch::Tensor> ret_dict;
    ret_dict["box_preds"] = box_pred.view({batch_size, -1, _box_code_size});
    ret_dict["cls_preds"] = cls_pred.view({batch_size, -1, _num_class});
    ret_dict["dir_cls_preds"] = dir_cls_preds.view({batch_size, -1, _num_direction_bins});
    return ret_dict;
}

void DefaultHead::load_weight(std::string head_type, std::map<std::string, torch::Tensor> &weights) {

//    for (auto & it : weights) {
//        std::cout<<it.first<<"\n";
//        std::cout<<it.second.sizes()<<"\n";
//    }
    torch::nn::Conv2dImpl *conv_box = dynamic_cast<torch::nn::Conv2dImpl *>(_conv_box.ptr()->ptr(0).get());
    conv_box->weight.set_data(weights[head_type + ".conv_box.weight"].view_as(conv_box->weight));
    conv_box->bias.set_data(weights[head_type + ".conv_box.bias"].view_as(conv_box->bias));

    torch::nn::Conv2dImpl *conv_cls = dynamic_cast<torch::nn::Conv2dImpl *>(_conv_cls.ptr()->ptr(0).get());
    conv_cls->weight.set_data(weights[head_type + ".conv_cls.weight"].view_as(conv_cls->weight));
    conv_cls->bias.set_data(weights[head_type + ".conv_cls.bias"].view_as(conv_cls->bias));

    torch::nn::Conv2dImpl *conv_dir_cls = dynamic_cast<torch::nn::Conv2dImpl *>(_conv_dir_cls.ptr()->ptr(0).get());
    conv_dir_cls->weight.set_data(weights[head_type + ".conv_dir_cls.weight"].view_as(conv_dir_cls->weight));
    conv_dir_cls->bias.set_data(weights[head_type + ".conv_dir_cls.bias"].view_as(conv_dir_cls->bias));

}

std::vector<int64_t> RPNV2::get_num_upsample_filters() {
    return _num_upsample_filters;
}
