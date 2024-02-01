
#include <iostream>
#include <fstream>
#include <vector>

#include "cuda_runtime.h"

#include "NvInfer.h"
#include "NvOnnxConfig.h"
#include "NvOnnxParser.h"
#include "NvInferRuntime.h"

#include "pointpillar.h"

#define checkCudaErrors(status)                                   \
{                                                                 \
  if (status != 0)                                                \
  {                                                               \
    std::cout << "Cuda failure: " << cudaGetErrorString(status)   \
              << " at line " << __LINE__                          \
              << " in file " << __FILE__                          \
              << " error status: " << status                      \
              << std::endl;                                       \
              abort();                                            \
    }                                                             \
}

TRT::~TRT(void) {
    context_->destroy();
    engine_->destroy();
    checkCudaErrors(cudaEventDestroy(start_));
    checkCudaErrors(cudaEventDestroy(stop_));
    return;
}

TRT::TRT(std::string modelFile, cudaStream_t stream) : stream_(stream) {
    std::string modelCache = modelFile + ".cache";
    std::fstream trtCache(modelCache, std::ifstream::in);
    checkCudaErrors(cudaEventCreate(&start_));
    checkCudaErrors(cudaEventCreate(&stop_));
    if (!trtCache.is_open()) {
        std::cout << "Building TRT engine." << std::endl;
        // define builder
        auto builder = (nvinfer1::createInferBuilder(gLogger_));

        // define network
        const auto explicitBatch =
                1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        auto network = (builder->createNetworkV2(explicitBatch));

        // define onnxparser
        auto parser = (nvonnxparser::createParser(*network, gLogger_));
        if (!parser->parseFromFile(modelFile.data(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
            std::cerr << ": failed to parse onnx model file, please check the onnx version and trt support op!"
                      << std::endl;
            exit(-1);
        }

        // define config
        auto networkConfig = builder->createBuilderConfig();
#if defined (__arm64__) || defined (__aarch64__)
        networkConfig->setFlag(nvinfer1::BuilderFlag::kFP16);
        std::cout << "Enable fp16!" << std::endl;
#endif
        // set max batch size
        builder->setMaxBatchSize(1);
        // set max workspace
        networkConfig->setMaxWorkspaceSize(size_t(1) << 30);

        engine_ = (builder->buildEngineWithConfig(*network, *networkConfig));

        if (engine_ == nullptr) {
            std::cerr << ": engine init null!" << std::endl;
            exit(-1);
        }

        // serialize the engine, then close everything down
        auto trtModelStream = (engine_->serialize());
        std::fstream trtOut(modelCache, std::ifstream::out);
        if (!trtOut.is_open()) {
            std::cout << "Can't store trt cache.\n";
            exit(-1);
        }

        trtOut.write((char *) trtModelStream->data(), trtModelStream->size());
        trtOut.close();
        trtModelStream->destroy();

        networkConfig->destroy();
        parser->destroy();
        network->destroy();
        builder->destroy();

    } else {
        std::cout << "load TRT cache." << std::endl;
        char *data;
        unsigned int length;

        // get length of file:
        trtCache.seekg(0, trtCache.end);
        length = trtCache.tellg();
        trtCache.seekg(0, trtCache.beg);

        data = (char *) malloc(length);
        if (data == NULL) {
            std::cout << "Can't malloc data.\n";
            exit(-1);
        }

        trtCache.read(data, length);
        // create context
        auto runtime = nvinfer1::createInferRuntime(gLogger_);

        if (runtime == nullptr) {
            std::cout << "load TRT cache0." << std::endl;
            std::cerr << ": runtime null!" << std::endl;
            exit(-1);
        }
        //plugin_ = nvonnxparser::createPluginFactory(gLogger_);
        engine_ = (runtime->deserializeCudaEngine(data, length, 0));
        if (engine_ == nullptr) {
            std::cerr << ": engine null!" << std::endl;
            exit(-1);
        }
        free(data);
        trtCache.close();
    }

    context_ = engine_->createExecutionContext();
    return;
}

int TRT::doinfer(void **buffers) {
    int status;

    status = context_->enqueueV2(buffers, stream_, &start_);

    if (!status) {
        return -1;
    }

    return 0;
}


//PointPillar::PointPillar(std::string modelFile, cudaStream_t stream) : stream_(stream) {
PointPillar::PointPillar(TSelfPcAlgParam * AlgParams){
    YAML::Node pc_cfg = YAML::LoadFile(AlgParams->m_strRootPath + AlgParams->m_stPcAlgParam.m_strCfgPath);
    ppOnnxPath = AlgParams->m_strRootPath + pc_cfg["POINTPILLAR_MODEL"].as<std::string>();
    AlgParams->m_stPcAlgParam.m_vecPcClass = {pc_cfg["CLASS_NAMES"][0].as<std::string>(),
                                                pc_cfg["CLASS_NAMES"][1].as<std::string>(), pc_cfg["CLASS_NAMES"][2].as<std::string>(),
                                                pc_cfg["CLASS_NAMES"][3].as<std::string>(), pc_cfg["CLASS_NAMES"][4].as<std::string>(),
                                                pc_cfg["CLASS_NAMES"][5].as<std::string>(), pc_cfg["CLASS_NAMES"][6].as<std::string>()};

    checkCudaErrors(cudaEventCreate(&start_));
    checkCudaErrors(cudaEventCreate(&stop_));

    pre_.reset(new PreProcessCuda(stream_));
//    trt_.reset(new TRT(modelFile, stream_));
    trt_.reset(new TRT(ppOnnxPath, stream_));
    post_.reset(new PostProcessCuda(stream_));

    //input of pre-process
    voxel_features_size_ = MAX_VOXELS * params_.max_num_points_per_pillar * 4 * sizeof(float);
    voxel_num_points_size_ = MAX_VOXELS * sizeof(float);
    coords_size_ = MAX_VOXELS * 4 * sizeof(float);

    checkCudaErrors(cudaMallocManaged((void **) &voxel_features_, voxel_features_size_));
    checkCudaErrors(cudaMallocManaged((void **) &voxel_num_points_, voxel_num_points_size_));
    checkCudaErrors(cudaMallocManaged((void **) &coords_, MAX_VOXELS * 4 * sizeof(float)));

    checkCudaErrors(cudaMemsetAsync(voxel_features_, 0, voxel_features_size_, stream_));
    checkCudaErrors(cudaMemsetAsync(voxel_num_points_, 0, voxel_num_points_size_, stream_));
    checkCudaErrors(cudaMemsetAsync(coords_, 0, MAX_VOXELS * 4 * sizeof(float), stream_));


    //TRT-input
    features_input_size_ = MAX_VOXELS * params_.max_num_points_per_pillar * 10 * sizeof(float);
    checkCudaErrors(cudaMallocManaged((void **) &features_input_, features_input_size_));
    checkCudaErrors(cudaMallocManaged((void **) &params_input_, 5 * sizeof(unsigned int)));

    checkCudaErrors(cudaMemsetAsync(features_input_, 0, features_input_size_, stream_));
    checkCudaErrors(cudaMemsetAsync(params_input_, 0, 5 * sizeof(float), stream_));


    //output of TRT -- input of post-process
    cls_size_ =
            params_.feature_x_size * params_.feature_y_size * params_.num_classes * params_.num_anchors * sizeof(float);
    box_size_ = params_.feature_x_size * params_.feature_y_size * params_.num_box_values * params_.num_anchors *
                sizeof(float);
    dir_cls_size_ = params_.feature_x_size * params_.feature_y_size * params_.num_dir_bins * params_.num_anchors *
                    sizeof(float);
    checkCudaErrors(cudaMallocManaged((void **) &cls_output_, cls_size_));
    checkCudaErrors(cudaMallocManaged((void **) &box_output_, box_size_));
    checkCudaErrors(cudaMallocManaged((void **) &dir_cls_output_, dir_cls_size_));

    //output of post-process
    bndbox_size_ = (params_.feature_x_size * params_.feature_y_size * params_.num_anchors * 9 + 1) * sizeof(float);
    checkCudaErrors(cudaMallocManaged((void **) &bndbox_output_, bndbox_size_));

    //res.resize(100);
    res_.reserve(500);
    return;
}

PointPillar::~PointPillar(void) {
    pre_.reset();
    trt_.reset();
    post_.reset();

    checkCudaErrors(cudaFree(voxel_features_));
    checkCudaErrors(cudaFree(voxel_num_points_));
    checkCudaErrors(cudaFree(coords_));

    checkCudaErrors(cudaFree(features_input_));
    checkCudaErrors(cudaFree(params_input_));

    checkCudaErrors(cudaFree(cls_output_));
    checkCudaErrors(cudaFree(box_output_));
    checkCudaErrors(cudaFree(dir_cls_output_));

    checkCudaErrors(cudaFree(bndbox_output_));

    checkCudaErrors(cudaEventDestroy(start_));
    checkCudaErrors(cudaEventDestroy(stop_));
    return;
}


xt::xarray<float> PointPillar::RUN(xt::xarray<float> &points_xt)
{
    size_t points_size = points_xt.shape(0);

    //  input cuda
    float *points_data = nullptr;
    unsigned int points_data_size = points_size * 4 * sizeof(float);
    checkCudaErrors(cudaMallocManaged((void **) &points_data, points_data_size));
    checkCudaErrors(cudaMemcpy(points_data, (float *)(points_xt.data()), points_data_size, cudaMemcpyDefault));
    checkCudaErrors(cudaDeviceSynchronize());

    //  voxel
    pre_->generateVoxels((float *) points_data, points_size,
                         params_input_,
                         voxel_features_,
                         voxel_num_points_,
                         coords_);
    unsigned int params_input_cpu[5];
    checkCudaErrors(cudaMemcpy(params_input_cpu, params_input_, 5 * sizeof(unsigned int), cudaMemcpyDefault));

    pre_->generateFeatures(voxel_features_,
                           voxel_num_points_,
                           coords_,
                           params_input_,
                           features_input_);
    void *buffers[] = {features_input_, coords_, params_input_, cls_output_, box_output_, dir_cls_output_};

    //  forward
    trt_->doinfer(buffers);
    checkCudaErrors(cudaMemsetAsync(params_input_, 0, 5 * sizeof(unsigned int), stream_));

    //  Postprocess
    post_->doPostprocessCuda(cls_output_, box_output_, dir_cls_output_,
                             bndbox_output_);
    checkCudaErrors(cudaDeviceSynchronize());
    float obj_count = bndbox_output_[0];
    int num_obj = static_cast<int>(obj_count);
    auto output = bndbox_output_ + 1;
    for (int i = 0; i < num_obj; i++) {
        auto Bb = Bndbox(output[i * 9],
                         output[i * 9 + 1], output[i * 9 + 2], output[i * 9 + 3],
                         output[i * 9 + 4], output[i * 9 + 5], output[i * 9 + 6],
                         static_cast<int>(output[i * 9 + 7]),
                         output[i * 9 + 8]);
        res_.push_back(Bb);
    }

    //  nms, output: nms_pred
    nms_cpu(res_, params_.nms_thresh, nms_pred);
    res_.clear();
    checkCudaErrors(cudaFree(points_data));
//    std::cout << "box num: " << nms_pred.size() << std::endl;

    //  optput xtensor
    xt::xarray<float> output_array = xt::zeros<float>({int(nms_pred.size()), 9});
    if (nms_pred.size() != 0)
    {
        for (size_t j = 0; j < nms_pred.size(); ++j) {
            if (nms_pred[j].score > 0.3){
                output_array(j, 0) = nms_pred[j].x;
                output_array(j, 1) = nms_pred[j].y;
                output_array(j, 2) = nms_pred[j].z;
                output_array(j, 3) = nms_pred[j].l;
                output_array(j, 4) = nms_pred[j].w;
                output_array(j, 5) = nms_pred[j].h;
                output_array(j, 6) = (nms_pred[j].rt + M_PI_2) * -1;
                // std::cout << "output_array(j, 6): " << output_array(j, 6) * (180 / M_PI) << std::endl;
                // output_array(j, 6) = 1.57 - nms_pred[j].rt + M_PI;
                output_array(j, 7) = nms_pred[j].id;
                output_array(j, 8) = nms_pred[j].score;
            }
        }
    }
//   else
//    {
//        output_array = xt::empty<float>((0, 9));
//    }
    nms_pred.clear();

    return output_array;
}

