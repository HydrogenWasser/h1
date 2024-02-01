#include "yolov5.h"
#include "config.h"
#include <cstring>
#include "xtensor/xbuilder.hpp"
#include "xtensor/xtensor_forward.hpp"
#include "xtensor/xview.hpp"

using namespace nvinfer1;

void prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer,
                     float **gpu_output_buffer, float **cpu_output_buffer) {
  assert(engine->getNbBindings() == 2);
  // In order to bind the buffers, we need to know the names of the input and
  // output tensors. Note that indices are guaranteed to be less than
  // IEngine::getNbBindings()
  const int inputIndex = engine->getBindingIndex(kInputTensorName);
  const int outputIndex = engine->getBindingIndex(kOutputTensorName);
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  // Create GPU buffers on device
  CUDA_CHECK(cudaMalloc((void **)gpu_input_buffer,
                        kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
  CUDA_CHECK(cudaMalloc((void **)gpu_output_buffer,
                        kBatchSize * kOutputSize * sizeof(float)));

  *cpu_output_buffer = new float[kBatchSize * kOutputSize];
}

void infer(IExecutionContext &context, cudaStream_t &stream, void **gpu_buffers,
           float *output, int batchsize) {
  context.enqueue(batchsize, gpu_buffers, stream, nullptr);
  CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1],
                             batchsize * kOutputSize * sizeof(float),
                             cudaMemcpyDeviceToHost, stream));
  cudaStreamSynchronize(stream);
}

void serialize_engine(unsigned int max_batchsize, bool &is_p6, float &gd,
                      float &gw, std::string &wts_name,
                      std::string &engine_name) {
  // Create builder
  IBuilder *builder = createInferBuilder(gLogger);
  IBuilderConfig *config = builder->createBuilderConfig();

  // Create model to populate the network, then set the outputs and create an
  // engine
  ICudaEngine *engine = nullptr;
  if (is_p6) {
    engine = build_det_p6_engine(max_batchsize, builder, config,
                                 DataType::kFLOAT, gd, gw, wts_name);
  } else {
    engine = build_det_engine(max_batchsize, builder, config, DataType::kFLOAT,
                              gd, gw, wts_name);
  }
  assert(engine != nullptr);

  // Serialize the engine
  IHostMemory *serialized_engine = engine->serialize();
  assert(serialized_engine != nullptr);

  // Save engine to file
  std::ofstream p(engine_name, std::ios::binary);
  if (!p) {
    std::cerr << "Could not open plan output file" << std::endl;
    assert(false);
  }
  p.write(reinterpret_cast<const char *>(serialized_engine->data()),
          serialized_engine->size());

  // Close everything down
  engine->destroy();
  builder->destroy();
  config->destroy();
  serialized_engine->destroy();
}

void deserialize_engine(std::string &engine_name, IRuntime **runtime,
                        ICudaEngine **engine, IExecutionContext **context) {
  std::ifstream file(engine_name, std::ios::binary);
  if (!file.good()) {
    std::cerr << "read " << engine_name << " error!" << std::endl;
    assert(false);
  }
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  char *serialized_engine = new char[size];
  assert(serialized_engine);
  file.read(serialized_engine, size);
  file.close();

  *runtime = createInferRuntime(gLogger);
  assert(*runtime);
  *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
  assert(*engine);
  *context = (*engine)->createExecutionContext();
  assert(*context);
  delete[] serialized_engine;
}

yolov5::yolov5(TSelfVideoAlgParam *m_AlgParams) 
{
  YAML::Node video_cfg = YAML::LoadFile(m_AlgParams->m_strRootPath + m_AlgParams->m_strVideoCfgPath);
  m_AlgParams->m_stVideoAlgParam.m_vecVideoClass = {video_cfg["YOLOV5_CLASS_NAMES"][0].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][1].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][2].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][3].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][4].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][5].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][6].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][7].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][8].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][9].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][10].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][11].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][12].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][13].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][14].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][15].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][16].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][17].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][18].as<std::string>()};
  yolov5_type = video_cfg["YOLOV5_NET_TYPE"].as<std::string>();
  yolov5_weight = m_AlgParams->m_strRootPath + video_cfg["YOLOV5_WEIGHT"].as<std::string>();

  if (yolov5_type == "n")
  {
    gd = 0.33;
    gw = 0.25;
  }
  else if (yolov5_type == "s")
  {
    gd = 0.33;
    gw = 0.50;
  }
  else if (yolov5_type == "m")
  {
    gd = 0.67;
    gw = 0.75;
  }
  else if (yolov5_type == "l")
  {
    gd = 1.0;
    gw = 1.0;
  }
  else if (yolov5_type == "x")
  {
    gd = 1.33;
    gw = 1.25;
  }
  else
  {
    return;
  }

  if (yolov5_weight.empty())
  {
    std::cout << "Load Failed. Please check the weight or engine path.\n";
  }
  int split_index = yolov5_weight.find_last_of('.');

  std::string file_type = yolov5_weight.substr(split_index+ 1, yolov5_weight.size() - split_index - 1);
  if (file_type == "wts")
  {
    wts_name = yolov5_weight;
    engine_name = yolov5_weight.substr(0, split_index + 1) + "engine";
  }
  else
  {
    engine_name = yolov5_weight;
  }
  if (access(engine_name.data(), F_OK) != 0)
  {
    if (access(wts_name.data(), F_OK) != 0)
    {
      std::cout<< "The .wts and .engine both not found.\n";
      return;
    }
    else
    {
      serialize_engine(kBatchSize, is_p6, gd, gw, wts_name, engine_name);
    }
  }

  deserialize_engine(engine_name, &runtime, &engine, &context);

  CUDA_CHECK(cudaStreamCreate(&stream));
  cuda_preprocess_init(kMaxInputImageSize);

  prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);
}


yolov5::~yolov5() {
  cudaStreamDestroy(stream);
  CUDA_CHECK(cudaFree(gpu_buffers[0]));
  CUDA_CHECK(cudaFree(gpu_buffers[1]));
  delete[] cpu_output_buffer;
  cuda_preprocess_destroy();
  // Destroy the engine
  context->destroy();
  engine->destroy();
  runtime->destroy();
}

void yolov5::standard_box(std::vector<std::vector<yolov5Detection>> &input)
{
    float l, r, t, b;
    float r_w = kInputW / (kOriInputW * 1.0);
    float r_h = kInputH / (kOriInputH * 1.0);
    for (int i = 0; i < input.size(); ++i)
    {
        for (int j = 0; j < input[i].size(); ++j)
        {
            if (r_h > r_w)
            {
                l = input[i][j].bbox[0] - input[i][j].bbox[2] / 2.f;
                r = input[i][j].bbox[0] + input[i][j].bbox[2] / 2.f;
                t = input[i][j].bbox[1] - input[i][j].bbox[3] / 2.f - (kInputH - r_w * kOriInputH) / 2;
                b = input[i][j].bbox[1] + input[i][j].bbox[3] / 2.f - (kInputH - r_w * kOriInputH) / 2;
                l = l / r_w;
                r = r / r_w;
                t = t / r_w;
                b = b / r_w;
            }
            else
            {
                l = input[i][j].bbox[0] - input[i][j].bbox[2] / 2.f - (kInputW- r_h * kOriInputW) / 2;
                r = input[i][j].bbox[0] + input[i][j].bbox[2] / 2.f - (kInputW - r_h * kOriInputW) / 2;
                t = input[i][j].bbox[1] - input[i][j].bbox[3] / 2.f;
                b = input[i][j].bbox[1] + input[i][j].bbox[3] / 2.f;
                l = l / r_w;
                r = r / r_w;
                t = t / r_w;
                b = b / r_w;
            }
            input[i][j].rect = {l, t, r - l, b - t};
        }
    }
}

std::vector<xt::xarray<float>> yolov5::RUN(std::vector<cv::Mat> & img_batch){
  // auto start = std::chrono::system_clock::now();
  std::vector<xt::xarray<float>> out_put(img_batch.size());

  // Preprocess
  cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

  // Run inference
  
  infer(*context, stream, (void **)gpu_buffers, cpu_output_buffer, kBatchSize);

  // NMS
  std::vector<std::vector<yolov5Detection>> res_batch;
  batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize,
            kConfThresh, kNmsThresh);
  standard_box(res_batch);

  // // Save images
  // for (size_t i = 0; i < img_batch.size(); i++) {
  //   cv::Mat &img = img_batch[i];
  //   for (size_t j = 0; j < res_batch[i].size(); j++)
  //   {
  //       cv::Rect r = {int (res_batch[i][j].rect.x), int (res_batch[i][j].rect.y), int (res_batch[i][j].rect.width), int (res_batch[i][j].rect.height)};
  //       cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
  //       cv::putText(img, std::to_string((int)res_batch[i][j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
  //   }
  //   // cv::imwrite(std::to_string(i) + ".png", img);
  //   cv::imwrite("/data/image_test/frame_" + std::to_string(i + 3 * images_times) + ".png", img);
  //   std::cout << "save ************* /data/image_test/frame_" + std::to_string(i + 3 * images_times) + ".png" << std::endl;
  //   std::cout << "images_times" << images_times << std::endl;
  //   images_times = images_times + 1;
  // }

  for (int i = 0; i < res_batch.size(); ++i) {
    xt::xarray<float> boxes = xt::zeros<float>({int(res_batch[i].size()), 6});
	  int k = 0;
    for (int j = 0; j < res_batch[i].size(); ++j) {
      if (res_batch[i][j].rect.width > 640 or res_batch[i][j].rect.width < 0 or res_batch[i][j].rect.height > 640 or res_batch[i][j].rect.height < 0 or
          res_batch[i][j].rect.x < 0 or res_batch[i][j].rect.y < 0){
          continue;
      }

      boxes(k, 0) = res_batch[i][j].rect.x;
      boxes(k, 1) = res_batch[i][j].rect.y;
      boxes(k, 2) = res_batch[i][j].rect.width;
      boxes(k, 3) = res_batch[i][j].rect.height;
      boxes(k, 4) = res_batch[i][j].conf;
      boxes(k, 5) = res_batch[i][j].class_id;
	    k += 1;
    }
    boxes = xt::view(boxes, xt::range(0, k));
    out_put[i] = boxes;
  }
  // for (int i = 0; i < res_batch.size(); ++i) {
  //   xt::xarray<float> boxes = xt::zeros<float>({int(res_batch[i].size()), 6});
  //   for (int j = 0; j < res_batch[i].size(); ++j) {
  //     boxes(i, 0) = res_batch[i][j].rect.x;
  //     boxes(i, 1) = res_batch[i][j].rect.y;
  //     boxes(i, 2) = res_batch[i][j].rect.width;
  //     boxes(i, 3) = res_batch[i][j].rect.height;
  //     boxes(i, 4) = res_batch[i][j].conf;
  //     boxes(i, 5) = res_batch[i][j].class_id;
  //   }
  //   out_put[i] = boxes;
  // }
  return out_put;
}