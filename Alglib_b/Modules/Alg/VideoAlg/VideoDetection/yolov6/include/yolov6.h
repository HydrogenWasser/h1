#include <fstream>
#include <iostream>
#include <sstream>
#include <numeric>
#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include "NvInfer.h"
#include "cuda_runtime_api.h"
#include "logging.h"
#include "yaml-cpp/yaml.h"

#include <opencv2/core/mat.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "IVideoAlgBase.h"
#include "TSelfVideoAlgParam.h"
#include "xtensor/xarray.hpp"

using namespace nvinfer1;

using namespace std;


#define DEVICE 0  // GPU id
#define NMS_THRESH 0.45
#define BBOX_CONF_THRESH 0.5
// stuff we know about the network and the input/output blobs
const int num_class = 14;
static const int INPUT_W = 736;
static const int INPUT_H = 416;



class yolov6 : public IVideoAlgBase {
public:
    yolov6(TSelfVideoAlgParam *m_AlgParams);
    ~yolov6();
    
    std::vector<xt::xarray<float>> RUN(std::vector<cv::Mat> & img_batch);
    xt::xarray<float> RUN(xt::xarray<float> &input) {}; 

private:
    // std::vector<std::string> class_names = {
    //     "person", "tool vehicle", "bicycle", "motorbike", "tricycle", "car", "Passenger car", "truck", "police car",
    //     "ambulance", "bus", "Dump truck", "tanker", "roadblock", "Fire car", "construction sign-board", "spillage", "rider"
    // };

    float* blob = nullptr;

    IRuntime* runtime;
    ICudaEngine* engine;
    IExecutionContext* context;
    // int output_size = 1;
    int iH, iW, in_size,maxBatchSize;
    std::vector<int> out_size;
    // float* prob;
    int* prob_num_dets; 
    float* prob_det_boxes; 
    float* prob_det_scores; 
    int* prob_det_classes;
    float scale;
    int img_w ;
    int img_h ;
    cv::Mat img; 
    void img_preprocess(cv::Mat& img); 
    std::string yolov6_weight;
    std::vector<float*>blobs;
};


