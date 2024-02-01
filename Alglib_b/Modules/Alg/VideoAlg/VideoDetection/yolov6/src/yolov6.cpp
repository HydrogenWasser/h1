#include "yolov6.h"
#include "xtensor/xbuilder.hpp"
#include "xtensor/xtensor_forward.hpp"
#include "xtensor/xview.hpp"
#include "Log/glog/logging.h"
const char *INPUT_BLOB_NAME = "images";
const char *OUTPUT_BLOB_NAME = "outputs";

#include "NvInferPlugin.h"
auto initP = initLibNvInferPlugins(nullptr, "");
#define CHECK(status)                                          \
    do                                                         \
    {                                                          \
        auto ret = (status);                                   \
        if (ret != 0)                                          \
        {                                                      \
            std::cerr << "Cuda failure: " << ret << std::endl; \
            abort();                                           \
        }                                                      \
    } while (0)

static Logger gLogger;

cv::Mat static_resize(cv::Mat &img)
{
    float r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
    int unpad_w = r * img.cols;
    int unpad_h = r * img.rows;
    cv::Mat re(unpad_h, unpad_w, CV_8UC3);
    cv::resize(img, re, re.size());
    cv::Mat out(INPUT_W, INPUT_H, CV_8UC3, cv::Scalar(114, 114, 114));
    re.copyTo(out(cv::Rect(0, 0, re.cols, re.rows)));
    return out;
}

struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

static inline float intersection_area(const Object &a, const Object &b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static void qsort_descent_inplace(std::vector<Object> &faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

#pragma omp parallel sections
    {
#pragma omp section
        {
            if (left < j)
                qsort_descent_inplace(faceobjects, left, j);
        }
#pragma omp section
        {
            if (i < right)
                qsort_descent_inplace(faceobjects, i, right);
        }
    }
}

static void qsort_descent_inplace(std::vector<Object> &objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

static void nms_sorted_bboxes(const std::vector<Object> &faceobjects, std::vector<int> &picked, float nms_threshold)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        const Object &a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            const Object &b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            // float IoU = inter_area / union_area
            if (inter_area / union_area > nms_threshold)
                keep = 0;
        }

        if (keep)
            picked.push_back(i);
    }
}

static void generate_yolo_proposals(float *feat_blob, int output_size, float prob_threshold, std::vector<Object> &objects)
{
    auto dets = output_size / (num_class + 5);
    for (int boxs_idx = 0; boxs_idx < dets; boxs_idx++)
    {
        const int basic_pos = boxs_idx * (num_class + 5);
        float x_center = feat_blob[basic_pos + 0];
        float y_center = feat_blob[basic_pos + 1];
        float w = feat_blob[basic_pos + 2];
        float h = feat_blob[basic_pos + 3];
        float x0 = x_center - w * 0.5f;
        float y0 = y_center - h * 0.5f;
        float box_objectness = feat_blob[basic_pos + 4];
        // std::cout<<*feat_blob<<std::endl;
        for (int class_idx = 0; class_idx < num_class; class_idx++)
        {
            float box_cls_score = feat_blob[basic_pos + 5 + class_idx];
            float box_prob = box_objectness * box_cls_score;
            if (box_prob > prob_threshold)
            {
                Object obj;
                obj.rect.x = x0;
                obj.rect.y = y0;
                obj.rect.width = w;
                obj.rect.height = h;
                obj.label = class_idx;
                obj.prob = box_prob;

                objects.push_back(obj);
            }

        } // class loop
    }
}

float *blobFromImage(cv::Mat &img)
{
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    float *blob = new float[img.total() * 3];
    int channels = 3;
    int img_h = img.rows;
    int img_w = img.cols;
    for (size_t c = 0; c < channels; c++)
    {
        for (size_t h = 0; h < img_h; h++)
        {
            for (size_t w = 0; w < img_w; w++)
            {
                blob[c * img_w * img_h + h * img_w + w] =
                    (((float)img.at<cv::Vec3b>(h, w)[c]) / 255.0f);
            }
        }
    }
    return blob;
}

static void decode_outputs(float *prob, int output_size, std::vector<Object> &objects, float scale, const int img_w, const int img_h)
{
    // auto postprocess_start = std::chrono::system_clock::now();

    std::vector<Object> proposals;
    generate_yolo_proposals(prob, output_size, BBOX_CONF_THRESH, proposals);

    qsort_descent_inplace(proposals);
    std::cout << "proposals nums:" << proposals.size() << std::endl;
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, NMS_THRESH);

    int count = picked.size();
    std::cout << "count: " << count << std::endl;
    objects.resize(count);
    for (int i = 0; i < count; i++)
    {
        objects[i] = proposals[picked[i]];

        // adjust offset to original unpadded
        // float x0 = (objects[i].rect.x) / scale;
        // float y0 = (objects[i].rect.y) / scale;
        // float x1 = (objects[i].rect.x + objects[i].rect.width) / scale;
        // float y1 = (objects[i].rect.y + objects[i].rect.height) / scale;

        float x0 = (objects[i].rect.x) / INPUT_W * img_w;
        float y0 = (objects[i].rect.y) / INPUT_H * img_h;
        float x1 = (objects[i].rect.x + objects[i].rect.width) / INPUT_W * img_w;
        float y1 = (objects[i].rect.y + objects[i].rect.height) / INPUT_H * img_h;

        // clip
        x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
        y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
        x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
        y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

        objects[i].rect.x = x0;
        objects[i].rect.y = y0;
        objects[i].rect.width = x1 - x0;
        objects[i].rect.height = y1 - y0;
    }

    // auto postprocess_end = std::chrono::system_clock::now();
    // double postprocess_time = std::chrono::duration_cast<std::chrono::milliseconds>(postprocess_end - postprocess_start).count();
    // std::cout <<"Yolov6 ---- postprocess Time:"<< postprocess_time << " ms" << std::endl;
}

//   doInference(*context, blob, prob, output_size, pr_img.size());
void doInference(IExecutionContext *context, std::vector<float *> input, int *prob_num_dets, float *prob_det_boxes, float *prob_det_scores, int *prob_det_classes, std::vector<int> out_size, int in_size, int maxBatchSize)
{
    // auto inference_start = std::chrono::system_clock::now();
    // const ICudaEngine& engine = context.getEngine();
    // assert(engine.getNbBindings() == 2);
    // void* buffers[2];

    // const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);

    // assert(engine.getBindingDataType(inputIndex) == nvinfer1::DataType::kFLOAT);    // nvinfer1::DataType::kFLOAT = 数据类型

    // const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);
    // assert(engine.getBindingDataType(outputIndex) == nvinfer1::DataType::kFLOAT);

    void *buffs[5];
    CHECK(cudaMalloc(&buffs[0], in_size * sizeof(float) * maxBatchSize));
    CHECK(cudaMalloc(&buffs[1], out_size[0] * sizeof(int) * maxBatchSize));
    CHECK(cudaMalloc(&buffs[2], out_size[1] * sizeof(float) * maxBatchSize));
    CHECK(cudaMalloc(&buffs[3], out_size[2] * sizeof(float) * maxBatchSize));
    CHECK(cudaMalloc(&buffs[4], out_size[3] * sizeof(int) * maxBatchSize));

    // CHECK(cudaMalloc(&buffers[inputIndex], 3 * input_shape.height * input_shape.width * sizeof(float)));
    // CHECK(cudaMalloc(&buffers[outputIndex], output_size*sizeof(float)));
    // Create stream
    cudaStream_t stream;
    CHECK(cudaStreamCreate(&stream));
    for (int i = 0; i < input.size(); i++)
    {
        CHECK(cudaMemcpyAsync(buffs[0] + in_size * i * sizeof(float), &(input[i][0]), in_size * sizeof(float), cudaMemcpyHostToDevice, stream));
    }
    context->enqueueV2(&buffs[0], stream, nullptr);
    CHECK(cudaMemcpyAsync(prob_num_dets, buffs[1], out_size[0] * sizeof(int) * maxBatchSize, cudaMemcpyDeviceToHost, stream));
    CHECK(cudaMemcpyAsync(prob_det_boxes, buffs[2], out_size[1] * sizeof(float) * maxBatchSize, cudaMemcpyDeviceToHost, stream));
    CHECK(cudaMemcpyAsync(prob_det_scores, buffs[3], out_size[2] * sizeof(float) * maxBatchSize, cudaMemcpyDeviceToHost, stream));
    CHECK(cudaMemcpyAsync(prob_det_classes, buffs[4], out_size[3] * sizeof(int) * maxBatchSize, cudaMemcpyDeviceToHost, stream));

    // CHECK(cudaMemcpyAsync(buffers[inputIndex], input, 3 * input_shape.height * input_shape.width * sizeof(float), cudaMemcpyHostToDevice, stream));
    // context.enqueue(1, buffers, stream, nullptr);
    // CHECK(cudaMemcpyAsync(output, buffers[outputIndex], output_size * sizeof(float), cudaMemcpyDeviceToHost, stream));

    cudaStreamSynchronize(stream);
    cudaStreamDestroy(stream);
    // CHECK(cudaFree(buffers[inputIndex]));
    // CHECK(cudaFree(buffers[outputIndex]));
    CHECK(cudaFree(buffs[0]));
    CHECK(cudaFree(buffs[1]));
    CHECK(cudaFree(buffs[2]));
    CHECK(cudaFree(buffs[3]));
    CHECK(cudaFree(buffs[4]));

    for (int i = 0; i < input.size(); i++)
    {
        delete input[i];
    }
    // delete input;
    // input = nullptr;
    // auto inference_end = std::chrono::system_clock::now();
    // double inference_time = std::chrono::duration_cast<std::chrono::milliseconds>(inference_end - inference_start).count();
    // std::cout <<"Yolov6 ---- inference Time:"<< inference_time << " ms" << std::endl;
}

yolov6::yolov6(TSelfVideoAlgParam *m_AlgParams)
{
    YAML::Node video_cfg = YAML::LoadFile(m_AlgParams->m_strRootPath + m_AlgParams->m_strVideoCfgPath);
    LOG(ERROR) << "video_cfg YOLOV6_CLASS_NAMES" << video_cfg["YOLOV6_CLASS_NAMES"] << endl;
    // m_AlgParams->m_stVideoAlgParam.m_vecVideoClass = {video_cfg["YOLOV6_CLASS_NAMES"][0].as<std::string>(),
    //         video_cfg["YOLOV6_CLASS_NAMES"][1].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][2].as<std::string>(),
    //         video_cfg["YOLOV6_CLASS_NAMES"][3].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][4].as<std::string>(),
    //         video_cfg["YOLOV6_CLASS_NAMES"][5].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][6].as<std::string>(),
    //         video_cfg["YOLOV6_CLASS_NAMES"][7].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][8].as<std::string>(),
    //         video_cfg["YOLOV6_CLASS_NAMES"][9].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][10].as<std::string>(),
    //         video_cfg["YOLOV6_CLASS_NAMES"][11].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][12].as<std::string>(),
    //         video_cfg["YOLOV6_CLASS_NAMES"][13].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][14].as<std::string>(),
    //         video_cfg["YOLOV6_CLASS_NAMES"][15].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][16].as<std::string>(),
    //         video_cfg["YOLOV6_CLASS_NAMES"][17].as<std::string>()};
    m_AlgParams->m_stVideoAlgParam.m_vecVideoClass = {video_cfg["YOLOV6_CLASS_NAMES"][0].as<std::string>(),
                                                      video_cfg["YOLOV6_CLASS_NAMES"][1].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][2].as<std::string>(),
                                                      video_cfg["YOLOV6_CLASS_NAMES"][3].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][4].as<std::string>(),
                                                      video_cfg["YOLOV6_CLASS_NAMES"][5].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][6].as<std::string>(),
                                                      video_cfg["YOLOV6_CLASS_NAMES"][7].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][8].as<std::string>(),
                                                      video_cfg["YOLOV6_CLASS_NAMES"][9].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][10].as<std::string>(),
                                                      video_cfg["YOLOV6_CLASS_NAMES"][11].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][12].as<std::string>(),
                                                      video_cfg["YOLOV6_CLASS_NAMES"][13].as<std::string>()};

    yolov6_weight = m_AlgParams->m_strRootPath + video_cfg["YOLOV6_WEIGHT"].as<std::string>();

    cudaSetDevice(DEVICE);
    char *trtModelStream{nullptr};
    size_t size{0};
    std::ifstream file(yolov6_weight, std::ios::binary);
    if (file.good())
    {
        file.seekg(0, file.end); // 让文件指针定位到文件末尾
        size = file.tellg();     // 得到文件指针当前指向的文件位置
        file.seekg(0, file.beg); // 让文件指针定位到文件开头
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
    }
    else
    {
        std::cout << "Yolov6 engine model path is empty." << std::endl;
        return;
    }

    runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    LOG(ERROR) << "engine deserialize success" << endl;

    auto in_dims = engine->getBindingDimensions(engine->getBindingIndex("images"));
    iH = in_dims.d[2];
    iW = in_dims.d[3];
    std::cout << "ih,iw:" << iH << " " << iW << std::endl;
    in_size = 1;
    for (int j = 1; j < in_dims.nbDims; j++)
    {
        in_size *= in_dims.d[j];
    }

    auto out_dims1 = engine->getBindingDimensions(engine->getBindingIndex("num_dets"));
    int out_size1 = 1;
    for (int j = 1; j < out_dims1.nbDims; j++)
    {
        out_size1 *= out_dims1.d[j];
    }
    out_size.push_back(out_size1);
    auto out_dims2 = engine->getBindingDimensions(engine->getBindingIndex("det_boxes"));
    int out_size2 = 1;
    for (int j = 1; j < out_dims2.nbDims; j++)
    {
        out_size2 *= out_dims2.d[j];
    }
    out_size.push_back(out_size2);
    auto out_dims3 = engine->getBindingDimensions(engine->getBindingIndex("det_scores"));
    int out_size3 = 1;
    for (int j = 1; j < out_dims3.nbDims; j++)
    {
        out_size3 *= out_dims3.d[j];
    }
    out_size.push_back(out_size3);
    auto out_dims4 = engine->getBindingDimensions(engine->getBindingIndex("det_classes"));
    int out_size4 = 1;
    for (int j = 1; j < out_dims4.nbDims; j++)
    {
        out_size4 *= out_dims4.d[j];
    }
    out_size.push_back(out_size4);

    context = engine->createExecutionContext();
    assert(context != nullptr);

    delete[] trtModelStream;

    // auto out_dims = engine->getBindingDimensions(1);

    // for(int j=0;j<out_dims.nbDims;j++) {
    //     output_size *= out_dims.d[j];
    //     LOG(ERROR)<< "out_dims.d[j]: "<< out_dims.d[j]<<endl;
    // }
    // LOG(ERROR)<< "out_dims.nbDims: "<< out_dims.nbDims<<endl;
    // prob = new float[output_size];
    maxBatchSize = 4;
    prob_num_dets = new int[out_size[0] * maxBatchSize];
    prob_det_boxes = new float[out_size[1] * maxBatchSize];
    prob_det_scores = new float[out_size[2] * maxBatchSize];
    prob_det_classes = new int[out_size[3] * maxBatchSize];
    LOG(ERROR) << "yolov6 init ------------" << endl;
}

yolov6::~yolov6()
{
    context->destroy();
    engine->destroy();
    runtime->destroy();
    // delete blob;
    // delete prob;
    delete prob_num_dets;
    delete prob_det_boxes;
    delete prob_det_scores;
    delete prob_det_classes;
}
// yuan xian tuxiang yu chuli fang fa
// void yolov6::img_preprocess(cv::Mat& img){
//     // auto preprocess_start = std::chrono::system_clock::now();
//     img_w = img.cols;
//     img_h = img.rows;
//     LOG(ERROR)<<"img.cols:"<<img.cols<<"img.rows:"<<img.rows<< endl;
//     scale = std::min(INPUT_W / (img.cols*1.0), INPUT_H / (img.rows*1.0));
//     img = static_resize(img);    // resize to 640*640
//     blob = blobFromImage(img);
//     // auto preprocess_end = std::chrono::system_clock::now();
//     // double preprocess_time = std::chrono::duration_cast<std::chrono::milliseconds>(preprocess_end - preprocess_start).count();
//     // std::cout <<"Yolov6 ---- Preprocess Time:"<< preprocess_time << " ms" << std::endl;

// }

// python dui ying yu chu li fang shi
void yolov6::img_preprocess(cv::Mat &img)
{
    // auto preprocess_start = std::chrono::system_clock::now();
    img_w = img.cols;
    img_h = img.rows;
    LOG(ERROR) << "img.cols:" << img.cols << "img.rows:" << img.rows << endl;
    scale = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
    cv::Mat re(INPUT_H, INPUT_W, CV_8UC3);
    cv::resize(img, re, re.size());
    img = re;
    blob = blobFromImage(img);
    // auto preprocess_end = std::chrono::system_clock::now();
    // double preprocess_time = std::chrono::duration_cast<std::chrono::milliseconds>(preprocess_end - preprocess_start).count();
    // std::cout <<"Yolov6 ---- Preprocess Time:"<< preprocess_time << " ms" << std::endl;
}

std::vector<xt::xarray<float>> yolov6::RUN(std::vector<cv::Mat> &img_batch)
{

    std::vector<xt::xarray<float>> out_put(img_batch.size());
    // std::cout<<"yolov6 run---------- " << std::endl;
    LOG(ERROR) << "yolov6 run---------- " << std::endl;
    std::cout << "before setBindingDimensions: " << context->getBindingDimensions(0).d[0] << " " << context->getBindingDimensions(0).d[1] << " " << context->getBindingDimensions(0).d[2] << " " << context->getBindingDimensions(0).d[3] << std::endl;
    context->setBindingDimensions(0, nvinfer1::Dims4(img_batch.size(), 3, 416, 736));
    std::cout << "after setBindingDimensions: " << context->getBindingDimensions(0).d[0] << " " << context->getBindingDimensions(0).d[1] << " " << context->getBindingDimensions(0).d[2] << " " << context->getBindingDimensions(0).d[3] << std::endl;

    for (int i = 0; i < img_batch.size(); ++i)
    {
        img = img_batch[i];
        img_preprocess(img);
        blobs.push_back(blob);
    }
    std::cout << "xcb---------inference begin" << std::endl;
    doInference(context, blobs, prob_num_dets, prob_det_boxes, prob_det_scores, prob_det_classes, out_size, in_size, maxBatchSize);
    for (int i = 0; i < img_batch.size(); ++i)
    {
        xt::xarray<float> boxes = xt::zeros<float>({prob_num_dets[i], 6});
        int k = 0;
        for (int j = 0; j < prob_num_dets[i]; j++)
        {
            float x0 = prob_det_boxes[i * out_size[1] + j * 4] / float(INPUT_W) * float(img_w);
            float y0 = prob_det_boxes[i * out_size[1] + j * 4 + 1] / float(INPUT_H) * float(img_h);
            float x1 = prob_det_boxes[i * out_size[1] + j * 4 + 2] / float(INPUT_W) * float(img_w);
            float y1 = prob_det_boxes[i * out_size[1] + j * 4 + 3] / float(INPUT_H) * float(img_h);

            x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
            y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
            x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
            y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

            float score = prob_det_scores[i * out_size[2] + j];
            int label = prob_det_classes[i * out_size[3] + j];

            boxes(k, 0) = x0;
            boxes(k, 1) = y0;
            boxes(k, 2) = x1;
            boxes(k, 3) = y1;
            boxes(k, 4) = score;
            boxes(k, 5) = label;
            k += 1;
        }
        boxes = xt::view(boxes, xt::range(0, k));
        out_put[i] = boxes;
    }
    blobs.clear();
    return out_put;

    // for (int i = 0; i < img_batch.size(); ++i)
    // {
    //     img = img_batch[i];
    //     img_preprocess(img);
    //     doInference(*context, blobs, prob, out_size, img.size());

    //     std::vector<Object> objects;
    //     decode_outputs(prob, output_size, objects, scale, img_w, img_h);

    //     // result
    //     xt::xarray<float> boxes = xt::zeros<float>({int(objects.size()), 6});
    //     int k = 0;
    //     for (size_t j = 0; j < objects.size(); j++)
    //     {
    //         const Object& obj = objects[j];
    //         std::cout<<"obj.rect:"<<obj.rect.width <<" "<<obj.rect.height<<std::endl;
    //         if (obj.rect.width > 1920 or obj.rect.width < 0 or
    //             obj.rect.height > 1080 or obj.rect.height < 0 or
    //             obj.rect.x < 0 or obj.rect.y < 0){
    //             continue;
    //         }

    //         boxes(k, 0) = obj.rect.x / 640 * 1920;
    //         boxes(k, 1) = obj.rect.y / 640 * 1080;
    //         boxes(k, 2) = obj.rect.width / 640 * 1920 + boxes(k, 0);
    //         boxes(k, 3) = obj.rect.height / 640 * 1080 + boxes(k, 1);
    //         boxes(k, 4) = obj.prob;
    //         boxes(k, 5) = obj.label;

    //         // boxes(k, 0) = obj.rect.x ;
    //         // boxes(k, 1) = obj.rect.y ;
    //         // boxes(k, 2) = obj.rect.width;
    //         // boxes(k, 3) = obj.rect.height;
    //         // boxes(k, 4) = obj.prob;
    //         // boxes(k, 5) = obj.label;
    //         k += 1;
    //     }
    //     boxes = xt::view(boxes, xt::range(0, k));
    //     out_put[i] = boxes;

    //     // for (size_t j = 0; j < objects.size(); j++)
    //     // {
    //     //     const Object& obj = objects[j];
    //     //     boxes(i, 0) = obj.rect.x;
    //     //     boxes(i, 1) = obj.rect.y;
    //     //     boxes(i, 2) = obj.rect.width;
    //     //     boxes(i, 3) = obj.rect.height;
    //     //     boxes(i, 4) = obj.prob;
    //     //     boxes(i, 5) = obj.label;
    //     // }
    //     // out_put[i] = boxes;
    // }
    // return out_put;
}