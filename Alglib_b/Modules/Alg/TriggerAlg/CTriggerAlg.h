/*******************************************************
 文件名：CTriggerAlg.h
 作者：
 描述：点云触发相机算法实现
 版本：v2.0
 日期：2023-04-10
 *******************************************************/

#pragma once
#ifndef TEST_CPP_TRIGGER
#define TEST_CPP_TRIGGER
#endif
#include "ITriggerAlg.h"
#include "TFusionResult.h"
#include "TPcResult.h"
#include "TSelfTriggerAlgParam.h"
#include "TTriggerAlgResult.h"
#include "TTrigAlgParam.h"


#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// 经纬度坐标点
struct GPSCoord
{
    double lon; //longitude
    double lat; //latitude
};

class CTriggerAlg :public ITriggerAlg
{
public:
    CTriggerAlg(const std::string& p_strOutPath);
    virtual ~CTriggerAlg();

    //设置初始化以及执行算法需要的各类参数，成功返回true，失败返回false
    virtual bool  SetAlgParam(const TSelfTriggerAlgParam* p_pAlgParam) override;

    //初始化算法接口对象，内部主要处理只需初始化一次的操作，比如模型加载之类的，成功返回true，失败返回false
    virtual bool  InitAlgorithm(void* p_pInitParam=nullptr) override;

    //执行算法函数，传入融合前的结果，算法执行成功返回融合后的结果（由算法类型而定），失败返回nullptr
    //参数1：数据源，目前约定传入 TFusionResult* 类型数据
    //参数2：融合算法类型，默认 FUSION_PC_VIDEO_WITH_TRACK
    //返回值：算法处理后的数据空间地址，根据算法类型不同可能有些差异，具体如下：
    //FUSION_PC_VIDEO_WITH_TRACK：返回 TFusionResult* 类型数据
    virtual void* RunAlgorithm(void* p_pSrcData, TRIGGER_ALG_TYPE p_eAlgType=TRIGGER) override;

private:
    bool read_trigger_xml(TSelfTriggerAlgParam* &m_stSelfTrigAlgParam, const char* trigger_xml_path);

    bool isPointinPolygon(GPSCoord& point, std::vector<GPSCoord>& rangelist);
    double calculate_distance(double jing1, double wei1, double jing2, double wei2);
    double get_angle(double lonA, double latA, double lonB, double latB);
    bool calculate_drop_feet(const GPSCoord& point,
                             const GPSCoord& line_point1,
                             const GPSCoord& line_point2,
                             GPSCoord& pOutPut);


    bool destination_point(double lon, double lat, double distance, double bearing, GPSCoord& OutPut);
    bool next_head_location(const GPSCoord& pre_center_location, double car_length, double car_width, double car_angle, GPSCoord& left_head,
                            GPSCoord& right_head);
    bool which_is_closer(GPSCoord& check_point, const TLanePara& Lane, int& index_min, std::string& angle_describe, double& min_dis);
    bool calcute_pixel_location(int colsest_point_key, std::string angle_describe, double sn_distance, double we_distance, const TLanePara& Lane, int* pOutPut);
    bool camera_trigger_alg(const TPcResult& cPcResult,
                            std::vector<uint16_t>& vecTriggeredId,
                            double angle_north_t,
                            unsigned long timeStamp,
                            unsigned int AlgDelaytime,
                            unsigned long frameTime,
                            TTriggerAlgResult& cTrigResult);

    const double JINGWEI_EPS = 0.00000000001; //经纬度坐标浮点数比较点位是否为同点，极小值


private:

//    TTrigAlgParam  m_tTrigAlgParam;//TSelfTriggerAlgParam
    TSelfTriggerAlgParam  *m_stSelfTrigAlgParam;//TSelfTriggerAlgParam

    std::vector<uint16_t> m_vecTriggeredId;// 记录触发起来的目标id

    // camera_trigger 主要算法函数用到的变量
    std::vector<GPSCoord> m_close_trigger_area; // = close_trigger_area
    std::vector<std::vector<GPSCoord>> m_lane_area_list; // 每条车道的触发区域
    std::vector<GPSCoord> m_assistant_point_list;       //# 用于对那种不在车道范围内的车的划分而准备的每个车道的左上角点（用来求距离）

    TTriggerAlgResult m_tTrigAlgResult;     //触发结果结构体

    std::ofstream m_outFile;

    std::string m_strOutPath;

    // std::vector<std::string> m_vecNoTrigger = {
    //         "person", "bicycle", "motorbicycle","food_bicycle","meituan" ,"motorbike","eleme","None"
    // };
    std::vector<std::string> m_vecNoTrigger = {
            "person","bicycle","tricycle","car","truck","bus","dump_truck","road_cones","others_bicycle_per",
            "MTfood_bicycle_per","ELMfood_bicycle_per","JD_tricycle","SF_tricycle","YZ_tricycle","YD_tricycle",
            "slagcar","tanker","road_work_sign","parking_tripod"
    };


};
