#include "CTriggerAlg.h"
#include <cmath>
#include "CommonFunctions.h"
#include "Log/glog/logging.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;


CTriggerAlg::CTriggerAlg(const std::string& p_strExePath)
{
    m_strOutPath = p_strExePath;
    LOG(INFO)<<"m_strOutPath: "<<m_strOutPath<<endl;
    m_vecTriggeredId.reserve(30);                   //先固定好长度，避免多次内存拷贝。
    m_tTrigAlgResult.m_vecTriggeredId.reserve(30);  //m_vecTriggeredId
}

CTriggerAlg::~CTriggerAlg()
{
    // delete m_CamTrigger;
}

bool CTriggerAlg::InitAlgorithm(void* p_pInitParam)
{
    int num = 0;

    num = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum;      //车道数

    // 所有车道的区域 ,4个点，封闭区域
    m_close_trigger_area.clear();           //整个触发区域的右上点和左下点经纬度
    GPSCoord l_stTmp;
    l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[0].m_vecLanePoint[1].m_fLaneLon;
    l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[0].m_vecLanePoint[1].m_fLaneLat;
    m_close_trigger_area.push_back(l_stTmp);
    l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[0].m_vecLanePoint[2].m_fLaneLon;
    l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[0].m_vecLanePoint[2].m_fLaneLat;
    m_close_trigger_area.push_back(l_stTmp);
    l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[num-1].m_vecLanePoint[3].m_fLaneLon;
    l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[num-1].m_vecLanePoint[3].m_fLaneLat;
    m_close_trigger_area.push_back(l_stTmp);
    l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[num-1].m_vecLanePoint[0].m_fLaneLon;
    l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[num-1].m_vecLanePoint[0].m_fLaneLat;
    m_close_trigger_area.push_back(l_stTmp);
    l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[0].m_vecLanePoint[1].m_fLaneLon;
    l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[0].m_vecLanePoint[1].m_fLaneLat;
    m_close_trigger_area.push_back(l_stTmp);

    //每条车道的触发区域
    m_lane_area_list.clear();

    for (int i = 0; i < num; i++)
    {
        std::vector<GPSCoord> area;
        m_lane_area_list.push_back(area);
        m_lane_area_list[i].clear();
        l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[0].m_fLaneLon;
        l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[0].m_fLaneLat;
        m_lane_area_list[i].push_back(l_stTmp);
        l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[1].m_fLaneLon;
        l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[1].m_fLaneLat;
        m_lane_area_list[i].push_back(l_stTmp);
        l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[2].m_fLaneLon;
        l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[2].m_fLaneLat;
        m_lane_area_list[i].push_back(l_stTmp);
        l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[3].m_fLaneLon;
        l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[3].m_fLaneLat;
        m_lane_area_list[i].push_back(l_stTmp);
        l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[0].m_fLaneLon;
        l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[0].m_fLaneLat;
        m_lane_area_list[i].push_back(l_stTmp);
    }
    //用于对那种不在车道范围内的车的划分而准备的每个车道的左上角点（用来求距离）
    m_assistant_point_list.clear();
    for (int i = 0; i < num; i++)
    {
        l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[0].m_fLaneLon;
        l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[0].m_fLaneLat;
        m_assistant_point_list.push_back(l_stTmp);
        l_stTmp.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[1].m_fLaneLon;
        l_stTmp.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[1].m_fLaneLat;
        m_assistant_point_list.push_back(l_stTmp);

    }
    return true;
}

bool CTriggerAlg::SetAlgParam(const TSelfTriggerAlgParam* p_pAlgParam)
{
    if (p_pAlgParam)
    {
        m_stSelfTrigAlgParam = const_cast<TSelfTriggerAlgParam*>(p_pAlgParam);
        const char* xml_path = "/data/AlgLib/OutPut/Configs/Alg/CppConfigs/trigger/TriggerParam_1_1_qifusuidao.xml";
        read_trigger_xml(m_stSelfTrigAlgParam, xml_path);
        for (int i = 0; i < m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDx1_de =
                        m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDx1;
                m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDy1_de =
                        m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDy1;
                m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDx2_de =
                        m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDx2;
                m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDy2_de =
                        m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDy2;
            }
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[1].m_fPixelDx1 *= -1;
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[1].m_fPixelDy1 *= -1;
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[2].m_fPixelDx1 *= -1;
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[2].m_fPixelDy1 *= -1;

            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[0].m_fPixelDx1_de *= -1;
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[0].m_fPixelDy1_de *= -1;
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[3].m_fPixelDx1_de *= -1;
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[3].m_fPixelDy1_de *= -1;
        }

    }
    else
    {
        LOG(ERROR)<< "The incoming parameter is empty"<<endl;
        return false;
    }
    return true;
}

void* CTriggerAlg::RunAlgorithm(void* p_pSrcData, TRIGGER_ALG_TYPE p_eAlgType)
{
    if (!p_pSrcData)
    {
        return nullptr;
    }

    // const char* xml_path = "/data/AlgLib/OutPut/Configs/Alg/CppConfigs/trigger/TriggerParam_1_1_qifusuidao.xml";
    // read_trigger_xml(xml_path);

    bool bOk;
    TPcResult m_cPcResult = *((TPcResult *)p_pSrcData);

    unsigned long l_ulNowTime = std::chrono::high_resolution_clock::now().time_since_epoch().count()/1000000;
    unsigned long l_ulPcFinish = m_cPcResult.m_ulBufTimeStamp[2];

    bOk = camera_trigger_alg(m_cPcResult,
                             m_vecTriggeredId,
                             m_cPcResult.m_fLidarNorthAngle,
                             l_ulNowTime,
                             m_cPcResult.m_unBufDelay[0],
                             m_cPcResult.m_ulBufTimeStamp[0],
                             m_tTrigAlgResult);     //m_tTrigAlgResult

    return &m_tTrigAlgResult;
}

bool CTriggerAlg::read_trigger_xml(TSelfTriggerAlgParam* &m_stSelfTrigAlgParam, const char* trigger_xml_path){
    //创建xml文件对象，并读取xml
    XMLDocument doc;
    if (doc.LoadFile(trigger_xml_path) != tinyxml2::XML_SUCCESS) {
        std::cout << "加载 卡口XML 文件失败" << std::endl;
        return false;
    }

    // 查找 TriggerParam_1_1 节点
    tinyxml2::XMLElement *triggerParam = doc.FirstChildElement("TriggerParam_1_1");

    // 解析子节点的值
    // std::cout<<triggerParam->Value()<<std::endl;
    std::vector<double> limit_angle;
    const char* allLimitAngleDou = triggerParam->FirstChildElement("limit_angle")->GetText();
    char* limitAngleToken = strtok(const_cast<char*>(allLimitAngleDou),",");
    while(limitAngleToken!=nullptr)
    {
        double value = atof(limitAngleToken);
        limit_angle.push_back(value);
        limitAngleToken = strtok(nullptr,",");
    }
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLimitAngle_1 = limit_angle[0];
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLimitAngle_2 = limit_angle[1];
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLimitAngle_3 = limit_angle[2];
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLimitAngle_4 = limit_angle[3];

    const char* angleDeescribe1Str = triggerParam->FirstChildElement("angle_describe_1")->GetText();
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_strIncrease = angleDeescribe1Str;

    const char* angleDeescribe2Str = triggerParam->FirstChildElement("angle_describe_2")->GetText();
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_strDecrease = angleDeescribe2Str;

    int laneNum = 0;
    triggerParam->FirstChildElement("lane_num")->QueryIntText(&laneNum);
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum = laneNum;

    std::vector<double> longitude;
    const char* allPointLongitudeStr = triggerParam->FirstChildElement("all_point_longitude")->GetText();
    char* longitudeToken = strtok(const_cast<char*>(allPointLongitudeStr), ",");
    while (longitudeToken != nullptr)
    {
        double value = atof(longitudeToken);
        longitude.push_back(value);
        longitudeToken = strtok(nullptr, ",");
    }

    std::vector<double> latitude;
    const char* allPointLatitudeStr = triggerParam->FirstChildElement("all_point_latitude")->GetText();
    char* latitudeToken = strtok(const_cast<char*>(allPointLatitudeStr), ",");
    while (latitudeToken != nullptr)
    {
        double value = atof(latitudeToken);
        latitude.push_back(value);
        latitudeToken = strtok(nullptr, ",");
    }

    std::vector<int> pixelX;
    const char* allPointPixelXStr = triggerParam->FirstChildElement("all_point_pixel_x")->GetText();
    char* pixelXToken = strtok(const_cast<char*>(allPointPixelXStr), ",");
    while (pixelXToken != nullptr)
    {
        int value = atoi(pixelXToken);
        pixelX.push_back(value);
        pixelXToken = strtok(nullptr, ",");
    }

    std::vector<int> pixelY;
    const char* allPointPixelYStr = triggerParam->FirstChildElement("all_point_pixel_y")->GetText();
    char* pixelYToken = strtok(const_cast<char*>(allPointPixelYStr), ",");
    while (pixelYToken != nullptr)
    {
        int value = atoi(pixelYToken);
        pixelY.push_back(value);
        pixelYToken = strtok(nullptr, ",");
    }

    std::vector<double> increaseChangePixelX1;
    const char* increaseChangePixelX1Str = triggerParam->FirstChildElement("increase_change_pixel_x1")->GetText();
    char* increaseChangePixelX1Token = strtok(const_cast<char*>(increaseChangePixelX1Str), ",");
    while (increaseChangePixelX1Token != nullptr)
    {
        double value = atof(increaseChangePixelX1Token);
        increaseChangePixelX1.push_back(value);
        increaseChangePixelX1Token = strtok(nullptr, ",");
    }

    std::vector<double> increaseChangePixelY1;
    const char* increaseChangePixelY1Str = triggerParam->FirstChildElement("increase_change_pixel_y1")->GetText();
    char* increaseChangePixelY1Token = strtok(const_cast<char*>(increaseChangePixelY1Str), ",");
    while (increaseChangePixelY1Token != nullptr)
    {
        double value = atof(increaseChangePixelY1Token);
        increaseChangePixelY1.push_back(value);
        increaseChangePixelY1Token = strtok(nullptr, ",");
    }

    std::vector<double> increaseChangePixelX2;
    const char* increaseChangePixelX2Str = triggerParam->FirstChildElement("increase_change_pixel_x2")->GetText();
    char* increaseChangePixelX2Token = strtok(const_cast<char*>(increaseChangePixelX2Str), ",");
    while (increaseChangePixelX2Token != nullptr)
    {
        double value = atof(increaseChangePixelX2Token);
        increaseChangePixelX2.push_back(value);
        increaseChangePixelX2Token = strtok(nullptr, ",");
    }

    std::vector<double> increaseChangePixelY2;
    const char* increaseChangePixelY2Str = triggerParam->FirstChildElement("increase_change_pixel_y2")->GetText();
    char* increaseChangePixelY2Token = strtok(const_cast<char*>(increaseChangePixelY2Str), ",");
    while (increaseChangePixelY2Token != nullptr)
    {
        double value = atof(increaseChangePixelY2Token);
        increaseChangePixelY2.push_back(value);
        increaseChangePixelY2Token = strtok(nullptr, ",");
    }
    assert(longitude.size()==m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum*4);
    assert(latitude.size()==m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum*4);
    assert(pixelX.size()==m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum*4);
    assert(pixelY.size()==m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum*4);
    assert(increaseChangePixelX1.size()==m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum*4);
    assert(increaseChangePixelY1.size()==m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum*4);
    assert(increaseChangePixelX2.size()==m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum*4);
    assert(increaseChangePixelY2.size()==m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum*4);


    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly.resize(m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum);
    for(int i=0;i<m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly.size();++i)
    {
        m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint.resize(longitude.size());
        for(int j=0;j<longitude.size();++j)
        {
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fLaneLon = longitude[j];
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fLaneLat = latitude[j];
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_unPixelX = pixelX[j];
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_unPixelY = pixelY[j];
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDx1 = increaseChangePixelX1[j];
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDx2 = increaseChangePixelY1[j];
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDy1 = increaseChangePixelX2[j];
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_vecLanePoint[j].m_fPixelDy2 = increaseChangePixelY2[j];
        }
    }


    int cameraPixelX = 0;
    triggerParam->FirstChildElement("camera_pixel_x")->QueryIntText(&cameraPixelX);
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX = cameraPixelX;

    int cameraPixelY = 0;
    triggerParam->FirstChildElement("camera_pixel_y")->QueryIntText(&cameraPixelY);
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelY = cameraPixelY;

    float laneStandardAngle = 0;
    triggerParam->FirstChildElement("lane_standard_angle")->QueryFloatText(&laneStandardAngle);
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLaneStandardAngle = laneStandardAngle;

    float speedCorrectionFactor = 0;
    triggerParam->FirstChildElement("speed_correction_factor")->QueryFloatText(&speedCorrectionFactor);
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fSpeedCorrectionFactor = speedCorrectionFactor;

    float pointCloudDelayTime = 0;
    triggerParam->FirstChildElement("point_cloud_delay_time")->QueryFloatText(&pointCloudDelayTime);
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fPCDelayTime = pointCloudDelayTime;

    float disCorrectionFactor = 0;
    triggerParam->FirstChildElement("dis_correction_factor")->QueryFloatText(&disCorrectionFactor);
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fDisCorrectionFactor = disCorrectionFactor;

    const char* axisYTriggerMode = triggerParam->FirstChildElement("axis_y_trigger_mode")->GetText();
    const char* axisYTriggerModeSta = "固定y坐标";
    if (*axisYTriggerMode==*axisYTriggerModeSta)
    {
        m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_bAxisYTriggerMode = true;
    }

    const char* axisXTriggerMode = triggerParam->FirstChildElement("axis_x_trigger_mode")->GetText();
    const char* axisXTriggerModeSta = "固定x坐标";

    if (*axisXTriggerMode==*axisXTriggerModeSta)
    {
        m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_bAxisXTriggerMode = true;
    }

    std::vector<double> certainLinesYPixel;
    const char* certainLinesYPixelStr = triggerParam->FirstChildElement("certain_lines_y_pixel")->GetText();
    char* certainLinesYPixelStrToken = strtok(const_cast<char*>(certainLinesYPixelStr), ",");
    while (certainLinesYPixelStrToken != nullptr)
    {
        double value = atof(certainLinesYPixelStrToken);
        certainLinesYPixel.push_back(value);
        certainLinesYPixelStrToken = strtok(nullptr, ",");
    }

    for(int i=0;i<m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly.size();++i)
    {
        m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_nCertainLinesPixelY = certainLinesYPixel[i];
    }


    std::vector<double> certainPointsXPixel;
    const char* certainPointsXPixelStr = triggerParam->FirstChildElement("certain_points_x_pixel")->GetText();
    char* certainPointsXPixelStrToken = strtok(const_cast<char*>(certainPointsXPixelStr), ",");
    while (certainPointsXPixelStrToken != nullptr)
    {
        double value = atof(certainPointsXPixelStrToken);
        certainPointsXPixel.push_back(value);
        certainPointsXPixelStrToken = strtok(nullptr, ",");
    }

    for(int i=0;i<m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly.size();++i)
    {
        for(int j = 0;j<certainPointsXPixel.size();j = j+2)
        {
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_nCertainLinesPixelX[0] = certainPointsXPixel[j];
            m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_nCertainLinesPixelX[1] = certainPointsXPixel[j+1];
        }

    }

    std::vector<double> xShiftingPixel;
    const char* xShiftingPixelStr = triggerParam->FirstChildElement("x_shifting_pixel")->GetText();
    char* xShiftingPixelStrToken = strtok(const_cast<char*>(xShiftingPixelStr), ",");
    while (xShiftingPixelStrToken != nullptr)
    {
        double value = atof(xShiftingPixelStrToken);
        xShiftingPixel.push_back(value);
        xShiftingPixelStrToken = strtok(nullptr, ",");
    }

    for(int i=0;i<m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly.size();++i)
    {
        m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_nShiftingPixelX = xShiftingPixel[i];
    }

    // std::vector<double> yShiftingPixel;
    // const char* yShiftingPixelStr = triggerParam->FirstChildElement("y_shifting_pixel")->GetText();
    // char* yShiftingPixelStrToken = strtok(const_cast<char*>(yShiftingPixelStr), ",");
    // while (yShiftingPixelStrToken != nullptr)
    // {
    //     double value = atof(yShiftingPixelStrToken);
    //     yShiftingPixel.push_back(value);
    //     yShiftingPixelStrToken = strtok(nullptr, ",");
    // }

    // for(int i=0;i<m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly.size();++i)
    // {
    //     m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[i].m_nShiftingPixelX = yShiftingPixel[i];
    // }

    // const char* isLineOrSquare = triggerParam->FirstChildElement("is_line_or_square")->GetText();
    // m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_strIsLineOrSquare = isLineOrSquare;

    // int xSquareExtendSmall = 0;
    // triggerParam->FirstChildElement("x_square_extend_small")->QueryIntText(&xSquareExtendSmall);
    // m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendSmall = xSquareExtendSmall;

    // int xSquareExtendBig = 0;
    // triggerParam->FirstChildElement("x_square_extend_big")->QueryIntText(&xSquareExtendBig);
    // m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendBig = xSquareExtendBig;

    // int xSquareExtendUpSmall = 0;
    // triggerParam->FirstChildElement("y_square_extend_up_small")->QueryIntText(&xSquareExtendUpSmall);
    // m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendUpSmall = xSquareExtendUpSmall;

    // int xSquareExtendDownSmall = 0;
    // triggerParam->FirstChildElement("y_square_extend_down_small")->QueryIntText(&xSquareExtendDownSmall);
    // m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendDownSmall = xSquareExtendDownSmall;

    // int xSquareExtendUpBig = 0;
    // triggerParam->FirstChildElement("y_square_extend_up_big")->QueryIntText(&xSquareExtendUpBig);
    // m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendUpBig = xSquareExtendUpBig;

    // int xSquareExtendDownBig = 0;
    // triggerParam->FirstChildElement("y_square_extend_down_big")->QueryIntText(&xSquareExtendDownBig);
    // m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendDownBig = xSquareExtendDownBig;

    // int isBigOrSmall = 0;
    // triggerParam->FirstChildElement("is_big_or_small")->QueryIntText(&isBigOrSmall);
    // m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_bIsBigOrSmall = isBigOrSmall;

    // int isSPeed = 0;
    // triggerParam->FirstChildElement("is_speed")->QueryIntText(&isSPeed);
    // m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_bIsSpeed = isSPeed;

    // int speedLimit = 0;
    // triggerParam->FirstChildElement("speed_limit")->QueryIntText(&speedLimit);
    // m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unSpeedLimit = speedLimit;

    // int lengthLimit = 0;
    // triggerParam->FirstChildElement("length_limit")->QueryIntText(&lengthLimit);
    // m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLengthLimit = lengthLimit;

    const char* execute_location =triggerParam->FirstChildElement("execute_location")->GetText();
    m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_strExcuteLocation=execute_location;;

    return true;
}



bool CTriggerAlg::camera_trigger_alg(const TPcResult& PcResult,
                                           std::vector<uint16_t>& vecTriggeredId,
                                           double angle_north_t,
                                           unsigned long timeStamp,
                                           unsigned int AlgDelaytime,
                                           unsigned long frameTime,
                                           TTriggerAlgResult& cTrigResult
)
{
    if (PcResult.m_usBoxNum < 1)
    {
        LOG(ERROR) << "camera_trigger_alg is failed.\n";
        return false;
    }

    bool b_Ok = true;
    int recalculate_prehead_flag;

    int trigger_status_flag = 0;
    uint16_t new_single_car_id = 0;
    std::string new_single_car_type;
    double now_jing_origin;
    double now_wei_origin;

    double boxAngle, now_angle, origin_angle;
    double now_car_speed;
    double car_length, car_width;

    double pre_distance;

    double pre_center_jing, pre_center_wei;
    GPSCoord pre_center_location, pre_center_location2;
    GPSCoord head_center_location, head_center_location2;
    GPSCoord now_location_origin;

    bool now_location_origin_check;
    bool next_head_center_check;

    GPSCoord next_left_head_location, next_right_head_location;
    GPSCoord next_left_head_location2, next_right_head_location2;
    int lane_index;
    int cur_idx;

    unsigned short l_BoxAng = 0; //boxAngle*100


    unsigned int lane_num = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLaneNum;
    std::vector<double> point_car_dis_list;
    point_car_dis_list.reserve(10); //最多三个车道，先给10个空间

    std::vector<double>::iterator itMin;
    double point_car_dis;
    double min_dis;
    int min_dis_index;

    TLanePara all_points_dic;
    GPSCoord far_trigger_line_point_1;
    GPSCoord far_trigger_line_point_2;
    GPSCoord close_trigger_line_point_1;
    GPSCoord close_trigger_line_point_2;

    GPSCoord drop_result;

    int certain_lines_y;
    int certain_points_x[2];
    int x_shifting;
    int y_shifting;
    int left_closest_point_key, right_closest_point_key;
    std::string left_angle_describe, right_angle_describe;
    double left_min_dis,right_min_dis;
    double right_sn_dis, right_we_dis;
    double left_sn_dis, left_we_dis;
    int right_pixelXY[2];  // 对应python  right_x right_y
    int left_pixelXY[2];  // 对应python  left  left
    std::vector<int> left_right_x_pixel_list(2);
    std::vector<int> left_right_y_pixel_list(2);

    std::vector<int> left_pixel_location(2);
    std::vector<int> right_pixel_location(2);

    unsigned short l_usTrigCnt = 0;
    unsigned int l_unFrameDelay = timeStamp - frameTime;  // 第0包时间戳到现在的时间延迟


    TTriggerBoxInfo l_cCTriggerBoxInfo;
    cTrigResult.m_vecTriggerBoxInfo.clear();
    // cout << "usBoxNum=" << PcResult.m_usBoxNum <<endl;

    for (int i = 0; i < PcResult.m_usBoxNum; i++)
    {
        trigger_status_flag = 0;
        new_single_car_id = PcResult.m_vecBox[i].m_usSingleId;
        new_single_car_type = PcResult.m_vecBox[i].m_strClass;
        now_jing_origin = PcResult.m_vecBox[i].m_dLon;
        now_wei_origin = PcResult.m_vecBox[i].m_dLat;
        now_location_origin.lon = now_jing_origin;
        now_location_origin.lat = now_wei_origin;



#ifdef TEST_TRIGGERALG
        boxAngle = fmod(PcResult.m_vecBox[i].usCourseAngle(), 360.0);
//        m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fPCDelayTime=0.1;
#else

        if (std::find(vecTriggeredId.begin(), vecTriggeredId.end(), new_single_car_id) != vecTriggeredId.end())
        {
        //    cout << "continue 1 id=" << new_single_car_id <<endl;
            continue;
        }
        if (std::find(m_vecNoTrigger.begin(), m_vecNoTrigger.end(), new_single_car_type) != m_vecNoTrigger.end())
        {
        //    cout << "continue 2 m_vecNoTrigger=" << new_single_car_type <<endl;
            continue;
        }

        boxAngle = fmodf((float)(PcResult.m_vecBox[i].m_usCourseAngle + (int)PcResult.m_fLidarNorthAngle + 180), 360.0);
#endif

        now_angle = boxAngle;
        now_car_speed = (PcResult.m_vecBox[i].m_usSpeed) / 100.0;
        car_length = (PcResult.m_vecBox[i].m_usLength) / 100.0;
        car_width = (PcResult.m_vecBox[i].m_usWidth) / 100.0;
        origin_angle = boxAngle;
        l_BoxAng = (uint16_t)(boxAngle * 100);


        // 计算预测距离，用到了速度修正系数和点云算法的延迟
        pre_distance = now_car_speed * m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fSpeedCorrectionFactor
                       * m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fPCDelayTime  + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fDisCorrectionFactor;

        //预测的车身中心经纬度
        b_Ok = destination_point(now_jing_origin, now_wei_origin, pre_distance, now_angle , pre_center_location);
        if (!b_Ok)
        {
            LOG(ERROR) << "[camera_trigger_alg]  destination_point() ---1 \n";
            return false;
        }

        now_location_origin_check = isPointinPolygon(now_location_origin, m_close_trigger_area); // 检查现在的实际位置是否进入了近端触发区域


        b_Ok = next_head_location(pre_center_location, car_length, car_width, now_angle, next_left_head_location, next_right_head_location);
        if (!b_Ok)
        {
            LOG(ERROR) << "[camera_trigger_alg]  next_head_location() ---1 \n";
            return false;
        }

        if (now_location_origin_check)
        {
            if (m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLaneStandardAngle - 15 > now_angle ||
                now_angle > m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLaneStandardAngle + 15 )
            {
                recalculate_prehead_flag = 1;
                //重新计算预测位置的左右车头和车头中心点
                now_angle = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLaneStandardAngle;
                destination_point(now_jing_origin, now_wei_origin, pre_distance, now_angle, pre_center_location);
                next_head_location(pre_center_location, car_length, car_width, now_angle, next_left_head_location, next_right_head_location);
            }
        }

        //计算预测的左右车头的中心位置
//        [head_center_location_jing, head_center_location_wei] = destination_point(pre_center_jing,pre_center_wei,car_length / 2, now_angle)
//        head_center_location = [head_center_location_jing, head_center_location_wei]
        destination_point(pre_center_location.lon,pre_center_location.lat,(double)car_length / 2, now_angle, head_center_location);
        next_head_center_check = isPointinPolygon(head_center_location, m_close_trigger_area);

        //实际位置、预测车头中心位置只要有一个在近端触发区域，就可以了
        if (now_location_origin_check ||  next_head_center_check)
        {
            if ((!now_location_origin_check )&& next_head_center_check) {
                trigger_status_flag = 1; //标准状态
            }else if (now_location_origin_check && next_head_center_check) {
                trigger_status_flag = 1; //标准状态
            }else if (now_location_origin_check && (!next_head_center_check)) {
                if (fabs(m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLaneStandardAngle - origin_angle ) > 15.0 ){
                    trigger_status_flag = 3; //车辆突然闪现且角度不对
                } else {
                    trigger_status_flag = 2; //车辆突然闪现且角度正确
                }
            }
        }
        else
        {
            continue;
        }

        std::vector<GPSCoord> tmpp;
        // 根据预测位置确定其所在车道lane_number
        lane_index = 0;
        for (int j = 0; j < lane_num; j++)
        {
            tmpp =  m_lane_area_list[j];
            if (isPointinPolygon(head_center_location, tmpp))
            {
                lane_index = j + 1;
                break;
            }
        }
        //若还是没有得到车道号，且车辆突然闪现，则根据原始位置确定车道号
        if (lane_index == 0 && (trigger_status_flag == 2 || trigger_status_flag == 3))
        {
            for (int j = 0; j < lane_num; j++)
            {
                tmpp =  m_lane_area_list[j];
                if (isPointinPolygon(now_location_origin, tmpp))
                {
                    lane_index = j + 1;
                    break;
                }
            }
        }

        if (lane_index == 0 && (trigger_status_flag == 2 || trigger_status_flag == 3))
        {
            point_car_dis_list.clear();
//            vector<double>().swap(point_car_dis_list);
            for (int j = 0; j < m_assistant_point_list.size(); j++)
            {
                point_car_dis = calculate_distance(now_location_origin.lon, now_location_origin.lat, m_assistant_point_list[j].lon,
                                                   m_assistant_point_list[j].lat);
                point_car_dis_list.push_back(point_car_dis);
            }

            itMin = std::min_element(point_car_dis_list.begin(), point_car_dis_list.end());
            min_dis = (double)*itMin;
            min_dis_index = (int)std::distance(point_car_dis_list.begin(), itMin);
            lane_index = int(min_dis_index / 2) + 1;
        }

        if (0 == lane_index)
        {
            destination_point(now_jing_origin, now_wei_origin, pre_distance, m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLaneStandardAngle, pre_center_location2);
            next_head_location(pre_center_location2, car_length, car_width, m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLaneStandardAngle,
                               next_left_head_location2, next_right_head_location2);
            head_center_location2.lon = 0.5 * (next_left_head_location2.lon + next_right_head_location2.lon);
            head_center_location2.lat = 0.5 * (next_left_head_location2.lat + next_right_head_location2.lat);
            for (int j = 0; j < lane_num; j++)
            {
                if (isPointinPolygon(head_center_location2, m_lane_area_list[j]))
                {
                    lane_index = j + 1;
                    break;
                }
            }
        }

        //若经过处理后还是没有得到车道号，则计算与各车道距离，强行匹配
        if (0 == lane_index)
        {
            point_car_dis_list.clear();
            for (int j = 0; j < m_assistant_point_list.size(); j++)
            {
                point_car_dis = calculate_distance(head_center_location.lon, head_center_location.lat, m_assistant_point_list[j].lon,
                                                   m_assistant_point_list[j].lat);
                point_car_dis_list.push_back(point_car_dis);
            }

            itMin = std::min_element(point_car_dis_list.begin(), point_car_dis_list.end());
            min_dis = (double)*itMin;
            min_dis_index = (int)std::distance(point_car_dis_list.begin(), itMin);
            lane_index = int(min_dis_index / 2) + 1;
        }


        if (lane_index < 1)
        {
            LOG(ERROR) << "[camera_trigger_alg]  lane_index < 1 \n";
            return false;
        }

        cur_idx = lane_index - 1;//当前车道索引号

        //--------------------------函数里的局部变量从全局列表、字典获取，被改从全局结构体变量获取---------------
        all_points_dic = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx];

        //--第n个车道的 1 2 3 4点----
        far_trigger_line_point_1.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[0].m_fLaneLon;
        far_trigger_line_point_1.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[0].m_fLaneLat;
        far_trigger_line_point_2.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[1].m_fLaneLon;
        far_trigger_line_point_2.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[1].m_fLaneLat;

        close_trigger_line_point_1.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[3].m_fLaneLon;
        close_trigger_line_point_1.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[3].m_fLaneLat;
        close_trigger_line_point_2.lon = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[2].m_fLaneLon;
        close_trigger_line_point_2.lat = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[2].m_fLaneLat;

        certain_lines_y = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_nCertainLinesPixelY;
        certain_points_x[0] = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_nCertainLinesPixelX[0];
        certain_points_x[1] = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_nCertainLinesPixelX[1];
        x_shifting = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_nShiftingPixelX;
        y_shifting = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_nShiftingPixelY;

        //如果没有超过近端线的,则与14个位置对比求最近点（计算左右车头）
        b_Ok = which_is_closer(next_left_head_location, all_points_dic, left_closest_point_key, left_angle_describe, left_min_dis);
        if (!b_Ok)
        {
            LOG(ERROR) << "[camera_trigger_alg]  which_is_closer  ---1 \n";
            return false;
        }
        b_Ok = which_is_closer(next_right_head_location, all_points_dic, right_closest_point_key, right_angle_describe, right_min_dis);
        if (!b_Ok)
        {
            LOG(ERROR) << "[camera_trigger_alg]  which_is_closer  ---2 \n";
            return false;
        }

        if (right_closest_point_key <= 1)
        {
            calculate_drop_feet(next_right_head_location, far_trigger_line_point_1, far_trigger_line_point_2, drop_result);

            //求右车头相对于特征点在南北方向的距离(垂足和最近特征点的距离)
            right_sn_dis = calculate_distance(drop_result.lon, drop_result.lat,
                                              m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[right_closest_point_key].m_fLaneLon,
                                              m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[right_closest_point_key].m_fLaneLat);
        }
        else
        {
            //求右车头相对于特征点在南北方向的距离(垂足和最近特征点的距离)
            calculate_drop_feet(next_right_head_location, close_trigger_line_point_1, close_trigger_line_point_2, drop_result);

            //求右车头相对于特征点在南北方向的距离(垂足和最近特征点的距离)
            right_sn_dis = calculate_distance(drop_result.lon, drop_result.lat,
                                              m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[right_closest_point_key].m_fLaneLon,
                                              m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[right_closest_point_key].m_fLaneLat);
        }
        right_we_dis = sqrt(fabs(right_min_dis*right_min_dis - right_sn_dis*right_sn_dis));
        calcute_pixel_location(right_closest_point_key, right_angle_describe,
                               right_sn_dis, right_we_dis,m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx],
                               right_pixelXY);

        //左车头相对于最近特征点南北方向和东西方向的行进距离
        if (left_closest_point_key <= 1)
        {
            calculate_drop_feet(next_left_head_location, far_trigger_line_point_1, far_trigger_line_point_2, drop_result);
            left_sn_dis = calculate_distance(drop_result.lon, drop_result.lat,
                                             m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[left_closest_point_key].m_fLaneLon,
                                             m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[left_closest_point_key].m_fLaneLat);
        }
        else //求左车头与近端线的垂足
        {
            calculate_drop_feet(next_left_head_location, close_trigger_line_point_1, close_trigger_line_point_2, drop_result);
            left_sn_dis = calculate_distance(drop_result.lon, drop_result.lat,
                                             m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[left_closest_point_key].m_fLaneLon,
                                             m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_vecLanePoint[left_closest_point_key].m_fLaneLat);
        }

        left_we_dis = sqrt(fabs(left_min_dis*left_min_dis - left_sn_dis*left_sn_dis));
        calcute_pixel_location(left_closest_point_key, left_angle_describe,
                               left_sn_dis, left_we_dis,m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx],
                               left_pixelXY);

        //如果是固定线触发，则固定y的数值
        if (m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_bAxisYTriggerMode)
        {
            right_pixelXY[1] = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_nCertainLinesPixelY;
        }
        //如果是固定线触发，则固定x的数值
        if (m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_bAxisYTriggerMode)
        {
            left_pixelXY[0] = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_nCertainLinesPixelX[0];
            right_pixelXY[0] = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_vecLanePoly[cur_idx].m_nCertainLinesPixelX[1];
        }

        //把这一帧所有触发的车辆的数据放到字典中
        //key:车辆id； value:车道号，触发状态flag，触发经度，触发纬度，算法耗时
//        dic_final_trigger_car_info[new_single_car_id] = [lane_index, trigger_status_flag, now_jing_origin, now_wei_origin, algorithm_time]
        //注意，组帧的id和识别结果本身的id不同,组帧的id可能要 +80000
        vecTriggeredId.push_back(new_single_car_id);

        // 存储触发车辆id个数在20个内
        std::vector<uint16_t>::iterator l_it;
        while (vecTriggeredId.size() > 20)
        {
            l_it = vecTriggeredId.begin();
            vecTriggeredId.erase(l_it);
        }

        // 进行偏移处理
        left_right_x_pixel_list[0] = (int)left_pixelXY[0];
        left_right_x_pixel_list[1] = (int)right_pixelXY[0];
        left_right_y_pixel_list[0] = (int)left_pixelXY[1];
        left_right_y_pixel_list[1] = (int)right_pixelXY[1];

        if (!m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_bAxisXTriggerMode)
        {
            for (int j = 0; j < left_right_x_pixel_list.size(); j++)
            {
                if (left_right_x_pixel_list[j] > 0 && left_right_x_pixel_list[j] < m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX)
                {
                    left_right_x_pixel_list[j] += x_shifting;
                }
            }

            for (int j = 0; j < left_right_x_pixel_list.size(); j++)
            {
                left_right_x_pixel_list[j] = std::max(left_right_x_pixel_list[j], 0);
                left_right_x_pixel_list[j] = std::min(left_right_x_pixel_list[j], (int)m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX);
            }
        }

        if (!m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_bAxisYTriggerMode)
        {
            for (int j = 0; j < left_right_x_pixel_list.size(); j++) {
                if (m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_bIsSpeed) {
                    if (left_right_x_pixel_list[j] > 0 &&
                        left_right_x_pixel_list[j] < m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX) {
                        if (now_car_speed > m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unSpeedLimit) {
                            left_right_y_pixel_list[j] += y_shifting;
                        } else {
                            left_right_y_pixel_list[j] -= y_shifting;
                        }
                    }
                } else {
                    left_right_y_pixel_list[j] -= y_shifting;
                }

            }
            for (int j = 0; j < left_right_y_pixel_list.size(); j++)
            {
                left_right_y_pixel_list[j] = std::max(left_right_y_pixel_list[j], 0);
                left_right_y_pixel_list[j] = std::min(left_right_y_pixel_list[j], (int)m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelY);
            }
        }

        //得到左车头的像素位置
        left_pixel_location[0] = left_right_x_pixel_list[0];
//        left_pixel_location[1] = right_pixelXY[1];
        left_pixel_location[1] = left_right_y_pixel_list[0];
        //得到右车头的像素位置
        right_pixel_location[0] = left_right_x_pixel_list[1];
//        right_pixel_location[1] = right_pixelXY[1];
        right_pixel_location[1] = left_right_y_pixel_list[1];

        // cout << "left_pixel_location =  " << left_pixel_location[0]<< " , " << left_pixel_location[1]<<endl;
        // cout << "right_pixel_location =  " << right_pixel_location[0]<< " , " << right_pixel_location[1]<<endl;

        if (m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_strIsLineOrSquare == "square") {
            if(m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_bIsBigOrSmall){
                if(m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unLengthLimit< car_length)
                {
                    left_pixel_location[0] = (left_pixel_location[0] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendBig >= 0 ? left_pixel_location[0] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendBig : 0);
                    left_pixel_location[1] = (left_pixel_location[1] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendUpBig >= 0 ? left_pixel_location[1] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendUpBig : 0);
                    right_pixel_location[0] = (right_pixel_location[0] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendBig <= m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX ? right_pixel_location[0] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendBig : m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX);
                    right_pixel_location[1] = (right_pixel_location[1] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendDownBig <= m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelY ? right_pixel_location[1] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendDownBig : m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelY);
                }
                else{
                    left_pixel_location[0] = (left_pixel_location[0] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendSmall >= 0 ? left_pixel_location[0] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendSmall : 0);
                    left_pixel_location[1] = (left_pixel_location[1] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendUpSmall >= 0 ? left_pixel_location[1] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendUpSmall : 0);
                    right_pixel_location[0] = (right_pixel_location[0] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendSmall <= m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX ? right_pixel_location[0] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendSmall : m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX);
                    right_pixel_location[1] = (right_pixel_location[1] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendDownSmall <= m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelY ? right_pixel_location[1] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendDownSmall : m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelY);
                }
            }
            else{
                left_pixel_location[0] = (left_pixel_location[0] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendSmall >= 0 ? left_pixel_location[0] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendSmall : 0);
                left_pixel_location[1] = (left_pixel_location[1] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendUpSmall >= 0 ? left_pixel_location[1] - m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendUpSmall : 0);
                right_pixel_location[0] = (right_pixel_location[0] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendSmall <= m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX ? right_pixel_location[0] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unXSquareExtendSmall : m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX);
                right_pixel_location[1] = (right_pixel_location[1] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendDownSmall <= m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelY ? right_pixel_location[1] + m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unySquareExtendDownSmall : m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelY);
            }
        }

        l_usTrigCnt++;

        l_cCTriggerBoxInfo.m_usBoxId = PcResult.m_vecBox[i].m_usSingleId;
        l_cCTriggerBoxInfo.m_nTriggerStatus = trigger_status_flag;
        l_cCTriggerBoxInfo.m_dLon = PcResult.m_vecBox[i].m_dLon;
        l_cCTriggerBoxInfo.m_dLat = PcResult.m_vecBox[i].m_dLat;
        l_cCTriggerBoxInfo.m_nLaneNum = lane_index;
        l_cCTriggerBoxInfo.m_nSpeed = (int32_t)PcResult.m_vecBox[i].m_usSpeed;
        l_cCTriggerBoxInfo.m_unLength = (int32_t)PcResult.m_vecBox[i].m_usLength;
        l_cCTriggerBoxInfo.m_unWidth = (int32_t)PcResult.m_vecBox[i].m_usWidth;

        l_cCTriggerBoxInfo.m_nLeftPixelX = left_pixel_location[0];
        l_cCTriggerBoxInfo.m_nLeftPixelY = left_pixel_location[1];
        l_cCTriggerBoxInfo.m_nRightPixelX = right_pixel_location[0];
        l_cCTriggerBoxInfo.m_nRightPixelY = right_pixel_location[1];

        memcpy(&l_cCTriggerBoxInfo.m_ucReserve[0], &l_BoxAng, 2);
        cTrigResult.m_vecTriggerBoxInfo.push_back(l_cCTriggerBoxInfo);
        string().swap(new_single_car_type);
        string().swap(left_angle_describe);
        string().swap(right_angle_describe);

    } //end for

    cTrigResult.m_unFrameId = PcResult.m_unFrameId;
    cTrigResult.m_vecTriggeredId = vecTriggeredId;
    cTrigResult.m_unBufDelay[0] = l_unFrameDelay; //组帧完成到执行trig算法时刻的时间延迟
    cTrigResult.m_unBufDelay[1] = AlgDelaytime; // 点云算法检测延时
    cTrigResult.m_ulBufTimeStamp[0] = frameTime; //点云第0包时间戳
    cTrigResult.m_ulBufTimeStamp[1] = timeStamp;//执行trig算法时间戳
    cTrigResult.m_ucReserve[0] = (char)l_usTrigCnt; //触发车辆个数

    // clear memory
    vector<int>().swap(left_right_x_pixel_list);
    vector<int>().swap(left_pixel_location);
    vector<int>().swap(right_pixel_location);
    vector<double>().swap(point_car_dis_list);

    vector<TLanePoint>().swap(all_points_dic.m_vecLanePoint);
//    VectorRelease(left_right_x_pixel_list);
//    VectorRelease(left_pixel_location);
//    VectorRelease(right_pixel_location);
//    VectorRelease(point_car_dis_list);
    return true;
}


bool CTriggerAlg::isPointinPolygon(GPSCoord& point, std::vector<GPSCoord>& rangelist)
{
//    '''
//    判读一个点是否在给定的范围内
//    :param point:  要判断的点
//    :param rangelist:  要判断的范围（双重列表闭环）比如  [[0,0],[0,1],[1,1],[1,0],[0,0]]
//    :return:  True 或者 False
//    '''

    int l_unCount = rangelist.size();
    std::vector<double> lnglist;
    std::vector<double> latlist;

    for (int i = 0; i < l_unCount - 1; i++)
    {
        lnglist.push_back(rangelist[i].lon);
        latlist.push_back(rangelist[i].lat);
    }

    double maxlng = *max_element(lnglist.begin(), lnglist.end());
    double minlng = *min_element(lnglist.begin(), lnglist.end());
    double maxlat = *max_element(latlist.begin(), latlist.end());
    double minlat = *min_element(latlist.begin(), latlist.end());

    if (point.lon > maxlng || point.lon < minlng ||
        point.lat > maxlat || point.lat < minlat)
    {
        return false;
    }

    int count = 0;
    GPSCoord point1 = rangelist[0];
    GPSCoord point2;
    double point12lng;
    for (int i = 1; i < l_unCount; i++)
    {
        point2 = rangelist[i];
        if ((fabs(point.lon - point1.lon) < JINGWEI_EPS && fabs(point.lat - point1.lat) < JINGWEI_EPS)
            ||(fabs(point.lon - point2.lon) < JINGWEI_EPS && fabs(point.lat - point2.lat) < JINGWEI_EPS))
        {
            return true;
        }

        if (point.lat > std::min(point1.lat, point2.lat)&& point.lat < std::max(point1.lat,point2.lat))
        {
            point12lng = point2.lon - (point2.lat - point.lat) * (point2.lon - point1.lon) / (point2.lat- point1.lat);
            if (fabs(point12lng - point1.lon) < JINGWEI_EPS)
            {
//                点在多边形边上
                return true;
            }
            if (point12lng < point.lon)
            {
                count++;
            }
        }
        point1 = point2;
    }
    if (count % 2 == 0)
        return false;
    else
        return true;

//    //下面算法先不用python转化做,自行实现.
//    for (int i = 0, j = l_unCount - 2; i < l_unCount - 1; j = i++)
//    {
//        //判断句1：既表达了测试点Y轴坐标在起始结束点之间（这样才会有交点），又同时解决了射线点如果和边框线顶点重合的问题
//        //判断句2：表示交点的X轴坐标在测试点向右的射线上（y - y1 = k（x - x1))
//        if(((rangelist[i].y > point.y) != (rangelist[j].y > point.y)) &&
//           (point.x < ((rangelist[j].x-rangelist[i].x) * (point.y-rangelist[i].y) / (rangelist[j].y-rangelist[i].y) + rangelist[i].x)))
//        {
//            //交点数+1
//            count++;
//        }
//    }
//    if (count % 2 == 0)
//        return false;
//    else
//        return true;


}


double CTriggerAlg::calculate_distance(double jing1, double wei1, double jing2, double wei2)
{
    double lng1 = Degree2Rad(jing1);
    double lat1 = Degree2Rad(wei1);
    double lng2 = Degree2Rad(jing2);
    double lat2 = Degree2Rad(wei2);

    double dlon = lng2 - lng1;
    double dlat = lat2 - lat1;
    double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
    //地球平均半径，6371.393km
    double distance = 2 * asin(sqrt(a)) * 6371.393 * 1000;

    return distance;
}


double CTriggerAlg::get_angle(double lonA, double latA, double lonB, double latB)
{
//    """
//    参数:
//    点A (lonA, latA)
//    点B (lonB, latB)
//    返回:
//    点B基于点A的方向，以北为基准,顺时针方向的角度，0~360度
//    """

    double radLatA = Degree2Rad(latA);
    double radLonA = Degree2Rad(lonA);
    double radLatB = Degree2Rad(latB);
    double radLonB = Degree2Rad(lonB);

    double dLon = radLonB - radLonA;
    double y = sin(dLon) * cos(radLatB);
    double x = cos(radLatA) * sin(radLatB) - sin(radLatA) * cos(radLatB) * cos(dLon);
    double angle = Rad2Degree(atan2(y, x));
    angle = fmod(angle + 360.0, 360.0);

    return angle;
}

bool CTriggerAlg::calculate_drop_feet(const GPSCoord& point,
                                            const GPSCoord& line_point1,
                                            const GPSCoord& line_point2,
                                            GPSCoord& pOutPut)
{
//    """
//    点到直线的垂足，但若直线的两个点重合，则垂足就是直线点
//    :param point:
//    :param line_point1:
//    :param line_point2:
//    :return: pOutPut
//    """

    double a = line_point2.lat - line_point1.lat;
    double b = line_point1.lon - line_point2.lon;
    double c = line_point2.lon * line_point1.lat - line_point1.lon * line_point2.lat;
    double drop_feet_x = 0.0;
    double drop_feet_y = 0.0;

    if (fabs(line_point1.lon - line_point2.lon) < JINGWEI_EPS &&
        fabs(line_point1.lat - line_point2.lat) < JINGWEI_EPS)
    {
        drop_feet_x = line_point1.lon;
        drop_feet_y = line_point1.lat;
    }
    else
    {
        drop_feet_x = (b * b * point.lon - a * b * point.lat - a * c) / (a * a + b * b);
        drop_feet_y = (a * a * point.lat - a * b * point.lon - b * c) / (a * a + b * b);
    }

    pOutPut.lon = drop_feet_x;
    pOutPut.lat = drop_feet_y;

    return true;

}

/* 改写def destination_point 方法c++化 */
bool CTriggerAlg::destination_point(double lon, double lat, double distance, double bearing, GPSCoord& OutPut)
{

//    """
//    :param lon: 现在的经度 (eg. 150.123)
//    :param lat: 现在的纬度 (e.g. 40.321)
//    :param distance: 距离（米）
//    :param bearing: 与正北方向的夹角作为方向角
//    :return:
//    """

    double radius = 6371393.0;//地球半径6371393米
    double delta = distance / radius;
    double thet = Degree2Rad(bearing);
    double phi_1 = Degree2Rad(lat);
    double lanbuda_1 = Degree2Rad(lon);

    double sin_phi_1 = sin(phi_1);
    double cos_phi_1 = cos(phi_1);
    double sin_delta = sin(delta);
    double cos_delta = cos(delta);
    double sin_thet = sin(thet);
    double cos_thet = cos(thet);

    double sin_phi_2 = sin_phi_1 * cos_delta + cos_phi_1 * sin_delta * cos_thet;
    double phi_2 = asin(sin_phi_2);

    double y = sin_thet * sin_delta * cos_phi_1;
    double x = cos_delta - sin_phi_1 * sin_phi_2;
    double lanbuda_2 = lanbuda_1 + atan2(y, x);

    double new_lon = fmod(Rad2Degree(lanbuda_2) + 540.0, 360.0) - 180.0;
    double new_lat = Rad2Degree(phi_2);

    //输出结果
    OutPut.lon = new_lon;
    OutPut.lat = new_lat;

    return true;

}


/* 改写 def next_head_location  */
bool CTriggerAlg::next_head_location(const GPSCoord& pre_center_location,
                                           double car_length,
                                           double car_width,
                                           double car_angle,
                                           GPSCoord& left_head,
                                           GPSCoord& right_head)
{
    //    """
    //    根据现在的中心位置，计算下一个间隔左右两个车头的位置
    //    :param center_location:
    //    :param car_length: 米
    //    :param car_width: 米
    //    :param car_angle:
    //    :param car_speed: 米/秒
    //    :return:
    //    """
    // pOutPut 函数出参 是二维数组 double [2][2]

    //车辆下一间隔位置
    double next_center_jing = pre_center_location.lon;
    double next_center_wei = pre_center_location.lat;
    //计算 左/右车头-中心点-航向角的角度
    double alpha = Rad2Degree(atan(car_width/car_length));

    // 计算中心点到左右车头的距离（米）
    double center_head_dis = (sqrt(car_length * car_length + car_width * car_width))/2;
    //下一间隔左车头位置
    double left_bearing = (car_angle-alpha > 0) ?  (car_angle-alpha) : (car_angle - alpha + 360.0);
    double OutTmp[2];
    GPSCoord tmpPoint;
    bool bOk = destination_point(next_center_jing, next_center_wei, center_head_dis, left_bearing, tmpPoint);
    if (!bOk)
    {
        LOG(ERROR) << "cal destination_point failed\n";
        return false;
    }
    double left_head_next_jing = tmpPoint.lon;
    double left_head_next_wei = tmpPoint.lat;

    //下一间隔右车头位置
    double right_bearing = (car_angle + alpha < 360) ? (car_angle + alpha) : (car_angle + alpha - 360.0);

    bOk = destination_point(next_center_jing, next_center_wei, center_head_dis, right_bearing, tmpPoint);
    if (!bOk)
    {
        LOG(ERROR) << "cal destination_point failed\n";
        return false;
    }

    double right_head_next_jing = tmpPoint.lon;
    double right_head_next_wei = tmpPoint.lat;

    left_head.lon = left_head_next_jing;
    left_head.lat = left_head_next_wei;
    right_head.lon = right_head_next_jing;
    right_head.lat = right_head_next_wei;

    return true;

}
bool CTriggerAlg::which_is_closer(GPSCoord& check_point, const TLanePara& Lane, int& index_min, std::string& angle_describe, double& min_dis)
{
//    """
//    check_point 和车道四个点里哪个点最近
//    :param check_point:
//    :param Lane:  一个车道对象
//    :return: index_min
//             angle_describe
//             min_dis

//    """

    if (Lane.m_vecLanePoint.size() < 4)
    {
        LOG(ERROR) << "which_is_closer failed\n";
        return false;

    }
    std::vector<double> distance_list;
    double dis = 0.0;


    for (int i = 0; i < Lane.m_vecLanePoint.size(); i++)
    {
        dis = calculate_distance(check_point.lon, check_point.lat, Lane.m_vecLanePoint[i].m_fLaneLon, Lane.m_vecLanePoint[i].m_fLaneLat);
        distance_list.push_back(dis);
    }
    std::vector<double>::iterator itMin = std::min_element(distance_list.begin(), distance_list.end());
    min_dis = (double)*itMin;
    index_min = (int)std::distance(distance_list.begin(), itMin);

//    然后计算要判断的点在位置点的方位
    double angle = get_angle(Lane.m_vecLanePoint[index_min].m_fLaneLon,
                             Lane.m_vecLanePoint[index_min].m_fLaneLat,
                             check_point.lon,
                             check_point.lat);

    if ((angle > m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLimitAngle_1 && angle < m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLimitAngle_2) ||
        ((angle > m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLimitAngle_3)&&(angle < m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_fLimitAngle_4)))
    {
        angle_describe = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_strIncrease;
    }
    else
    {
        angle_describe = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_strDecrease;
    }

    return true;
}

bool CTriggerAlg::calcute_pixel_location(int colsest_point_key, std::string angle_describe, double sn_distance, double we_distance, const TLanePara& Lane, int* pOutPut)
{
//    """
//    :param colsest_point_key: int, 代表距离车辆最近的点的id   0~3
//    :param angle_describe: 方向的描述 str
//    :param sn_distance: 两个点南北方向距离，float
//    :param we_distance: 两个点东西方向距离，float
//    :param Lane: 这个车道对象
//    :return:
//    x_sn_we：南北、东西方向距离所造成的像素改变之后的位置坐标 x int
//    y_sn_we：南北、东西方向距离所造成的像素改变之后的位置坐标 y int
//    """
    if (! pOutPut)
    {
        LOG(ERROR) << "calcute_pixel_location failed\n";
        return false;
    }

    int closed_id = colsest_point_key;

    unsigned int origin_pixel_location[2] = {0, 0};
    origin_pixel_location[0] = Lane.m_vecLanePoint[closed_id].m_unPixelX;
    origin_pixel_location[1] = Lane.m_vecLanePoint[closed_id].m_unPixelY;
    double x_sn, y_sn;
    double x_sn_we, y_sn_we;
    int result[2];

    //# 南北方向距离所造成的像素改变之后的位置坐标
    if (angle_describe == m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_strIncrease)
    {
        x_sn = origin_pixel_location[0] + sn_distance * Lane.m_vecLanePoint[closed_id].m_fPixelDx1;
        y_sn = origin_pixel_location[1] + sn_distance * Lane.m_vecLanePoint[closed_id].m_fPixelDy1;
    }
    else
    {
        x_sn = origin_pixel_location[0] + sn_distance * Lane.m_vecLanePoint[closed_id].m_fPixelDx1_de;
        y_sn = origin_pixel_location[1] + sn_distance * Lane.m_vecLanePoint[closed_id].m_fPixelDy1_de;
    }

    //# 南北、东西方向距离所造成的像素改变之后的位置坐标
    if (angle_describe == m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_strIncrease)
    {
        x_sn_we = x_sn + we_distance * Lane.m_vecLanePoint[closed_id].m_fPixelDx2;
        y_sn_we = y_sn + we_distance * Lane.m_vecLanePoint[closed_id].m_fPixelDy2;
    }
    else
    {
        x_sn_we = x_sn + we_distance * Lane.m_vecLanePoint[closed_id].m_fPixelDx2_de;
        y_sn_we = y_sn + we_distance * Lane.m_vecLanePoint[closed_id].m_fPixelDy2_de;
    }
    result[0] = (int)(x_sn_we + 0.5);
    result[1] = (int)(y_sn_we + 0.5);
    if (result[0] < 0)
        result[0] = 0;
    else if (result[0] > m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX)
        result[0] = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelX;

    if (result[1] < 0)
        result[1] = 0;
    else if (result[1] > m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelY)
        result[1] = m_stSelfTrigAlgParam->m_stTriggerAlgParam.m_unCameraPixelY;

    pOutPut[0] = result[0];
    pOutPut[1] = result[1];


    return true;
}

