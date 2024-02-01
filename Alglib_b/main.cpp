#include <opencv2/core/mat.hpp>
#include <stdio.h>
#include <chrono>
#include <assert.h>
#include <iostream>
#include <istream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/stat.h>
#include <unistd.h>
#include "ExportAlgLib.h"
#include "IPcAlg.h"
#include "IVideoAlg.h"
#include "TSelfPcAlgParam.h"
#include "TPcSrcData.h"
#include "TCameraParam.h"
#include "TPcAlgResult.h"
#include "TSelfVideoAlgParam.h"
#include "TVideoSrcData.h"
#include "tinyxml2.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "xtensor/xnpy.hpp"
#include "xtensor/xcsv.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xstrided_view.hpp"
#include "xtensor-blas/xlinalg.hpp"

#include "ExportAlgLib.h"
#include "IFusionAlg.h"
#include "TSelfFusionAlgParam.h"
#include "TFusionResult.h"
#include "TFusionSrcData.h"
#include "TEventB5Data.h"
// #include "CSceneAlg.h"
#include <unistd.h>

#include "IEventAlg.h"


// zqj20230514 for nms 需要把box_ops_nms.cpp代码拷贝到该文件中, 在CMakeLists.txt中增加后，可以直接使用
#include <set>
#include "./Modules/Alg/PointCloudAlg/PcPostProcess/CPostProcess.h"

#include "Log/glog/logging.h"
#define PATH_SIZE 255
/*相机内外参   此处为上地西67，68，126ip的相机参数*/
const float camera_param_in[3][9]     = {{1174.07467031771, 0, 975.559052741506, 0, 1167.92176860689, 572.720893252979, 0, 0, 1},
                                         {1154.07467031771, 0, 975.559052741506, 0, 1207.92176860689, 572.720893252979, 0, 0, 1},
                                         {1155.023180,      0, 967.178167,       0, 1149.531704,      541.645587,       0, 0, 1}};
const float camera_param_rotate[3][3] = {{1.6303330135419367, -1.4019180977442631,  0.8723481431372321},
                                         {1.3731985950150833,  1.5004915129867642  -1.1267008444124609},
                                         {-2.4249080614788,   -1.2204040053466,    -2.31865241843}};
const float camera_param_trans[3][3]  = {{680.48,             303.28000000000003, -250.43},
                                         {-367.4599999999999, 393.5,              4.88},
                                         {500.000,            -25.652,            70.477}};
const float camera_param_dis[3][5]    = {{-0.318846891959627, 0.0902775208989156, 0.000282528081340622, 0.000800223746856249, 0},
                                         {-0.318846891959627, 0.0902775208989156, 0.000282528081340622, 0.000800223746856249, 0},
                                         {-0.317613,          0.088368,          -0.001007,            -0.000789,             0}};   

// /*相机内外参   此处为8号门51，50ip的相机参数*/
// const float camera_param_in[2][9]     = {{1152.4, 0, 959.409, 0, 1147.4, 550.065, 0, 0, 1},
//                                          {1147.9, 0, 951.689, 0, 1143.4, 566.33,  0, 0, 1}};
// const float camera_param_rotate[2][3] = {{1.85233,  0.624684, -0.484852},
//                                          {2.02309,  0.128325, -0.105201}};
// const float camera_param_trans[2][3]  = {{234.929,  151.025, -0.075},
//                                          {95.262,   164.969,  15.345}};
// const float camera_param_dis[2][5]    = {{-0.3341,  0.1085,  0.000413,   0.00070638, 0},
                                        //  {-0.3335,  0.109,  -0.00068918, 0.0013,     0}};                                    

using namespace std;

void save_data(std::string m_strProjectPath,const xt::xarray<double> &data, const int &num)
{
    assert(data.shape().size() == 2);
    std::string save_path = m_strProjectPath+"OutPut/hungariancpp_data/";
    if (access(save_path.c_str(), 0) == -1)
    {
        mkdir(save_path.c_str(), S_IRWXU);
    }
    std::string name = "frame_" + std::to_string(num) + ".csv";
    std::ofstream f(save_path + name);
    xt::dump_csv(f, data);
}

void ReadPcAlgXml(TSelfPcAlgParam *&l_stPcAlgParam, std::string pc_xml_path)
{
    tinyxml2::XMLDocument alg_doc;
    alg_doc.LoadFile(pc_xml_path.c_str());
    tinyxml2::XMLElement *alg_config = alg_doc.FirstChildElement();
    tinyxml2::XMLElement *pc_config = alg_config->FirstChildElement();
    // pointcloud value
    tinyxml2::XMLElement *PcProcessNum = pc_config->FirstChildElement();
    tinyxml2::XMLElement *PcDetectNum = PcProcessNum->NextSiblingElement();
    tinyxml2::XMLElement *PcTrackNum = PcDetectNum->NextSiblingElement();
    l_stPcAlgParam->m_stPcAlgParam.m_PcProcessNum = atoi(PcProcessNum->GetText());
    l_stPcAlgParam->m_stPcAlgParam.m_PcDetectNum = atoi(PcDetectNum->GetText());
    l_stPcAlgParam->m_stPcAlgParam.m_PcTrackNum = atoi(PcTrackNum->GetText());
}

void ReadVideoAlgXml(TSelfVideoAlgParam *&l_stVideoAlgParam, std::string video_xml_path)
{
    tinyxml2::XMLDocument alg_doc;
    alg_doc.LoadFile(video_xml_path.c_str());
    tinyxml2::XMLElement *alg_config = alg_doc.FirstChildElement();
    tinyxml2::XMLElement *pc_config = alg_config->FirstChildElement();
    tinyxml2::XMLElement *video_config = pc_config->NextSiblingElement();
    // video value
    tinyxml2::XMLElement *VideoDetectNum = video_config->FirstChildElement();
    tinyxml2::XMLElement *VideoTrackNum = VideoDetectNum->NextSiblingElement();
    l_stVideoAlgParam->m_VideoDetectNum = atoi(VideoDetectNum->GetText());
    l_stVideoAlgParam->m_VideoTrackNum = atoi(VideoTrackNum->GetText());
}

void ReadFusionAlgXml(TSelfFusionAlgParam *&l_stFuAlgParam, std::string fusion_xml_path)
{
    tinyxml2::XMLDocument alg_doc;
    alg_doc.LoadFile(fusion_xml_path.c_str());
    tinyxml2::XMLElement *alg_config = alg_doc.FirstChildElement();
    tinyxml2::XMLElement *pc_config = alg_config->FirstChildElement();
    tinyxml2::XMLElement *video_config = pc_config->NextSiblingElement();
    tinyxml2::XMLElement *fusion_config = video_config->NextSiblingElement();
    // fusion value
    tinyxml2::XMLElement *FusionNum = fusion_config->FirstChildElement();
    l_stFuAlgParam->m_FusionNum = atoi(FusionNum->GetText());
}

inline void file_to_string(vector<string> &record, const string &line, char delimiter)
{
    int linepos = 0;
    char c;
    int linemax = line.length();
    string curstring;
    record.clear();
    while (linepos < linemax)
    {
        c = line[linepos];
        if (isdigit(c) || c == '.' || c == '-')
        {
            curstring += c;
        }
        else if (c == delimiter && curstring.size())
        {
            record.push_back(curstring);
            curstring = "";
        }
        ++linepos;
    }
    if (curstring.size())
        record.push_back(curstring);
    return;
}

inline float string_to_float(string str)
{
    int i = 0, len = str.length();
    float sum = 0;
    while (i < len)
    {
        if (str[i] == '.')
            break;
        sum = sum * 10 + str[i] - '0';
        ++i;
    }
    ++i;
    float t = 1, d = 1;
    while (i < len)
    {
        d *= 0.1;
        t = str[i] - '0';
        sum += t * d;
        ++i;
    }
    return sum;
}

void read_data(const std::string &data_path, TPcSrcData &l_stPcSrcData)
{
    l_stPcSrcData.m_vecPoints.clear();
    ifstream in(data_path);
    if (in.fail())
    {
        return;
    }
    vector<string> row;
    string line;
    while (getline(in, line) && in.good())
    {
        file_to_string(row, line, ','); // 把line里的单元格数字字符提取出来，“,”为单元格分隔符
        TPcPoint l_cPcPoint;
        l_cPcPoint.m_fX = atof(row[0].data());
        l_cPcPoint.m_fY = atof(row[1].data());
        l_cPcPoint.m_fZ = atof(row[2].data())+1;   // add 1 by zqj20230530 for python
        l_cPcPoint.m_fIntensity = atof(row[3].data());
        l_stPcSrcData.m_vecPoints.push_back(l_cPcPoint);
    }
    in.close();
}

void read_data_fusion(const std::string &data_path, int i, int cam_num, TFusionResult &l_stFuSrcData, TSelfFusionAlgParam *&l_stFuAlgParam)
{

    l_stFuSrcData.m_stVideoFusionResult.m_unFrameId = i;

    // std::cout << "liluo-------------> 000000000" << std::endl;
    // video
    
    vector<TVideoResult> video_result;
    for (int j = 0; j < cam_num; j++)
    {
        std::string path = data_path + "VideoBox_Channel"  + "/" + to_string(j + 1);
        std::string pc_idx_new = data_path + "/PcBox";
        std::string pc_idx = "Frame_"+to_string(i%3000+1000000).substr(1,6);
        // std::cout << "liluo--------> video_idx: " << pc_idx << std::endl;
        std::string video_path = data_path + "VideoBox_Channel" + to_string(j) +"/" + pc_idx + ".npy";
        std::cout<<"liluo------------->video_path:"<<video_path<<std::endl;
        xt::xarray<double> video_src_data = xt::load_npy<double>(video_path);
        video_src_data = xt::empty<float>({0, 18});
        TVideoResult videoSrcResult;
        videoSrcResult.m_ucCameraId = j + 1;
        videoSrcResult.m_usBoxNum = video_src_data.shape(0);
        for (int k = 0; k < videoSrcResult.m_usBoxNum; k++)
        {
            TVideoBoxInfo m_videobox;
            
            m_videobox.m_sXCoord = video_src_data(k, 0);
            std::cout<<"m_videobox.m_sXCoord"<< m_videobox.m_sXCoord<<std::endl;
            m_videobox.m_sYCoord = video_src_data(k, 1);
            m_videobox.m_usWidth = video_src_data(k, 2);
            m_videobox.m_usLength = video_src_data(k, 3);
            m_videobox.m_sTheta = video_src_data(k, 4);
            m_videobox.m_sZCoord = video_src_data(k, 5);
            m_videobox.m_usHeight = video_src_data(k, 6);
            m_videobox.m_strClass = l_stFuAlgParam->m_fusion_parameter["fusion_param"]["m_strVideoClass"][int(video_src_data(k,7))];
            m_videobox.m_sSpeed = video_src_data(k, 8);
            m_videobox.m_usSingleId = video_src_data(k, 9);
            m_videobox.m_ucConfidence = char(video_src_data(k, 10));

            m_videobox.m_fTopLeftX = video_src_data(k, 11);
            m_videobox.m_fTopLeftY = video_src_data(k, 12);
            m_videobox.m_fBottomRightX = video_src_data(k, 13);
            m_videobox.m_fBottomRightY = video_src_data(k, 14);

            m_videobox.m_sDataSource = video_src_data(k, 15);
            m_videobox.m_sChannel = (video_src_data(k, 16));
        
            videoSrcResult.m_vecBox.push_back(m_videobox);
        }

        video_result.push_back(videoSrcResult);
    }
    l_stFuSrcData.m_stVideoFusionResult.m_vecResult = video_result;

    // pc
    //pointcloud
    std::string pc_idx_new = data_path + "PcBox/";
    std::string pc_idx = "Frame_"+to_string(i%3000+1000000).substr(1,6);
    std::cout << "liluo--------> pc_idx: " << pc_idx << std::endl;
    std::string pc_path = pc_idx_new + pc_idx+ ".npy";
    std::cout<<"liluo------------->pc_path: "<<pc_path<<std::endl;

    xt::xarray<double> pc_src_data =  xt::load_npy<double>(pc_path);
    std::cout<<"1111"<<std::endl;
    l_stFuSrcData.m_stPcFusionResult.m_unFrameId = i;
    vector<TPcBoxInfo> pc_result;

    l_stFuSrcData.m_stPcFusionResult.m_usBoxNum = int(pc_src_data.shape(0));
    for (int j = 0; j < l_stFuSrcData.m_stPcFusionResult.m_usBoxNum; j++)
    {
        TPcBoxInfo m_pcbox;
        m_pcbox.m_sXCoord = int(pc_src_data(j, 0) * 100);
        // std::cout<<"m_pcbox.m_sXCoord---:"<<m_pcbox.m_sXCoord<<std::endl;
        m_pcbox.m_sYCoord = int(pc_src_data(j, 1) * 100);
        m_pcbox.m_sZCoord = int(pc_src_data(j, 2) * 100);
        m_pcbox.m_usWidth = int(pc_src_data(j, 3) * 100);
        m_pcbox.m_usLength = int(pc_src_data(j, 4) * 100);
        m_pcbox.m_usHeight = int(pc_src_data(j, 5) * 100);
        m_pcbox.m_usCourseAngle = pc_src_data(j, 6);
        m_pcbox.m_strClass = l_stFuAlgParam->m_fusion_parameter["fusion_param"]["m_strPcClass"][int(pc_src_data(j,7))];
        m_pcbox.m_ucConfidence = int(pc_src_data(j,8) * 100);
        m_pcbox.m_usSpeed = 0;
        m_pcbox.m_usSingleId = 0;
        pc_result.push_back(m_pcbox);
    }
    l_stFuSrcData.m_stPcFusionResult.m_vecBox = pc_result;
}

void read_data_fusion_new(const std::string &data_path, int i, int cam_num, TFusionResult &l_stFuSrcData, TSelfFusionAlgParam *&l_stFuAlgParam)
{
    // edited by zqj20230613 for new offline data
    // video
    l_stFuSrcData.m_stVideoFusionResult.m_unFrameId = i;
    vector<TVideoResult> video_result;
    for (int j = 0; j < 1; j++)
    {
        std::string video_path = data_path + "VideoBox_Channel" + to_string(j) + "/Frame_" + to_string(i) + ".npy" ;
        std::cout<<"--zqj-debug-video_path:"<<video_path<<std::endl;
        xt::xarray<float> video_src_data = xt::load_npy<float>(video_path);
        TVideoResult videoSrcResult;
        videoSrcResult.m_ucCameraId = j + 1;
        videoSrcResult.m_usBoxNum = video_src_data.shape(0);
        for (int k = 0; k < videoSrcResult.m_usBoxNum; k++)
        {
            TVideoBoxInfo m_videobox;
            m_videobox.m_sXCoord = video_src_data(k, 0);
            m_videobox.m_sYCoord = video_src_data(k, 1);
            m_videobox.m_usWidth = video_src_data(k, 2);
            m_videobox.m_usLength = video_src_data(k, 3);
            m_videobox.m_sTheta = video_src_data(k, 4);
            m_videobox.m_sZCoord = video_src_data(k, 5);
            m_videobox.m_usHeight = video_src_data(k, 6);
            m_videobox.m_strClass = l_stFuAlgParam->m_fusion_parameter["fusion_param"]["m_strVideoClass"][int(video_src_data(k,7))];
            m_videobox.m_sSpeed = video_src_data(k, 8);
            m_videobox.m_usSingleId = video_src_data(k, 9);
            m_videobox.m_ucConfidence = int(video_src_data(k, 10) * 100);
            m_videobox.m_fTopLeftX = video_src_data(k, 11);
            m_videobox.m_fTopLeftY = video_src_data(k, 12);
            m_videobox.m_fBottomRightX = video_src_data(k, 13);
            m_videobox.m_fBottomRightY = video_src_data(k, 14);
            m_videobox.m_sDataSource = video_src_data(k, 15);
            m_videobox.m_sChannel = video_src_data(k, 16);
            videoSrcResult.m_vecBox.push_back(m_videobox);
        }
        video_result.push_back(videoSrcResult);
    }
    l_stFuSrcData.m_stVideoFusionResult.m_vecResult = video_result;

    // pointcloud
    std::string pc_path = data_path + "PcBox/Frame_" + to_string(i) + ".npy";
    std::cout<<"--zqj-debug-pc_path: "<<pc_path<<std::endl;
    xt::xarray<float> pc_src_data =  xt::load_npy<float>(pc_path);
    l_stFuSrcData.m_stPcFusionResult.m_unFrameId = i;
    vector<TPcBoxInfo> pc_result;
    l_stFuSrcData.m_stPcFusionResult.m_usBoxNum = int(pc_src_data.shape(0));
    for (int j = 0; j < l_stFuSrcData.m_stPcFusionResult.m_usBoxNum; j++)
    {
        TPcBoxInfo m_pcbox;
        m_pcbox.m_sXCoord = int(pc_src_data(j, 0) * 100);
        m_pcbox.m_sYCoord = int(pc_src_data(j, 1) * 100);
        m_pcbox.m_sZCoord = int(pc_src_data(j, 2) * 100);
        m_pcbox.m_usWidth = int(pc_src_data(j, 3) * 100);
        m_pcbox.m_usLength = int(pc_src_data(j, 4) * 100);
        m_pcbox.m_usHeight = int(pc_src_data(j, 5) * 100);
        m_pcbox.m_usCourseAngle = pc_src_data(j, 6);
        m_pcbox.m_strClass = l_stFuAlgParam->m_fusion_parameter["fusion_param"]["m_strPcClass"][int(pc_src_data(j,7))];
        m_pcbox.m_ucConfidence = int(pc_src_data(j,8) * 100);
        m_pcbox.m_usSpeed = pc_src_data(j,9);
        m_pcbox.m_usSingleId = pc_src_data(j,10);
        m_pcbox.m_ucSource = pc_src_data(j,11);
        pc_result.push_back(m_pcbox);
    }
    l_stFuSrcData.m_stPcFusionResult.m_vecBox = pc_result;
}

void testPc(std::string m_strProjectPath)
{
    std::string pc_path = m_strProjectPath+"data/pc_data/city_64sig/";
    // std::string pc_path = "/data/csv_qlg_1002/";
    //算法接口调用流程基本如下：
    IPcAlg* l_pObj = CreatePcAlgObj(m_strProjectPath+"OutPut");
    //准备算法参数
    TSelfPcAlgParam *l_stPcAlgParam = new TSelfPcAlgParam();
    TLidarDev a;
    std::string pc_xml_path = m_strProjectPath+"OutPut/Configs/Alg/Alg.xml";
    ReadPcAlgXml(l_stPcAlgParam, pc_xml_path);
    l_stPcAlgParam->m_stPcAlgParam.m_PcRead = true;         // 是否进行点云预处理, paras1:m_bCropFilter(bmp_crop)
    l_stPcAlgParam->m_stPcAlgParam.m_bPcProType = 0;        // add by zqj20230518 是否进行点云postprocess
    l_stPcAlgParam->m_stPcAlgParam.m_nGridSizeSmall = 0;    // add by zqj20230527 是否进行点云postfilter 0|others
    l_stPcAlgParam->m_stPcAlgParam.m_bCropFilter = 1;       // bmp_crop
    l_stPcAlgParam->m_stPcAlgParam.m_strCfgPath = "/Configs/Alg/CppConfigs/point_cloud/point_cloud.yaml";
    l_stPcAlgParam->m_stLidarParam.m_vecLidarDev.push_back(a);
    l_stPcAlgParam->m_nLidarIndex = 0; // 对应雷达ID

    // //设置算法参数
    l_pObj->SetAlgParam(l_stPcAlgParam);

    // //初始化算法接口对象
    l_pObj->InitAlgorithm();

    // //准备点云数据。。。
    TPcSrcData l_stPcSrcData;
    for(int i = 1; i < 9; ++i)
    {   
        std::string pc_name = pc_path + "/Frame_" + std::to_string(i) + ".csv";
        // std::string pc_idx_new = "000000"+std::to_string(i);
        // std::string pc_name = pc_path + pc_idx_new.substr(pc_idx_new.length()-6, 6) + ".csv";
        std::cout << "--zqj-debug-read_offline_path:" << pc_name << std::endl;
        read_data(pc_name, l_stPcSrcData);
        
        l_pObj->RunAlgorithm(&l_stPcSrcData);
    }
    delete l_pObj;
}
//xcb add
void testEvent(std::string m_strProjectPath)
{
    std::cout<<"xcb---------------testevent"<<std::endl;
    IEventAlg *l_pObj = CreateEventAlgObj(m_strProjectPath+"OutPut");
    l_pObj->InitAlgorithm();
    std::cout<<"xcb------11111111111"<<std::endl;
    std::string event_root_path = "/data/isfp-baseline/OutPut/Configs/Alg/CppConfigs/event/event_source/";
    int num = 1198;
    // int start_index = 225401;
    xt::xarray<double> event_data;
    int p_nRadarId = 1;
    std::ifstream in_file;
    for (int i = 0; i < num; i++)
    {
        std::cout<<"xcb------"<<i<<std::endl;
        std::string file_name = std::to_string(i);
        // file_name = "000000" + file_name;
        // file_name = "event_FrameIn_" + file_name; //.substr(file_name.length()-6)
        std::string event_path = event_root_path + file_name + ".npy";
        std::cout<<"event_path:"<<event_path<<std::endl;
        // in_file.open(event_path);
        event_data = xt::load_npy<double>(event_path);
        // TEventB5Data* pEventB5Data = new TEventB5Data;
        l_pObj->RunAlgorithm(&event_data, &p_nRadarId);
        in_file.close();
    }
    delete l_pObj;
}

void testVideo(std::string m_strProjectPath)
{
    // std::string img_path = m_strProjectPath+"data/video_data/";
    std::string img_path = "/data/git/Alglib-intersection/data/video_data/";
    int CamNUm = 1;
    //算法接口调用流程基本如下：
    IVideoAlg* l_pObj = CreateVideoAlgObj(m_strProjectPath+"OutPut");
    //准备算法参数
    TSelfVideoAlgParam *l_stVideoAlgParam = new TSelfVideoAlgParam();

    std::string video_xml_path = m_strProjectPath+"OutPut/Configs/Alg/Alg.xml";
    ReadVideoAlgXml(l_stVideoAlgParam, video_xml_path);
    l_stVideoAlgParam->m_strVideoCfgPath = "/Configs/Alg/CppConfigs/video/video.yaml";
    for (int i = 0; i < CamNUm; ++i)
    {
        TCameraDev l_stCameraDev;
        l_stCameraDev.m_ucCameraId = i;
        l_stVideoAlgParam->m_stCameraParam.m_vecCameraDev.push_back(l_stCameraDev);
    }

    // //设置算法参数     
    l_pObj->SetAlgParam(l_stVideoAlgParam);
    LOG(ERROR) << "SetAlgParam SUCCESS" << endl;
    // //初始化算法接口对象
    l_pObj->InitAlgorithm();
    LOG(ERROR) << "InitAlgorithm SUCCESS" << endl;
    // //准备视频数据。。。
    TVideoSrcDataTimematch l_pVideoSrcData;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 1; ++j)
        {
            cv::Mat img = cv::imread(img_path + std::to_string(i) + ".png");
            LOG(ERROR)<<"img SIZE: "<< img.size()<< endl;
            std::vector<uint8_t> l_MatBuff = std::vector<uint8_t>(img.reshape(1, 1));
            LOG(ERROR)<<"img reshape SIZE: "<< img.reshape(1, 1).size()<< endl;
            // LOG(ERROR)<<"l_MatBuff: " << l_MatBuff[0] <<endl;
            LOG(ERROR)<<"img read SUCCESS"<<endl;

            TVideoSrcData l_stVideoData;
            l_stVideoData.m_unFrameId = i;
            l_stVideoData.m_ulTimeStamp = i;
            l_stVideoData.m_ulRtpTimeStamp = i;
            l_stVideoData.m_unCameraId = 0;
            l_stVideoData.m_ucVideoSource = 1;
            l_stVideoData.m_usBmpLength = 1920;
            l_stVideoData.m_usBmpWidth = 1080;
            l_stVideoData.m_vecImageBuf = l_MatBuff;
            l_stVideoData.m_unBmpBytes = l_MatBuff.size();
            l_pVideoSrcData.m_vecSrcData.push_back(l_stVideoData);
        }
        l_pObj->RunAlgorithm(&l_pVideoSrcData);
        l_pVideoSrcData.m_vecSrcData.clear();
    }

    //offline test
    // for (int i = 0; i < 400; ++i)
    // {
    //     for (int j = 0; j < 3; ++j)
    //     {
    //         std::string save_idx_new = "000000" + std::to_string(i);
    //         std::string data_path = img_path + save_idx_new.substr(save_idx_new.length() - 6, 6) + ".png";
    //         std::cout << "data_path:" << data_path << std::endl;
    //         cv::Mat img = cv::imread(data_path);
    //         LOG(ERROR) << "img SIZE: " << img.size() << endl;
    //         std::vector<uint8_t> l_MatBuff = std::vector<uint8_t>(img.reshape(1, 1));
    //         LOG(ERROR) << "img reshape SIZE: " << img.reshape(1, 1).size() << endl;
    //         // LOG(ERROR)<<"l_MatBuff: " << l_MatBuff[0] <<endl;
    //         LOG(ERROR) << "img read SUCCESS" << endl;

    //         TVideoSrcData l_stVideoData;
    //         l_stVideoData.m_unFrameId = i;
    //         l_stVideoData.m_ulTimeStamp = i;
    //         l_stVideoData.m_ulRtpTimeStamp = i;
    //         l_stVideoData.m_unCameraId = 0;
    //         l_stVideoData.m_ucVideoSource = 1;
    //         l_stVideoData.m_usBmpLength = 1080;
    //         l_stVideoData.m_usBmpWidth = 1920;
    //         l_stVideoData.m_vecImageBuf = l_MatBuff;
    //         l_stVideoData.m_unBmpBytes = l_MatBuff.size();
    //         l_pVideoSrcData.m_vecSrcData.push_back(l_stVideoData);
    //     }

    //     l_pObj->RunAlgorithm(&l_pVideoSrcData);
    //     l_pVideoSrcData.m_vecSrcData.clear();
    // }
    delete l_pObj;
}

// TwinDisplaySceneAlg
int testFusion(std::string m_strProjectPath)
{
    std::cout << "liluo--------> testFusion 00000" << std::endl;
    int cam_num = 1;   // zqj20230613 for read_data_fusion_new
    char path[PATH_SIZE];
    if (!getcwd(path, PATH_SIZE))
    {
        std::cout << "Get path fail!" << std::endl;
        return 0;
    }
    std::cout << "path:" << path << std::endl;

    IFusionAlg* l_pObj = CreateFusionAlgObj(m_strProjectPath+"OutPut");
    TSelfFusionAlgParam *l_stFuAlgParam = new TSelfFusionAlgParam();

    // SceneAlg
    
    TLidarDev Lidar1;
    std::cout << "liluo--------> testFusion 11111" << std::endl;
    // /* 雷达参数    8号门87*/
    // Lidar1.m_dLon = 116.28698024;
    // Lidar1.m_dLat = 40.05169478;
    // Lidar1.m_fAngle = 160;

    /* 雷达参数    上地西96*/
    Lidar1.m_dLon = 116.2915824879496;
    Lidar1.m_dLat = 40.0533207678991;
    Lidar1.m_fAngle = 260.3;

    l_stFuAlgParam->m_stLidarParam.m_vecLidarDev.push_back(Lidar1);

    l_stFuAlgParam->m_stCameraParam.m_vecCameraDev.resize(cam_num);
    l_stFuAlgParam->m_stCameraParam.m_bUse485 = false;
    l_stFuAlgParam->m_stCameraParam.m_unCameraCount = cam_num;
    for (int i = 0; i < cam_num; i++)
    {
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter.resize(9);
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecRotateMatrix.resize(3);
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecTranslationMatrix.resize(3);
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix.resize(5);
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[0] = camera_param_in[i][0];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[1] = camera_param_in[i][1];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[2] = camera_param_in[i][2];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[3] = camera_param_in[i][3];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[4] = camera_param_in[i][4];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[5] = camera_param_in[i][5];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[6] = camera_param_in[i][6];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[7] = camera_param_in[i][7];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[8] = camera_param_in[i][8];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecRotateMatrix[0] = camera_param_rotate[i][0];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecRotateMatrix[1] = camera_param_rotate[i][1];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecRotateMatrix[2] = camera_param_rotate[i][2];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecTranslationMatrix[0] = camera_param_rotate[i][0];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecTranslationMatrix[1] = camera_param_rotate[i][1];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecTranslationMatrix[2] = camera_param_rotate[i][2];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix[0] = camera_param_dis[i][0];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix[1] = camera_param_dis[i][1];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix[2] = camera_param_dis[i][2];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix[3] = camera_param_dis[i][3];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix[4] = camera_param_dis[i][4];
    }
    std::cout << "liluo--------> testFusion 22222" << std::endl;
    std::string fusion_xml_path = m_strProjectPath+"OutPut/Configs/Alg/Alg.xml";

    ReadFusionAlgXml(l_stFuAlgParam, fusion_xml_path);//读取算法配置xml文件
    l_stFuAlgParam->m_strFusionCfgPath = "/Configs/Alg/CppConfigs/fusion/fusion.yaml";
    l_pObj->SetAlgParam(l_stFuAlgParam);
    l_pObj->InitAlgorithm();
    TFusionResult p_pSrcData;
    std::cout << "liluo--------> testFusion 33333" << std::endl;
    
    for (int i = 0; i < 4; ++i)
    {   int temp_cin;
//        std::cout<<"hhhhhhhhhhh:"<<std::endl;
//        std::cin>>temp_cin;
        // read_data_fusion_new(m_strProjectPath+"data/fusion_test_data_new/" , i, cam_num, p_pSrcData, l_stFuAlgParam);  // zqj20230613 data format is now same
        read_data_fusion(m_strProjectPath+"data/taiguo01/" , i, 4, p_pSrcData, l_stFuAlgParam);
        auto t_start = std::chrono::steady_clock::now();
        l_pObj->RunAlgorithm(&p_pSrcData);
        auto t_end = std::chrono::steady_clock::now();
        double latency = std::chrono::duration<double, std::milli>(t_end - t_start).count();


    }
    return 0;
}



/***************************************************************
 * @file       main.cpp
 * @brief      nms for mosaic
 * @input      AlgParams: void
 * @author     zhangqijun
 * @date       2023.05.15
 **************************************************************/
void test_nms()
{
    std::cout << "--zqj-debug-alg-test_nms" << std::endl;
    // xt::xarray<float> x_arr1 = xt::ones<float>({3,5});
    xt::xarray<float> x_arr1 = {{1,1,1,1,0},{5,5,1,1,0},{1,1,2,2,0},{1,1,2,2,0},{5,5,2,2,0},{10,10,2,2,0}};
    xt::xarray<float> x_arr2 = xt::xarray<float>{{1,1,0.5,0.5,0},{1,1,1,1,0}};
    auto iou_res = box_ops_nms::rotate_nms_cc(x_arr1, x_arr1);
    xt::xarray<float> iou_matrix = iou_res.first;  // 交并比
    xt::xarray<float> iou_matrix_new = iou_res.second;
    int row_iou = iou_matrix.shape(0);
    int col_iou = iou_matrix.shape(1);
    // std::set<int> ab;
    set<int> set_org;
    for(int i=0;i<row_iou-1;i++)
    {
        for (int j=i+1;j<col_iou;j++)
        {
            std::cout << "--zqj-debug-alg-test_iou_right_tri:" << iou_matrix(i,j) <<  std::endl;
            if (iou_matrix(i,j)>0)
            {
                // set_org.insert(i);
                set_org.insert(j);  // 只删除后一个框
            }
        }
    }
    std::cout << "--zqj-debug-alg-test_nms_set:" <<  set_org.size() <<  std::endl;
    xt::xarray<float> iou_save = xt::zeros<float>({row_iou-(int)set_org.size(),5});
    int save_row = 0;
    for (int j = 0; j < row_iou; j++)
    {
        bool flag_save = true;
        for(auto i=set_org.begin();i!=set_org.end();i++)
        {
            // std::cout << "--zqj-debug-alg-test_nms_set:" << *i <<  std::endl;
            if (j == *i)
            {
                flag_save = false;
            }
        }
        if(flag_save)
        {
            xt::view(iou_save, save_row, xt::all()) = xt::view(x_arr1, j, xt::all());
            save_row++;
        }
    }

    // std::cout << "--zqj-debug-alg-test_nms_range:" << xt::xarray<int>({0,1}) <<  std::endl;
    // std::cout << "--zqj-debug-alg-test_nms_shape:" << row_iou << "--" << col_iou <<  std::endl;
    std::cout << "--zqj-debug-alg-test_nms_arr:" << x_arr1 <<  std::endl;
    std::cout << "--zqj-debug-alg-test_nms_iou:" << iou_matrix <<  std::endl;
    std::cout << "--zqj-debug-alg-test_nms_iou_save:" << iou_save <<  std::endl;
    // std::cout << "--zqj-debug-alg-test_nms_arr_shape:" << x_arr1.shape(0) << std::endl;
}

void test_filter()
{
    //code 验证KT过滤的准确
    std::cout << "--zqj-debug-alg-test_filter" << std::endl;
    // int mode = 0;
    // std::vector<int> labels(5,1);
    // std::vector<float> scores{0.05, 0.1, 0.5,0.001,0.6};
    // std::vector<int> taked_indics =  filter_label_for_KT(labels, scores, mode);

    // code 验证wj过滤的准确
    // 后处理参数 ['car:0', 'bicycle:1', 'bus:2', 'tricycle:3', 'pedestrian:4', 'semitrailer:5', 'truck:6']
    std::vector<int> labels1{0,1};
    xt::xarray<float> pc_boxes1 = {{-5,-5,-5, 2, 4, 2, 0}, {-5,-5,-5,1,1,2,0}};
    std::vector<float> scores1{0.55, 0.3};
    std::vector<int> point_num1{15, 7};
    std::vector<float> heights1 = {-25,-25};
    xt::xarray<float> area1 ={{-6,-4},{-5.2,-3.5}};
    std::vector<int> take_idx1 =  filter_label_for_WJ(labels1, pc_boxes1, scores1, point_num1, heights1, area1);
    for (int i =0; i<take_idx1.size();i++)
    {
        std::cout<<"--zqj-debug-alg-test-filter_label_for_WJ_takid:  " << take_idx1[i] <<std::endl;
        std::cout<<"--zqj-debug-alg-test-filter_label_for_WJ_label:  " << labels1[take_idx1[i]] <<std::endl;
    }
    // code  验证点数计算的准确
    // xt::xarray<float> point_arr_test = {{1,1,1},{-10,-10,-10},{-9,-9,-9},{-5.1,-5.1,-5.1}};
    // xt::xarray<float> xtensor_3Dbox_test = {{-10,-10,-10,10,10,10,0}};
    // std::shared_ptr<Count_Box_Result> num_point_per_boxes_height_area_test = points_count_rbbox(point_arr_test, xtensor_3Dbox_test);
    // for (int i =0; i <num_point_per_boxes_height_area_test->vec_result_point.size() ;i++)
    // {
    //     std::cout << "points_num_in_box:" << num_point_per_boxes_height_area_test->vec_result_point[i] <<std::endl;
    //     std::cout << "avg_height_in_box:" << num_point_per_boxes_height_area_test->vec_result_hight[i]/num_point_per_boxes_height_area_test->vec_result_point[i] <<std::endl;
    //     std::cout << "range_of_z_in_box:" << num_point_per_boxes_height_area_test->arr_result_area <<std::endl;
    // }

    // code 对比vector和xtensor的速度差异
    // xt::xarray<float> x_arr3 = {{1,1,1,1,0}};
    // int num_polygons =150000;
    // for (int k=0;k<10;k++)
    // {
    //     auto t_start = std::chrono::high_resolution_clock::now();
    //     xt::xarray<float> h_area1 = xt::zeros<float>({num_polygons, 2});  // to do 需要修改为vector
    //     // for(int i=0; i<num_polygons; i++){
    //     //     // h_area(i, 0) = 0;
    //     //     h_area1(i, 1) = -100;
    //     // }
    //     for(int j=0; j<num_polygons; j++)
    //     {
    //         h_area1(j, 1) = std::max(h_area1(j, 1), x_arr3(0,0));  // init -100
    //         h_area1(j, 1) = std::max(h_area1(j, 1), x_arr3(0,0));  // init -100
    //         // vec_h_area_min[j] = std::min(vec_h_area_min[j], points(i, 2));
    //     }
    //     auto t_end = std::chrono::high_resolution_clock::now();
    //     auto time_spend = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    //     std::cout<<"--zqj-debug-time all spend00000:  "<<time_spend<<" ms"<<std::endl;

    //     auto t_end2 = std::chrono::high_resolution_clock::now();
    //     std::vector<float> vec_h_area_min(num_polygons, 0);
    //     std::vector<float> vec_h_area_max(num_polygons, -100);
    //     for(int j=0; j<num_polygons; j++)
    //     {
    //         vec_h_area_min[j] = std::min(vec_h_area_min[j], x_arr3(0,0));
    //         vec_h_area_min[j] = std::min(vec_h_area_min[j], x_arr3(0,0));
    //     }
    //     auto t_end1 = std::chrono::high_resolution_clock::now();
    //     auto time_spend1 = std::chrono::duration<double, std::milli>(t_end1 - t_end2).count();
    //     std::cout<<"--zqj-debug-time all spend1111:  "<<time_spend1<<" ms"<<std::endl;
    // }

}



//xcb add
void testLoad()
{
    std::cout<<"xcb---------------testevent"<<std::endl;
    std::cout<<"xcb------11111111111"<<std::endl;
    std::string event_root_path = "/data/OfflineData/fusion/";
    int num = 84;
    xt::xarray<double> event_data;
    for (int i=0; i < num; i++)
    {
        std::cout<<"xcb------"<<i<<std::endl;
        std::string file_name = std::to_string(i);
        file_name = "000000" + file_name;
        file_name = "Frame_" + file_name.substr(file_name.length()-6);
        std::string event_path = event_root_path + file_name + ".npy";
        std::cout<<"event_path:"<<event_path<<std::endl;
        // in_file.open(event_path);
        event_data = xt::load_npy<double>(event_path);
    }
}


int testScene(std::string m_strProjectPath)
{
    std::cout << "liluo--------> testFusion 00000" << std::endl;
    int cam_num = 1;   // zqj20230613 for read_data_fusion_new
    char path[PATH_SIZE];
    if (!getcwd(path, PATH_SIZE))
    {
        std::cout << "Get path fail!" << std::endl;
        return 0;
    }
    std::cout << "path:" << path << std::endl;

    IFusionAlg* l_pObj = CreateFusionAlgObj(m_strProjectPath+"OutPut");
    TSelfFusionAlgParam *l_stFuAlgParam = new TSelfFusionAlgParam();
    

    // SceneAlg
    ISceneAlg* l_pSceneObj = CreateSceneAlgObj(m_strProjectPath+"OutPut");
    TSelfSceneAlgParam *l_stSceneAlgParam = new TSelfSceneAlgParam();
    
    
    TLidarDev Lidar1;
    std::cout << "liluo--------> testFusion 11111" << std::endl;
    // /* 雷达参数    8号门87*/
    // Lidar1.m_dLon = 116.28698024;
    // Lidar1.m_dLat = 40.05169478;
    // Lidar1.m_fAngle = 160;

    /* 雷达参数    上地西96*/
    Lidar1.m_dLon = 116.2915824879496;
    Lidar1.m_dLat = 40.0533207678991;
    Lidar1.m_fAngle = 260.3;

    l_stFuAlgParam->m_stLidarParam.m_vecLidarDev.push_back(Lidar1);

    l_stFuAlgParam->m_stCameraParam.m_vecCameraDev.resize(cam_num);
    l_stFuAlgParam->m_stCameraParam.m_bUse485 = false;
    l_stFuAlgParam->m_stCameraParam.m_unCameraCount = cam_num;
    for (int i = 0; i < cam_num; i++)
    {
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter.resize(9);
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecRotateMatrix.resize(3);
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecTranslationMatrix.resize(3);
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix.resize(5);
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[0] = camera_param_in[i][0];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[1] = camera_param_in[i][1];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[2] = camera_param_in[i][2];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[3] = camera_param_in[i][3];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[4] = camera_param_in[i][4];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[5] = camera_param_in[i][5];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[6] = camera_param_in[i][6];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[7] = camera_param_in[i][7];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter[8] = camera_param_in[i][8];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecRotateMatrix[0] = camera_param_rotate[i][0];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecRotateMatrix[1] = camera_param_rotate[i][1];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecRotateMatrix[2] = camera_param_rotate[i][2];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecTranslationMatrix[0] = camera_param_rotate[i][0];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecTranslationMatrix[1] = camera_param_rotate[i][1];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecTranslationMatrix[2] = camera_param_rotate[i][2];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix[0] = camera_param_dis[i][0];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix[1] = camera_param_dis[i][1];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix[2] = camera_param_dis[i][2];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix[3] = camera_param_dis[i][3];
        l_stFuAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix[4] = camera_param_dis[i][4];
    }
    std::cout << "liluo--------> testFusion 22222" << std::endl;
    std::string fusion_xml_path = m_strProjectPath+"OutPut/Configs/Alg/Alg.xml";

    ReadFusionAlgXml(l_stFuAlgParam, fusion_xml_path);//读取算法配置xml文件
    l_stFuAlgParam->m_strFusionCfgPath = "/Configs/Alg/CppConfigs/fusion/fusion.yaml";
    l_pObj->SetAlgParam(l_stFuAlgParam);
    l_pObj->InitAlgorithm();
    l_stSceneAlgParam->m_bFusion = l_stFuAlgParam->m_bFusion;
    l_stSceneAlgParam->m_ucStationId = l_stFuAlgParam->m_ucStationId;
    l_stSceneAlgParam->m_vecUsedCameraId = l_stFuAlgParam->m_vecUsedCameraId;
    l_stSceneAlgParam->m_usFusionWaitTime = l_stFuAlgParam->m_usFusionWaitTime;
    l_stSceneAlgParam->m_FusionNum = l_stFuAlgParam->m_FusionNum;
    l_stSceneAlgParam->m_strFusionCfgPath = l_stFuAlgParam->m_strFusionCfgPath;
    l_stSceneAlgParam->m_strRootPath = l_stFuAlgParam->m_strRootPath;
    l_stSceneAlgParam->m_CameraId = l_stFuAlgParam->m_CameraId;
    l_stSceneAlgParam->m_fusion_parameter = l_stFuAlgParam->m_fusion_parameter;
    l_stSceneAlgParam->m_stLidarParam = l_stFuAlgParam->m_stLidarParam;
    l_pSceneObj->SetAlgParam(l_stSceneAlgParam);
    scene_type  a = twin_display;
    l_pSceneObj->InitAlgorithm(&(a));
    TFusionResult p_pSrcData;
    std::cout << "liluo--------> testFusion 33333" << std::endl;
    
    for (int i = 0; i < 1; ++i)
    {   int temp_cin;
//        std::cout<<"hhhhhhhhhhh:"<<std::endl;
//        std::cin>>temp_cin;
        // read_data_fusion_new(m_strProjectPath+"data/fusion_test_data_new/" , i, cam_num, p_pSrcData, l_stFuAlgParam);  // zqj20230613 data format is now same
        read_data_fusion(m_strProjectPath+"data/taiguo01/" , i, 4, p_pSrcData, l_stFuAlgParam);
        auto t_start = std::chrono::steady_clock::now();
        TFusionOutputResult * scene_data = (TFusionOutputResult *) l_pObj->RunAlgorithm(&p_pSrcData);
        l_pSceneObj->RunAlgorithm(scene_data);
        auto t_end = std::chrono::steady_clock::now();
        double latency = std::chrono::duration<double, std::milli>(t_end - t_start).count();


    }
    return 0;
}

//测试算法接口调用流程
int main(int argc, char* argv[])
{
    char *p = NULL;
    int len = 1024;
    char l_pExecutionPath[len];
    memset(l_pExecutionPath, 0, len);
    int n = readlink("/proc/self/exe", l_pExecutionPath, len);
    if (NULL != (p = strrchr(l_pExecutionPath,'/')))
    {
        *p = '\0';
    }
    std::string m_strExecutionPath = l_pExecutionPath;  // "OutPut"
    std::string m_strProjectPath = m_strExecutionPath.substr(0, m_strExecutionPath.length()-6); // "AlgLib/""
    std::cout<<"m_strProjectPath:"<<m_strProjectPath<<std::endl;
    // testPc(m_strProjectPath);
    // testVideo(m_strProjectPath);
    testFusion(m_strProjectPath);
    // testScene(m_strProjectPath);
    // test_nms();
    // test_filter();
    // testLoad();
//    testEvent(m_strProjectPath);
    return 0;
}