#include "CSceneAlg.h"
#include "Log/glog/logging.h"
#include "iostream"
void CSceneAlg::setFusionResult(TFusionResult *m_tFuResult, TSelfSceneAlgParam *p_pAlgParam, const Fusion_Res_scene &result)
{
    auto &pc_result = m_tFuResult->m_stPcFusionResult;

    xt::xarray<float> pointcloud = result.pc_res;
    int pc_shape = int(pointcloud.shape(0));
    pc_result.m_vecBox.clear();
    pc_result.m_usBoxNum = pc_shape;

    // //行人统计
    // xt::xarray<float> PeopleCount = result.PeopleCount;
    // uint8_t people_count0,people_count1,people_count2,people_count3;

    // people_count0 = (uint8_t)result.PeopleCount(0,0);
    // people_count1 = (uint8_t)result.PeopleCount(1,0);
    // people_count2 = (uint8_t)result.PeopleCount(2,0);
    // people_count3 = (uint8_t)result.PeopleCount(3,0);

    // *(uint8_t*)&pc_result.m_fBufFps[9] = people_count0;
    // *((uint8_t*)&pc_result.m_fBufFps[9] + 1) = people_count1;
    // *((uint8_t*)&pc_result.m_fBufFps[9] + 2) = people_count2;
    // *((uint8_t*)&pc_result.m_fBufFps[9] + 3) = people_count3;

    // 点云结果处理
    if (pc_shape > 0)
    {
        for (int i = 0; i < pc_shape; ++i)
        {
            // [0, 1, 2, 3,   4,   5, 6,   7,     8,    9,   10,     11,    12,    13,    14,     15,         16,     17,    18]
            // [x, y, w, l, theta, z, h, label, speed, id, scores, x_min, y_min, x_max, y_max, data_source, channel, lane, occur_time]
            TPcBoxInfo fu_cPcBoxInfo;

            fu_cPcBoxInfo.m_sXCoord = pointcloud(i, 0) * 100;  // X轴坐标:m
            fu_cPcBoxInfo.m_sYCoord = pointcloud(i, 1) * 100;  // Y轴坐标:m
            fu_cPcBoxInfo.m_usWidth = pointcloud(i, 2) * 100;  // 宽度
            fu_cPcBoxInfo.m_usLength = pointcloud(i, 3) * 100; // 长度
            fu_cPcBoxInfo.m_usCourseAngle = pointcloud(i, 4);  // 航向角 角度
            // std::cout<<"angleout:"<<pointcloud(i, 4)<<std::endl;
            fu_cPcBoxInfo.m_sZCoord = pointcloud(i, 5) * 100;  // Z轴坐标:m
            fu_cPcBoxInfo.m_usHeight = pointcloud(i, 6) * 100; // 高度
            int label = int(pointcloud(i, 7));
            if (label == 5 || label == 7){
                label = 6;
            }
            fu_cPcBoxInfo.m_strClass = p_pAlgParam->m_fusion_parameter["fusion_param"]["m_strFusionClass"][label]; // 目标类型(点云算法传出的类型与视频传出的类型映射关系不同,所以此处采用字符串形式统一)

            fu_cPcBoxInfo.m_usSpeed = pointcloud(i, 8) * 100;            // 速度：m/s
            fu_cPcBoxInfo.m_usSingleId = pointcloud(i, 9);               // 单基站检测的ID
            fu_cPcBoxInfo.m_ucConfidence = int(pointcloud(i, 10) * 100); // 置信度

            fu_cPcBoxInfo.usVideoInfo[0] = (uint16_t)pointcloud(i, 11);  // x_min
            fu_cPcBoxInfo.usVideoInfo[1] = (uint16_t)pointcloud(i, 12);  // y_min
            fu_cPcBoxInfo.usVideoInfo[2] = (uint16_t)pointcloud(i, 13);  // x_max
            fu_cPcBoxInfo.usVideoInfo[3] = (uint16_t)pointcloud(i, 14);  // x_max
            fu_cPcBoxInfo.m_ucSource = (unsigned char)(pointcloud(i, 15)); // data_source
            fu_cPcBoxInfo.m_ucReserve[0] = (unsigned char)(pointcloud(i, 16)); // channel
            fu_cPcBoxInfo.m_ucLanId          = (unsigned char)pointcloud(i, 17); // lane

            // fu_cPcBoxInfo.m_ucLanId          = pointcloud(i, 12);               // chedao
            // fu_cPcBoxInfo.m_ucReserve[0]     = (char)pointcloud(i, 11);         // hits
            // fu_cPcBoxInfo.m_ucReserve[1]     = (char)pointcloud(i, 13);         // time-since-update
            // std::cout<<"video_id"<<pointcloud(i, 20)<<std::endl;
         
            // source pc or video or fusion
            //  fu_cPcBoxInfo.usVideoInfo[0] = (int)pointcloud(i, 16);    // leftx
            //  fu_cPcBoxInfo.usVideoInfo[1] = (int)pointcloud(i, 17);    // lefty
            //  fu_cPcBoxInfo.usVideoInfo[2] = (int)pointcloud(i, 18);    // rightx
            //  fu_cPcBoxInfo.usVideoInfo[3] = (int)pointcloud(i, 19);    // righty
            //  fu_cPcBoxInfo.usVideoInfo[4] = (int)pointcloud(i, 20);    // camid
            // pc_result.m_unFrameId = (int)pointcloud(i, 15);
            // std::cout <<"--zqj-debug-alg-scene-1111 size: "<<m_stSelfSceneAlgParam->m_stLidarParam.m_vecLidarDev.size() <<std::endl;
            double l_dBoxLongitude = 0.0, l_dBoxLatitude=0.0;
            // _dLidarLon = m_stSelfSceneAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLon;  // add by zqj20230620
            // _dLidarLat = m_stSelfSceneAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLat;  // add by zqj20230620
            // _fLidarNorthAngle =  m_stSelfSceneAlgParam->m_stLidarParam.m_vecLidarDev[0].m_fAngle;
            double _dLidarLon= pc_result.m_dLidarLon, _dLidarLat=pc_result.m_dLidarLat;   // add by szh20230719
            float _fLidarNorthAngle = pc_result.m_fLidarNorthAngle;

            XYZ_To_BLH(
                _dLidarLon, _dLidarLat,
                pointcloud(i,0),+
                pointcloud(i,1),
                _fLidarNorthAngle,
                l_dBoxLongitude,
                l_dBoxLatitude);
            fu_cPcBoxInfo.m_dLon = l_dBoxLongitude;
            fu_cPcBoxInfo.m_dLat = l_dBoxLatitude;
            // if (i==0)
            // {
            //     printf("l_dBoxLongitude, l_dBoxLatitude: %.8f, %.8f\n", l_dBoxLongitude, l_dBoxLatitude);
            //     std::cout << "--zqj-debug-alg-scence-l_dBoxLongitude, l_dBoxLatitude: " << _dLidarLon << ", " << _dLidarLat<< std::endl;
            // //     std::cout << "--zqj-debug-alg-scence-l_dBoxLongitude, l_dBoxLatitude: " << l_dBoxLongitude << ", " << l_dBoxLatitude << std::endl; 
            // }
            pc_result.m_dLidarLon = _dLidarLon;
            pc_result.m_dLidarLat = _dLidarLat;
            pc_result.m_fLidarNorthAngle = _fLidarNorthAngle;
            pc_result.m_vecBox.push_back(fu_cPcBoxInfo);
        }
    
    }
    // std::cout<<"szh----------> scene out size  00000000000000: "<<pc_result.m_vecBox.size()<<std::endl;
}
ISceneAlgBase *CSceneAlg::create_alg(scene_type type)
{
    std::cout << "!!!!!!!!!Secene Type: " <<  std::endl;
    if (type == v2x)
    {
        ISceneAlgBase *CSceneAlg_temp = new V2xSceneAlg(&(m_stSelfSceneAlgParam->m_fusion_parameter));
        return CSceneAlg_temp;
    }

    if (type == twin_display){
        
        ISceneAlgBase *CSceneAlg_temp = new TwinDisplaySceneAlg(&(m_stSelfSceneAlgParam->m_fusion_parameter));
        std::cout << "Init TwinDisplaySceneAlg success !!!" << std::endl;
        return CSceneAlg_temp;
        
    }

    // if (type == ServiceArea){
        
    //     ISceneAlgBase *CSceneAlg_temp = new ServiceAreaSceneAlg(&(m_stSelfSceneAlgParam->m_fusion_parameter));
    //     std::cout << "Init ServiceAreaSceneAlg success !!!" << std::endl;
    //     return CSceneAlg_temp;
        
    // }

}

CSceneAlg::CSceneAlg(const std::string &p_strOutPath)
{
    m_strOutPath = p_strOutPath;
    LOG(INFO) << "m_strOutPath: " << m_strOutPath << std::endl;
}


// CSceneAlg::CSceneAlg()
// {

// }

CSceneAlg::~CSceneAlg()
{
    if (m_scene_alg_handler != nullptr){
        delete m_scene_alg_handler;
        m_scene_alg_handler = nullptr;
    }
}

bool CSceneAlg::SetAlgParam(const TSelfSceneAlgParam *p_pAlgParam)
{
    
    LOG(INFO) << "SetSceneAlgParam" << std::endl;
    if (p_pAlgParam)
    {
        m_stSelfSceneAlgParam = const_cast<TSelfSceneAlgParam *>(p_pAlgParam);
    }
    else
    {
        LOG(ERROR) << "The incoming parameter is empty" << std::endl;
        return false;
    }

    return true;
}

bool CSceneAlg::InitAlgorithm(void *p_pInitParam)
{
    
    LOG(INFO) << "init SceneAlg" << std::endl;
    scene_type* scene_type_temp=(scene_type*)p_pInitParam;
    m_scene_alg_handler = create_alg(*((scene_type*)p_pInitParam));
    return true;
}

void *CSceneAlg::RunAlgorithm(void *p_pSrcData)
{   
    TFusionOutputResult &temp = *(TFusionOutputResult *)p_pSrcData; // 
    TFusionTrackResult *l_pFusionResult=&(temp.m_stFusionTrackResult);
    TFusionResult *m_tFusionResult = new TFusionResult();
    Fusion_Res_scene output = m_scene_alg_handler->RUN(l_pFusionResult);
    
    m_tFusionResult->m_stPcFusionResult.m_strTerminalId = l_pFusionResult->m_strTerminalId;
    m_tFusionResult->m_stPcFusionResult.m_unFrameId = l_pFusionResult->m_unFrameId;
    m_tFusionResult->m_stPcFusionResult.m_ucLidarId = l_pFusionResult->m_ucLidarId;

    m_tFusionResult->m_stPcFusionResult.m_dLidarLat = l_pFusionResult->m_dLidarLat;
    m_tFusionResult->m_stPcFusionResult.m_dLidarLon = l_pFusionResult->m_dLidarLon;
    m_tFusionResult->m_stPcFusionResult.m_fLidarNorthAngle = l_pFusionResult->m_fLidarNorthAngle;
    _dLidarLon = l_pFusionResult->m_dLidarLon;
    _dLidarLat = l_pFusionResult->m_dLidarLat;
	_fLidarNorthAngle = l_pFusionResult->m_fLidarNorthAngle;
    
    for (int i = 0; i < 10; i++){
        m_tFusionResult->m_stPcFusionResult.m_ulBufTimeStamp[i] = l_pFusionResult->m_ulBufTimeStamp[i];
        m_tFusionResult->m_stPcFusionResult.m_fBufFps[i] = l_pFusionResult->m_fBufFps[i];
    }
    for (int i = 0; i < 5; i++){
        m_tFusionResult->m_stPcFusionResult.m_unBufDelay[i] = l_pFusionResult->m_unBufDelay[i];
    }

    m_tFusionResult->m_stPcFusionResult.m_dLidarLon = l_pFusionResult->m_dLidarLon;
    m_tFusionResult->m_stPcFusionResult.m_dLidarLat = l_pFusionResult->m_dLidarLat;
	m_tFusionResult->m_stPcFusionResult.m_fLidarNorthAngle = l_pFusionResult->m_fLidarNorthAngle;
    
    setFusionResult(m_tFusionResult, m_stSelfSceneAlgParam, output);
    LOG(INFO)<<"scene out size: "<<m_tFusionResult->m_stPcFusionResult.m_vecBox.size()<<std::endl;
    return m_tFusionResult;
   
}
