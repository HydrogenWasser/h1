#include "CFusionAlg.h"
#include "Log/glog/logging.h"
#include "CommonFunctions.h"

using namespace std;

CFusionAlg::CFusionAlg(const std::string &p_strExePath)
{
    m_strOutPath = p_strExePath;
    LOG(INFO) << "m_strOutPath: " << m_strOutPath << endl;
}

CFusionAlg::~CFusionAlg()
{
    delete m_Fusionalgorithm;
}

void save_trackdata(const xt::xarray<double> &track_data, const int &num)
{
    assert(track_data.shape().size() == 2);
    std::string save_path = "/data/track/103/";
    if (access(save_path.c_str(), 0) == -1)
    {
        mkdir(save_path.c_str(), S_IRWXU);
    }
    std::string name = "trackdata_" + std::to_string(num) + ".csv";
    std::ofstream f(save_path + name);
    xt::dump_csv(f, track_data);
}

bool CFusionAlg::SetAlgParam(const TSelfFusionAlgParam *p_pAlgParam)

{
    LOG(INFO) << "SetFusionAlgParam" << endl;
    if (p_pAlgParam)
    {
        m_stSelfFuAlgParam = const_cast<TSelfFusionAlgParam *>(p_pAlgParam);
    }
    else
    {
        LOG(ERROR) << "The incoming parameter is empty" << endl;
        return false;
    }

    ParaMgr *temp_paramgr = new ParaMgr();
    std::string filepath = "Configs/Alg/CppConfigs/fusion/ServiceArea/fusion_param.json";
    bool isload = temp_paramgr->jsonload(filepath);
    nlohmann::json temp_param;
    bool flag = temp_paramgr->get(temp_param);
    m_stSelfFuAlgParam->m_fusion_parameter = temp_param;
    if (temp_paramgr != nullptr)
    {
        delete temp_paramgr;
        temp_paramgr = nullptr;
    }
    
    return true;
}

bool CFusionAlg::InitAlgorithm(void *p_pInitParam)
{
    LOG(INFO) << "m_stSelfFuAlgParam->m_FusionNum 111: " << std::endl;
    LOG(INFO) << "InitAlgorithm--m_strOutPath=" << m_strOutPath << endl;
    LOG(INFO) << "m_stSelfFuAlgParam->m_FusionNum 222: " << std::endl;
    m_stSelfFuAlgParam->m_strRootPath = m_strOutPath;

    // 融合算法
    if (m_stSelfFuAlgParam->m_FusionNum == 1)
    { // 路口融合跟踪算法

        m_Fusionalgorithm = new Fusion_Algorithm_crossing(m_stSelfFuAlgParam);
        cout << "InitAlgorithm-crossing-fusion -> sucess" << endl;
        LOG(INFO) << "InitAlgorithm-crossing-fusion -> sucess";
    }
    else if (m_stSelfFuAlgParam->m_FusionNum == 2)
    { // 隧道融合算法
        m_Fusionalgorithm = new Fusion_Algorithm_TunnelAll(m_stSelfFuAlgParam);
        cout << "InitAlgorithm-tunnel-fusion -> sucess" << endl;
        LOG(INFO) << "InitAlgorithm-tunnel-fusion -> sucess";
    }
    else if (m_stSelfFuAlgParam->m_FusionNum == 3)
    { // 苏研院全息路口-融合跟踪算法
        // 加载相机反映射限制csv:
        load_csv("Configs/Alg/CppConfigs/fusion/intersection/reflect/camera_reflect_limit.csv");

        cout << "InitAlgorithm-suyan-intersection-fusion -> init" << endl;
        m_Fusionalgorithm = new Fusion_Algorithm_Intersection(m_stSelfFuAlgParam);
        cout << "InitAlgorithm-suyan-intersection-fusion -> sucess" << endl;
        LOG(INFO) << "InitAlgorithm-suyan-intersection-fusion -> sucess";
    }
    else if (m_stSelfFuAlgParam->m_FusionNum == 4)
    { // 苏研服务区融合算法

        // 加载相机反映射限制csv: 服务区好像没有？不知道先写着
        load_csv("Configs/Alg/CppConfigs/fusion/intersection/reflect/camera_reflect_limit.csv");

        cout << "InitAlgorithm-suyan-ServiceArea-fusion -> init" << endl;
        m_Fusionalgorithm = new Fusion_Algorithm_ServiceArea(m_stSelfFuAlgParam);
        cout << "InitAlgorithm-suyan-ServiceArea-fusion -> sucess" << endl;
        LOG(INFO) << "InitAlgorithm-suyan-ServiceArea-fusion -> sucess";
    }
    else
    {
        LOG(INFO) << "InitAlgorithm--fusion -> failed";
    }
    return true;
}

void *CFusionAlg::RunAlgorithm(void *p_pSrcData)
{
    std::cout << "liluo------------------>"
              << "RunAlgorithm begin" << std::endl;
    auto &l_tSrcData = *(TFusionResult *)p_pSrcData;
   
    // TFusionResult *m_tFusionResult = new TFusionResult();
    // service for sceneAlg
   
    // TAllVideoResult *m_video_results;
    // TPcResult *m_pc_result;

    auto &video_results = l_tSrcData.m_stVideoFusionResult;
    auto &pc_result = l_tSrcData.m_stPcFusionResult;

    auto l_tPcTran1 = std::chrono::high_resolution_clock::now();

    unsigned long pc_timestamp = pc_result.m_ulBufTimeStamp[0]; // 雷达时间戳

    int camera_num = video_results.m_vecResult.size();
    std::vector<unsigned long> camera_timestamp; // 相机时间戳
    for (int i = 0; i < camera_num; i++)
    {
        camera_timestamp.push_back(video_results.m_ulBufTimeStamp[0]);
    }
    m_Fusionalgorithm->m_pc_timestamp = pc_timestamp;
    m_Fusionalgorithm->m_camera_timestamp = camera_timestamp;

    // xt::xarray<float> pc_result;
    // std::vector<xt::xarray<float>> video_results;

    // 点云获取数据转换
    auto pc = PcAlgResTransfer(&pc_result, m_stSelfFuAlgParam);
    auto l_tPcTran2 = std::chrono::high_resolution_clock::now();
    // 图像获取数据转换
    std::vector<xt::xarray<float>> video_con;
    
    int i = 0;
    for (auto &v : video_results.m_vecResult)
    {
        xt::xarray<float> video = VideoAlgResTransfer(&v, m_stSelfFuAlgParam);
        if (m_stSelfFuAlgParam->m_FusionNum == 3 || m_stSelfFuAlgParam->m_FusionNum == 4)
        {
            /* 根据相机反映射配置，对相机检测目标进行过滤*/
            auto filter_box_index = xt::where(xt::col(video, m_camera_reflect_limit(i, 0)) > m_camera_reflect_limit(i, 2) || xt::col(video, m_camera_reflect_limit(i, 0)) < m_camera_reflect_limit(i, 1))[0];
            video = xt::view(video, xt::drop(filter_box_index));

            auto col = xt::view(video, xt::all(), 10);
            col = xt::where(col >= 1.0, 0.98, col);
            
            auto filter_box_x_index = xt::where(xt::col(video, 0) < 0.001f && xt::col(video, 0) >= 0.0f)[0];
            video = xt::view(video, xt::drop(filter_box_x_index));
        }
        video_con.push_back(video);
        i = i + 1;
    }
    // 融合进程
    Fusion_Res output = m_Fusionalgorithm->RUN(video_con, pc, 4);

    // 保存融合结果
    // save_trackdata(output.pc_res, l_tSrcData.m_stPcFusionResult.m_unFrameId);
    
    // 设置融合结果
    if (m_stSelfFuAlgParam->m_FusionNum == 3 || m_stSelfFuAlgParam->m_FusionNum == 4)
    {  // suzhou
        // shared_ptr<TFusionOutputResult> p_tFusionResult(new TFusionOutputResult());
        
        TFusionOutputResult *p_tFusionResult = new TFusionOutputResult();

        //add by szh 20230719
        p_tFusionResult->m_stFusionTrackResult.m_dLidarLat=l_tSrcData.m_stPcFusionResult.m_dLidarLat;
        p_tFusionResult->m_stFusionTrackResult.m_dLidarLon=l_tSrcData.m_stPcFusionResult.m_dLidarLon;
        p_tFusionResult->m_stFusionTrackResult.m_fLidarNorthAngle=l_tSrcData.m_stPcFusionResult.m_fLidarNorthAngle;

        p_tFusionResult->m_stFusionTrackResult.m_strTerminalId = l_tSrcData.m_stPcFusionResult.m_strTerminalId;
        p_tFusionResult->m_stFusionTrackResult.m_unFrameId = l_tSrcData.m_stPcFusionResult.m_unFrameId;
        p_tFusionResult->m_stFusionTrackResult.m_ucLidarId = l_tSrcData.m_stPcFusionResult.m_ucLidarId;
        p_tFusionResult->m_stFusionTrackResult.m_ulBufTimeStamp[0] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[0];
        p_tFusionResult->m_stFusionTrackResult.m_ulBufTimeStamp[1] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[1];
        p_tFusionResult->m_stFusionTrackResult.m_ulBufTimeStamp[2] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[2];
        p_tFusionResult->m_stFusionTrackResult.m_ulBufTimeStamp[3] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[3];
        p_tFusionResult->m_stFusionTrackResult.m_unBufDelay[0] = l_tSrcData.m_stPcFusionResult.m_unBufDelay[0];
        // ts add by zqj20230615 for log
        p_tFusionResult->m_stFusionTrackResult.m_ulBufTimeStamp[4] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[4];
        p_tFusionResult->m_stFusionTrackResult.m_ulBufTimeStamp[5] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[5];
        p_tFusionResult->m_stFusionTrackResult.m_ulBufTimeStamp[6] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[6];
        p_tFusionResult->m_stFusionTrackResult.m_ulBufTimeStamp[7] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[7];
        p_tFusionResult->m_stFusionTrackResult.m_ulBufTimeStamp[8] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[8];
        p_tFusionResult->m_stFusionTrackResult.m_ulBufTimeStamp[9] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[9];
        
        p_tFusionResult->m_stVideoFusionResult.m_ulBufTimeStamp[0] = l_tSrcData.m_stVideoFusionResult.m_ulBufTimeStamp[0];
        // fps add by zqj20230615 for log
        // std::cout << "--zqj--debug-alg-fps:"<< l_tSrcData.m_stPcFusionResult.m_fBufFps[0] <<"**" <<l_tSrcData.m_stPcFusionResult.m_fBufFps[1]<< std::endl;
        // std::cout << "--zqj--debug-alg-fps:"<< l_tSrcData.m_stPcFusionResult.m_fBufFps[2] <<"**" <<l_tSrcData.m_stPcFusionResult.m_fBufFps[3]<< std::endl;
        p_tFusionResult->m_stFusionTrackResult.m_fBufFps[0] = l_tSrcData.m_stPcFusionResult.m_fBufFps[0];
        p_tFusionResult->m_stFusionTrackResult.m_fBufFps[1] = l_tSrcData.m_stPcFusionResult.m_fBufFps[1];
        p_tFusionResult->m_stFusionTrackResult.m_fBufFps[2] = l_tSrcData.m_stPcFusionResult.m_fBufFps[2];
        p_tFusionResult->m_stFusionTrackResult.m_fBufFps[3] = l_tSrcData.m_stPcFusionResult.m_fBufFps[3];
        p_tFusionResult->m_stFusionTrackResult.m_fBufFps[4] = l_tSrcData.m_stPcFusionResult.m_fBufFps[4];
         p_tFusionResult->m_stFusionTrackResult.m_fBufFps[5] = l_tSrcData.m_stPcFusionResult.m_fBufFps[5];
        // delay add by zqj20230615 for log
        p_tFusionResult->m_stFusionTrackResult.m_unBufDelay[0] = l_tSrcData.m_stPcFusionResult.m_unBufDelay[0];
        p_tFusionResult->m_stFusionTrackResult.m_unBufDelay[1] = l_tSrcData.m_stPcFusionResult.m_unBufDelay[1];
        p_tFusionResult->m_stFusionTrackResult.m_unBufDelay[2] = l_tSrcData.m_stPcFusionResult.m_unBufDelay[2];
        p_tFusionResult->m_stFusionTrackResult.m_unBufDelay[3] = l_tSrcData.m_stPcFusionResult.m_unBufDelay[3];
        p_tFusionResult->m_stFusionTrackResult.m_unBufDelay[4] = l_tSrcData.m_stPcFusionResult.m_unBufDelay[4];
        
        p_tFusionResult->m_stVideoFusionResult.m_unBufDelay[0] = l_tSrcData.m_stVideoFusionResult.m_unBufDelay[0];
        p_tFusionResult->m_stVideoFusionResult.m_unBufDelay[1] = l_tSrcData.m_stVideoFusionResult.m_unBufDelay[1];
        p_tFusionResult->m_stVideoFusionResult.m_unBufDelay[2] = l_tSrcData.m_stVideoFusionResult.m_unBufDelay[2];

        setFusionResult4Scene(p_tFusionResult, m_stSelfFuAlgParam, output);
        // add by zqj20230615 for log  setFusionResult4Scene() with clear vector
        int cam_num = output.video_res.size();
        // std::cout << "--zqj--debug-alg-camnum:"<< cam_num<< std::endl;
        for (int i = 0; i < cam_num; i++){
            TVideoResult result;
            // p_tFusionResult->m_stVideoFusionResult.m_vecResult.push_back(result);
            p_tFusionResult->m_stVideoFusionResult.m_vecResult[i].m_ulRtpTimeStamp = l_tSrcData.m_stVideoFusionResult.m_vecResult[i].m_ulRtpTimeStamp;
            // std::cout << "--zqj--debug-alg-timestamp:"<< p_tFusionResult->m_stVideoFusionResult.m_vecResult[i].m_ulRtpTimeStamp <<"**" <<l_tSrcData.m_stVideoFusionResult.m_vecResult[i].m_ulRtpTimeStamp<< std::endl;
        }

        // std::cout << "liluo------------------>setFusionResult end" << std::endl;
        return p_tFusionResult;


    }else{
        TFusionResult *p_tFusionResult = new TFusionResult();
        p_tFusionResult->m_stPcFusionResult.m_strTerminalId = l_tSrcData.m_stPcFusionResult.m_strTerminalId;
        p_tFusionResult->m_stPcFusionResult.m_unFrameId = l_tSrcData.m_stPcFusionResult.m_unFrameId;
        p_tFusionResult->m_stPcFusionResult.m_ucLidarId = l_tSrcData.m_stPcFusionResult.m_ucLidarId;
        p_tFusionResult->m_stPcFusionResult.m_ulBufTimeStamp[0] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[0];
        p_tFusionResult->m_stPcFusionResult.m_ulBufTimeStamp[1] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[1];
        p_tFusionResult->m_stPcFusionResult.m_ulBufTimeStamp[2] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[2];
        p_tFusionResult->m_stPcFusionResult.m_ulBufTimeStamp[3] = l_tSrcData.m_stPcFusionResult.m_ulBufTimeStamp[3];
        p_tFusionResult->m_stPcFusionResult.m_unBufDelay[0] = l_tSrcData.m_stPcFusionResult.m_unBufDelay[0];
        setFusionResult(p_tFusionResult, m_stSelfFuAlgParam, output);
        return p_tFusionResult;
    }
    

    
}

void CFusionAlg::setFusionResult(TFusionResult *m_tFuResult, TSelfFusionAlgParam *p_pAlgParam, const Fusion_Res &result)
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

            fu_cPcBoxInfo.m_strClass = p_pAlgParam->m_fusion_parameter["fusion_param"]["m_strFusionClass"][int(pointcloud(i, 7))]; // 目标类型(点云算法传出的类型与视频传出的类型映射关系不同,所以此处采用字符串形式统一)

            fu_cPcBoxInfo.m_usSpeed = pointcloud(i, 8) * 100;            // 速度：m/s
            fu_cPcBoxInfo.m_usSingleId = pointcloud(i, 9);               // 单基站检测的ID
            fu_cPcBoxInfo.m_ucConfidence = int(pointcloud(i, 10) * 100); // 置信度

            // fu_cPcBoxInfo.m_ucLanId          = pointcloud(i, 12);               // chedao
            // fu_cPcBoxInfo.m_ucReserve[0]     = (char)pointcloud(i, 11);         // hits
            // fu_cPcBoxInfo.m_ucReserve[1]     = (char)pointcloud(i, 13);         // time-since-update
            // std::cout<<"video_id"<<pointcloud(i, 20)<<std::endl;
            if (pointcloud(i, 15) != 0 && pointcloud(i, 15) != -1)
            {
                fu_cPcBoxInfo.m_ucSource = 2; // 融合结果
            }
            else
            {
                fu_cPcBoxInfo.m_ucSource = 0; // 点云结果
            }
            // source pc or video or fusion
            //  fu_cPcBoxInfo.usVideoInfo[0] = (int)pointcloud(i, 16);    // leftx
            //  fu_cPcBoxInfo.usVideoInfo[1] = (int)pointcloud(i, 17);    // lefty
            //  fu_cPcBoxInfo.usVideoInfo[2] = (int)pointcloud(i, 18);    // rightx
            //  fu_cPcBoxInfo.usVideoInfo[3] = (int)pointcloud(i, 19);    // righty
            //  fu_cPcBoxInfo.usVideoInfo[4] = (int)pointcloud(i, 20);    // camid
            //  double l_dBoxLongitude = 0.0, l_dBoxLatitude=0.0;

            // XYZ_To_BLH(
            //     _dLidarLon, _dLidarLat,
            //     pointcloud(i,0),
            //     pointcloud(i,1),
            //     _fLidarNorthAngle,
            //     l_dBoxLongitude,
            //     l_dBoxLatitude);
            // fu_cPcBoxInfo.m_dLon = l_dBoxLongitude;
            // fu_cPcBoxInfo.m_dLat = l_dBoxLatitude;
            // pc_result.m_vecBox.push_back(fu_cPcBoxInfo);
            // pc_result.m_unFrameId = (int)pointcloud(i, 15);
            // pc_result.m_dLidarLon = _dLidarLon;
            // pc_result.m_dLidarLat = _dLidarLat;
            // pc_result.m_fLidarNorthAngle = _fLidarNorthAngle;
            pc_result.m_vecBox.push_back(fu_cPcBoxInfo);
        }
    }

    //图像结果处理
    auto &allvideo_result = m_tFuResult->m_stVideoFusionResult;
    allvideo_result.m_vecResult.clear();

    // 得到相机个数
    // int cam_num =int(p_pAlgParam->m_stCameraParam.m_vecCameraDev.size());
    int cam_num = result.video_res.size();

    allvideo_result.m_vecResult.resize(cam_num);

    // 相机检测结果
    std::vector<xt::xarray<float>> video_temp = result.video_res; 
    // x, y, w, l, theta(角度), z,    h,       label, speed, id, score, x_min,y_min,x_max,y_max, data_source, data_channel
    for (int i = 0; i < cam_num; i++)
    {
        int video_shape=video_temp[i].shape(0);
        allvideo_result.m_vecResult[i].m_vecBox.clear();
        allvideo_result.m_vecResult[i].m_ucCameraId = i;
        allvideo_result.m_vecResult[i].m_usBoxNum = video_shape;
        if (video_shape > 0)
        {
            for (int j = 0; j < video_shape; ++j)
            {
                TVideoBoxInfo fu_cVideoBoxInfo;
                //CVideoBoxInfo
                fu_cVideoBoxInfo.m_fTopLeftX        = video_temp[i](j, 11);       // left x
                fu_cVideoBoxInfo.m_fTopLeftY        = video_temp[i](j, 12);       // left y
                fu_cVideoBoxInfo.m_fBottomRightX    = video_temp[i](j, 13);       // right x
                fu_cVideoBoxInfo.m_fBottomRightY    = video_temp[i](j, 14);       // right y
                fu_cVideoBoxInfo.m_ucConfidence     = int(video_temp[i](j, 10) * 100);  // confidience
                fu_cVideoBoxInfo.m_strClass         = p_pAlgParam->m_stFusionAlgParam.m_vecFusionClass[int(video_temp[i](j, 7))];  //label
                fu_cVideoBoxInfo.m_usSingleId       = int(video_temp[i](j, 9));  // lidarid
                fu_cVideoBoxInfo.m_ucReserve[0]     = char(video_temp[i](j, 15)); // fusionflag  true:1 false:0
                allvideo_result.m_vecResult[i].m_vecBox.push_back(fu_cVideoBoxInfo);
            }
        }
    }
}

void CFusionAlg::setFusionResult4Scene(TFusionOutputResult *m_tFuTrackResult, TSelfFusionAlgParam *p_pAlgParam, const Fusion_Res &result)
{
    auto &pc_result = m_tFuTrackResult->m_stFusionTrackResult;

    xt::xarray<float> pointcloud = result.pc_res;  // 获取融合跟踪结果
    int pc_shape = int(pointcloud.shape(0));
    pc_result.m_vecBox.clear();
    pc_result.m_usBoxNum = pc_shape;

    // 把融合跟踪结果装入结构体中
    if (pc_shape > 0)
    {
        for (int i = 0; i < pc_shape; ++i)
        {
            // [0, 1, 2, 3,   4,   5, 6,   7,     8,    9,   10,     11,    12,    13,    14,     15,         16,     17,    18                19,         20,        21 ]
            // [x, y, w, l, theta, z, h, label, speed, id, scores, x_min, y_min, x_max, y_max, data_source, channel, lane, occur_time, time_sicne_update, hits, flag_delete_track]
            TFusionTrackBoxInfo fu_cPcBoxInfo;

            fu_cPcBoxInfo.m_sXCoord = pointcloud(i, 0) * 100;  // X轴坐标:m    0
            fu_cPcBoxInfo.m_sYCoord = pointcloud(i, 1) * 100;  // Y轴坐标:m    1
            fu_cPcBoxInfo.m_usWidth = pointcloud(i, 2) * 100;  // 宽度         2
            fu_cPcBoxInfo.m_usLength = pointcloud(i, 3) * 100; // 长度         3
            fu_cPcBoxInfo.m_usCourseAngle = pointcloud(i, 4);  // 航向角 角度   4
            // std::cout<<"angleout:"<<pointcloud(i, 4)<<std::endl;
            fu_cPcBoxInfo.m_sZCoord = pointcloud(i, 5) * 100;  // Z轴坐标:m    5
            fu_cPcBoxInfo.m_usHeight = pointcloud(i, 6) * 100; // 高度         6

            fu_cPcBoxInfo.m_strClass = p_pAlgParam->m_fusion_parameter["fusion_param"]["m_strFusionClass"][int(pointcloud(i, 7))]; // 目标类型(点云算法传出的类型与视频传出的类型映射关系不同,所以此处采用字符串形式统一)

            fu_cPcBoxInfo.m_usSpeed = pointcloud(i, 8) * 100;            // 速度：m/s
            fu_cPcBoxInfo.m_usSingleId = pointcloud(i, 9);               // 单基站检测的ID
            fu_cPcBoxInfo.m_ucConfidence = int(pointcloud(i, 10) * 100); // 置信度
            
            fu_cPcBoxInfo.m_usVideoInfo[0] = (uint16_t)pointcloud(i, 11);  // x_min
            fu_cPcBoxInfo.m_usVideoInfo[1] = (uint16_t)pointcloud(i, 12);  // y_min
            fu_cPcBoxInfo.m_usVideoInfo[2] = (uint16_t)pointcloud(i, 13);  // x_max
            fu_cPcBoxInfo.m_usVideoInfo[3] = (uint16_t)pointcloud(i, 14);  // x_max
            fu_cPcBoxInfo.m_ucSource = (unsigned char)(pointcloud(i, 15)); // data_source
            fu_cPcBoxInfo.m_ucReserve[0] = (unsigned char)(pointcloud(i, 16)); // channel
            fu_cPcBoxInfo.m_ucLanId          = (unsigned char)pointcloud(i, 17); // lane
            fu_cPcBoxInfo.m_ulOccurTime          = float(pointcloud(i, 18)); // occur_time
            fu_cPcBoxInfo.m_uiTimeSinceUpdate = uint32_t(pointcloud(i, 19));   // time_since_update
            fu_cPcBoxInfo.m_ulHits = uint64_t(pointcloud(i, 20));   // hits
            if (pointcloud(i, 21) == 0){
                fu_cPcBoxInfo.m_bDeleteFlag = false;
            }else{
                fu_cPcBoxInfo.m_bDeleteFlag = true;
            }
            // add by szh20230719
            double l_dBoxLongitude = 0.0, l_dBoxLatitude=0.0;
            double _dLidarLon= pc_result.m_dLidarLon, _dLidarLat=pc_result.m_dLidarLat;   
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

            // TODO:
            // if((fu_cPcBoxInfo.m_uiTimeSinceUpdate) <12 && (fu_cPcBoxInfo.m_ulHits > 3)){
            //     pc_result.m_vecBox.push_back(fu_cPcBoxInfo);

            // }
            pc_result.m_vecBox.push_back(fu_cPcBoxInfo);
            
        }
    }

    //图像结果处理
    auto &allvideo_result = m_tFuTrackResult->m_stVideoFusionResult;
    allvideo_result.m_vecResult.clear();

    // 得到相机个数
    // int cam_num =int(p_pAlgParam->m_stCameraParam.m_vecCameraDev.size());
    int cam_num = result.video_res.size();

    allvideo_result.m_vecResult.resize(cam_num);
    // 相机检测结果
    std::vector<xt::xarray<float>> video_temp = result.video_res; 
    // x, y, w, l, theta(角度), z,    h,       label, speed, id, score, x_min,y_min,x_max,y_max, data_source, data_channel
    for (int i = 0; i < cam_num; i++)
    {
        int video_shape=video_temp[i].shape(0);
        allvideo_result.m_vecResult[i].m_vecBox.clear();
        allvideo_result.m_vecResult[i].m_ucCameraId = i;
        allvideo_result.m_vecResult[i].m_usBoxNum = video_shape;

        if (video_shape > 0)
        {
            for (int j = 0; j < video_shape; ++j)
            {
                TVideoBoxInfo fu_cVideoBoxInfo;
                //CVideoBoxInfo
                fu_cVideoBoxInfo.m_fTopLeftX        = video_temp[i](j, 11);       // left x
                fu_cVideoBoxInfo.m_fTopLeftY        = video_temp[i](j, 12);       // left y
                fu_cVideoBoxInfo.m_fBottomRightX    = video_temp[i](j, 13);       // right x
                fu_cVideoBoxInfo.m_fBottomRightY    = video_temp[i](j, 14);       // right y
                fu_cVideoBoxInfo.m_ucConfidence     = int(video_temp[i](j, 10) * 100);  // confidience
                fu_cVideoBoxInfo.m_strClass         = p_pAlgParam->m_fusion_parameter["fusion_param"]["m_strVideoClass"][int(video_temp[i](j, 7))];  //label
                fu_cVideoBoxInfo.m_usSingleId       = int(video_temp[i](j, 9));  // lidarid
                fu_cVideoBoxInfo.m_ucReserve[0]     = char(video_temp[i](j, 15)); // fusionflag  true:1 false:0
                allvideo_result.m_vecResult[i].m_vecBox.push_back(fu_cVideoBoxInfo);
            }
        }
    }

}


int CFusionAlg::VideoAlgClassToFusionId(const std::string &p_strClass, TSelfFusionAlgParam *p_pAlgParam)
{
    for (int i = 0; i < 12; i++)
    {
        if (p_strClass == p_pAlgParam->m_stFusionAlgParam.m_vecFusionClass[i])
        {
            return i;
        }
    }
    if (p_strClass == "truck")
    {
        return 11;
    }
    if (p_strClass == "bus")
    {
        return 3;
    }
    if (p_strClass == "dump_truck")
    {
        return 2;
    }
    if (p_strClass == "road_work_sign" || "parking_tripod")
    {
        return 11;
    }
    if (p_strClass == "others_bicycle_per" || "MTfood_bicycle_per" || "ELMfood_bicycle_per")
    {
        return 5;
    }
    if (p_strClass == "tricycle" || "JD_tricycle" || "SF_tricycle" || "YZ_tricycle" || "YD_tricycle")
    {
        return 6;
    }
    if (p_strClass == "slagcar" || "tanker")
    {
        return 8;
    }
    return 1; // car
}

int CFusionAlg::PcAlgClassToFusionId(const std::string &p_strClass, TSelfFusionAlgParam *p_pAlgParam)
{
    for (int i = 0; i < 12; i++)
    {
        if (p_strClass == p_pAlgParam->m_stFusionAlgParam.m_vecFusionClass[i])
        {
            return i;
        }
    }
    if (p_strClass == "bus")
    {
        return 3;
    }
    if (p_strClass == "tricycle")
    {
        return 6;
    }
    if (p_strClass == "dump_truck")
    {
        return 3;
    }
    if (p_strClass == "truck")
    {
        return 11;
    }
    return 1; // car
}

void CFusionAlg::load_csv(string _file_path)
{
    std::ifstream file(_file_path, std::ios::in);
    if (!file.good())
    {
        perror("can't open file");
    }
    std::string line;
    int row = 0;
    int column = 0;
    std::vector<int> data;
    while (std::getline(file, line))
    {
        /* code */
        column = 0;
        row++;

        // if(row == 1){
        // 	continue;
        // }

        std::stringstream ss(line);
        float value;

        // std::vector<float> row_data;

        while (ss >> value)
        {
            /* code */

            data.push_back(value);
            if (ss.peek() == ',')
            {
                ss.ignore();
            }
            column++;
        }

        // data.push_back(row_data);
    }
    file.close();

    // for(int i = 0; i<data.size(); i++){
    // 	for (int j = 0; j<data[i].size(); j++){
    // 		std::cout << data[i][j] << " ";
    // 	}
    // 	std::cout << "\n";
    // }

    m_camera_reflect_limit = xt::adapt(data, {row, column});
}

xt::xarray<float> CFusionAlg::VideoAlgResTransfer(TVideoResult *p_pVideoResult, TSelfFusionAlgParam *p_pAlgParam)
{
    if (p_pAlgParam->m_FusionNum == 3 || m_stSelfFuAlgParam->m_FusionNum == 4)
    {

        xt::xarray<float> each_video_for_fusion_box;
        int each_video_box_num = p_pVideoResult->m_vecBox.size();
        if (each_video_box_num == 0)
        { // 该路相机的目标框集个数为0
            each_video_for_fusion_box = xt::empty<float>({0, 17});
        }
        else
        {
            each_video_for_fusion_box = xt::zeros<float>({each_video_box_num, 17});
            for (int j = 0; j < each_video_box_num; j++)
            {                                                                                                                                                // 遍历某一路相机的所有目标框
                each_video_for_fusion_box(j, 0) = float(p_pVideoResult->m_vecBox[j].m_sXCoord) / 100;                                                        // x
                each_video_for_fusion_box(j, 1) = float(p_pVideoResult->m_vecBox[j].m_sYCoord) / 100;                                                        // y
                each_video_for_fusion_box(j, 2) = float(p_pVideoResult->m_vecBox[j].m_usWidth) / 100;                                                        // w
                each_video_for_fusion_box(j, 3) = float(p_pVideoResult->m_vecBox[j].m_usLength) / 100;                                                       // l
                each_video_for_fusion_box(j, 4) = float(p_pVideoResult->m_vecBox[j].m_sTheta) / 100;                                                         // theta
                each_video_for_fusion_box(j, 5) = float(p_pVideoResult->m_vecBox[j].m_sZCoord) / 100;                                                        // z
                each_video_for_fusion_box(j, 6) = float(p_pVideoResult->m_vecBox[j].m_usHeight) / 100;                                                       // h
                each_video_for_fusion_box(j, 7) = FunctionHub::videoClass2label(p_pVideoResult->m_vecBox[j].m_strClass, &(p_pAlgParam->m_fusion_parameter)); // label
                each_video_for_fusion_box(j, 8) = float(p_pVideoResult->m_vecBox[j].m_sSpeed) / 100;
                each_video_for_fusion_box(j, 9) = p_pVideoResult->m_vecBox[j].m_usSingleId;
                each_video_for_fusion_box(j, 10) = float(p_pVideoResult->m_vecBox[j].m_ucConfidence) / 100;
                each_video_for_fusion_box(j, 11) = p_pVideoResult->m_vecBox[j].m_fTopLeftX;
                each_video_for_fusion_box(j, 12) = p_pVideoResult->m_vecBox[j].m_fTopLeftY;
                each_video_for_fusion_box(j, 13) = p_pVideoResult->m_vecBox[j].m_fBottomRightX;
                each_video_for_fusion_box(j, 14) = p_pVideoResult->m_vecBox[j].m_fBottomRightY;
                each_video_for_fusion_box(j, 15) = p_pVideoResult->m_vecBox[j].m_sDataSource;
                each_video_for_fusion_box(j, 16) = p_pVideoResult->m_vecBox[j].m_sChannel;
            }
        }
        return each_video_for_fusion_box;
    }
    xt::xarray<float> l_arrFaild = xt::ones<float>({1, 1});
    if (p_pVideoResult)
    {
        xt::xarray<float> l_TransferOut = xt::ones<float>({int(p_pVideoResult->m_usBoxNum), 7});
        for (int i = 0; i < p_pVideoResult->m_usBoxNum; i++)
        {
            l_TransferOut(i, 0) = p_pVideoResult->m_vecBox[i].m_fTopLeftX;
            l_TransferOut(i, 1) = p_pVideoResult->m_vecBox[i].m_fTopLeftY;
            l_TransferOut(i, 2) = p_pVideoResult->m_vecBox[i].m_fBottomRightX;
            l_TransferOut(i, 3) = p_pVideoResult->m_vecBox[i].m_fBottomRightY;
            l_TransferOut(i, 5) = float(p_pVideoResult->m_vecBox[i].m_ucConfidence / 100.0f);
            l_TransferOut(i, 4) = VideoAlgClassToFusionId(p_pVideoResult->m_vecBox[i].m_strClass, p_pAlgParam);
            l_TransferOut(i, 6) = p_pVideoResult->m_vecBox[i].m_usSingleId;
        }
        return l_TransferOut;
    }
    else
    {
        return l_arrFaild;
    }
}

xt::xarray<float> CFusionAlg::PcAlgResTransfer(TPcResult *p_pPcResult, TSelfFusionAlgParam *p_pAlgParam)
{
    if (p_pAlgParam->m_FusionNum == 3 || m_stSelfFuAlgParam->m_FusionNum == 4)
    {
        // x,y,w,l,theta(角度),z,h,label,speed, id, score, [x_min,y_min,x_max,y_max,] data_source, [data_channel]
        xt::xarray<float> lidar_box = xt::zeros<float>({int(p_pPcResult->m_vecBox.size()), 17});

        // std::vector<int> fake_det_index;
        // float fake_object_1_x = -71.50;
        // float fake_object_1_y = -3.30;
        // float fake_object_2_x = -3.06;
        // float fake_object_2_y = -6.72;
        // float fake_object_3_x = -26.33;
        // float fake_object_3_y = -5.83;

        // float fake_object_4_x = -2.01;
        // float fake_object_4_y = -6.43;

        // float fake_object_5_x = 46.19;
        // float fake_object_5_y = -20.26;

        // float fake_object_6_x = 8.67;
        // float fake_object_6_y = -8.38;

        // float fake_object_7_x = 40.82;
        // float fake_object_7_y = -26.40;

        // float fake_object_8_x = -192.5;
        // float fake_object_8_y = -33.32;

        // bool real_flag_1 = true;
        // bool real_flag_2 = true;
        // bool real_flag_3 = true;
        // bool real_flag_4 = true;
        // bool real_flag_5 = true;
        // bool real_flag_6 = true;
        // bool real_flag_7 = true;
        // bool real_flag_8 = true;

        int label;
        for (int i = 0; i < int(p_pPcResult->m_vecBox.size()); i++)
        {
            lidar_box(i, 0) = float(p_pPcResult->m_vecBox[i].m_sXCoord) / 100;         // x
            lidar_box(i, 1) = float(p_pPcResult->m_vecBox[i].m_sYCoord) / 100;         // y
            

            // if ((abs(lidar_box(i, 0) - fake_object_1_x)<1)&&(abs(lidar_box(i, 1) - fake_object_1_y)<1)){
            //     real_flag_1 = false;
            // }
            // if ((abs(lidar_box(i, 0) - fake_object_2_x)<1)&&(abs(lidar_box(i, 1) - fake_object_2_y)<1)){
            //     real_flag_2 = false;
            // }
            // if ((abs(lidar_box(i, 0) - fake_object_3_x)<1)&&(abs(lidar_box(i, 1) - fake_object_3_y)<1)){
            //     real_flag_3 = false;
            // }
            // if ((abs(lidar_box(i, 0) - fake_object_4_x)<1)&&(abs(lidar_box(i, 1) - fake_object_4_y)<1)){
            //     real_flag_4 = false;
            // }
            // if ((abs(lidar_box(i, 0) - fake_object_5_x)<1)&&(abs(lidar_box(i, 1) - fake_object_5_y)<1)){
            //     real_flag_5 = false;
            // }
            // if ((abs(lidar_box(i, 0) - fake_object_6_x)<1)&&(abs(lidar_box(i, 1) - fake_object_6_y)<1)){
            //     real_flag_6 = false;
            // }
            // if ((abs(lidar_box(i, 0) - fake_object_7_x)<1)&&(abs(lidar_box(i, 1) - fake_object_7_y)<1)){
            //     real_flag_7 = false;
            // }
            // if ((abs(lidar_box(i, 0) - fake_object_8_x)<1)&&(abs(lidar_box(i, 1) - fake_object_8_y)<1)){
            //     real_flag_8 = false;
            // }
            // if (!(real_flag_1&&real_flag_2&&real_flag_3 && real_flag_4&&real_flag_5&&real_flag_6 && real_flag_7&&real_flag_8)){
            //     // std::cout << "liluo----------------------> filter fake det!" << std::endl;
            //     fake_det_index.push_back(i);
            // }


            lidar_box(i, 2) = float(p_pPcResult->m_vecBox[i].m_usWidth) / 100;         // w
            lidar_box(i, 3) = float(p_pPcResult->m_vecBox[i].m_usLength) / 100;        // l
            lidar_box(i, 4) = (float(p_pPcResult->m_vecBox[i].m_usCourseAngle) / 100); //* 180 / PI; // theta
            lidar_box(i, 5) = float(p_pPcResult->m_vecBox[i].m_sZCoord) / 100;         // z
            lidar_box(i, 6) = float(p_pPcResult->m_vecBox[i].m_usHeight) / 100;        // h

            label = FunctionHub::pcClass2label(p_pPcResult->m_vecBox[i].m_strClass, &(p_pAlgParam->m_fusion_parameter));
            // if (label == 2 or label == 8)
            // {
            //     label = 2;
            // }
            for (auto item : (p_pAlgParam->m_fusion_parameter)["fusion_param"]["lidar_class_to_fusion_class"].items())
            {
                if (label == int(stof(item.key())))
                    label = item.value();
            }
            lidar_box(i, 7) = label; // label

            lidar_box(i, 8) = float(p_pPcResult->m_vecBox[i].m_usSpeed);             // speed
            lidar_box(i, 9) = p_pPcResult->m_vecBox[i].m_usSingleId;                 // id
            lidar_box(i, 10) = float(p_pPcResult->m_vecBox[i].m_ucConfidence) / 100; // score

            lidar_box(i, 11) = -1000; // x_min
            lidar_box(i, 12) = -1000; // y_min
            lidar_box(i, 13) = -1000; // x_max
            lidar_box(i, 14) = -1000; // y_max
            lidar_box(i, 15) = 2;     // data_source
            lidar_box(i, 16) = 0;     // data_channel
        }
        // if(fake_det_index.size() > 0){
        //     lidar_box = xt::view(lidar_box, xt::drop(fake_det_index));
        // }
        return lidar_box;
    }
    else
    {
        xt::xarray<float> l_arrTrackOut = xt::zeros<float>({int(p_pPcResult->m_vecBox.size()), 16});
        //{x, y, z, w, l, h, yaw, class, speed, id, conf, 8 * 3 }
        for (int i = 0; i < p_pPcResult->m_vecBox.size(); i++)
        {
            l_arrTrackOut(i, 0) = p_pPcResult->m_vecBox[i].m_sXCoord / 100.0; // cm->m
            l_arrTrackOut(i, 1) = p_pPcResult->m_vecBox[i].m_sYCoord / 100.0;
            l_arrTrackOut(i, 2) = p_pPcResult->m_vecBox[i].m_sZCoord / 100.0;
            l_arrTrackOut(i, 3) = p_pPcResult->m_vecBox[i].m_usWidth / 100.0;
            l_arrTrackOut(i, 4) = p_pPcResult->m_vecBox[i].m_usLength / 100.0;
            l_arrTrackOut(i, 5) = p_pPcResult->m_vecBox[i].m_usHeight / 100.0;
            l_arrTrackOut(i, 6) = p_pPcResult->m_vecBox[i].m_usCourseAngle; // 角度 y偏x
            // std::cout<<"angle:"<<l_arrTrackOut(i, 6)<<std::endl;
            l_arrTrackOut(i, 7) = PcAlgClassToFusionId(p_pPcResult->m_vecBox[i].m_strClass, p_pAlgParam);
            l_arrTrackOut(i, 8) = p_pPcResult->m_vecBox[i].m_usSpeed / 100.0; // 输入一般为0
            l_arrTrackOut(i, 9) = p_pPcResult->m_vecBox[i].m_usSingleId;
            l_arrTrackOut(i, 10) = p_pPcResult->m_vecBox[i].m_ucConfidence / 100.0; // 100->1
        }
        _dLidarLon = p_pPcResult->m_dLidarLon;
        _dLidarLat = p_pPcResult->m_dLidarLat;
        _fLidarNorthAngle = p_pPcResult->m_fLidarNorthAngle;

        xt::xarray<float> output = xt::zeros<float>({int(l_arrTrackOut.shape(0)), 35});
        xt::view(output, xt::all(), xt::range(0, 11)) = xt::view(l_arrTrackOut, xt::all(), xt::range(0, 11));
        for (int i = 0; i < output.shape(0); ++i)
        {
            xt::xarray<float> loc = xt::view(output, i, xt::range(0, 3));
            loc = loc.reshape({1, 3});
            xt::xarray<float> dim = xt::view(output, i, xt::range(3, 6));
            dim = dim.reshape({1, 3});
            xt::xarray<float> out_box = box_ops_crossing::center_to_corner_box3d(loc, dim, output(i, 6));
            out_box = out_box.reshape({out_box.size()});
            xt::view(output, i, xt::range(11, 35)) = out_box;
        }
        return output;
    }
}