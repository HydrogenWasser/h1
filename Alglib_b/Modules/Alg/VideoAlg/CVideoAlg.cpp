#include "CVideoAlg.h"
#include "Log/glog/logging.h"
#include "TSelfVideoAlgParam.h"
#include <cstdio>
int xcb_int = 0;

CVideoAlg::CVideoAlg(const std::string &p_strExePath) : m_strOutPath(p_strExePath)
{
    LOG(INFO) << "m_strOutPath: " << m_strOutPath << std::endl;
}

CVideoAlg::~CVideoAlg()
{
    delete m_VideoDetector;
    for (int i = 0; i < m_arrVideoTracker.size(); ++i)
    {
        delete m_arrVideoTracker[i];
    }
}

bool CVideoAlg::SetAlgParam(const TSelfVideoAlgParam *p_pAlgParam)
{
    if (p_pAlgParam)
    {
        m_stSelfVideoAlgParam = const_cast<TSelfVideoAlgParam *>(p_pAlgParam);
    }
    else
    {
        LOG(ERROR) << "The incoming parameter is empty" << std::endl;
        return false;
    }
    return true;
}

bool CVideoAlg::InitAlgorithm(void *p_pInitParam)
{
    try
    {
        LOG(INFO) << "InitAlgorithm-------" << std::endl;
        m_stSelfVideoAlgParam->m_strRootPath = m_strOutPath;
        // 视频检测
        if (m_stSelfVideoAlgParam->m_VideoDetectNum == 1)
        { // yolov5
            m_VideoDetector = new yolov5(m_stSelfVideoAlgParam);
            LOG(INFO) << "InitAlgorithm--yolov5 -> sucess";
        }
        else if (m_stSelfVideoAlgParam->m_VideoDetectNum == 2)
        { // yolov6
            m_VideoDetector = new yolov6(m_stSelfVideoAlgParam);
            LOG(INFO) << "InitAlgorithm--yolov6 -> sucess";
        }
        else
        {
            m_VideoDetector = nullptr; // 不使用
            LOG(INFO) << "InitAlgorithm--video_detector -> failed";
        }

        // 视频跟踪
        // if (m_stSelfVideoAlgParam->m_VideoTrackNum == 1) { //路口视频跟踪算法
        //     for (size_t i = 0; i < m_stSelfVideoAlgParam->m_stCameraParam.m_vecCameraDev.size(); ++i) {
        //         m_arrVideoTracker.push_back(new Use_DeepSort(m_stSelfVideoAlgParam));
        //     }
        //     LOG(INFO) << "InitAlgorithm--video_tracker -> sucess";
        // }
        // else{
        //     m_arrVideoTracker.push_back(nullptr); //不使用
        //     LOG(INFO) << "InitAlgorithm--video_tracker -> failed";
        // }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }

    // reflect init
    try
    {
        LOG(INFO) << "Init reflect-------" << std::endl;
        std::string m_rorate_file_path = "Configs/Alg/CppConfigs/video/ReflectFiles/calibration_rotate.csv";
        std::string m_trans_file_path = "Configs/Alg/CppConfigs/video/ReflectFiles/calibration_trans.csv";
        std::string m_filter_image_dir = "Configs/Alg/CppConfigs/video/ReflectFiles/";
        std::string m_pixel_xyz_dir = "Configs/Alg/CppConfigs/video/ReflectFiles/";
        int m_camera_dev_num = 4; // m_stSelfVideoAlgParam->m_stCameraParam.m_vecCameraDev.size();
        // TCameraParam *m_camera_param = &(m_stSelfVideoAlgParam->m_stCameraParam);
        TCameraParam *m_camera_param = nullptr;
        ParaMgrVideo *fusion_paramgr = new ParaMgrVideo();
        std::string filepath = "Configs/Alg/CppConfigs/video/ReflectFiles/fusion_param.json";
        fusion_paramgr->jsonload(filepath);
        bool flag = fusion_paramgr->get(m_parameter);
        m_AlgRsultReflect = new Process_reflect(&m_parameter);
        m_AlgRsultReflect->init_reflect(m_rorate_file_path, m_trans_file_path, m_camera_param, m_camera_dev_num, m_filter_image_dir, m_pixel_xyz_dir);
        m_AlgRsultReflect->load_fusion_param();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    return true;
}

void *CVideoAlg::RunAlgorithm(void *p_pSrcData)
{
    auto t_start = std::chrono::high_resolution_clock::now();
    if (!p_pSrcData)
    {
        return &m_stVideoAlgResult;
    }

    // 视频数据处理
    auto t_1 = std::chrono::high_resolution_clock::now();
    TVideoSrcDataTimematch *l_pVideoSrcData = static_cast<TVideoSrcDataTimematch *>(p_pSrcData);
    LOG(INFO) << "RunAlgorithm                FrameID : " << l_pVideoSrcData->m_vecSrcData[0].m_unFrameId << " ." << std::endl;
    int l_nVideoNumber = l_pVideoSrcData->m_vecSrcData.size();
    std::vector<cv::Mat> l_vecVideoSrcData;
    LOG(INFO) << "l_nVideoNumber:  " << l_nVideoNumber << std::endl;
    for (int i = 0; i < l_nVideoNumber; ++i)
    {
        cv::Mat l_matData = cv::Mat(l_pVideoSrcData->m_vecSrcData[i].m_vecImageBuf);
        l_matData = l_matData.reshape(3, l_pVideoSrcData->m_vecSrcData[i].m_usBmpLength).clone();
        l_vecVideoSrcData.push_back(l_matData);
    }
    auto t_2 = std::chrono::high_resolution_clock::now();
    auto latency12 = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_2 - t_1).count();
    LOG(INFO) << "RunAlgorithm       Load_" << l_nVideoNumber << "_VideoData : " << latency12 << " us." << std::endl;

    // 视频检测算法
    auto t_3 = std::chrono::high_resolution_clock::now();
    std::vector<xt::xarray<float>> video_detect_res = m_VideoDetector->RUN(l_vecVideoSrcData);
    // LOG(INFO)<<"video_detect_res0: " << video_detect_res[0] << std::endl;
    // LOG(INFO)<<"video_detect_res1: " << video_detect_res[1] << std::endl;
    // LOG(INFO)<<"video_detect_res2: " << video_detect_res[2] << std::endl;
    std::vector<xt::xarray<float>> video_detect_res_reflect;                                 // xcb add
    m_AlgRsultReflect->get_tracker_videoBoxInfo(video_detect_res, video_detect_res_reflect); // xcb add boxes reflect

    // for (int i = 1; i < video_detect_res.size(); ++i) {
    //     std::cout << video_detect_res[i];
    // }
    auto t_4 = std::chrono::high_resolution_clock::now();
    auto latency34 = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_4 - t_3).count();
    LOG(INFO) << "RunAlgorithm               VideoDet : " << latency34 << " us." << std::endl;

    int total_object = 0;
    m_stVideoAlgResult.m_vecVideoResult.m_vecResult.clear();

    // 判断是否进行视频跟踪算法
    //  if (m_arrVideoTracker[0])
    //  {
    //      LOG(INFO)<<"xcb----------333333333333333331111: " << std::endl;
    //      auto t_5 = std::chrono::high_resolution_clock::now();
    //      for (int i = 0; i < l_nVideoNumber; ++i)
    //      {
    //          xt::xarray<float> & l_arrVideoSortInput = video_detect_res[i];
    //          // 视频跟踪
    //          auto l_arrTrackResult = m_arrVideoTracker[i]->RUN(l_arrVideoSortInput);
    //          TVideoResult m_stVideoRes;                                                          // m_vecAllVideoResult的内容
    //          std::vector<TVideoBoxInfo> m_vecBox(l_arrTrackResult.shape(0));            // m_stVideoRes的成员
    //          total_object += l_arrTrackResult.shape(0);

    //         std::vector<float> l_vecOutBoxes(l_arrTrackResult.shape(0) * 4);
    //         std::vector<int> out_class_idxs(l_arrTrackResult.shape(0));
    //         std::vector<float> out_score(l_arrTrackResult.shape(0));
    //         std::vector<int> out_track_id(l_arrTrackResult.shape(0));
    //         for (int j = 0; j < l_arrTrackResult.shape(0); ++j) {
    //             l_vecOutBoxes[j * 4 + 0] = l_arrTrackResult(j, 0);
    //             l_vecOutBoxes[j * 4 + 1] = l_arrTrackResult(j, 1);
    //             l_vecOutBoxes[j * 4 + 2] = l_arrTrackResult(j, 2);
    //             l_vecOutBoxes[j * 4 + 3] = l_arrTrackResult(j, 3);
    //             out_class_idxs[j] = int(l_arrTrackResult(j, 5));
    //             out_score[j] = l_arrTrackResult(j, 4);
    //             out_track_id[j] = int(l_arrTrackResult(j, 6));
    //         }

    //         for(int j = 0; j < l_arrTrackResult.shape(0); j++)
    //         {
    //             TVideoBoxInfo l_cVideoBoxInfo;
    //             l_cVideoBoxInfo.m_fTopLeftX = l_arrTrackResult(j, 0);
    //             l_cVideoBoxInfo.m_fTopLeftY = l_arrTrackResult(j, 1);
    //             l_cVideoBoxInfo.m_fBottomRightX = l_arrTrackResult(j, 2);
    //             l_cVideoBoxInfo.m_fBottomRightY = l_arrTrackResult(j, 3);
    //             l_cVideoBoxInfo.m_strClass = m_stSelfVideoAlgParam->m_stVideoAlgParam.m_vecVideoClass[out_class_idxs[j]];
    //             l_cVideoBoxInfo.m_ucConfidence = out_score[j] * 100;
    //             l_cVideoBoxInfo.m_usGlobalId = out_track_id[j];
    //             l_cVideoBoxInfo.m_usSingleId = out_track_id[j];
    //             m_vecBox[j] = l_cVideoBoxInfo;
    //         }
    //         m_stVideoRes.m_ucCameraId = l_pVideoSrcData->m_vecSrcData[i].m_unCameraId;
    //         m_stVideoRes.m_ulTimeStamp = l_pVideoSrcData->m_vecSrcData[i].m_ulTimeStamp;
    //         m_stVideoRes.m_usBoxNum = (unsigned short)m_vecBox.size();
    //         m_stVideoRes.m_vecBox = m_vecBox;

    //         m_stVideoAlgResult.m_vecVideoResult.m_unFrameId = l_pVideoSrcData->m_vecSrcData[i].m_unFrameId;
    //         // l_cAllVideoResult.m_ulTimeStampMatch(l_pVideoSrcData->ulBufTimeStamp()[0]);
    //         m_stVideoAlgResult.m_vecVideoResult.m_ulBufTimeStamp[0] = l_pVideoSrcData->m_ulBufTimeStamp[0];
    //         m_stVideoAlgResult.m_vecVideoResult.m_ucResultSource = l_pVideoSrcData->m_vecSrcData[i].m_ucVideoSource;

    //         m_stVideoAlgResult.m_vecVideoResult.m_vecResult.push_back(m_stVideoRes);
    //     }
    //     auto t_6 = std::chrono::high_resolution_clock::now();
    //     auto latency56 = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_6 - t_5).count();
    //     LOG(INFO)<<"RunAlgorithm      TotalObjectNum : " << total_object << " ." << std::endl;
    //     LOG(INFO)<<"RunAlgorithm VideoTrackAndReturn : " << latency56 << " us." << std::endl;

    // }

    // 视频算法结果处理
    auto t_5 = std::chrono::high_resolution_clock::now();
    // offline test
    //  std::string save_idx_new_label = "000000" + std::to_string(xcb_int++);
    //  std::string data_path_label = "/data/test_data/cross_data/qlg_west_1/dts/"  + save_idx_new_label.substr(save_idx_new_label.length()-6, 6) + ".txt";
    //  ofstream write(data_path_label, ios::out);
    for (int i = 0; i < l_nVideoNumber; ++i)
    {
        xt::xarray<float> &l_arrVideoDetRes = video_detect_res_reflect[i];
        TVideoResult m_stVideoRes;                                      // m_vecAllVideoResult的内容
        std::vector<TVideoBoxInfo> m_vecBox(l_arrVideoDetRes.shape(0)); // m_stVideoRes的成员
        total_object += l_arrVideoDetRes.shape(0);

        for (int j = 0; j < l_arrVideoDetRes.shape(0); j++)
        {
            TVideoBoxInfo l_cVideoBoxInfo;
            // l_cVideoBoxInfo.m_fTopLeftX = l_arrVideoDetRes(j, 0);
            // l_cVideoBoxInfo.m_fTopLeftY = l_arrVideoDetRes(j, 1);
            // // l_cVideoBoxInfo.m_fBottomRightX = l_arrVideoDetRes(j, 0) + l_arrVideoDetRes(j, 2);
            // // l_cVideoBoxInfo.m_fBottomRightY = l_arrVideoDetRes(j, 1) + l_arrVideoDetRes(j, 3);
            // l_cVideoBoxInfo.m_fBottomRightX = l_arrVideoDetRes(j, 2);
            // l_cVideoBoxInfo.m_fBottomRightY = l_arrVideoDetRes(j, 3);
            // l_cVideoBoxInfo.m_strClass = m_stSelfVideoAlgParam->m_stVideoAlgParam.m_vecVideoClass[int(l_arrVideoDetRes(j, 5))];
            // l_cVideoBoxInfo.m_ucConfidence = l_arrVideoDetRes(j, 4) * 100;
            // l_cVideoBoxInfo.m_usGlobalId = 0;
            // l_cVideoBoxInfo.m_usSingleId = 0;
            // m_vecBox[j] = l_cVideoBoxInfo;

            // offline test use
            // write << int(l_arrVideoDetRes(j, 5)) << " "
            // << (l_arrVideoDetRes(j, 0) + l_arrVideoDetRes(j, 2) / 2) / 1920<< " "
            // << (l_arrVideoDetRes(j, 1) + l_arrVideoDetRes(j, 3) / 2) /1080 << " "
            // << l_arrVideoDetRes(j, 2)/1920 << " "
            // << l_arrVideoDetRes(j, 3) /1080<< " "
            // << l_arrVideoDetRes(j, 4) << endl;

            l_cVideoBoxInfo.m_usGlobalId = 0;
            l_cVideoBoxInfo.m_usSingleId = 0;
            l_cVideoBoxInfo.m_sXCoord = l_arrVideoDetRes(j, 0) * 100;
            l_cVideoBoxInfo.m_sYCoord = l_arrVideoDetRes(j, 1) * 100;
            l_cVideoBoxInfo.m_usWidth = l_arrVideoDetRes(j, 3) * 100;
            l_cVideoBoxInfo.m_usLength = l_arrVideoDetRes(j, 4) * 100;
            l_cVideoBoxInfo.m_sTheta = l_arrVideoDetRes(j, 6) * 100;
            l_cVideoBoxInfo.m_sZCoord = l_arrVideoDetRes(j, 2) * 100;
            l_cVideoBoxInfo.m_usHeight = l_arrVideoDetRes(j, 5) * 100;
            l_cVideoBoxInfo.m_strClass = (m_parameter)["fusion_param"]["m_strVideoClass"][int(l_arrVideoDetRes(j, 7))];
            l_cVideoBoxInfo.m_sSpeed = l_arrVideoDetRes(j, 8);
            l_cVideoBoxInfo.m_usSingleId = l_arrVideoDetRes(j, 9);
            l_cVideoBoxInfo.m_ucConfidence = l_arrVideoDetRes(j, 10) * 100;
            l_cVideoBoxInfo.m_fTopLeftX = l_arrVideoDetRes(j, 11);
            l_cVideoBoxInfo.m_fTopLeftY = l_arrVideoDetRes(j, 12);
            l_cVideoBoxInfo.m_fBottomRightX = l_arrVideoDetRes(j, 13);
            l_cVideoBoxInfo.m_fBottomRightY = l_arrVideoDetRes(j, 14);
            l_cVideoBoxInfo.m_sDataSource = l_arrVideoDetRes(j, 15);
            l_cVideoBoxInfo.m_sChannel = l_arrVideoDetRes(j, 16);
            m_vecBox[j] = l_cVideoBoxInfo;
            // LOG(ERROR)<<"l_cVideoBoxInfo.m_sXCoord : " << l_cVideoBoxInfo.m_sXCoord << std::endl;
            
        }
        m_stVideoRes.m_ucCameraId = l_pVideoSrcData->m_vecSrcData[i].m_unCameraId;
        m_stVideoRes.m_ulTimeStamp = l_pVideoSrcData->m_vecSrcData[i].m_ulTimeStamp;
        m_stVideoRes.m_usBoxNum = (unsigned short)m_vecBox.size();
        m_stVideoRes.m_vecBox = m_vecBox;

        m_stVideoRes.m_ucCameraId = l_pVideoSrcData->m_vecSrcData[i].m_unCameraId;
        m_stVideoRes.m_ulTimeStamp = l_pVideoSrcData->m_vecSrcData[i].m_ulTimeStamp;
        m_stVideoRes.m_usBoxNum = (unsigned short)m_vecBox.size();
        m_stVideoRes.m_vecBox = m_vecBox;
        
        m_stVideoRes.m_ulRtpTimeStamp = l_pVideoSrcData->m_vecSrcData[i].m_ulRtpTimeStamp;  // zqj20230615 for log
        // std::cout<<"--zqj-debug-alg-m_stVideoRes.m_ulRtpTimeStamp : "<<m_stVideoRes.m_ulRtpTimeStamp<<std::endl;

        m_stVideoAlgResult.m_vecVideoResult.m_unFrameId = l_pVideoSrcData->m_vecSrcData[i].m_unFrameId;
        // l_cAllVideoResult.m_ulTimeStampMatch(l_pVideoSrcData->ulBufTimeStamp()[0]);
        m_stVideoAlgResult.m_vecVideoResult.m_ulBufTimeStamp[0] = l_pVideoSrcData->m_ulBufTimeStamp[0];
        m_stVideoAlgResult.m_vecVideoResult.m_ucResultSource = l_pVideoSrcData->m_vecSrcData[i].m_ucVideoSource;
        m_stVideoAlgResult.m_vecVideoResult.m_vecResult.push_back(m_stVideoRes);
    }
    // write.close(); //offline test

    auto t_6 = std::chrono::high_resolution_clock::now();
    auto latency56 = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_6 - t_5).count();
    LOG(INFO) << "RunAlgorithm        TotalObjectNum : " << total_object << " ." << std::endl;
    LOG(INFO) << "RunAlgorithm     VideoDetResReturn : " << latency56 << " us." << std::endl;

    auto t_end = std::chrono::high_resolution_clock::now();
    auto latency_all = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_end - t_start).count();
    LOG(INFO) << "RunAlgorithm ---- End >>> All Time : " << latency_all << " us." << std::endl;
    return &m_stVideoAlgResult;
}