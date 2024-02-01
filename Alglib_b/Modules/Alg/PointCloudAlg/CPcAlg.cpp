#include "CPcAlg.h"
#include "TSelfPcAlgParam.h"
// #include "cmath"
#include <cmath>
#include "CommonFunctions.h"
#include "real_time_function.h"
#include "Log/glog/logging.h"

using namespace std;
using namespace cv;

CPcAlg::CPcAlg(const std::string& p_strExePath)
{
    m_strOutPath = p_strExePath;
    LOG(INFO)<<"m_strOutPath: "<<m_strOutPath<<endl;
}

CPcAlg::~CPcAlg()
{
    delete m_PcDetector;
    delete m_PcProcess;
    delete m_tTracker;
    delete m_PcPostProcess;
}

bool CPcAlg::InitAlgorithm(void* p_pInitParam)
{
    LOG(INFO) << "InitAlgorithm-pc-m_strOutPath=" << m_strOutPath << endl;
    // m_stSelfPcAlgParam->m_stPcAlgParam.m_vecFixTruckParam = {-70, 70, -3.9, 0, 0, -70, 70, 50000};   // TODO : 这一行报错
    m_stSelfPcAlgParam->m_stPcAlgParam.m_fTrackMaxAge = 4;
    m_stSelfPcAlgParam->m_stPcAlgParam.m_fTrackMinHits = 2;
    m_stSelfPcAlgParam->m_stPcAlgParam.m_DeltaThresh = 0;
    m_stSelfPcAlgParam->m_stPcAlgParam.m_fConeThreshold = 0.2;
    m_stSelfPcAlgParam->m_strRootPath = m_strOutPath;

    // 点云处理
    if (m_stSelfPcAlgParam->m_stPcAlgParam.m_PcRead) {       
        m_PcProcess = new pc_process(m_stSelfPcAlgParam); //用于算法库点云处理，离线使用
        LOG(INFO) << "InitAlgorithm--pc_process -> sucess";
    }
    else{
        m_PcProcess = nullptr;  //融合软件中不使用
        LOG(INFO) << "InitAlgorithm--pc_process -> failed";
    }
    
    if (m_stSelfPcAlgParam->m_stPcAlgParam.m_bPcProType)
    {
        // zqj add for pc_postprocess
        m_PcPostProcess = new CPostProcess(m_stSelfPcAlgParam, 0);
        LOG(INFO) << "--zqj-debug-alg-InitAlgorithm--CPostProcess -> sucess";
    }
    else
	{
        LOG(INFO) << "--zqj-debug-alg-InitAlgorithm--CPostProcess -> failed";
    }
    

    // 点云检测
    if (m_stSelfPcAlgParam->m_stPcAlgParam.m_PcDetectNum == 1) { // Second
        m_PcDetector = new VoxelNet(m_stSelfPcAlgParam);
        std::cout << "--zqj-debug-alg-Alglib_Init_VoxelNet_sucess" << std::endl;
        LOG(INFO) << "InitAlgorithm--VoxelNet -> sucess";
    }
    else if (m_stSelfPcAlgParam->m_stPcAlgParam.m_PcDetectNum == 2) { // Pointpillar
        m_PcDetector = new PointPillar(m_stSelfPcAlgParam);
        LOG(INFO) << "InitAlgorithm--PointPillar -> sucess";
    }
    else{
        m_PcDetector = nullptr;  // 不使用
        LOG(INFO) << "InitAlgorithm--pc_detector -> failed";
    }

    // 点云跟踪
    if (m_stSelfPcAlgParam->m_stPcAlgParam.m_PcTrackNum == 1) {        // 隧道点云跟踪算法
        m_tTracker = new Sort(m_stSelfPcAlgParam);
        LOG(INFO) << "InitAlgorithm--PcSort -> sucess";
    }
    else{ 
        m_tTracker = nullptr;  // 不使用
        LOG(INFO) << "InitAlgorithm--pc_tracker -> failed";
    }

    return true;
}

//lzq 点云预处理算法中增加一个接口参数:p_sIpSeq(传入雷达的ip) 作为算法选择bmp图标志
bool CPcAlg::Pretreatment(void* p_pPcSrcData)
{
    TPcSrcData* l_pPcSrcData = (TPcSrcData*)p_pPcSrcData;
    if(!l_pPcSrcData || l_pPcSrcData->m_vecPoints.size() == 0)
    {
        return false;
    }

    xt::xarray<float> l_arrInput = xt::zeros<float>({int (l_pPcSrcData->m_vecPoints.size()), 4});
    for(int i=0; i<l_pPcSrcData->m_vecPoints.size(); i++)
    {
        l_arrInput(i, 0) = l_pPcSrcData->m_vecPoints[i].m_fX;
        l_arrInput(i, 1) = l_pPcSrcData->m_vecPoints[i].m_fY;
        l_arrInput(i, 2) = l_pPcSrcData->m_vecPoints[i].m_fZ;
        l_arrInput(i, 3) = l_pPcSrcData->m_vecPoints[i].m_fIntensity;
    }

    int ori_point_num = l_pPcSrcData->m_vecPoints.size();
    l_pPcSrcData->m_vecPoints.clear();
    // bool is_crop = m_stSelfPcAlgParam->m_stPcAlgParam.m_bUseCropRoi;
    // bool is_filter = m_stSelfPcAlgParam->m_stPcAlgParam.m_bUseGroundPointsFilter;
    bool is_crop = m_stSelfPcAlgParam->m_stPcAlgParam.m_bCropFilter;
    // bool is_filter = m_stSelfPcAlgParam->m_stPcAlgParam.m_bCropFilter;
    bool is_filter = 0;   // zqj no filter_ground
    m_PcProcess->filter_points(l_arrInput, is_crop, is_filter);
    LOG(INFO)<<"RunAlgorithm Pre >>> Crop: "<< is_crop << " Filter: " << is_filter << ". "
                << ori_point_num << " --> " << l_arrInput.shape(0) << endl;
    //坐标旋转, 此处设计多个雷达的旋转，暂不可使用模块化接口   cc by zqj20230601 for data_preprocess
    // auto& l_refVecDev = m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev;
    // int l_nLidarIndex = m_stSelfPcAlgParam->m_nLidarIndex;
    // if(l_refVecDev.size() > l_nLidarIndex && l_refVecDev[l_nLidarIndex].m_bRotationTranslation)
    // {
    //     std::cout << "--zqj-debug-+++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    //     std::vector<float> angle = {(float)(l_refVecDev[l_nLidarIndex].m_fSelfRotTrans[0] / (float)180.0 * M_PI), 
    //                         (float)(l_refVecDev[l_nLidarIndex].m_fSelfRotTrans[1] / (float)180.0 * M_PI), (float)(l_refVecDev[l_nLidarIndex].m_fSelfRotTrans[2] / (float)180.0 * M_PI)};
    //     std::vector<float> trans = {l_refVecDev[l_nLidarIndex].m_fSelfRotTrans[0], l_refVecDev[l_nLidarIndex].m_fSelfRotTrans[1], l_refVecDev[l_nLidarIndex].m_fSelfRotTrans[2]};
    //     if(l_arrInput.shape(0) > 0)
    //     {
    //         rotate<float>(l_arrInput, angle, trans);
    //     }
    // }
    
    if(l_arrInput.shape(0) > 0)
    {
        auto& l_refPcFinalRes = l_arrInput;
        for (size_t i = 0; i < l_arrInput.shape(0); i++)
        {
            TPcPoint l_stPcPoint;
            l_stPcPoint.m_fX = l_refPcFinalRes(i, 0);
            l_stPcPoint.m_fY = l_refPcFinalRes(i, 1);
            l_stPcPoint.m_fZ = l_refPcFinalRes(i, 2);
            l_stPcPoint.m_fIntensity = l_refPcFinalRes(i, 3);
            l_pPcSrcData->m_vecPoints.push_back(l_stPcPoint);
        }
        l_pPcSrcData->m_unPointNums = l_arrInput.shape(0);
    }
    else
    {
    }

    return true;
}

void* CPcAlg::RunAlgorithm(void* p_pSrcData)
{
    auto t_start = std::chrono::high_resolution_clock::now();
    if (!p_pSrcData)
    {
        LOG(INFO)<<"RunAlgorithm ---- End >>> Failed : No PointCloud Data." << endl;
        return nullptr;
    }

    //判断是否点云处理
    if (m_PcProcess){
        auto t_p1 = std::chrono::high_resolution_clock::now();
        //获取雷达IP和端口
        // auto& l_refVecLidarDev = m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev;
        // if (l_refVecLidarDev.size() <= m_stSelfPcAlgParam->m_nLidarIndex)
        // {
        //     LOG(INFO)<<"RunAlgorithm ---- End >>> Failed : No Lidar Device." << endl;
        //     return nullptr;
        // }
        // string l_strIpAndPort = l_refVecLidarDev[m_stSelfPcAlgParam->m_nLidarIndex].m_strLidarSrcIP;
        // string l_strIpInfo = l_strIpAndPort.assign(l_strIpAndPort.begin(), l_strIpAndPort.begin()+l_strIpAndPort.find('-'));
        // string l_strIpSeq = l_strIpInfo.substr(l_strIpInfo.find_last_of(".")+1); //提取雷达ip地址的最后一位的数字,作为读取bmp图的名字
        // Pretreatment(p_pSrcData, true, "70");
        Pretreatment(p_pSrcData);
        auto t_p2 = std::chrono::high_resolution_clock::now();
        auto latency_pre = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_p2 - t_p1).count();
        LOG(INFO)<<"Pretreatment         time : " << latency_pre << " us." << endl;
    }

    //点云数据处理
    auto t_1 = std::chrono::high_resolution_clock::now();
    TPcSrcData l_stPcSrcData = *((TPcSrcData *)p_pSrcData);

    m_stPcAlgResult.m_tPcSrcAlgResult.m_vecPcRecogResult.clear();
    m_stPcAlgResult.m_tPcResult.m_vecBox.clear();

    xt::xarray<float> l_arrInput = xt::zeros<float>({int (l_stPcSrcData.m_vecPoints.size()), 4});
    for(int i=0; i<l_stPcSrcData.m_vecPoints.size(); i++){
        l_arrInput(i, 0) = l_stPcSrcData.m_vecPoints[i].m_fX;
        l_arrInput(i, 1) = l_stPcSrcData.m_vecPoints[i].m_fY;
        l_arrInput(i, 2) = l_stPcSrcData.m_vecPoints[i].m_fZ;
        l_arrInput(i, 3) = l_stPcSrcData.m_vecPoints[i].m_fIntensity;  // 提交代码中禁止修改此处数据
        // l_arrInput(i, 3) = 0.0;
    }
    auto t_2 = std::chrono::high_resolution_clock::now();
    auto latency12 = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_2 - t_1).count();
    LOG(INFO)<<"RunAlgorithm DataType Trans : " << latency12 << " us." << endl;

    //判断是否需要生成bmp图
    if (m_stSelfPcAlgParam->m_stPcAlgParam.m_bCropFilter == 0 && (m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_strBmpPath1).empty())
    {
        m_PcProcess = new pc_process(m_stSelfPcAlgParam);
        std::string save_bmp_path = m_stSelfPcAlgParam->m_strRootPath + "/Configs/Alg/CppConfigs/point_cloud/crop_images/save_new.bmp";   
        std::cout << "save_bmp_path: "<< save_bmp_path << ", bmp_size: " << 3000 <<std::endl;        
        m_PcProcess->m_save_bmp(l_arrInput, save_bmp_path, 3000);
        std::cout << " save bmp success!!! " << std::endl;
        return nullptr;  
    }

    try
    {
        //点云检测算法

        // 平移至路口中央
        // xt::view(l_arrInput, xt::all(), 0) -= m_stSelfPcAlgParam->m_stPcAlgParam.m_fRoughLanRotationX;
        // xt::view(l_arrInput, xt::all(), 1) -= m_stSelfPcAlgParam->m_stPcAlgParam.m_fRoughLanRotationY;
        // 保存点云数据
        // static int frame_num = 0;
        // static int det_num = 0;
        // xt::dump_npy("/root/data_Frame_" + std::to_string(frame_num++) + ".npy", l_arrInput);
        // xt::dump_npy("/root/l_arrInput_new.npy", l_arrInput);

        auto t_3 = std::chrono::high_resolution_clock::now();
        xt::xarray<float> PcDet_out_1 = m_PcDetector->RUN(l_arrInput);
        auto t_4 = std::chrono::high_resolution_clock::now();
        double latency34  = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_4 - t_3).count();
        LOG(INFO)<<"RunAlgorithm         Pc_Detect : " << latency34 << " us."  << "   PcDet_num:" << PcDet_out_1.shape(0) << endl;
        // 保存点云检测结果
        // xt::dump_npy("/root/res_Frame_" + std::to_string(det_num++) + ".npy", PcDet_out);
        // 再平移回来
        // xt::view(PcDet_out, xt::all(), 0) += m_stSelfPcAlgParam->m_stPcAlgParam.m_fRoughLanRotationX;
        // xt::view(PcDet_out, xt::all(), 1) += m_stSelfPcAlgParam->m_stPcAlgParam.m_fRoughLanRotationY;
        xt::xarray<float> PcDet_out;
        if(m_PcProcess){
        std::vector<int> bbindex = m_PcProcess->crop_bboxes2(PcDet_out_1);
         PcDet_out = xt::view(PcDet_out_1, xt::keep(bbindex));
        }
        else{
          PcDet_out = PcDet_out_1;
        }
        LOG(INFO)<<"RunAlgorithm         Pc_Detect2 : " << latency34 << " us."  << "   PcDet_num2:" << PcDet_out.shape(0) << endl;


        if (PcDet_out.shape(0) == 0) {
            return &m_stPcAlgResult;
        }

        //判断是否进行点云跟踪算法
        if (m_tTracker) {
            auto t_1 = std::chrono::high_resolution_clock::now();
            xt::xarray<float> tracker_out = m_tTracker->RUN(PcDet_out);
            auto t_2 = std::chrono::high_resolution_clock::now();
            auto latency = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_2 - t_1).count();
            LOG(INFO)<<"RunAlgorithm       Pc_Track : " << latency << " us." << endl;

            if(tracker_out.size() == 0)
            {
                LOG(ERROR) << "RunAlgorithm ---- End >>> Pc_Track Failed!" << endl;
            }
            std::vector<float> l_vecResult(tracker_out.size());
            std::copy(tracker_out.cbegin(), tracker_out.cend(), l_vecResult.begin());

            m_stPcAlgResult.m_tPcSrcAlgResult.m_vecPcRecogResult.insert(
                m_stPcAlgResult.m_tPcSrcAlgResult.m_vecPcRecogResult.begin(),
                l_vecResult.begin(),
                l_vecResult.end());
            m_stPcAlgResult.m_tPcSrcAlgResult.m_usResultColumns = 16;

            for (int i = 0; i < tracker_out.shape(0); ++i)
            {
                TPcBoxInfo l_stPcBoxInfo;
                l_stPcBoxInfo.m_sXCoord          = tracker_out(i, 0) * 100;			    //X轴坐标:m
                l_stPcBoxInfo.m_sYCoord          = tracker_out(i, 1) * 100;			    //Y轴坐标:m
                l_stPcBoxInfo.m_usWidth          = tracker_out(i, 2) * 100;			    // 宽度
                l_stPcBoxInfo.m_usLength         = tracker_out(i, 3) * 100;			    //长度
                l_stPcBoxInfo.m_usCourseAngle    = tracker_out(i, 4);		            // 航向角
                l_stPcBoxInfo.m_sZCoord          = tracker_out(i, 5) * 100;			    // Z轴坐标:m
                l_stPcBoxInfo.m_usHeight         = tracker_out(i, 6) * 100;	            // 高度
                // l_stPcBoxInfo.m_strClass         = m_strClass[int(tracker_out(i, 7))];  //l_dData[34*i + 7];   //目标类型(点云算法传出的类型与视频传出的类型映射关系不同,所以此处采用字符串形式统一)
                l_stPcBoxInfo.m_strClass         = m_stSelfPcAlgParam->m_stPcAlgParam.m_vecPcClass[int(tracker_out(i, 7))];
                l_stPcBoxInfo.m_usSpeed          = tracker_out(i, 8) * 100;			    //速度：m/s
                l_stPcBoxInfo.m_usSingleId       = tracker_out(i, 9);		            //单基站检测的ID
                l_stPcBoxInfo.m_ucConfidence     = int(tracker_out(i, 10) * 100);		//置信度


                double l_dBoxLongitude = 0.0, l_dBoxLatitude=0.0;
                auto& l_refVecLidarDev = m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev;
                double l_dLidarLongitude = l_refVecLidarDev.size() > 0 ? l_refVecLidarDev[0].m_dLon : 0;
                double l_dLidarLatitude = l_refVecLidarDev.size() > 0 ? l_refVecLidarDev[0].m_dLat : 0;
                float  l_fLidarAngle = l_refVecLidarDev.size() > 0 ? l_refVecLidarDev[0].m_fAngle : 0;
                XYZ_To_BLH(l_dLidarLongitude, l_dLidarLatitude, tracker_out(i, 0), tracker_out(i, 1), l_fLidarAngle, l_dBoxLongitude, l_dBoxLatitude);
                l_stPcBoxInfo.m_dLon = l_dBoxLongitude;
                l_stPcBoxInfo.m_dLat = l_dBoxLatitude;
                m_stPcAlgResult.m_tPcResult.m_vecBox.push_back(l_stPcBoxInfo);
                m_stPcAlgResult.m_tPcResult.m_ulBufTimeStamp[0] = (l_stPcSrcData.m_stSrcInfo.m_ulBufTimeStamp[0]);
            }

            m_stPcAlgResult.m_tPcResult.m_usBoxNum = tracker_out.shape(0);
            m_stPcAlgResult.m_tPcResult.m_ulBufTimeStamp[0] = l_stPcSrcData.m_stSrcInfo.m_ulBufTimeStamp[0];

            auto t_3 = std::chrono::high_resolution_clock::now();
            latency = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_3 - t_2).count();
            LOG(INFO)<<"RunAlgorithm         Output : " << latency << " us." << endl;

        }
       
        // 点云结果postprocess
        auto t_5 = std::chrono::high_resolution_clock::now();
        bool is_flag_nms = m_stSelfPcAlgParam->m_stPcAlgParam.m_bPcProType;
        if(is_flag_nms)
        {
            for (int i = 0; i < PcDet_out.shape(0); ++i) 
            {
                m_PcPostProcess->BoxSplit(PcDet_out(i, 0), PcDet_out(i, 1), PcDet_out(i, 2), PcDet_out(i, 6));
            }
            // std::cout << "--zqj-debug-alg-test_iou_BoxSplit success" <<  std::endl;
            auto iou_res = box_ops_nms::rotate_nms_cc(PcDet_out, PcDet_out);
            xt::xarray<float> iou_matrix = iou_res.first;  // 交并比
            // xt::xarray<float> iou_matrix_new = iou_res.second;  // 矩形1面积
            int row_iou = iou_matrix.shape(0);
            int col_iou = iou_matrix.shape(1);
            std::set<int> set_org;
            for(int i=0; i<row_iou-1; i++)
            {
                for (int j=i+1;j<col_iou;j++)
                {
                    if (iou_matrix(i,j)>0)
                    {
                        // std::cout << "--zqj-debug-alg-test_iou_half:" << iou_matrix(i,j) <<  std::endl;
                        set_org.insert(j);  // 只删除后一个框
                    }
                }
            }
            // std::cout << "--zqj-debug-alg-test_iou_set_org success:" << set_org.size() << "--"<< row_iou << std::endl;
            xt::xarray<float> iou_save = xt::zeros<float>({row_iou-(int)set_org.size(),(int)PcDet_out.shape(1)});
            int save_row = 0;
            for (int j = 0; j < row_iou; j++)
            {
                bool flag_save = true;
                for(auto i=set_org.begin();i!=set_org.end();i++)
                {
                    if (j == int(*i))  // 过滤条件
                    {
                        flag_save = false;
                    }
                }
                if(flag_save)
                {
                    xt::view(iou_save, save_row, xt::all()) = xt::view(PcDet_out, j, xt::all());
                    save_row++;
                }
            }
            // std::cout << "--zqj-debug-alg-test_iou_iou_save success" <<  std::endl;
            // std::cout << "--zqj-debug-alg-test_nms_iou:" << iou_matrix <<  std::endl;
            // std::cout << "--zqj-debug-alg-test_nms_iou_save:" << iou_save <<  std::endl;
            for (int i = 0; i < iou_save.shape(0); ++i)
            {
                TPcBoxInfo l_stPcBoxInfo;
                l_stPcBoxInfo.m_sXCoord  = iou_save(i, 0) * 100;			        // X轴坐标:m
                l_stPcBoxInfo.m_sYCoord  = iou_save(i, 1) * 100;			        // Y轴坐标:m
                l_stPcBoxInfo.m_sZCoord  = iou_save(i, 2) * 100;			        // Z轴坐标:m
                l_stPcBoxInfo.m_usWidth  = iou_save(i, 3) * 100;			        // 宽度
                l_stPcBoxInfo.m_usLength = iou_save(i, 4) * 100;			        // 长度
                l_stPcBoxInfo.m_usHeight = iou_save(i, 5) * 100;	                // 高度
                l_stPcBoxInfo.m_usCourseAngle = iou_save(i, 6) *(180 / M_PI)*100;   // 航向角
                l_stPcBoxInfo.m_strClass = m_stSelfPcAlgParam->m_stPcAlgParam.m_vecPcClass[int(iou_save(i, 7))]; // 类别
                l_stPcBoxInfo.m_ucConfidence = int(iou_save(i, 8) * 100);		    // 置信度
            
                double l_dBoxLongitude = 0.0, l_dBoxLatitude=0.0;
                // std::cout << "l_dLidarLongitude, l_dLidarLatitude: " << m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLon << ", " << m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLat << std::endl;
                // std::cout << "iou_save: " <<  iou_save(i, 0) << ", " << iou_save(i, 1) << std::endl;
                // std::cout << "m_fLidarAngle: " <<  m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_fAngle << std::endl;
                XYZ_To_BLH(
                    m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLon, 
                    m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLat, 
                    iou_save(i, 0), 
                    iou_save(i, 1), 
                    m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_fAngle, 
                    l_dBoxLongitude, 
                    l_dBoxLatitude);
                // std::cout << "l_dBoxLongitude, l_dBoxLatitude: " <<  l_dBoxLongitude << ", " << l_dBoxLatitude << std::endl;

                l_stPcBoxInfo.m_dLon = l_dBoxLongitude;
                l_stPcBoxInfo.m_dLat = l_dBoxLatitude;
                m_stPcAlgResult.m_tPcResult.m_vecBox.push_back(l_stPcBoxInfo);
            }
            // std::cout << "--zqj-debug-alg-test_iou_process success" <<  std::endl;
            m_stPcAlgResult.m_tPcResult.m_usBoxNum = iou_save.shape(0);
        }
        
        else
        {
            for (int i = 0; i < PcDet_out.shape(0); ++i)
            {
                // std::cout<< "--zqj-debug-alg-pcdet_xyz_11:" << PcDet_out(i, 0)<<"--"<< PcDet_out(i, 1)<<"--"<< PcDet_out(i, 2) << std::endl;
                // if (m_stSelfPcAlgParam->m_stPcAlgParam.m_bPcProType)
                // {
                //     m_PcPostProcess->BoxSplit(PcDet_out(i, 0), PcDet_out(i, 1), PcDet_out(i, 2), PcDet_out(i, 6));
                // }
                // std::cout<< "--zqj-debug-alg-pcdet_xyz_22:" << PcDet_out(i, 0)<<"--"<< PcDet_out(i, 1)<<"--"<< PcDet_out(i, 2) << std::endl;
                TPcBoxInfo l_stPcBoxInfo;
                l_stPcBoxInfo.m_sXCoord  = PcDet_out(i, 0) * 100;			        // X轴坐标:m
                l_stPcBoxInfo.m_sYCoord  = PcDet_out(i, 1) * 100;			        // Y轴坐标:m
                l_stPcBoxInfo.m_sZCoord  = PcDet_out(i, 2) * 100;			        // Z轴坐标:m
                l_stPcBoxInfo.m_usWidth  = PcDet_out(i, 3) * 100;			        // 宽度
                l_stPcBoxInfo.m_usLength = PcDet_out(i, 4) * 100;			        // 长度
                l_stPcBoxInfo.m_usHeight = PcDet_out(i, 5) * 100;	                // 高度
                l_stPcBoxInfo.m_usCourseAngle = PcDet_out(i, 6) *(180 / M_PI);		// 航向角
                l_stPcBoxInfo.m_strClass = m_stSelfPcAlgParam->m_stPcAlgParam.m_vecPcClass[int(PcDet_out(i, 7))]; // 类别
                l_stPcBoxInfo.m_ucConfidence = int(PcDet_out(i, 8) * 100);		    // 置信度
            
                double l_dBoxLongitude = 0.0, l_dBoxLatitude=0.0;
                // std::cout << "l_dLidarLongitude, l_dLidarLatitude: " << m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLon << ", " << m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLat << std::endl;
                // std::cout << "PcDet_out: " <<  PcDet_out(i, 0) << ", " << PcDet_out(i, 1) << std::endl;
                // std::cout << "m_fLidarAngle: " <<  m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_fAngle << std::endl;
                XYZ_To_BLH(
                    m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLon, 
                    m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLat, 
                    PcDet_out(i, 0), 
                    PcDet_out(i, 1), 
                    m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_fAngle, 
                    l_dBoxLongitude, 
                    l_dBoxLatitude);
                // std::cout << "l_dBoxLongitude, l_dBoxLatitude: " <<  l_dBoxLongitude << ", " << l_dBoxLatitude << std::endl;

                l_stPcBoxInfo.m_dLon = l_dBoxLongitude;
                l_stPcBoxInfo.m_dLat = l_dBoxLatitude;
                m_stPcAlgResult.m_tPcResult.m_vecBox.push_back(l_stPcBoxInfo);
            }
            m_stPcAlgResult.m_tPcResult.m_usBoxNum = PcDet_out.shape(0);
        }
        m_stPcAlgResult.m_tPcResult.m_dLidarLon = m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLon;
        m_stPcAlgResult.m_tPcResult.m_dLidarLat = m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_dLat;
        m_stPcAlgResult.m_tPcResult.m_fLidarNorthAngle = m_stSelfPcAlgParam->m_stLidarParam.m_vecLidarDev[0].m_fAngle;
        

        auto t_6 = std::chrono::high_resolution_clock::now();
        auto latency56 = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_6 - t_5).count();
        LOG(INFO)<<"RunAlgorithm         Track : " << latency56 << " us." << endl;
    }
    catch(const std::exception& e)
    {
        LOG(ERROR) << e.what() << '\n';
        return NULL;
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    auto latency_all = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_end - t_start).count();

    LOG(INFO)<<"RunAlgorithm ---- End >>> All Time : " << latency_all << " us." << endl;
    return &m_stPcAlgResult;    
}

bool CPcAlg::SetAlgParam(const TSelfPcAlgParam* p_pAlgParam)
{
    if (p_pAlgParam)
    {
        m_stSelfPcAlgParam = const_cast<TSelfPcAlgParam*>(p_pAlgParam);
    }
    else
    {
        LOG(ERROR)<< "The incoming parameter is empty"<<endl;
        return false;
    }
    return true;
}

