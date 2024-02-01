#include "CEventAlg.h"
// #include "CommonDef.h"
// #include "CCameraParam.h"
#include "traffic_flow_matrix.h"
#include "CommonFunctions.h"
#include "CGeneralTrans.h"
#include <json.hpp>
#include <cstdint>
using namespace std;
#define OFFLINE

CEventAlg::CEventAlg(const std::string &p_strExePath):m_strOutPath(p_strExePath)
{
    LOG(INFO) << "CEventAlgorithm::CEventAlgorithm";
    // LoadAlgEnv("LibTrafficEventAlg", "TrafficEventAlg");
    LOG(INFO) << "m_strOutPath: " << m_strOutPath << std::endl;
    m_traffic_flow = nullptr;
}

CEventAlg::~CEventAlg()
{
    delete m_traffic_flow;
}

bool CEventAlg::InitAlgorithm(void* p_pInitParam)
{
    if (!m_traffic_flow)
    {
//        std::string lane_config_path = "/data/isfp_radar/Modules/Handlers/AlgHandler/SuiDaoAlg/EventAlg/virtual_config";
        std::string lane_config_path = m_strOutPath + "/Configs/Alg/CppConfigs/event";
        std::cout << lane_config_path << std::endl;
        m_traffic_flow = new Traffic_Flow(lane_config_path, 1);
    }
    return true;
}

//xuyao cunfang ge peizhi wenjian
bool CEventAlg::SetAlgParam(const TSelfEventAlgParam* p_pAlgParam)
{
    if (p_pAlgParam)
    {
        m_stSelfEventAlgParam = const_cast<TSelfEventAlgParam*>(p_pAlgParam);
        // CCameraParam l_cAlgParam = *((CCameraParam *)p_pConfigParam);
        // auto l_nDataLen = l_cAlgParam.vecCameraDev().size() * 20; // 20 = 9 +3 +3 +5
        auto l_nDataLen = m_stSelfEventAlgParam->m_stCameraParam.m_vecCameraDev.size() * 20;
        auto *l_pAlgParamData = new float[l_nDataLen];
        int l_nIndex = 0;
        for (int i = 0; i < m_stSelfEventAlgParam->m_stCameraParam.m_unCameraCount; i++)
        {
            for (float j : m_stSelfEventAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecInParameter)
            {
                l_pAlgParamData[l_nIndex++] = j;
            }
            for (float j : m_stSelfEventAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecRotateMatrix)
            {
                l_pAlgParamData[l_nIndex++] = j;
            }
            for (float j : m_stSelfEventAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecTranslationMatrix)
            {
                l_pAlgParamData[l_nIndex++] = j;
            }
            for (float j : m_stSelfEventAlgParam->m_stCameraParam.m_vecCameraDev[i].m_vecDistMatrix)
            {
                l_pAlgParamData[l_nIndex++] = j;
            }
        }
    }
    else
    {
        LOG(ERROR) << "The incoming parameter is empty" << endl;
        return false;
    }

    // PyObject *pResult = PyObject_CallMethod(pInstance2, "setConfigParam", "Os", m_pAlgParamMap, m_strOutPath.data());
    // if (pResult == NULL)
    // {
    //     LOG(ERROR) << "Failed to use setConfigParam!" << endl;
    //     return false;
    // }

    return true;
}

void* CEventAlg::RunAlgorithm(void* p_pSrcData, void* p_nRadarId)
{
    #ifdef OFFLINE
    // TEventB3Data* pEventB3Data = (TEventB3Data*)pB3Data;
    // TEventB4Data* pEventB4Data = (TEventB4Data*)pB4Data;
    // TEventB5Data* pEventB5Data = &pEventB5Result;
    auto lidar_info = *(xt::xarray<double>*)p_pSrcData;
    int stationId = *((int *)p_nRadarId);
    if (lidar_info.size() > 0)
    {
        long B5_time = ops::getTimeStamp();
        bool B3_flag = false;
        bool B4_flag = false;
        bool B5_flag = false;

        std::string path = m_strOutPath + "/Configs/Alg/CppConfigs/event/use_time_traffic";
        if (access(path.c_str(), 0) == -1) {
            mkdir(path.c_str(), S_IRWXU);
        }
        path += "/event_result.txt";
        if (access(path.c_str(), 0) == 0) {
            if (std::remove(path.c_str()) == 0) {
                std::cout << ">>> Delete use_time_traffic.txt. --> Sucess.\n";
            } else {
                std::cout << ">>> Delete use_time_traffic.txt. --> Failed.\n";
            }
        }
        // std::ofstream event_traffic_fd;
        // event_traffic_fd.open(path, std::ios::out | std::ios::app);

        static bool firstFrame = false;
        double t_B4_now = lidar_info(0, 44);
        double t_B3_now = lidar_info(0, 44);

        static double t_B4, t_B3; // t_B3, t_B3_now why?
        if (!firstFrame) {
            t_B4 = lidar_info(0, 44);
            t_B3 = lidar_info(0, 44);
        }
        firstFrame = true;
        long t_now = ops::getTimeStamp();

        if (t_B3_now - t_B3 >= 1 * 1000) {
            B3_flag = true;
            t_B3 = t_B3_now;
        } else {
            B3_flag = false;
        }
        if (t_B4_now - t_B4 >= 60 * 1000) {
            B4_flag = true;
            t_B4 = t_B4_now;
        } else {
            B4_flag = false;
        }
        if (t_now - B5_time >= 5 * 1000) {
            B5_flag = true;
            B5_time = t_now;
        } else {
            B5_flag = false;
        }

        xt::xarray<double> temp = xt::empty<double>({0, 0});
        if (B4_flag) {
            m_traffic_flow->use(lidar_info, true, temp, stationId);
        } else {
            m_traffic_flow->use(lidar_info, false, temp, stationId);
        }

        // cjm 0918
        // get_B3:report B3 result
        std::shared_ptr<Event_Output> event_res;
        if (B3_flag) {
            std::vector<std::vector<double>> b3_output; // yk
            int b3_cnt;
            int lane_num;
            m_traffic_flow->get_B3(event_res);
            b3_output = event_res->out_content; // yk
            lane_num = event_res->out_normal_lane_num;


            b3_cnt = event_res->out_B3_cnt; 

             //将B3结果数据转换成对应的DDS数据体
            pEventB3Data.m_usDeviceId = 0;                     //设备ID
            pEventB3Data.m_ulTimeStamp = 0;                   //时间戳
            pEventB3Data.m_ulB3FrameId = b3_cnt;             //帧号
            pEventB3Data.m_ucLaneNum = lane_num;
            pEventB3Data.m_ucCrossingNum = 0;
            pEventB3Data.m_vecCrossingEvent.resize(1);
            pEventB3Data.m_vecLaneEvent.resize(b3_output.size());
            for (int i = 0; i < b3_output.size(); i++)
            {   
                pEventB3Data.m_vecLaneEvent[i].m_ucLaneNo = b3_output[i][0];
                pEventB3Data.m_vecLaneEvent[i].m_ucLaneType = b3_output[i][1];
                pEventB3Data.m_vecLaneEvent[i].m_usLaneDirection = b3_output[i][2];
                pEventB3Data.m_vecLaneEvent[i].m_usLaneSpeedLimit = b3_output[i][3];
                pEventB3Data.m_vecLaneEvent[i].m_usLaneSpeedAvg = b3_output[i][4];
                pEventB3Data.m_vecLaneEvent[i].m_uiQueueLength = b3_output[i][5];
                pEventB3Data.m_vecLaneEvent[i].m_ucQueueState= b3_output[i][6];
                pEventB3Data.m_vecLaneEvent[i].m_usLaneHeadSpace = b3_output[i][7];
                pEventB3Data.m_vecLaneEvent[i].m_ucSpaceOccupancy = b3_output[i][8];
                pEventB3Data.m_vecLaneEvent[i].m_usVehicleNum = b3_output[i][9];
                pEventB3Data.m_vecLaneEvent[i].m_usNonVehicleNum = b3_output[i][10];
                pEventB3Data.m_vecLaneEvent[i].m_bTakeMap = false;
                pEventB3Data.m_vecLaneEvent[i].m_uiWaitingTime = b3_output[i][12];
                pEventB3Data.m_vecLaneEvent[i].m_usBaseLocation = b3_output[i][11];
            }


        }

        if (B4_flag) {
            m_traffic_flow->get_B4(event_res);
            // std::vector<std::vector<double>> b4_output;  // cjm 0927
            std::vector<std::shared_ptr<B4_context_info>> b4_output;  
            int b4_cnt;                   
            b4_output = event_res->out_context; 
            b4_cnt = event_res->out_B4_cnt;                                                             // yk

            //////////////////////////modify by szh ////////////////////////////////
            //将B4结果数据转换成对应的DDS数据体
            pEventB4Data.m_usDeviceId = 0;    //设备ID
            pEventB4Data.m_ulstartTimeStamp = 0;   //开始时间戳
            pEventB4Data.m_ulendTimeStamp = 0;     //结束时间戳
            pEventB4Data.m_ulB4FrameId = b4_cnt;           //帧号
            pEventB4Data.m_ucLaneNum = b4_output.size(); //车道数量

            // std::vector<TEventB4LaneInfo> m_vecEvent;

            pEventB4Data.m_vecLaneEvent.resize(b4_output.size());
            for (int i = 0; i < b4_output.size(); i++)
            {
                // for (int j = 0; i < b4_output[i][6].size(); j++)
                // {

                // }
                // TEventB4LaneInfo l_eventB4LaneInfo;
                pEventB4Data.m_vecLaneEvent[i].m_ucLaneNo = b4_output[i]->item0;

                pEventB4Data.m_vecLaneEvent[i].m_ucLaneType = b4_output[i]->item1;

                pEventB4Data.m_vecLaneEvent[i].m_usLaneDirection = b4_output[i]->item2;

                pEventB4Data.m_vecLaneEvent[i].m_usLaneSpeedLimit = b4_output[i]->item3;

                pEventB4Data.m_vecLaneEvent[i].m_ucFollowPercent = b4_output[i]->item7;
                pEventB4Data.m_vecLaneEvent[i].m_ucTimeOccupancy = b4_output[i]->item8;
                pEventB4Data.m_vecLaneEvent[i].m_usHeadSpaceAvg = b4_output[i]->item9;

                TEventB4VehicheInfo m_tB4CarInfo;
                m_tB4CarInfo.m_usFlow = b4_output[i]->item10;
                m_tB4CarInfo.m_usSpeedAvg = b4_output[i]->item15;
                pEventB4Data.m_vecLaneEvent[i].m_tCarInfo = m_tB4CarInfo;

                TEventB4VehicheInfo m_tB4LargeTruckInfo;
                m_tB4LargeTruckInfo.m_usFlow = b4_output[i]->item11;
                m_tB4LargeTruckInfo.m_usSpeedAvg = b4_output[i]->item16;
                pEventB4Data.m_vecLaneEvent[i].m_tLargeTruckInfo = m_tB4LargeTruckInfo;

                TEventB4VehicheInfo m_tB4BusInfo;
                m_tB4BusInfo.m_usFlow = b4_output[i]->item12;
                m_tB4BusInfo.m_usSpeedAvg = b4_output[i]->item17;
                pEventB4Data.m_vecLaneEvent[i].m_tBusInfo = m_tB4BusInfo;

                TEventB4VehicheInfo m_tB4MiniBusInfo;
                m_tB4MiniBusInfo.m_usFlow = b4_output[i]->item13;
                m_tB4MiniBusInfo.m_usSpeedAvg = b4_output[i]->item18;
                pEventB4Data.m_vecLaneEvent[i].m_tMiniBusInfo = m_tB4MiniBusInfo;

                TEventB4VehicheInfo m_tB4TruckInfo;
                m_tB4TruckInfo.m_usFlow = b4_output[i]->item20;
                m_tB4TruckInfo.m_usSpeedAvg = b4_output[i]->item22;
                pEventB4Data.m_vecLaneEvent[i].m_tTruckInfo = m_tB4TruckInfo;

                TEventB4VehicheInfo m_tB4LittleTruckInfo;
                m_tB4LittleTruckInfo.m_usFlow = b4_output[i]->item21;
                m_tB4LittleTruckInfo.m_usSpeedAvg = b4_output[i]->item23;
                pEventB4Data.m_vecLaneEvent[i].m_tLittleTruckInfo = m_tB4LittleTruckInfo;

                TEventB4VehicheInfo m_tB4AmbulanceInfo;
                m_tB4AmbulanceInfo.m_usFlow = 0;
                m_tB4AmbulanceInfo.m_usSpeedAvg = 0;
                pEventB4Data.m_vecLaneEvent[i].m_tAmbulanceInfo = m_tB4AmbulanceInfo;

                TEventB4VehicheInfo m_tB4FireengineInfo;
                m_tB4FireengineInfo.m_usFlow = 0;
                m_tB4FireengineInfo.m_usSpeedAvg = 0;
                pEventB4Data.m_vecLaneEvent[i].m_tFireengineInfo = m_tB4FireengineInfo;


                // m_vecEvent.push_back(l_eventB4LaneInfo);
            }

            // GetB4AlgResult(b4_output, stationId, pEventB4Data, b4_cnt);
            std::cout << "-----------" << stationId << "-b4--------------------------------------" << std::endl;

// //        for (int j = 0; j < b4_output.size(); ++j) {
// //            for (int k = 0; k < b4_output[j].size(); ++k) {
// //                std::cout << b4_output[j][k] << ",";
// //            }
// //            std::cout << "." << std::endl;
// //        }
        }

        // get_B5:report B5 result
        std::shared_ptr<Event_Output> b5 = m_traffic_flow->get_B5();
        std::vector<std::shared_ptr<B5_event_info>> event_result; // save all event in the variable
        event_result = b5->out_all_event_output_for_B5;
        int event_num = b5->out_input4;
        std::vector<std::vector<double>> b5_output;

        std::cout << "stationId: "<< stationId << ", PublishEventB5Data" << event_num << ", " << event_result.size() << std::endl;
        /* muqian zaiyong
        if (event_num > 0) {
            std::cout << "---------------"<<stationId<<"-b5--------------------------------------" << std::endl;
            for (int j = 0; j < event_num; ++j) {
                std::vector<double> one_event_info;
                std::cout.precision(8);                                                                                                                                                                                                                                                                                                                                   // yk:save the 小数位数
                std::cout << event_result[j]->item0 << "," << event_result[j]->item1 << "," << event_result[j]->item2 << ","
                            << event_result[j]->item6 << "," << event_result[j]->item7 << "," << event_result[j]->item9[0][0] << ","
                            << event_result[j]->item9[0][1]<< "," << event_result[j]->item9[0][2]<< "," << event_result[j]->item9[0][3]<< ","
                            << event_result[j]->item10 <<"," << event_result[j]->item11 <<"," << event_result[j]->item12 << ","
                            << event_result[j]->item13<< std::endl; // print out_all_event_output_for_B5 event
                std::shared_ptr<B5_event_info> result = event_result[j];
                // double start_time = (result->item11) * 1000 + (result->item12);
                // double id = result->item8[0][0];
                // one_event_info.push_back(result->item0);
                // one_event_info.push_back(result->item1);
                // one_event_info.push_back(3);
                // one_event_info.push_back(3);
                // one_event_info.push_back(result->item6);
                // one_event_info.push_back(result->item9);
                // one_event_info.push_back(result->item10);
                // one_event_info.push_back(start_time);
                // one_event_info.push_back(result->item15);
                // one_event_info.push_back(id);
                // one_event_info.push_back(10);
                //mu qian zaiyong
                double start_time = result->item12;//start_time
                double id = result->item9[0][0]; //id
                one_event_info.push_back(result->item0); //event_no
                one_event_info.push_back(result->item1); //abnormal_class_index
                one_event_info.push_back(3);
                one_event_info.push_back(3);
                one_event_info.push_back(result->item6); //lane
                one_event_info.push_back(result->item10);//jingdu
                one_event_info.push_back(result->item11);//weidu
                one_event_info.push_back(start_time);
                one_event_info.push_back(result->item13);//end_time
                one_event_info.push_back(id);
                one_event_info.push_back(10);
                b5_output.push_back(one_event_info);
            }
            // pEventB5Data->vecB5EventInfo = m_vecEvent;
            std::cout<<"xcb------get b5 result------"<<std::endl;


            GetB5AlgResult(b5_output, stationId, m_pEventB5Result);
            // ofstream out(
            //         m_strOutPath + "/Configs/Alg/CppConfigs/event/use_time_traffic/event_result" + to_string(stationId) + ".txt",
            //         ios::out | ios::app);
            // for (int j = 0; j < event_result.size(); ++j) {
            //     event_traffic_fd << "ID:" << event_result[j]->item8[0][0] << std::endl;
            //     event_traffic_fd << event_result[j]->item0 << "," << event_result[j]->item1 << "," << event_result[j]->item2
            //         << "," << event_result[j]->item3 << "," << event_result[j]->item4 << ","
            //         << event_result[j]->item5 << "," << event_result[j]->item6 << "," << event_result[j]->item7
            //         << "," << event_result[j]->item8[0][0] << "," << event_result[j]->item9 << ","
            //         << event_result[j]->item10 << "," << event_result[j]->item11 << ","
            //         << event_result[j]->item12 << "," << event_result[j]->item13 << ","
            //         << event_result[j]->item14 << "," << event_result[j]->item15 << std::endl;
            // }
            // event_traffic_fd.close();
            return &m_pEventB5Result;*/
        if (event_num > 0)
        {
            GetB5AlgResult(event_result, stationId, m_pEventB5Result);

            pEventData.m_B3Data = pEventB3Data;
            pEventData.m_B4Data = pEventB4Data;
            pEventData.m_B5Data = m_pEventB5Result;

            return &pEventData;
        }
        pEventData.m_B3Data = pEventB3Data;
        pEventData.m_B4Data = pEventB4Data;
        pEventData.m_B5Data = m_pEventB5Result;
        return &pEventData;
    }
    else
    {
        pEventData.m_B3Data = pEventB3Data;
        pEventData.m_B4Data = pEventB4Data;
        pEventData.m_B5Data = m_pEventB5Result;
        return &pEventData;
    }
    

    #endif
}

bool CEventAlg::GetB5AlgResult(std::vector<std::shared_ptr<B5_event_info>> &p_refB5Data, int stationId, TB5Data& eventB5Data)
{
    eventB5Data.m_ulTimeStamp = 0;
    eventB5Data.m_usDeviceId = stationId;
    eventB5Data.m_ucEventNum = p_refB5Data.size();
    LOG(INFO)<<">>> ----------alg event num:"<<std::to_string(eventB5Data.m_ucEventNum)<<endl;
    int event_num = 0;
    for (auto b5_event : p_refB5Data) // each event
    {
        int i = 0;
        TTargetMsg l_targrtInfo;
        l_targrtInfo.m_uiTargetId = b5_event->item9[0][0];  
        l_targrtInfo.m_uiTargetType = b5_event->item9[0][1];
        l_targrtInfo.m_ucEventConfidence = b5_event->item9[0][2];
        l_targrtInfo.m_ucEventInfoNum = b5_event->item9[0][3];
        l_targrtInfo.m_length = b5_event->item4;
        l_targrtInfo.m_width = b5_event->item3;
        l_targrtInfo.m_height = b5_event->item5;
        for(auto event_info : b5_event->eventInfoList)
        {
            TEventInfo l_eventInfo;
            l_eventInfo.m_ucEventInfoType = event_info[0];
            l_eventInfo.m_fdLongitude = event_info[1];
            l_eventInfo.m_fdLatitude = event_info[2];
            l_eventInfo.m_usLeftTopX = event_info[3];
            l_eventInfo.m_usLeftTopY = event_info[4] ;
            l_eventInfo.m_usRightDownX = event_info[5];
            l_eventInfo.m_usRightDownY = event_info[6];
            l_eventInfo.m_ucStationId = event_info[7];
            l_eventInfo.m_ucCameraId = event_info[8];
            // l_eventInfo.m_ucCameraNum = event_info[1];
            l_eventInfo.m_ulHeadCamTimeStamp = event_info[9];
            l_eventInfo.m_ulBodyCamTimeStamp = event_info[10];
            l_eventInfo.m_ulTailCamTimeStamp = event_info[11];
            l_eventInfo.m_uiGlobalFrameNo = event_info[12];
            l_eventInfo.m_ulGlobalTimeStamp = event_info[13];

            l_targrtInfo.m_vecEventInfo[i] = l_eventInfo;
            i++;
        }
        TB5LaneEvent l_b5LaneEvent;
        // TEventB5Pos l_evenB5Pos;
        // l_evenB5Pos.dLatitude = b5_event->item10;
        // l_evenB5Pos.dLongitude = b5_event->item11;

        l_b5LaneEvent.m_ulB5EventId = b5_event->item0;
        // std::cout<<"cjm----> m_ulB5EventId:"<<b5_event->item0<<std::endl;
        l_b5LaneEvent.m_usBaseLocation = b5_event->item7;
        l_b5LaneEvent.m_ucB5EventType = b5_event->item1;
        LOG(INFO)<<"targetid: "<<std::to_string(l_targrtInfo.m_uiTargetId)
        <<"eventid: "<<std::to_string(l_b5LaneEvent.m_ulB5EventId)
        <<"eventTYPE: "<<std::to_string(l_b5LaneEvent.m_ucB5EventType)<<endl;
        l_b5LaneEvent.m_ucStationId = b5_event->item2;
        l_b5LaneEvent.m_ucLaneNo = b5_event->item6;
        l_b5LaneEvent.m_ucTargetNum = b5_event->item8;
        l_b5LaneEvent.m_vecTargetInfo[0] = l_targrtInfo;
        l_b5LaneEvent.m_fdEventLongitude = b5_event->item10;
        l_b5LaneEvent.m_fdEventLatitude = b5_event->item11;
        // l_b5LaneEvent.m_fdEventAltitude;
        l_b5LaneEvent.m_ulStartTimeStamp = b5_event->item12;
        l_b5LaneEvent.m_ulEndTimeStamp = b5_event->item13;
        l_b5LaneEvent.m_strSource="null";
        l_b5LaneEvent.m_ucAction=0;
        l_b5LaneEvent.m_usRadius=0;
        l_b5LaneEvent.m_strSensor="hello";
        // l_b5LaneEvent.vecPath.push_back(l_evenB5Pos);
        // l_b5LaneEvent.m_strImage;

        eventB5Data.m_vecB5LaneEvent[event_num] = l_b5LaneEvent;
        ++event_num;
        if (event_num >= 32){
            break;
        }
    }
    return true;
}

// xcb note CEventAlgorithm::GetB3AlgResult
bool CEventAlg::GetB3AlgResult(std::vector<std::vector<double>> &p_refB3Data, int stationId, TEventB3Data& eventB3Data, int b3_cnt)
{
    //将B3结果数据转换成对应的DDS数据体
    eventB3Data.m_usDeviceId = 0;                     //设备ID
    eventB3Data.m_ulTimeStamp = 0;                   //时间戳
    eventB3Data.m_ulB3FrameId = b3_cnt;             //帧号
    eventB3Data.m_ucLaneNum = p_refB3Data.size();
    eventB3Data.m_ucCrossingNum = 0;
    // std::vector<TEventB3LaneInfo> m_vecCrossingEvent;
    eventB3Data.m_vecCrossingEvent.resize(1);
    // eventB3Data.m_vecCrossingEvent = m_vecCrossingEvent;

    // std::vector<TEventB3LaneInfo> m_vecEvent;
    eventB3Data.m_vecLaneEvent.resize(p_refB3Data.size());
    // m_vecEvent.reserve(p_refB3Data.size());
    for (int i = 0; i < p_refB3Data.size()-1; i++)
    {
        // TEventB3LaneInfo l_eventB3LaneInfo;
        // std::cout<<"szh-------> GetB3AlgResult 0000000000 size"<<p_refB3Data[i].size()<<std::endl;
        // std::cout<<"szh-------> GetB3AlgResult 111111111 "<<p_refB3Data[i][0]<<std::endl;
        eventB3Data.m_vecLaneEvent[i].m_ucLaneNo = p_refB3Data[i][0];
        // std::cout<<"szh-------> GetB3AlgResult 2222222222 "<<p_refB3Data[i][0]<<std::endl;
        eventB3Data.m_vecLaneEvent[i].m_ucLaneType = p_refB3Data[i][1];
        eventB3Data.m_vecLaneEvent[i].m_usLaneDirection = p_refB3Data[i][2];
        eventB3Data.m_vecLaneEvent[i].m_usLaneSpeedLimit = p_refB3Data[i][3];
        eventB3Data.m_vecLaneEvent[i].m_usLaneSpeedAvg = p_refB3Data[i][4];
        eventB3Data.m_vecLaneEvent[i].m_uiQueueLength = p_refB3Data[i][5];
        eventB3Data.m_vecLaneEvent[i].m_ucQueueState= p_refB3Data[i][6];
        eventB3Data.m_vecLaneEvent[i].m_usLaneHeadSpace = p_refB3Data[i][7];
        eventB3Data.m_vecLaneEvent[i].m_ucSpaceOccupancy = p_refB3Data[i][8];
        eventB3Data.m_vecLaneEvent[i].m_usVehicleNum = p_refB3Data[i][9];
        eventB3Data.m_vecLaneEvent[i].m_usNonVehicleNum = p_refB3Data[i][10];

        eventB3Data.m_vecLaneEvent[i].m_bTakeMap = false;
        eventB3Data.m_vecLaneEvent[i].m_usNonVehicleNum = p_refB3Data[i][12];
        // m_vecEvent[i]=l_eventB3LaneInfo;
        // m_vecEvent.push_back(l_eventB3LaneInfo);
    }
    // eventB3Data.m_vecLaneEvent.resize(m_vecEvent.size());
    // eventB3Data.m_vecLaneEvent = m_vecEvent;
}

// xcb note CEventAlgorithm::GetB4AlgResult cjm 0927
bool CEventAlg::GetB4AlgResult(std::vector<std::shared_ptr<B4_context_info>> &p_refB4Data, int stationId, TEventB4Data& eventB4Data, int b4_cnt)
{
    //将B4结果数据转换成对应的DDS数据体

    eventB4Data.m_usDeviceId = 0;    //设备ID
    eventB4Data.m_ulstartTimeStamp = 0;   //开始时间戳
    eventB4Data.m_ulendTimeStamp = 0;     //结束时间戳
    eventB4Data.m_ulB4FrameId = b4_cnt;           //帧号
    eventB4Data.m_ucLaneNum = p_refB4Data.size(); //车道数量

    // std::vector<TEventB4LaneInfo> m_vecEvent;

    eventB4Data.m_vecLaneEvent.resize(p_refB4Data.size());
    for (int i = 0; i < p_refB4Data.size(); i++)
    {
        // for (int j = 0; i < p_refB4Data[i][6].size(); j++)
        // {

        // }
        // TEventB4LaneInfo l_eventB4LaneInfo;
        eventB4Data.m_vecLaneEvent[i].m_ucLaneNo = p_refB4Data[i]->item0;

        eventB4Data.m_vecLaneEvent[i].m_ucLaneType = p_refB4Data[i]->item1;

        eventB4Data.m_vecLaneEvent[i].m_usLaneDirection = p_refB4Data[i]->item2;

        eventB4Data.m_vecLaneEvent[i].m_usLaneSpeedLimit = p_refB4Data[i]->item3;

        eventB4Data.m_vecLaneEvent[i].m_ucFollowPercent = p_refB4Data[i]->item7;
        eventB4Data.m_vecLaneEvent[i].m_ucTimeOccupancy = p_refB4Data[i]->item8;
        eventB4Data.m_vecLaneEvent[i].m_usHeadSpaceAvg = p_refB4Data[i]->item9;

        TEventB4VehicheInfo m_tB4CarInfo;
        m_tB4CarInfo.m_usFlow = p_refB4Data[i]->item10;
        m_tB4CarInfo.m_usSpeedAvg = p_refB4Data[i]->item15;
        eventB4Data.m_vecLaneEvent[i].m_tCarInfo = m_tB4CarInfo;

        TEventB4VehicheInfo m_tB4LargeTruckInfo;
        m_tB4LargeTruckInfo.m_usFlow = p_refB4Data[i]->item11;
        m_tB4LargeTruckInfo.m_usSpeedAvg = p_refB4Data[i]->item16;
        eventB4Data.m_vecLaneEvent[i].m_tLargeTruckInfo = m_tB4LargeTruckInfo;

        TEventB4VehicheInfo m_tB4BusInfo;
        m_tB4BusInfo.m_usFlow = p_refB4Data[i]->item12;
        m_tB4BusInfo.m_usSpeedAvg = p_refB4Data[i]->item17;
        eventB4Data.m_vecLaneEvent[i].m_tBusInfo = m_tB4BusInfo;

        TEventB4VehicheInfo m_tB4MiniBusInfo;
        m_tB4MiniBusInfo.m_usFlow = p_refB4Data[i]->item13;
        m_tB4MiniBusInfo.m_usSpeedAvg = p_refB4Data[i]->item18;
        eventB4Data.m_vecLaneEvent[i].m_tMiniBusInfo = m_tB4MiniBusInfo;

        TEventB4VehicheInfo m_tB4TruckInfo;
        m_tB4TruckInfo.m_usFlow = p_refB4Data[i]->item20;
        m_tB4TruckInfo.m_usSpeedAvg = p_refB4Data[i]->item22;
        eventB4Data.m_vecLaneEvent[i].m_tTruckInfo = m_tB4TruckInfo;

        TEventB4VehicheInfo m_tB4LittleTruckInfo;
        m_tB4LittleTruckInfo.m_usFlow = p_refB4Data[i]->item21;
        m_tB4LittleTruckInfo.m_usSpeedAvg = p_refB4Data[i]->item23;
        eventB4Data.m_vecLaneEvent[i].m_tLittleTruckInfo = m_tB4LittleTruckInfo;

        TEventB4VehicheInfo m_tB4AmbulanceInfo;
        m_tB4AmbulanceInfo.m_usFlow = 0;
        m_tB4AmbulanceInfo.m_usSpeedAvg = 0;
        eventB4Data.m_vecLaneEvent[i].m_tAmbulanceInfo = m_tB4AmbulanceInfo;

        TEventB4VehicheInfo m_tB4FireengineInfo;
        m_tB4FireengineInfo.m_usFlow = 0;
        m_tB4FireengineInfo.m_usSpeedAvg = 0;
        eventB4Data.m_vecLaneEvent[i].m_tFireengineInfo = m_tB4FireengineInfo;


        // m_vecEvent.push_back(l_eventB4LaneInfo);
    }
    // eventB4Data.m_vecLaneEvent.resize(m_vecEvent.size());
    // eventB4Data.m_vecLaneEvent = m_vecEvent;
}

/*
// xcb note CEventAlgorithm::GetB4AlgResult 
// bool CEventAlg::GetB4AlgResult(std::vector<std::vector<double>> &p_refB4Data, int stationId, TEventB4Data& eventB4Data, int b4_cnt) cjm 0927
// {
//     //将B4结果数据转换成对应的DDS数据体
//     eventB4Data.m_usDeviceId = 0;    //设备ID
//     eventB4Data.m_ulstartTimeStamp = 0;   //开始时间戳
//     eventB4Data.m_ulendTimeStamp = 0;     //结束时间戳
//     eventB4Data.m_ulB4FrameId = b4_cnt;           //帧号
//     eventB4Data.m_ucLaneNum = p_refB4Data.size(); //车道数量

//     std::vector<TEventB4LaneInfo> m_vecEvent;
//     for (int i = 0; i < p_refB4Data.size(); i++)
//     {
//         // for (int j = 0; i < p_refB4Data[i][6].size(); j++)
//         // {

//         // }
//         TEventB4LaneInfo l_eventB4LaneInfo;

//         l_eventB4LaneInfo.m_ucLaneNo = p_refB4Data[i][0];
//         l_eventB4LaneInfo.m_ucLaneType = p_refB4Data[i][1];
//         l_eventB4LaneInfo.m_usLaneDirection = p_refB4Data[i][2];
//         l_eventB4LaneInfo.m_usLaneSpeedLimit = p_refB4Data[i][3];

//         l_eventB4LaneInfo.m_ucFollowPercent = p_refB4Data[i][7];
//         l_eventB4LaneInfo.m_ucTimeOccupancy = p_refB4Data[i][8];
//         l_eventB4LaneInfo.m_usHeadSpaceAvg = p_refB4Data[i][9];

//         TEventB4VehicheInfo m_tB4CarInfo;
//         m_tB4CarInfo.m_usFlow = p_refB4Data[i][10];
//         m_tB4CarInfo.m_usSpeedAvg = p_refB4Data[i][15];
//         l_eventB4LaneInfo.m_tCarInfo = m_tB4CarInfo;

//         TEventB4VehicheInfo m_tB4LargeTruckInfo;
//         m_tB4LargeTruckInfo.m_usFlow = p_refB4Data[i][11];
//         m_tB4LargeTruckInfo.m_usSpeedAvg = p_refB4Data[i][16];
//         l_eventB4LaneInfo.m_tLargeTruckInfo = m_tB4LargeTruckInfo;

//         TEventB4VehicheInfo m_tB4BusInfo;
//         m_tB4BusInfo.m_usFlow = p_refB4Data[i][12];
//         m_tB4BusInfo.m_usSpeedAvg = p_refB4Data[i][17];
//         l_eventB4LaneInfo.m_tBusInfo = m_tB4BusInfo;

//         TEventB4VehicheInfo m_tB4MiniBusInfo;
//         m_tB4MiniBusInfo.m_usFlow = p_refB4Data[i][13];
//         m_tB4MiniBusInfo.m_usSpeedAvg = p_refB4Data[i][18];
//         l_eventB4LaneInfo.m_tMiniBusInfo = m_tB4MiniBusInfo;

//         TEventB4VehicheInfo m_tB4TruckInfo;
//         m_tB4TruckInfo.m_usFlow = p_refB4Data[i][20];
//         m_tB4TruckInfo.m_usSpeedAvg = p_refB4Data[i][22];
//         l_eventB4LaneInfo.m_tTruckInfo = m_tB4TruckInfo;

//         TEventB4VehicheInfo m_tB4LittleTruckInfo;
//         m_tB4LittleTruckInfo.m_usFlow = p_refB4Data[i][21];
//         m_tB4LittleTruckInfo.m_usSpeedAvg = p_refB4Data[i][23];
//         l_eventB4LaneInfo.m_tLittleTruckInfo = m_tB4LittleTruckInfo;

//         TEventB4VehicheInfo m_tB4AmbulanceInfo;
//         m_tB4AmbulanceInfo.m_usFlow = 0;
//         m_tB4AmbulanceInfo.m_usSpeedAvg = 0;
//         l_eventB4LaneInfo.m_tAmbulanceInfo = m_tB4AmbulanceInfo;

//         TEventB4VehicheInfo m_tB4FireengineInfo;
//         m_tB4FireengineInfo.m_usFlow = 0;
//         m_tB4FireengineInfo.m_usSpeedAvg = 0;
//         l_eventB4LaneInfo.m_tFireengineInfo = m_tB4FireengineInfo;


//         m_vecEvent.push_back(l_eventB4LaneInfo);
//     }
//     eventB4Data.m_vecLaneEvent = m_vecEvent;
// }
*/

/*xcb note muqian yong GetB5AlgResult
bool CEventAlg::GetB5AlgResult(std::vector<std::vector<double>> &p_refB5Data, int stationId, TEventB5Data& eventB5Data)
{
    //调用算法接口获取B5帧结果数据

    //将B5结果数据转换成对应的数据体
    std::cout<<"xcb---GetB5AlgResult------0000000"<<std::endl;
    std::vector<TEventB5Info> m_vecEvent;
    std::string stationId_s = std::to_string(stationId);
    std::cout<<"xcb---GetB5AlgResult------11111111"<<std::endl;
    eventB5Data.strDeviceId = stationId_s;
    std::cout<<"xcb---GetB5AlgResult------2222222"<<std::endl;
    std::cout << "stationId_s: " << stationId_s << ", eventB5Data.strDeviceId(stationId_s): " <<eventB5Data.strDeviceId << std::endl;
    eventB5Data.ullTimestamp = 0;
    eventB5Data.ocEventNum = p_refB5Data.size();
    for (int i = 0; i < p_refB5Data.size(); i++)
    {
        TEventB5Info l_eventB5Info;
        l_eventB5Info.ulEventId = p_refB5Data[i][0];
        l_eventB5Info.ocEventType = p_refB5Data[i][1];
        l_eventB5Info.strSource = "3";
        l_eventB5Info.strSensor = "3";
        l_eventB5Info.vecLaneNos.push_back(p_refB5Data[i][4]);
        l_eventB5Info.dEventLongitude = p_refB5Data[i][5];
        l_eventB5Info.dEventLatitude = p_refB5Data[i][6];
        l_eventB5Info.dEventAltitude = 0;
        l_eventB5Info.ullEventTimestampStart = p_refB5Data[i][7];
        l_eventB5Info.ullEventTimestampEnd = p_refB5Data[i][8];
        l_eventB5Info.vecRefVehicles.push_back(p_refB5Data[i][9]);
        l_eventB5Info.strRadius = "10";
//        l_eventB5Info.vecPath();
//        l_eventB5Info.strImage(0);
        m_vecEvent.push_back(l_eventB5Info);
    }

    eventB5Data.vecB5EventInfo = m_vecEvent;
    return true;
}
*/

//xcb note CEventAlgorithm::TrackAlgResTransfer
// xt::xarray<double> CEventAlgorithm::TrackAlgResTransfer(CTrackResult *p_pTrackResult, int p_nRadarId)
// {
//     std::cout<<"p_nRadarId: "<<p_nRadarId<<std::endl;
//     if (p_pTrackResult){
//         xt::xarray<double> l_TransferOut = xt::ones<double>({int (p_pTrackResult->vecBox().size()) ,53})*(-1);
//         static int p = 0;
//         static int k = 0;
//         std::string event_data_path;
//         if (p_nRadarId == 1)
//         {
//             p++;
//             event_data_path = "/data/isfp_radar/Modules/Handlers/AlgHandler/SuiDaoAlg/EventAlg/event_data/"+ to_string(p_nRadarId) + "/track_" +
//                                       to_string(p) +".csv";}
//         else{
//             k++;
//             event_data_path = "/data/isfp_radar/Modules/Handlers/AlgHandler/SuiDaoAlg/EventAlg/event_data/"+ to_string(p_nRadarId) + "/track_" +
//                                           to_string(k) +".csv";
//         }
//         ofstream out(event_data_path,ios::out | ios::app);
//         for(int i=0; i<p_pTrackResult->vecBox().size(); i++){
//             l_TransferOut(i,0) = p_pTrackResult->vecBox()[i].sXCoord();
//             l_TransferOut(i,1) = p_pTrackResult->vecBox()[i].sYCoord();
//             l_TransferOut(i,2) = p_pTrackResult->vecBox()[i].sZCoord();
//             l_TransferOut(i,3) = p_pTrackResult->vecBox()[i].ucWidth();
//             l_TransferOut(i,4) = p_pTrackResult->vecBox()[i].ucLength();
//             l_TransferOut(i,5) = p_pTrackResult->vecBox()[i].ucHeight();
//             l_TransferOut(i,6) = p_pTrackResult->vecBox()[i].fCourseAngle();
//             l_TransferOut(i,7) = p_pTrackResult->vecBox()[i].ucClass();
//             l_TransferOut(i,8) = p_pTrackResult->vecBox()[i].fSpeedY();
//             l_TransferOut(i,9) = p_pTrackResult->vecBox()[i].usTargetId();
//             l_TransferOut(i,10) = p_pTrackResult->vecBox()[i].ucConfidence();
//             l_TransferOut(i,11) = p_pTrackResult->vecBox()[i].ucSource();
//             l_TransferOut(i,13) = 1;
//             l_TransferOut(i,15) = p_pTrackResult->vecBox()[i].dLon();
//             l_TransferOut(i,16) = p_pTrackResult->vecBox()[i].dLat();
//             getConners(&l_TransferOut(i,20),l_TransferOut(i,0),l_TransferOut(i,1),l_TransferOut(i,2),
//                        l_TransferOut(i,3),l_TransferOut(i,4),l_TransferOut(i,5),l_TransferOut(i,6));
//             l_TransferOut(i,44) = p_pTrackResult->vecBox()[i].ullTimestamp();
//             l_TransferOut(i,46) = p_pTrackResult->vecBox()[i].usLeftTopX();
//             l_TransferOut(i,47) = p_pTrackResult->vecBox()[i].usLeftTopY();
//             l_TransferOut(i,48) = p_pTrackResult->vecBox()[i].usRightDownX();
//             l_TransferOut(i,49) = p_pTrackResult->vecBox()[i].usRightDownY();
// //            cout<<"x:"<<l_TransferOut(i,0)<<endl;
// //            cout<<"y:"<<l_TransferOut(i,1)<<endl;
// //            cout<<"TIME:"<<(unsigned long long)l_TransferOut(i,44)<<endl;
//             std::cout.precision(8);
//             for (int j =0;j<53;j++){
//                 if(j<44){
//                     out<<l_TransferOut(i,j)<<',';
//                 }
//                 if(j==44){
//                     out<<(unsigned long long)l_TransferOut(i,j)<<',';
//                 }
//                 if(j>44 and j<52){
//                     out<<l_TransferOut(i,j)<<',';
//                 }
//                 if(j==52){
//                     out<<l_TransferOut(i,j);
//                 }
//             }
//             out<<endl;
//         }
// //        std::cout<<"l_TransferOut.shape(0):"<<l_TransferOut.shape(1)<<std::endl;
//         out.close();
// //        std::cout << "liluo----------> l_TransferOut(17-43): " << xt::view(l_TransferOut, xt::all(), xt::range(17, 44)) << std::endl;
// //        std::cout << "liluo----------> l_TransferOut(17-43): " << l_TransferOut << std::endl;
//         return l_TransferOut;
//     }
// }


/* xcb note CEventAlgorithm::RunAlgorithm
bool CEventAlgorithm::RunAlgorithm(void *p_pInputEvent, void* pB3Data, void* pB4Data, void* pB5Data, void* p_nRadarId) {
    std::cout<<"------------------------------RunAlgorithm--------------------------"<<std::endl;
    if (!p_pInputEvent) {
        return false;
    }
    static int i=0;
    i++;
    CEventB3Data* pEventB3Data = (CEventB3Data*)pB3Data;
    CEventB4Data* pEventB4Data = (CEventB4Data*)pB4Data;
    CEventB5Data* pEventB5Data = (CEventB5Data*)pB5Data;

    cout<<"frame:"<<i<<endl;
    auto &l_tSrcData = *(CTrackResult *) p_pInputEvent;
    CTrackResult &e_data = l_tSrcData;
    int stationId = *((int *) p_nRadarId);
    auto lidar_info = TrackAlgResTransfer(&e_data, stationId);

    if (lidar_info.size() > 0) {

        std::cout << "CEventAlgorithm::TIME: " << (unsigned long long) lidar_info(0, 44) << std::endl;

        cout <<"stationId: " << stationId << ", input:" << lidar_info.shape(0) << endl;

//    long B3_time = ops::getTimeStamp();
//    long B4_time = ops::getTimeStamp();
        bool B3_flag = false;
        bool B4_flag = false;

        std::string path = m_strOutPath + "/Configs/Alg/CppConfigs/event/use_time_traffic";
        if (access(path.c_str(), 0) == -1) {
            mkdir(path.c_str(), S_IRWXU);
        }
        path += "event_result.txt";
        if (access(path.c_str(), 0) == 0) {
            if (std::remove(path.c_str()) == 0) {
                std::cout << ">>> Delete use_time_traffic.txt. --> Sucess.\n";
            } else {
                std::cout << ">>> Delete use_time_traffic.txt. --> Failed.\n";
            }
        }
        std::ofstream event_traffic_fd;
        event_traffic_fd.open(path, std::ios::out | std::ios::app);

        static bool firstFrame = false;
        double t_B4_now = lidar_info(0, 44);
        double t_B3_now = lidar_info(0, 44);

        static double t_B4, t_B3; // t_B3, t_B3_now why?
        if (!firstFrame) {
            t_B4 = lidar_info(0, 44);
            t_B3 = lidar_info(0, 44);
        }
        firstFrame = true;

        long t_now = ops::getTimeStamp();

//    cout<<"t_B4_now"<<(unsigned long long)t_B4_now<<endl;
//    cout<<"t_B4"<<(unsigned long long)t_B4<<endl;

        if (t_B3_now - t_B3 >= 1 * 1000) {
            B3_flag = true;
            t_B3 = t_B3_now;
        } else {
            B3_flag = false;
        }
        if (t_B4_now - t_B4 >= 60 * 1000) {
            B4_flag = true;
            t_B4 = t_B4_now;
        } else {
            B4_flag = false;
        }


        xt::xarray<double> temp = xt::empty<double>({0, 0});
        if (B4_flag) {
            m_traffic_flow->use(lidar_info, true, temp, stationId);
        } else {
            m_traffic_flow->use(lidar_info, false, temp, stationId);
        }

        // get_B3:report B3 result
        std::vector<std::vector<double>> b3_output; // yk
        std::shared_ptr<Event_Output> event_res;
        if (B3_flag) {
            m_traffic_flow->get_B3(event_res);
            b3_output = event_res->out_content; // yk
            GetB3A->get_B3lgResult(b3_output,stationId, *pEventB3Data);

        std::cout << "-----------" << stationId << "-b3--------------------------------------" << std::endl;
//        std::cout
//                << "# lane name,lane type,lane direction,lane limit speed,lane avg speed,lane queue,lane head space,lane occupy,0,lane vehicle num,lane car num,lane truck num,lane bus num,lane mini num,lane people num,lane non_motor num,lane medium truck num,lane little truck num"
//                << std::endl;
//        for (int j = 0; j < b3_output.size(); ++j) {
//            for (int k = 0; k < b3_output[j].size(); ++k) {
//                std::cout << b3_output[j][k] << ",";
//            }
//            std::cout << "." << std::endl;
//        }
//        long b3_time = ops::getTimeStamp();
//        cout<<"ttime_1:"<<b3_time<<endl;
        }
        // get_B4:report B4 result
        if (B4_flag) {
            m_traffic_flow->get_B4(event_res);
            std::vector<std::vector<double>> b4_output;                                                     // yk
            b4_output = event_res->out_context;                                                             // yk
            GetB4AlgResult(b4_output, stationId,*pEventB4Data);
            std::cout << "-----------" << stationId << "-b4--------------------------------------" << std::endl;
//        std::cout << "--------------------------b4--------------------------------------"
//                  << std::endl; // yk:10.28 comment:congestion state isn't be couted.
//        std::cout
//                << "#lane name, lane type, lane direct, lane speed, 基站数量, 0, 0, lane follow_car_percent, lane time_occupancy, lane mean_sh, lane car num, lane truck num, lane bus num, lane minibus num, lane car mean_v, lane truck mean_v, lane bus mean_v, lane minibus mean_v, lane medium truck num, lane little truck num, lane medium truck mean_v ,lane little truck mean_v"
//                << std::endl;
//        for (int j = 0; j < b4_output.size(); ++j) {
//            for (int k = 0; k < b4_output[j].size(); ++k) {
//                std::cout << b4_output[j][k] << ",";
//            }
//            std::cout << "." << std::endl;
//        }
        }
        // get_B5:report B5 result
        std::shared_ptr<Event_Output> b5 = m_traffic_flow->get_B5();
        std::vector<std::shared_ptr<B5_event_info>> event_result; // save all event in the variable
        event_result = b5->out_all_event_output_for_B5;
        int event_num = b5->out_input4;
        std::vector<std::vector<double>> b5_output;

        std::cout << "stationId: "<< stationId << ", PublishEventB5Data" << event_num << ", " << event_result.size() << std::endl;
//    CEventB5Data CEventB5Data;
        std::vector<CEventB5Info> m_vecEvent;
        if (event_num > 0) {
            std::cout << "---------------"<<stationId<<"-b5--------------------------------------" << std::endl;
            for (int j = 0; j < event_num; ++j) {
                std::vector<double> one_event_info;
                std::cout.precision(8);                                                                                                                                                                                                                                                                                                                                   // yk:save the 小数位数
//            std::cout << event_result[j]->item0 << "," << event_result[j]->item1 << "," << event_result[j]->item2 << ","
//                      << event_result[j]->item3 << "," << event_result[j]->item4 << "," << event_result[j]->item5 << ","
//                      << event_result[j]->item6 << "," << event_result[j]->item7 << "," << event_result[j]->item8[0][0]
//                      << "," << event_result[j]->item9 << std::endl; // print out_all_event_output_for_B5 event
                std::shared_ptr<B5_event_info> result = event_result[j];
                double start_time = (result->item11) * 1000 + (result->item12);
                double id = result->item8[0][0];
                one_event_info.push_back(result->item0);
                one_event_info.push_back(result->item1);
                one_event_info.push_back(3);
                one_event_info.push_back(3);
                one_event_info.push_back(result->item6);
                one_event_info.push_back(result->item9);
                one_event_info.push_back(result->item10);
                one_event_info.push_back(start_time);
                one_event_info.push_back(result->item15);
                one_event_info.push_back(id);
                one_event_info.push_back(10);
                b5_output.push_back(one_event_info);
            }
            pEventB5Data->vecB5EventInfo(m_vecEvent);
            GetB5AlgResult(b5_output, stationId,*pEventB5Data);
            ofstream out(
                    "/data/isfp_radar/Modules/Handlers/AlgHandler/SuiDaoAlg/EventAlg/use_time_traffic/event_result" + to_string(stationId) + ".txt",
                    ios::out | ios::app);
            for (int j = 0; j < event_result.size(); ++j) {
                out << "ID:" << event_result[j]->item8[0][0] << std::endl;
                out << event_result[j]->item0 << "," << event_result[j]->item1 << "," << event_result[j]->item2
                    << "," << event_result[j]->item3 << "," << event_result[j]->item4 << ","
                    << event_result[j]->item5 << "," << event_result[j]->item6 << "," << event_result[j]->item7
                    << "," << event_result[j]->item8[0][0] << "," << event_result[j]->item9 << ","
                    << event_result[j]->item10 << "," << event_result[j]->item11 << ","
                    << event_result[j]->item12 << "," << event_result[j]->item13 << ","
                    << event_result[j]->item14 << "," << event_result[j]->item15 << std::endl;
            }
            out.close();
        }

        return true;
    }else{
        std::cout << "Inupt TrackResult is Empty !!!!!!! " << std::endl;
        return false;
    }
    delete pEventB3Data;
    delete pEventB4Data;
    delete pEventB5Data;
}
*/

/* xcb note CEventAlgorithm::GetB3AlgResult
bool CEventAlgorithm::GetB3AlgResult(std::vector<std::vector<double>> &p_refB3Data, int stationId,  CEventB3Data& eventB3Data)
{
    //调用算法接口获取B3帧结果数据

    //将B3结果数据转换成对应的DDS数据体
    std::string stationId_s = std::to_string(stationId);
    std::vector<CEventB3LaneInfo> m_vecEvent;
    eventB3Data.strDeviceId(stationId_s);                     //设备ID
    std::cout << "stationId_s: " << stationId_s << ", eventB3Data.strDeviceId(stationId_s): " <<eventB3Data.strDeviceId() << std::endl;
    eventB3Data.ullTimestamp(0);        //时间戳
    eventB3Data.ulEventFrameId(7897);           //帧号
    eventB3Data.ocLaneNum(p_refB3Data.size());
    for (int i = 0; i < p_refB3Data.size(); i++)
    {
        CEventB3LaneInfo l_eventB3LaneInfo;
        l_eventB3LaneInfo.ocLaneNo(p_refB3Data[i][0]);
        l_eventB3LaneInfo.ocLaneType(p_refB3Data[i][1]);
        l_eventB3LaneInfo.sLaneDirection(p_refB3Data[i][2]);
        l_eventB3LaneInfo.usLaneSpeedLimit(p_refB3Data[i][3]);
        l_eventB3LaneInfo.usLaneSpeedAvg(p_refB3Data[i][4]);
        l_eventB3LaneInfo.ulQueueLength(p_refB3Data[i][5]);
        l_eventB3LaneInfo.usLaneHeadSpace(p_refB3Data[i][6]);

        l_eventB3LaneInfo.usLaneHeadperiod(0);

        l_eventB3LaneInfo.ocSpaceOccupancy(p_refB3Data[i][7]);
        l_eventB3LaneInfo.ocTimeOccupancy(0);

        l_eventB3LaneInfo.usVehicleNum(p_refB3Data[i][9]);
        l_eventB3LaneInfo.usCarNum(p_refB3Data[i][10]);
        l_eventB3LaneInfo.usBusNum(p_refB3Data[i][12]);
        l_eventB3LaneInfo.usMediumBusNum(p_refB3Data[i][13]);
        l_eventB3LaneInfo.usPedestrianNum(p_refB3Data[i][15]);
        l_eventB3LaneInfo.usNonVehicleNum(p_refB3Data[i][16]);
        l_eventB3LaneInfo.usTruckNum(p_refB3Data[i][17]);
        l_eventB3LaneInfo.usSmallTruckNum(p_refB3Data[i][18]);
        m_vecEvent.push_back(l_eventB3LaneInfo);
    }

    eventB3Data.vecLaneInfo(m_vecEvent);
}
*/

/* xcb note CEventAlgorithm::GetB4AlgResult
bool CEventAlgorithm::GetB4AlgResult(std::vector<std::vector<double>> &p_refB4Data, int stationId, CEventB4Data& eventB4Data)
{
    //调用算法接口获取B4帧结果数据

    //将B4结果数据转换成对应的DDS数据体
    std::vector<CEventB4LaneInfo> m_vecEvent;
    std::string stationId_s = std::to_string(stationId);

    eventB4Data.strDeviceId(stationId_s);                     //设备ID
    std::cout << "stationId_s: " << stationId_s << ", eventB4Data.strDeviceId(stationId_s): " <<eventB4Data.strDeviceId() << std::endl;
    eventB4Data.ullStartTimestamp(0);   //开始时间戳
    eventB4Data.ullEndTimestamp(60);     //结束时间戳
    eventB4Data.ulEventFrameId(23);           //帧号
    eventB4Data.ocLaneNum(p_refB4Data.size());                        //车道数量

    for (int i = 0; i < p_refB4Data.size(); i++)
    {
        CEventB4LaneInfo l_eventB4LaneInfo;
        l_eventB4LaneInfo.ocLaneNo(p_refB4Data[i][0]);
        l_eventB4LaneInfo.ocLaneType(p_refB4Data[i][1]);
        l_eventB4LaneInfo.sLaneDirection(p_refB4Data[i][2]);
        l_eventB4LaneInfo.sLaneSpeedLimit(p_refB4Data[i][3]);

        l_eventB4LaneInfo.ocFllowPercent(p_refB4Data[i][7]);
        l_eventB4LaneInfo.ocTimeOccupancy(p_refB4Data[i][8]);
        l_eventB4LaneInfo.usHeadSpaceAvg(p_refB4Data[i][9]);
        l_eventB4LaneInfo.usCarFlow(p_refB4Data[i][10]);
        l_eventB4LaneInfo.usBusFlow(p_refB4Data[i][12]);
        l_eventB4LaneInfo.usMediumBusFlow(p_refB4Data[i][13]);
        l_eventB4LaneInfo.usCarSpeedAvg(p_refB4Data[i][15]);
        l_eventB4LaneInfo.usBusSpeedAvg(p_refB4Data[i][17]);
        l_eventB4LaneInfo.usMediumBusSpeedAvg(p_refB4Data[i][18]);
        l_eventB4LaneInfo.usTruckFlow(p_refB4Data[i][21]);
        l_eventB4LaneInfo.usSmallTruckFlow(p_refB4Data[i][20]);
        l_eventB4LaneInfo.usTruckSpeedAvg(p_refB4Data[i][23]);
        l_eventB4LaneInfo.usSmallTruckSpeedAvg(p_refB4Data[i][22]);
        m_vecEvent.push_back(l_eventB4LaneInfo);
    }
    eventB4Data.vecLaneInfo(m_vecEvent);
}
*/

/* xcb note CEventAlgorithm::GetB5AlgResult
//bool CEventAlgorithm::GetB5AlgResult(CEventB5Data &p_refB5Data)
//{
//    //调用算法接口获取B5帧结果数据
//
//    //将B5结果数据转换成对应的DDS数据体
//    p_refB5Data.strDeviceId("0");
//    p_refB5Data.ullTimestamp(0);
//    p_refB5Data.ocEventNum(0);
//    for (int i = 0; i < p_refB5Data.ocEventNum(); i++)
//    {
//        CEventB5Info l_eventB5Info;
//        l_eventB5Info.ulEventId(0);
//        l_eventB5Info.ocEventType(0);
//        l_eventB5Info.strSource("3");
//        l_eventB5Info.strSensor("3");
//        l_eventB5Info.vecLaneNos().push_back(0);
//        l_eventB5Info.dEventLongitude(0);
//        l_eventB5Info.dEventLatitude(0);
//        l_eventB5Info.dEventAltitude(0);
//        l_eventB5Info.ullEventTimestampStart(0);
//        l_eventB5Info.ullEventTimestampEnd(0);
//        l_eventB5Info.vecRefVehicles();
//        l_eventB5Info.strRadius("0");
//        l_eventB5Info.vecPath();
//        l_eventB5Info.strImage("");
//        p_refB5Data.vecB5EventInfo().push_back(l_eventB5Info);
//    }
//
//    return true;
//}

/* xcb note
bool CEventAlgorithm::GetB5AlgResult(std::vector<std::vector<double>> &p_refB5Data, int stationId, CEventB5Data& eventB5Data)
{
    //调用算法接口获取B5帧结果数据

    //将B5结果数据转换成对应的DDS数据体
    std::vector<CEventB5Info> m_vecEvent;
    std::string stationId_s = std::to_string(stationId);
    eventB5Data.strDeviceId(stationId_s);
    std::cout << "stationId_s: " << stationId_s << ", eventB5Data.strDeviceId(stationId_s): " <<eventB5Data.strDeviceId() << std::endl;
    eventB5Data.ullTimestamp(0);
    eventB5Data.ocEventNum(p_refB5Data.size());
    for (int i = 0; i < p_refB5Data.size(); i++)
    {
        CEventB5Info l_eventB5Info;
        l_eventB5Info.ulEventId(p_refB5Data[i][0]);
        l_eventB5Info.ocEventType(p_refB5Data[i][1]);
        l_eventB5Info.strSource("3");
        l_eventB5Info.strSensor("3");
        l_eventB5Info.vecLaneNos().push_back(p_refB5Data[i][4]);
        l_eventB5Info.dEventLongitude(p_refB5Data[i][5]);
        l_eventB5Info.dEventLatitude(p_refB5Data[i][6]);
        l_eventB5Info.dEventAltitude(0);
        l_eventB5Info.ullEventTimestampStart(p_refB5Data[i][7]);
        l_eventB5Info.ullEventTimestampEnd(p_refB5Data[i][8]);
        l_eventB5Info.vecRefVehicles().push_back(p_refB5Data[i][9]);
        l_eventB5Info.strRadius("10");
//        l_eventB5Info.vecPath();
//        l_eventB5Info.strImage(0);
        m_vecEvent.push_back(l_eventB5Info);
    }

    eventB5Data.vecB5EventInfo(m_vecEvent);
}


/* xcb note
std::string CEventAlgorithm::GetInterfaceName()
{
    return "IEventAlgorithm";
}

IAlgHandler *CEventAlgorithm::Clone()
{
    return new CEventAlgorithm;
}
*/