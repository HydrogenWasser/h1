#pragma once

#include <vector>
#include <string>

//定义FastDDS通信的topic
#define TOPIC_GETDATAREPLY_EVENTB4DATA		    "getDataReply_EventB4Data"				//S端发送命令号，回复Radar结果
#define TOPIC_GETDATA_EVENTB4DATA               "getData_EventB4Data"                   //C端发送命令号，查询radar识别结果


struct TEventB4VehicheInfo
{
    uint8_t m_usFlow;
    uint8_t m_usSpeedAvg;
};

struct TEventB4LaneInfo
{
    uint8_t m_ucLaneNo;                   //车道号 从左往右1开始
    uint8_t m_ucLaneType;                 //车道类型0-未知,1-机动车,2-非机动车,3-应急车道
    short m_usLaneDirection;               //车道方向 正北夹角，顺时针为正
    unsigned short m_usLaneSpeedLimit;    //车道限速 m/s
    uint8_t m_ucStationNum;
    std::vector<uint8_t> m_vecStationCongestion;   //MEC拥堵状况列表  4
    uint8_t m_ucFollowPercent;                   //跟车百分比
    uint8_t m_ucTimeOccupancy;                  //时间占用率
    unsigned short m_usHeadSpaceAvg;          //平均车头间距 m

    TEventB4VehicheInfo m_tCarInfo;
    TEventB4VehicheInfo m_tLargeTruckInfo;
    TEventB4VehicheInfo m_tBusInfo;
    TEventB4VehicheInfo m_tMiniBusInfo;
    TEventB4VehicheInfo m_tTruckInfo;
    TEventB4VehicheInfo m_tLittleTruckInfo;
    TEventB4VehicheInfo m_tAmbulanceInfo;
    TEventB4VehicheInfo m_tFireengineInfo;
};

struct TEventB4Data
{
    uint8_t m_usDeviceId;                    //设备ID
    unsigned long long m_ulstartTimeStamp;   //开始时间戳
    unsigned long long m_ulendTimeStamp;     //结束时间戳
    unsigned long m_ulB4FrameId;           //帧号
    uint8_t m_ucLaneNum;                        //车道数量
    std::vector<TEventB4LaneInfo> m_vecLaneEvent; //B4车道事件信息
};