#pragma once

#include <vector>
#include <string>

//定义FastDDS通信的topic
#define TOPIC_GETDATAREPLY_EVENTB3DATA		    "getDataReply_EventB3Data"				//S端发送命令号，回复Radar结果
#define TOPIC_GETDATA_EVENTB3DATA               "getData_EventB3Data"                   //C端发送命令号，查询radar识别结果


struct TEventB3LaneInfo
{
    uint8_t m_ucLaneNo;                   //车道号 从左往右1开始
    uint8_t m_ucLaneType;                 //车道类型0-未知,1-机动车,2-非机动车,3-应急车道
    short m_usLaneDirection;               //车道方向 正北夹角，顺时针为正
    unsigned short m_usLaneSpeedLimit;    //车道限速 m/s
    unsigned short m_usLaneSpeedAvg;      //平均速度 m/s
    unsigned long m_uiQueueLength;        //排队长度 m
    unsigned long m_ucQueueState;                 //排队状态 cjm 0925  6
    unsigned short m_usLaneHeadSpace;     //车头间距 m
    uint8_t m_ucSpaceOccupancy;           //空间占用率
    unsigned short m_usVehicleNum;        //机动车车辆数
    unsigned short m_usNonVehicleNum;     //非机动车车辆数
    bool m_bTakeMap;
    unsigned short m_uiWaitingTime;      
    unsigned short m_usBaseLocation;    

};

struct TEventB3Data
{
    uint8_t m_usDeviceId;                    //设备ID
    unsigned long long m_ulTimeStamp;            //时间戳
    unsigned long m_ulB3FrameId;               //帧号
    uint8_t m_ucLaneNum;                          //车道数量
    uint8_t m_ucCrossingNum;                      
    std::vector<TEventB3LaneInfo> m_vecLaneEvent;  //B3车道信息
    std::vector<TEventB3LaneInfo> m_vecCrossingEvent;  
};