/*******************************************************
 文件名：TTriggerAlgResult.h
 作者：
 描述：触发算法结果结构体
 版本：v1.0
 日期：2021-11-02
 *******************************************************/

#ifndef TTRIGGERALGRESULT_H
#define TTRIGGERALGRESULT_H

#include <vector>
#include <string>


#define TOPIC_GETDATAREPLY_TRIGGERRESULT			"getDataReply_TriggerResult"		//S端发送命令号，回复触发结果数据帧


//触发车辆信息
struct TTriggerBoxInfo
{
    unsigned short           m_usBoxId;          //id
    int                      m_nTriggerStatus;   //触发状态
    int                      m_nLaneNum;         //所在车道号
    double                   m_dLon;             //经度
    double                   m_dLat;             //纬度
    int                      m_nSpeed;           //速度 cm/s
    unsigned int             m_unLength;         //长 cm
    unsigned int             m_unWidth;          //宽 cm

    // 目标 左右车头的X，Y坐标
    int                      m_nLeftPixelX;
    int                      m_nLeftPixelY;
    int                      m_nRightPixelX;
    int                      m_nRightPixelY;

    // 0~1 存储目标角度，uint16_t数据，占用2字节
    unsigned char            m_ucReserve[10];    // 预留
};

//触发算法结果数据结构体
struct TTriggerAlgResult
{
    unsigned int                    m_unFrameId;            //帧号，来自点云结果数据（结果数据帧号来自原始点云数据，读取时自己累计的，最大不超过65535）
    unsigned char                   m_ucResultBuff[3000];   //结果数据帧
    unsigned short                  m_usBufMsgLen;          //数据帧长度
    std::vector<unsigned short>     m_vecTriggeredId;       //已触发车辆id列表,累计统计触发车辆id，最多20个

    // 时间戳信息记录数组，预留
    // 0 - 点云第0包时间戳
    // 1 - 执行trig算法时间戳
    unsigned long                   m_ulBufTimeStamp[10];

    //记录耗时数组，预留
    // 0 - 组帧完成到执行trig算法时刻的时间延迟 ms
    // 1 - 点云算法检测延时 ms
    unsigned int                    m_unBufDelay[10];

    //触发目标信息
    std::vector<TTriggerBoxInfo>    m_vecTriggerBoxInfo;    //当前帧的触发车辆info

    // 0 - 当前帧触发车辆个数 ，一般情况是0 或者 1 ，很少有2以上。
    unsigned char                   m_ucReserve[10]; // 预留
};

#endif