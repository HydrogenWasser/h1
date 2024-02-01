#ifndef ALGORITHM_TEVENTSTRUCT_H
#define ALGORITHM_TEVENTSTRUCT_H


#include <iostream>
#include <vector>


//B0车道信息
struct TB0ChannelData{
    unsigned short m_usChannelId ;              //ID
    unsigned short m_usChannelNum = 0;          //车道编号
    unsigned short m_usChannelDirection = 0;    //车道方向 正北方向顺时针夹角，0~360度
    unsigned char m_ucChannelLineDir = 0;       //车道线方向 0 — 直行1 — 左转2 — 右转3 — 直左4 — 直右5 — 未识别
    unsigned char m_ucChannelType =5;           //车道类型 0-  机动车道  1-  非机动车道
    unsigned short m_usChannelSpeedLimit = 0;   //车道限速  单位km/h
    unsigned short m_usChannelReservel1 = 0;    //预留 
    unsigned int m_unQueueLength = 0;           //排队长度
    unsigned int m_unSpcaeOccupancy = 0;        //空间占有率
    unsigned char m_ucCarNum = 0;               //车辆数
    unsigned char m_uc3ChannelReservel2[3] ;  //3 unsigned char;
    std::vector< unsigned short> m_vecCarHeadDis ; //车头间距
    unsigned short m_usChannelReservel3 = 0;    //预留
};

//B0帧消息内容
struct TB0MsgContent{
    unsigned short m_usEquiqId = 0;                 //设备ID
    unsigned short m_usReserve1 = 0;                //预留        
    unsigned int m_unTimeStamp1 = 0;              //时间戳信息 前32位为整数部分，单位为s
    unsigned int m_unTimeStamp2 = 0;              //时间戳信息 后32位为小数部分
    unsigned short m_usCarChannelNum = 0;           //车道数量
    unsigned short m_usReserve2 = 0;                //预留
    std::vector< TB0ChannelData> m_vecChannelData ; //
    unsigned short m_usReserve3 = 0;                //预留
};

//B0帧瞬时事件
struct TB0TransientEvent{
    unsigned short m_usBeginBit = (unsigned short)(0xffff);//帧开始位，固定0xFF 0xFF
    unsigned char m_ucSerialNum = 0;                //数据帧序列号0x00～0x0F
    unsigned char m_ucCommandNum = 0xB0;               //主命令号
    unsigned char m_ucSubComNum = 0;                //子命令号；00H，无意义
    unsigned char m_ucStatusBit = 0;                //状态位：00H,此时无意义
    unsigned short m_usMsgLen = 0;                  //消息长度
    //std::vector< TB0MsgContent> m_tMsgContent ;
    TB0MsgContent m_tMsgContent ;                   //消息内容信息
    unsigned char m_ucCheck = 0;                    //BCC奇偶校验位
    unsigned char m_ucCutoffBit = 0xFF;                //帧结束位,固定0xFF
};


//B1帧车道信息
struct TB1ChannelData{
    unsigned short m_usChannelId = 0;               //车道1ID
    unsigned short m_usChannelNum = 0;              //车道编号
    unsigned short m_usChannelDirection = 0;        //车道方向
    unsigned char m_ucChannelLineDir = 0;           //车道线方向
    unsigned char m_ucChannelType = 0;              //车道类型
    unsigned short m_usChannelSpeedLimit = 0;       //车道限速
    unsigned char m_ucCongestionStatus = 0;         //拥堵状况
    unsigned short m_usTraffic = 0;                 //车流量
    unsigned short m_usTimeShare = 0;               //时间占有率
    unsigned short m_usCar = 00;                    //汽车
    unsigned short m_usTruck = 0;                   //卡车、货车
    unsigned short m_usBus = 0;                     //大巴车
    unsigned short m_usMotorbike = 0;               //摩托车/电动车
    unsigned short m_usChannelReservel1 = 0;        //预留
};
//B1帧消息内容 
struct TB1MsgContent{
    unsigned short m_usEquiqId = 0;                 //设备ID
    unsigned short m_usReserve1 = 0;                //预留
    unsigned int m_unStartTimeStamp1 = 0;           //统计开始时间戳信息 前32位为整数部分，单位为s
    unsigned int m_unStartTimeStamp2 = 0;           //时间戳信息 后32位为小数部分
    unsigned int m_unEndTimeStamp1 = 0;             //统计截止时间戳信息 前32位为整数部分，单位为s
    unsigned int m_unEndTimeStamp2 = 0;             //时间戳信息 后32位为小数部分
    unsigned short m_usCarChannelNum = 0;           //车道数量
    unsigned short m_usReserve2 = 0;                //预留
    std::vector< TB1ChannelData> m_vecB1ChannelData;//
};
//B1帧统计事件 每隔一分钟主动发送一次，统计时长为1分钟
struct TB1TransientEvent{
    unsigned short m_usBeginBit = (unsigned short)(0xffff);//帧开始位，固定0xFF 0xFF
    unsigned char m_ucSerialNum = 0;                //数据帧序列号0x00～0x0F
    unsigned char m_ucCommandNum = 0xB1;               //主命令号
    unsigned char m_ucSubComNum = 0;                //子命令号；00H，无意义
    unsigned char m_ucStatusBit = 0;                //状态位：00H,此时无意义
    unsigned short m_usMsgLen = 0;                  //消息长度
    TB1MsgContent m_tMsgContent ;                   //消息内容信息
    unsigned char m_ucCheck = 0;                    //BCC奇偶校验位
    unsigned char m_ucCutoffBit = 0xFF;                //帧结束位,固定0xFF
};


//B2 通用异常事件信息
struct TB2ExceptEventMsg{
    unsigned short m_usExceptId = 0;                //ID
    unsigned short m_usExceptType = 0;              //异常类型
    unsigned int m_unCameraNumbers = 0;             //目标所在相机个数
    std::vector< unsigned int> m_vecCameraIp;          //相机IP
    unsigned int m_unExceptBeginTime1 = 0;          //异常事件开始时间 前32位为整数部分，单位为s  检测到异常事件的工控机时间戳
    unsigned int m_unExceptBeginTime2 = 0;          //
    unsigned long long m_ullExceptViolationTime = 0;  //异常事件持续时间  单位秒
    unsigned short m_usReservel1 = 0;               //预留
};
//B2帧消息内容
struct TB2MsgContent{

    unsigned short m_usEquiqId = 0;                 //设备ID
    unsigned short m_usReserve1 = 0;                //预留
    unsigned int m_unStartTimeStamp1 = 0;         //时间戳信息 前32位为整数部分，单位为s
    unsigned int m_unStartTimeStamp2 = 0;         //
    unsigned short m_usExceptEentNumbers = 0;       //异常事件数量
    unsigned short m_usReserve2 = 0;                //预留
    std::vector< TB2ExceptEventMsg> m_vecTargetMsg ;//
    unsigned short m_usReserve3 = 0;                //预留
};
//B2交通异常事件
struct TB2TransientEvent{

    unsigned short m_usBeginBit = (unsigned short)(0xffff);//帧开始位，固定0xFF 0xFF
    unsigned char m_ucSerialNum = 0;                //数据帧序列号0x00～0x0F
    unsigned char m_ucCommandNum = 0xB2;               //主命令号
    unsigned char m_ucSubComNum = 0;                //子命令号；00H，无意义
    unsigned char m_ucStatusBit = 0;                //状态位：00H,此时无意义
    unsigned short m_usMsgLen = 0;                  //消息长度
    TB2MsgContent m_tMsgContent ;                   //消息内容信息
    unsigned char m_ucCheck = 0;                    //BCC奇偶校验位
    unsigned char m_ucCutoffBit = 0xFF;                //帧结束位,固定0xFF
};


//B3车道信息
struct TB3ChannelData{
    unsigned char m_ucChannelId = 0;
    unsigned char m_ucChannelType = 0;
    unsigned short m_usChannelDirection = 0;
    unsigned char m_ucChannelSpeedLimit = 0;
    unsigned char m_ucMeanSpeed = 0;
    unsigned int m_unQueueLength = 0;
    unsigned short m_usChannelInterval = 0;
    unsigned char m_ucSpcaeOccupancy = 0;
    unsigned char m_ucReservel1 = 0;
    unsigned short m_usMotorVehicles = 00;
    unsigned short m_usCarNum = 0;
    unsigned short m_usTruckFreight = 0;
    unsigned short m_usBigBus = 0;
    unsigned short m_usMediumBus = 0;
    unsigned short m_usTankCar = 0;
    unsigned short m_usPeople = 0 ;
    unsigned short m_usNoMotorVehicles = 0 ;
    unsigned int m_unReservel2 = 0;
};
//B3消息内容
struct TB3MsgContent{
    unsigned short m_usEquiqId = 0;
    unsigned short m_usReserve1 = 0;
    unsigned int m_unTimeStampS = 0;
    unsigned int m_unTimeStampUs = 0;
    unsigned int m_unFrameId = 0;
    unsigned char m_ucCarChannelNums = 0;
    unsigned char m_u3cReserve2[3];
    std::vector< TB3ChannelData> m_vecInfoData;
    unsigned int m_unReserve3 = 0;
};
//B3帧瞬时事件
struct TB3TransientEvent{
    unsigned short m_usBeginBit = 0;
    unsigned char m_ucSerialNum = 0;
    unsigned char m_ucCommandNum = 0;
    unsigned char m_ucSubComNum = 0 ;
    unsigned char m_ucStatusBit = 0;
    unsigned short m_usMsgLen = 0;
    //std::vector< TB3MsgContent> m_tMsgContent ;
    TB3MsgContent m_tMsgContent ;
    unsigned char m_ucCheck = 0;
    unsigned char m_ucCutoffBit = 0;
};


//B4基站信息
struct TB4StationData{
    unsigned char m_ucStationId = 0;
    unsigned char m_ucCongestion = 0;
    unsigned short m_usReservel = 0;
};
//B4帧车道信息
struct TB4ChannelData{
    unsigned char m_ucChannelId = 0;
    unsigned char m_ucChannelType = 0;
    unsigned char m_usChannelDirection = 0;
    unsigned char m_ucChannelSpeedLimit = 0;
    unsigned char m_ucStationNums = 0;
    unsigned short m_usReservel1 = 0;
    std::vector< TB4StationData> m_vecStationData ;
    unsigned char m_ucCarfollowpercen  = 0;
    unsigned char m_ucTimeOccupancy = 0;
    unsigned short m_usCarHeadDis = 0;
    unsigned short m_usCarFlow = 0;
    unsigned short m_usTruckFreightFlow = 0;
    unsigned short m_usBigBusFlow = 0;
    unsigned short m_usMediumBusFlow = 0;
    unsigned short m_usTankCarFlow = 0;
    unsigned char m_ucCarAverSpeed  = 0;
    unsigned char m_ucTruckFreightAverSpeed = 0;
    unsigned char m_ucBigBusAverSpeed = 0;
    unsigned char m_ucMediumBusAverSpeed = 0;
    unsigned char m_ucTankCarAverSpeed = 0;
    unsigned char m_ucReservel2 = 0;
};
//B4帧消息内容
struct TB4MsgContent{
    unsigned short m_usEquiqId = 0;
    unsigned short m_usReserve1 = 0;
    unsigned int m_unStartTimeStampS = 0;
    unsigned int m_unStartTimeStampUs = 0;
    unsigned int m_unEndTimeStampS = 0;
    unsigned int m_unEndTimeStampUs = 0;
    unsigned int m_unFrameId = 0;
    unsigned short m_ucCarChannelNums = 0;
    unsigned char m_u3cReserve2[3] ;
    std::vector< TB4ChannelData> m_vecInfoData ;
};
//B4帧统计事件
struct TB4TransientEvent{
    unsigned short m_usBeginBit = 0;
    unsigned char m_ucSerialNum = 0;
    unsigned char m_ucCommandNum = 0;
    unsigned char m_ucSubComNum = 0;
    unsigned char m_ucStatusBit = 0;
    unsigned short m_usMsgLen = 0;
    // std::vector< TB4MsgContent> m_tMsgContent ;
    TB4MsgContent m_tMsgContent ;
    unsigned char m_ucCheck = 0;
    unsigned char m_ucCutoffBit = 0;
};


//B5 通用异常事件信息
struct TB5ExceptTargetData{
    unsigned short m_usTargetId = 0;
    unsigned short m_usReservel = 0;
};
//B5 通用异常事件信息
struct TB5ExceptEventMsg{
    unsigned int m_unExceptId = 0;
    unsigned char m_ucExceptType = 0;
    unsigned char m_ucExceptOfStation = 0;
    unsigned char m_ucExceptOfChannel = 0;
    unsigned char m_ucExceptTargetNums = 0;
    std::vector< TB5ExceptTargetData> m_vecExceptTarget ;       //??????;
    unsigned int m_unExceptLong = 0;
    unsigned int m_unExceptLat = 0;
    unsigned int m_unExceptBeginTimeS = 0;
    unsigned int m_unExceptBeginTimeUs = 0;
    unsigned int m_unReservel = 0;
};
//B5帧消息内容
struct TB5MsgContent{
    unsigned short m_usEquiqId = 0;
    unsigned short m_usReserve1 = 0;
    unsigned int m_unStartTimeStampS = 0;
    unsigned int m_unStartTimeStampUs = 0;
    unsigned char m_ucExceptEentNums = 0;
    unsigned short m_usReserve2 = 0;
    std::vector< TB5ExceptEventMsg> m_vecInfoData ;
    unsigned int m_unReserve3 = 0;
};
//B5交通异常事件
struct TB5TransientEvent{
    unsigned short m_usBeginBit = 0;
    unsigned char m_ucSerialNum = 0;
    unsigned char m_ucCommandNum = 0;
    unsigned char m_ucSubComNum = 0;
    unsigned char m_ucStatusBit = 0;
    unsigned short m_usMsgLen = 0;
    // std::vector< TB5MsgContent> m_tMsgContent;
    TB5MsgContent m_tMsgContent;
    unsigned char m_ucCheck = 0;
    unsigned char m_ucCutoffBit = 0;
};

#endif