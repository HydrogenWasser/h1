#pragma once

#include <vector>
#include <string>

//定义FastDDS通信的topic
#define TOPIC_GETDATAREPLY_EVENTB5DATA		    "getDataReply_EventB5Data"				//S端发送命令号，回复Radar结果
#define TOPIC_GETDATA_EVENTB5DATA               "getData_EventB5Data"                   //C端发送命令号，查询radar识别结果

#define MAX_DEVICE_COUNT 16
#define MAX_EVENTB5_COUNT 32
#define MAX_EVENT_COUNT 256
#define MAX_TARGET_COUNT 2048
#define MAX_RADAR_COUNT 4
#define MAX_CAMERA_COUNT  16
#define MAX_STATION_COUNT 16
#define MAX_RADARDEVICE_LENGTH 30
#define MAX_CAMERADEVICE_LENGTH 30


struct TEventB5Pos
{
    double dLongitude;                          //经度
    double dLatitude;                           //纬度
};

struct TEventB5Info
{
    unsigned long ulEventId;                    //事件ID
    uint8_t ocEventType;                          //事件类型
    std::string strSource;                           //事件来源
    std::string strSensor;                           //来源序列号拼串 0-激光，1-视频，2-视频激光融合，3-毫米波，4-视频毫米波融合
    std::vector<uint8_t> vecLaneNos;                 //车道号，事发点所在车道
    double dEventLongitude;                     //事件位置经度1e-7
    double dEventLatitude;                      //事件位置纬度1e-7
    double dEventAltitude;                      //事件位置高程 米
    unsigned long long ullEventTimestampStart;  //事件开始时间戳
    unsigned long long ullEventTimestampEnd;    //事件结束时间戳
    std::vector<unsigned long> vecRefVehicles;     //关联车辆 ["id1", "id2"]
    std::string strRadius;                           //事件影响半径 米
    std::vector<TEventB5Pos> vecPath;              //影响路径，事件区域描述，使用有序点集合描述事件影响路径
    std::string strImage;                            //事件截图，对图片jpg压缩后base64编码
};

struct TEventB5Data
{
    std::string strDeviceId;                         //设备ID
    unsigned long long ullTimestamp;            //时间戳
    uint8_t ocEventNum;                           //事件数量
    std::vector<TEventB5Info> vecB5EventInfo;      //B5事件信息
};
//************************B5 Event Data****************************



//基站相机信息
struct TCameraInfo
{
    uint64_t m_ulTimeStamp; //unix时间（单位：毫秒）1970年开始累积的秒数后的毫秒数
    uint8_t m_ucCameraID;   //相机ID
};


//事件描述信息
struct TEventInfo
{
    uint8_t m_ucEventInfoType;                         //0-事件前；1-事件中；2-事件后
    double m_fdLongitude;                              //事件位置经度
    double m_fdLatitude;                               //事件位置纬度
    uint16_t m_usLeftTopX;                             //车身框左上方x像素值；0表示无框
    uint16_t m_usLeftTopY;                             //车身框左下方y像素值；0表示无框
    uint16_t m_usRightDownX;                           //车身框右下方x像素值；0表示无框
    uint16_t m_usRightDownY;                           //车身框右下方y像素值；0表示无框
    uint8_t m_ucStationId;                             //基站ID
    uint8_t m_ucCameraId;                              //该目标所属的相机编号
    uint8_t m_ucCameraNum;                             //基站相机数量
    TCameraInfo m_vecCameraInfo[MAX_DEVICE_COUNT]; //基站相机信息集合
    uint64_t m_ulHeadCamTimeStamp;                     //车头方向ms级时间戳
    uint64_t m_ulBodyCamTimeStamp;                     //车身方向ms级时间戳
    uint64_t m_ulTailCamTimeStamp;                     //车尾方向ms级时间戳
    uint32_t m_uiGlobalFrameNo;                        //全域数据帧号
    uint64_t m_ulGlobalTimeStamp;                      //全域上报时间
};

//目标信息
struct TTargetMsg
{
    uint32_t m_uiTargetId;                            //目标全域ID
    uint16_t m_uiTargetType;                          //目标类型
    uint8_t m_ucEventConfidence;                      //事件置信度
    uint8_t m_ucEventInfoNum;                         //事件信息描述数量
    TEventInfo m_vecEventInfo[MAX_EVENTB5_COUNT];     //事件信息描述列表
    uint64_t m_length;                                 //mubiao chang
    uint64_t m_width;                                  //mubiao kuan
    uint64_t m_height;                                 //mubiao gao
};


//目标信息
struct TB5LaneEvent
{
    uint64_t m_ulB5EventId;                            //事件编码
    uint16_t m_usBaseLocation;                         //目标所在方位
    uint8_t m_ucB5EventType;                           //事件类型
    uint8_t m_ucStationId;                             //基站ID
    uint8_t m_ucLaneNo;                                //车道号
    uint8_t m_ucTargetNum;                             //事件目标数量
    TTargetMsg m_vecTargetInfo[MAX_EVENTB5_COUNT]; //目标集合
    double m_fdEventLongitude;                         //事件位置经度
    double m_fdEventLatitude;                          //事件位置纬度
    double m_fdEventAltitude;                          //事件位置高程
    uint64_t m_ulStartTimeStamp;                       //事件开始时间戳:
    uint64_t m_ulEndTimeStamp;                         //事件结束时间戳:
    std::string m_strSource;                           //事件来源（福泉协议0-激光，1-视频，2-视频激光融合，3-毫米波，4-毫米波视频融合）
    uint8_t m_ucAction;                                //事件状态
    uint16_t m_usRadius;                               //事件影响半径
    std::string m_strSensor;                           //来源序列号拼串
    std::vector<TEventB5Pos> vecPath;                  //影响路径，事件区域描述，使用有序点集合描述事件影响路径
    std::string m_strImage;                            //事件截图，对图片jpg压缩后base64编码
};

//B5交通事件数据
struct TB5Data
{
    uint8_t            m_ucRadarNum;                         //毫米波雷达数量
    uint8_t            m_ucCameraNum;                        //相机数量
    char        m_arrayRadarDevieId[MAX_RADAR_COUNT][MAX_RADARDEVICE_LENGTH];   //毫米波雷达设备号
    char        m_arrayCameraDevieId[MAX_CAMERA_COUNT][MAX_CAMERADEVICE_LENGTH]; //相机设备号
    TB5LaneEvent m_vecB5LaneEvent[MAX_EVENTB5_COUNT]; //事件集合
    uint64_t m_ulTimeStamp;                               //统计截止时间戳
    uint64_t m_ulB5FrameId;                               //统计事件帧号
    uint16_t m_usDeviceId;                                //设备ID
    uint8_t m_ucEventNum;                                 //事件数量
};