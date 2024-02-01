/*******************************************************
 文件名：TPcSrcData.h
 作者：
 描述：点云原始数据结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TPCSRCDATA_H
#define TPCSRCDATA_H

#include <vector>
#include <string>

//定义FastDDS通信的topic
// #define TOPIC_GETDATA_PCSRC			    "getData_pcSrc"                 //C端发送命令号，查询点云数据
#define TOPIC_GETDATAREPLY_PCSRC	        "getDataReply_pcSrc"            //点云数据TPcSrcData，显示端订阅该topic
#define TOPIC_GETDATAREPLY_PCSRC_TO_ALG	    "getDataReply_pcSrc_to_Alg"     //点云数据TPcSrcData,算法活动订阅该topic，与显示端区别开，避免多个显示端连接后导致发布耗时增大，进而影响服务端效率
#define TOPIC_GETDATAREPLY_PCSRC_ORG_TO_ALG	"getDataReply_pcSrc_org_to_Alg" //未经处理的点云数据TPcSrcData
#define TOPIC_GETDATAREPLY_PCSRCINFO        "getDataReply_pcSrcInfo"        //S端发送命令号，回复纯粹的点云信息TPcSrcInfo，不包含点云数据
#define TOPIC_GETDATAREPLY_PCSRCPACKET      "getDataReply_pcSrcPacket"      //雷达原始数据包结构体TPcSrcPacketData
#define TOPIC_GETDATAREPLY_PCSRC_TIMEMATCH  "getDataReply_pcSrc_timeMatch"  //点云离线读取活动订阅客户端发布的时间对齐的离线点云数据

const float c_fAngleIntervalPerPackage = 360.0/180;                 //10Hz雷达每两包数据包之间的角度间隔，单位度，每一包数据包里有10个数据块，每个数据块均分每包的角度值
const float c_fTimeIntervalPerPackage = 100.0/180;                  //10Hz雷达每两包数据包之间的时间间隔，单位ms

//点云点的结构体
struct TPcPoint
{
    float m_fX;             // 点的x坐标值,单位m
    float m_fY;             // 点的y坐标值,单位m
    float m_fZ;             // 点的z坐标值,单位m
    float m_fIntensity;     // 点的强度值
    float m_fConfidence;    // 点的置信度
};

//点云原始数据相关信息结构体
struct TLidarInfo
{
    unsigned char                m_ucLidarId;   //雷达编号
    unsigned char                m_ucPcSource;  //雷达数据来源：1：在线雷达输出；2：离线pcap文件；3：预留
    std::string                  m_strIp;       //雷达ip
    bool                         m_bStatus;     //雷达数据来源：1：在线；2：离线
    double                       m_dLon;        //雷达经度
    double                       m_dLat;        //雷达纬度
    float                        m_fAngle;      //雷达北向夹角
    unsigned int                 m_uiPointNum;  //雷达点数
    uint16_t                     m_usLineNum;   //雷达线束
};


//点云原始数据相关信息结构体，跟视频做时间匹配，使用该结构体即可
struct TPcSrcInfo
{
    unsigned int                 m_unFrameId;   //帧号
    unsigned char                m_ucLidarId;   //雷达编号
    unsigned char                m_ucPcSource;  //雷达数据来源：1：在线雷达输出；2：离线pcap文件；3：预留
    std::vector<TLidarInfo>      m_vecLidaInfo;        //雷达信息

    //存放各个阶段各种数据的时间戳：ms，从前往后依次是：
    //0.点云0度数据包的时间戳
    //1、2、3、4依次是：雷达点云旋转到对应相机角度时的pcap数据包到达网卡的时间戳(点云与视频进行时间匹配时，使用该时间戳匹配)
    //5.点云发布出去的时间戳
    //6.原始点云帧输入到点云算法活动的输入时间戳(回调)
    //7.10字节存放雷达产生的时间戳
    //9. 组帧完成时间戳
    //预留
    unsigned long               m_ulBufTimeStamp[10];

    //存放各个阶段各种过程耗时信息：ms，从前往后依次是:
    //0.点云预处理算法的耗时
    //预留
    unsigned int                m_unBufDelay[3];

    //存放各个阶段的各种数据的帧率，从前往后依次是：
    //0.原始点云帧的输入帧率，表示从雷达输入数据的帧率
    //1.原始点云帧的输出帧率，表示点云读取活动最终输出的帧率（考虑出现发布耗时多导致队列满，进而丢帧的可能）
    //2.原始点云帧输入到点云算法活动的输入帧率(回调)
    //预留
	float			            m_fBufFps[3];		
};

//点云原始数据结构体
struct TPcSrcData
{
    TPcSrcInfo              m_stSrcInfo;        //点云数据的相关信息
    unsigned int            m_unPointNums;      //点数量
    std::vector<uint32_t>   m_vecPointNums;     //各雷達点数量
    std::vector<TPcPoint>   m_vecPoints;        //点集
};

//时间对齐的多路点云数据，匹配时间戳以0号雷达的0度数据包的时间戳为基准
struct TPcSrcDataTimeMatch
{
	std::vector<TPcSrcData>    m_vecPcSrcData;
};

//雷达原始数据包结构体
struct TPcSingleData
{
    //一帧点云的原始数据包，每个元素都是1350字节的包数据转换成的string
    std::vector<std::string>   m_vecPacketData;
};

//雷达原始数据包结构体
struct TPcSrcPacketData
{
    //存放各个阶段各种数据的时间戳：ms，从前往后依次是：
    //0.点云0度数据包的时间戳
    //1.点云组帧完成的时间戳
    //预留
    unsigned long               m_ulBufTimeStamp[10];

    //一帧点云的原始数据包，每个元素都是1350字节的包数据转换成的string
    std::vector<TPcSingleData>  m_vecSingleData;
};

#endif
