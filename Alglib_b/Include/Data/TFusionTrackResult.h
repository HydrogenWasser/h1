#ifndef TFUSIONTRACKRESULT_H
#define TFUSIONTRACKRESULT_H
#include <vector>
#include <string>

// 融合跟踪的目标框信息结构体
struct TFusionTrackBoxInfo
{
    uint16_t m_usGlobalId;           // 表述的为多基站情况下全域ID
    uint16_t m_usSingleId;           // 单基站检测的ID
    std::string m_strClass;          // 目标类型(点云算法传出的类型与视频传出的类型映射关系不同,所以此处采用字符串形式统一)
    unsigned char m_ucConfidence;    // 置信度
    unsigned char m_ucColor;         // 颜色
    unsigned char m_ucSource;        // 0 — 激光、1 — 视频、2 — 视频激光融合
    unsigned char m_ucSignFlag;      // 坐标符号标识（bit0、bit1、bit2分别代表X轴、Y轴、Z轴坐标值符号位：0表示有符号，1表示无符号，主要用于在坐标值超出其类型范围时，依据此取值）
    unsigned char m_ucStationSource; // 基站来源（1~255，若是高速公路，按行驶方向递增，若是路口，按照与正北顺时针夹角从小到大递增）
    double m_dLon;                   // 经度
    double m_dLat;                   // 纬度
    int16_t m_usElevation;           // 海拔:cm
    int16_t m_usSpeed;               // 速度：cm/s
    int16_t m_usCourseAngle;         // 航向角
    uint16_t m_usLength;             // 长度
    uint16_t m_usWidth;              // 宽度
    uint16_t m_usHeight;             // 高度
    int16_t m_sXCoord;               // X轴坐标:cm
    int16_t m_sYCoord;               // Y轴坐标:cm
    int16_t m_sZCoord;               // Z轴坐标:cm
    unsigned char m_ucLanId;         // 车道号

    uint64_t m_ulOccurTime;       // 目标第一次出现的时间戳
    uint32_t m_uiTimeSinceUpdate; //
    uint64_t m_ulHits;            // 击中次数
    bool m_bDeleteFlag;           // 是否删除该轨迹

    uint16_t m_usVideoInfo[20];    // 视频目标框相关信息
    unsigned char m_ucReserve[10]; // 预留10字节
    // 原始数据 n*16     后五项暂时存在 m_ucReserve中
    // x y 宽 长 航向角 z 高 类型id 速度 id 置信度 击中次数 0 0 0 0
};

// 点云识别结果数据结构体
struct TFusionTrackResult
{
    std::string m_strTerminalId;    // 终端ID，用于kafka识别基站等用途
    uint32_t m_unFrameId;           // 结果数据帧号
    unsigned char m_ucLidarId;      // 雷达编号
    unsigned char m_ucResultSource; // 识别结果数据来源：1：在线雷达数据；2：离线pcap文件；3：预留
    double m_dLidarLon;             // 激光器原点经度
    double m_dLidarLat;             // 激光器原点纬度
    float m_fLidarNorthAngle;       // 激光器与正北方向夹角
    uint16_t m_usBoxNum;            // 交通参与者(即识别出的目标框)数量

    // 存放各个阶段各种数据的时间戳或者过程耗时信息：ms，从前往后依次是：
    // 0.对应帧点云数据包0度的时间戳，用于后续融合算法匹配视频结果
    // 1.点云算法活动获取到该帧点云时的时间戳，用于观察点云数据发布订阅过程的延时
    // 2.点云算法活动发布结果数据出去时的时间戳
    // 3.融合算法活动获取到该帧点云结果时的时间戳，用于观察点云结果数据发布订阅过程的延时
    // 4.融合算法活动获取到该帧视频结果时的时间戳，用于观察视频结果数据发布订阅过程的延时
    // 5.融合算法活动发布结果数据出去时的时间戳
    // 预留
    uint64_t m_ulBufTimeStamp[10];

    // 存放各个阶段各种过程耗时信息：ms，从前往后依次是:
    // 0.纯点云算法（检测+跟踪）执行的耗时
    // 1.纯融合算法执行的耗时
    // 预留
    uint32_t m_unBufDelay[5];

    // 存放各个阶段的各种数据的帧率，从前往后依次是：
    // 0.原始点云的输入帧率，表示点云帧实际输入到点云算法处理活动的帧率
    // 1.点云结果的输出帧率，表示点云算法处理活动最终输出结果的帧率(考虑到出现算法耗时多导致处理不及时，进而丢帧的可能)
    // 2.点云结果的输入帧率，表示点云结果帧实际输入到融合算法处理活动的帧率
    // 3.视频结果的输入帧率，表示视频结果帧实际输入到融合算法处理活动的帧率
    // 4.融合结果的输出帧率，表示融合算法处理活动最终输出融合后结果的帧率(考虑到出现算法耗时多导致处理不及时，进而丢帧的可能)
    // 预留
    float m_fBufFps[10];

    std::vector<TFusionTrackBoxInfo> m_vecBox; // 目标框集
};

#endif