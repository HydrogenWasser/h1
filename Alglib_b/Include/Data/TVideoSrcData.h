/*******************************************************
 文件：TVideoSrcData.h
 作者：
 描述：视频原始数据结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TVIDEOSRCDATA_H
#define TVIDEOSRCDATA_H

#include <vector>

//定义FastDDS通信的topic
// #define TOPIC_GETDATA_VIDEOSRC						"getData_videoSrc"						 //C端发送命令号，查询视频数据
#define TOPIC_GETDATAREPLY_VIDEOSRC						"getDataReply_videoSrc"					 //S端发送命令号，回复视频数据

// #define TOPIC_GETDATA_VIDEOSRC_TIMEMATCH				"getData_videoSrc_timeMatch"			 //C端发送命令号，查询视频数据
#define TOPIC_GETDATAREPLY_VIDEOSRC_TIMEMATCH			"getDataReply_videoSrc_timeMatch"		 //S端发送命令号，回复视频数据

//服务其内部算法活动订阅该topic，与显示端区别开，避免多个显示端连接后导致发布耗时增大，进而影响服务端效率
#define TOPIC_GETDATAREPLY_VIDEOSRC_TIMEMATCH_TO_ALG	"getDataReply_videoSrc_timeMatch_to_Alg" 


struct TVideoSrcData
{
	unsigned int 				m_unFrameId;				//一一匹配的帧号
	unsigned long 				m_ulTimeStamp;				//本路相机图像自身的时间戳转换到主机时间轴：ms
	unsigned long 				m_ulRtpTimeStamp;			//本路相机图像自身的Rtp时间戳（从rtsp协议中解析得到）：ms
	unsigned char 				m_unCameraId;				//相机编号
	unsigned char 				m_ucVideoSource;			//视频数据来源：1：在线摄像头输出；2：离线视频；3：预留
	unsigned short 				m_usBmpLength;				//图片长度
	unsigned short 				m_usBmpWidth;				//图片宽度
	unsigned int 				m_unBmpBytes;				//图片数据字节长度
    float                       m_fVideoSrcDataInputFps;   	//单路原始视频的输入帧率，表示从单路相机输入数据的帧率
	std::vector<unsigned char> 	m_vecImageBuf;				//获取的相机数据，是libav输出的图像数据，uinit8类型的数组，数组大小由图片宽高决定
};

struct TVideoSrcDataTimematch
{
	//存放各个阶段各种数据的时间戳：ms，从前往后依次是：
	//0.与点云匹配好的点云0度数据包的时间戳、
	//1.将该匹配好的视频数据发布出去时的时间戳
	//点云匹配好的原始视频的输入到视频算法活动中的输入时间戳(回调函数拿到数据时填充)
    //预留
    unsigned long               m_ulBufTimeStamp[10];

	//存放各个阶段各种过程耗时信息：ms，从前往后依次是:
    //0.时间匹配过程的耗时
    //预留
    unsigned int                m_unBufDelay[5];

	//存放各个阶段的各种数据的帧率，从前往后依次是：
	//0.时间匹配好的视频数据的输入帧率，表示匹配好的视频数据实际输入到视频算法处理活动的帧率
	//1.视频识别结果的输出帧率，表示视频算法处理活动最终输出结果的帧率(考虑出现算法耗时多导致处理不及时，进而丢帧的可能)
    //2.视频结果输入到融合算法活动的输入帧率(回调函数拿到数据时填充)
	//预留
	float						m_fBufFps[5];

	std::vector<TVideoSrcData> 	m_vecSrcData;				//与点云匹配好的每一路相机数据
};


#endif
