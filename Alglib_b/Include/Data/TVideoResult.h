/*******************************************************
 文件：TVideoResult.h
 作者：
 描述：视频识别结果数据结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TVIDEORESULT_H
#define TVIDEORESULT_H

#include <vector>
#include <string>

//定义FastDDS通信的topic
// #define TOPIC_GETDATA_VIDEORESULT		"getData_videoResult"			//C端发送命令号，查询视频识别结果数据
#define TOPIC_GETDATAREPLY_VIDEORESULT	"getDataReply_videoResult"		//S端发送命令号，回复视频识别结果数据
#define TOPIC_GETDATAREPLY_VIDEOFUSIONRESUL "getDataReply_videoFusionResult"		//S端发送命令号，回复视频识别结果数据

//视频识别的目标框信息结构体
struct TVideoBoxInfo
{
    unsigned short 	m_usGlobalId;				//表述的为多基站情况下全域ID	
	unsigned short 	m_usSingleId;				//单基站检测的ID
	std::string 	m_strClass;					//目标类型(点云算法传出的类型与视频传出的类型映射关系不同,所以此处采用字符串形式统一)
	unsigned char 	m_ucConfidence;			    //置信度
	unsigned char 	m_ucColor;					//颜色
	float 			m_fTopLeftX;				//X轴坐标:m
	float 			m_fTopLeftY;				//X轴坐标:m
	float 			m_fBottomRightX;			//Y轴坐标:m/*  */
	float 			m_fBottomRightY;			//Y轴坐标:m
	unsigned char 	m_ucReserve[10];			//预留10字节

	short           m_sXCoord;
	short           m_sYCoord;
	short           m_sZCoord;
	unsigned short  m_usLength;
	unsigned short  m_usWidth;
	unsigned short  m_usHeight;
	short           m_sTheta;
	short           m_sSpeed;
	short           m_sDataSource;
	short           m_sChannel;
};

//单路视频识别结果数据结构体
struct TVideoResult
{
	unsigned long 				m_ulTimeStamp;			//本路相机图像自身的时间戳转换到主机时间轴：ms
	unsigned long 				m_ulRtpTimeStamp;		//本路相机图像自身的Rtp时间戳（从rtsp协议中解析得到）：ms
	unsigned char 				m_ucCameraId;   		//相机编号
	unsigned short				m_usBoxNum;				//交通参与者(即识别出的目标框)数量
	std::vector<TVideoBoxInfo> 	m_vecBox;				//目标框集
};
 
//多路视频识别结果数据集合结构体	// 视频算法runAlg输出类型
struct TAllVideoResult
{
    unsigned int 				m_unFrameId;			//结果数据帧号	
    unsigned char 				m_ucResultSource;   	//识别结果数据来源：1：在线相机数据；2：离线avi文件；3：预留

	//存放各个阶段各种数据的时间戳或者过程耗时信息：ms，从前往后依次是：
	//0.与点云匹配好的点云0度数据包的时间戳（用于融合算法匹配点云结果）、
	//1.视频算法活动获取到该帧时间匹配好的视频数据时的时间戳（用于观察视频数据发布订阅过程的延时）、
	//2.视频算法活动发布结果数据出去的时间戳、
	//3.融合算法活动获取到该帧视频结果时的时间戳，(回调)
	//4.其他活动获取到融合后的视频识别结果数据的时间戳
    //预留
	unsigned long  				m_ulBufTimeStamp[10];

	//存放各个阶段各种过程耗时信息：ms，从前往后依次是:
    //0.纯视频算法执行的耗时、
    //1.订阅到匹配好的视频数据的延时(订阅到的匹配好的视频数据时的时间戳-接收的第一路相机图片的时间戳(未使用485时，该时间戳是主机时间-130ms）)
    //2.视频算法进程总延时(视频结果发布时间戳-接收的第一路相机图片的时间戳(未使用485时，该时间戳是主机时间-130ms）)
    //预留
    unsigned int                m_unBufDelay[5];

	//存放各个阶段的各种数据的帧率，从前往后依次是：
	//0.时间匹配好的视频数据的输入帧率，表示匹配好的视频数据实际输入到视频算法处理活动的帧率
	//1.视频识别结果的输出帧率，表示视频算法处理活动最终输出结果的帧率(考虑出现算法耗时多导致处理不及时，进而丢帧的可能)
    //2.视频结果输入到融合算法活动的输入帧率(回调函数拿到数据时填充)
	//3.融合后的视频识别结果数据输入到其他活动中的输入帧率
	//预留
	float						m_fBufFps[5];

	std::vector<TVideoResult> 	m_vecResult;			//单路相机识别结果集
};


#endif
