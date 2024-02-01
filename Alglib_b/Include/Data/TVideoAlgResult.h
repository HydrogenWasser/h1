/*******************************************************
 文件名：TVideoSrcAlgResult.h
 作者：
 描述：原始视频结果结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TVIDEOSRCALGRESULT_H
#define TVIDEOSRCALGRESULT_H

#include <vector>
#include <string>

#include "TVideoResult.h"
//定义FastDDS通信的topic
#define TOPIC_GETDATA_ALLVIDEOSRCALGRESULT				"getData_allVideoSrcAlgResult"			//C端发送命令号，查询点云识别结果
#define TOPIC_GETDATAREPLY_ALLVIDEOSRCALGRESULT			"getDataReply_allVideoSrcAlgResult"		//S端发送命令号，回复点云识别结果

struct TVideoSrcAlgResult
{
	std::vector<float > vecVideoBoxesResult;
	std::vector<float > vecVideoScoreResult;
	std::vector<int >  vecVideoClassIDResult;	  
	std::vector<int > vecVideoTrackIdResult;

};
struct TAllVideoSrcAlgResult
{
	/* data */
	unsigned int 			unFrameId;				//结果数据帧号	
	unsigned long long		m_ulTimeStampMatch;		//与点云匹配好的时间戳：ms
    char  					ucResultSource;   		//识别结果数据来源：1：在线相机数据；2：离线avi文件；3：预留
	std::vector<TVideoSrcAlgResult> tVideoSrcAlgResult;
};

struct TVideoAlgResult
{
	TAllVideoResult m_vecVideoResult;			//视频识别结果
	TAllVideoSrcAlgResult m_tVideoSrcAlgResult;
	// bool m_bPcAlgSucFlag;
};



#endif
