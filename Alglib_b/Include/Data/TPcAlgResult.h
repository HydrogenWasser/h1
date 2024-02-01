/*******************************************************
 文件名：TPcAlgResult.h
 作者：
 描述：原始点云算法结果结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TPCALGRESULT_H
#define TPCALGRESULT_H

#include <vector>
#include <string>
#include "TPcSrcData.h"
#include "TPcResult.h"
//定义FastDDS通信的topic
#define TOPIC_GETDATA_PCSRCALGRESULT				"getData_pcSrcAlgResult"			//C端发送命令号，查询点云识别结果
#define TOPIC_GETDATAREPLY_PCSRCALGRESULT			"getDataReply_pcSrcAlgResult"		//S端发送命令号，回复点云识别结果
#define TOPIC_GETDATAREPLY_PCALGDEALSRCDATA			"getDataReply_pcAlgDealSrcData"		//S端发送命令号，回复点云识别结果

struct TPcSrcAlgResult
{
	std::vector<float> m_vecPcRecogResult;	//点云识别结果原始值 n*16
	unsigned short m_usResultColumns = 16;
	unsigned long long	m_ulTimeStamp;			//时间戳：ms，对应点云数据包0度的时间
};

struct TPcAlgDealSrcData
{
	std::vector<float>  m_vecSrcData;		//处理后点云原始数据 x y z i    n*4
	unsigned short m_usSrcDataColumns = 4;
};

struct TPcAlgResult
{
	TPcSrcData 		m_tPcSrcData;		//处理后点云原始数据结构体 
	TPcResult 		m_tPcResult;		//点云识别结果
	TPcSrcAlgResult m_tPcSrcAlgResult;	//原始点云算法结果结构体
};

#endif
