/*******************************************************
 文件：TFusionSrcData.h
 作者：
 描述：视频原始数据结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TFUSIONSRCDATA_H
#define TFUSIONSRCDATA_H

#include "TVideoSrcData.h"
#include "TVideoResult.h"
#include "TPcAlgResult.h"

struct TFusionSrcData
{
	// CVideoSrcDataTimematch m_cVideoSrcData;		//视频原始数据结构体
	TVideoResult m_cVideoSrcAlgResult;	//视频原始算法结构体
	TPcAlgResult m_cPcAlgResult;				//点云算法结构体
};


#endif
