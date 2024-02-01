/*******************************************************
 文件：TFusionResult.h
 作者：
 描述：视频识别结果数据结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TFUSIONRESULT_H
#define TFUSIONRESULT_H

#include <vector>
#include <string>
#include "TPcResult.h"
#include "TVideoResult.h"
#include "TFusionTrackResult.h"

struct TFusionOutputResult
{
    TFusionTrackResult  m_stFusionTrackResult;   //融合后的点云识别结果，以点云算法结果结构体为模板填充
    TAllVideoResult     m_stVideoFusionResult; // 融合后的视频识别结果，以视频算法结果结构体为模板填充
    std::vector<int>    m_vecFusionId;        // 融合id list
};

struct TFusionResult
{
	TPcResult 			m_stPcFusionResult; 	//融合后的点云识别结果，以点云算法结果结构体为模板填充
	TAllVideoResult 	m_stVideoFusionResult; 	//融合后的视频识别结果，以视频算法结果结构体为模板填充
	std::vector<int> 	m_vecFusionId;			//融合id list
};

#endif
