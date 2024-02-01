/*******************************************************
 文件：TFusionAlgParam.h
 作者：
 描述：融合算法参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TFUSIONPARAM_H
#define TFUSIONPARAM_H

#include <iostream>
#include <vector>
#include "TVideoCropRect.h"

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_FUSIONALG		"getParam_fusionAlg"			//C端发送命令号，查询融合算法参数内容
#define TOPIC_GETPARAMREPLY_FUSIONALG	"getParamReply_fusionAlg"		//S端发送命令号，返回融合算法参数内容
#define TOPIC_SETPARAM_FUSIONALG		"setParam_fusionAlg"			//C端发送命令号，设置融合算法参数内容
#define TOPIC_SETPARAMREPLY_FUSIONALG	"setParamReply_fusionAlg"		//S端发送命令号，返回设置成功消息

//融合算法参数结构体
struct TFusionAlgParam
{
	std::vector<TVideoCropRect> m_vecCropRect;		//视频界面框选的裁剪矩形框
	/*******融合配置文件路径*******/
	std::string m_strFusionConfPath;
	std::vector<std::string> 	m_vecPcClass;
	std::vector<std::string> 	m_vecVideoClass;
	std::vector<std::string> 	m_vecFusionClass;

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TFusionAlgParam);
		int l_nSizeTemp = 0;
		for (size_t i = 0; i < m_vecCropRect.size(); i++){
			l_nSizeTemp += m_vecCropRect[i].GetSize();
		}
		return l_nBaseSize + l_nSizeTemp;
	}
};

#endif
