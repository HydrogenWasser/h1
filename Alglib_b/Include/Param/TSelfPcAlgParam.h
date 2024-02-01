/*******************************************************
 文件：TSelfPcAlgParam.h
 作者：
 描述：点云算法内部使用的参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/
#pragma once
#include "TLidarParam.h"
#include "TPcAlgParam.h"

//点云算法参数结构体(需兼容多雷达)
struct TSelfPcAlgParam
{
	TSelfPcAlgParam() : m_nLidarIndex(0) {}
	virtual ~TSelfPcAlgParam(){}

	int				m_nLidarIndex;			//雷达序号，可根据该序号对指定雷达数据做处理
	// std::string		m_strCfgPath;			//配置文件所在目录
	TLidarParam     m_stLidarParam;			//雷达参数结构体
	TPcAlgParam     m_stPcAlgParam;			//点云算法参数结构体	
	// int 			m_PcProcessNum;			//点云处理选择
	// int 			m_PcDetectNum;			//点云检测选择
	// int 			m_PcTrackNum;			//点云跟踪选择
	std::string		m_strRootPath;			//算法配置文件根目录路径
};
