/*******************************************************
 文件：TSelfVideoAlgParam.h
 作者：
 描述：视频算法内部使用的参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/
#pragma once
#include <string>
#include "TCameraParam.h"
#include "TVideoAlgParam.h"

//视频算法参数结构体
struct TSelfVideoAlgParam
{
	TSelfVideoAlgParam() : m_nVideoIndex(0) {}
	virtual ~TSelfVideoAlgParam(){}

	int					m_nVideoIndex;				//当前选取的相机序号，可根据该序号对指定相机数据做处理
	// std::string			m_strCfgPath;				//配置文件所在目录
	TCameraParam  		m_stCameraParam;			//相机参数结构体
	TVideoAlgParam  	m_stVideoAlgParam;			//视频算法参数结构体
	int 				m_VideoDetectNum;			//视频检测选择
	int 				m_VideoTrackNum;			//视频跟踪选择
	std::string 		m_strVideoCfgPath;			//视频检测权重路径配置文件
	std::string			m_strRootPath;			//算法配置文件根目录路径
};
