/*******************************************************
 文件：TVideoAlgParam.h
 作者：
 描述：视频算法参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TVIDEOALGPARAM_H
#define TVIDEOALGPARAM_H

#include <vector>
#include "TVideoCropRect.h"

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_VIDEOALG			"getParam_videoAlg"			//C端发送命令号，查询视频算法参数内容
#define	TOPIC_GETPARAMREPLY_VIDEOALG	"getParamReply_videoAlg"	//S端发送命令号，返回视频算法参数内容
#define	TOPIC_SETPARAM_VIDEOALG			"setParam_videoAlg"			//C端发送命令号，设置视频算法参数内容
#define	TOPIC_SETPARAMREPLY_VIDEOALG	"setParamReply_videoAlg"	//S端发送命令号，返回设置成功消息


//视频算法参数结构体
struct TVideoAlgParam
{
	unsigned int 				m_unResizeVideoWidth;		//算法使用的图片宽度
	unsigned int 				m_unResizeVideoHeight;		//算法使用的图片高度
	std::vector<int> 			m_vecUsedCameraId;			//当前使用所有相机的Id
	std::vector<TVideoCropRect> m_vecCropRect;				//界面框选的裁剪矩形框
	std::vector<std::string>	m_vecVideoClass;			//视频检测类别
	// std::string					m_Weight_Path;				// 视频原始权重路径， 非engine
	// std::string					m_NET_TYPE = "s";			// 视频算法网络类型
	// float 						m_NMS_THRESH = 0.4;			// NMS阈值
	// float 						m_CONF_THRESH = 0.5;		// conf阈值
	// int 						m_MAX_BATCH_SIZE = 4;		// 模型最大输入batch
	
	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TVideoAlgParam);
        int l_nSizeTemp = m_vecUsedCameraId.size() * sizeof(int);
		for (size_t i = 0; i < m_vecCropRect.size(); i++)
		{
			l_nSizeTemp += m_vecCropRect[i].GetSize();
		}
		return l_nBaseSize + l_nSizeTemp;
	}
};


#endif
