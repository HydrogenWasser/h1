/*******************************************************
 文件：TAllParam.h
 作者：
 描述：合并所有参数体到该参数结构体中
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TALLPARAM_H
#define TALLPARAM_H

#include "TStationParam.h"
#include "TStartControlParam.h"
#include "TWorkflowParam.h"
#include "TAuthorizationParam.h"
#include "TLidarParam.h"
#include "TCameraParam.h"
#include "TPcAlgParam.h"
#include "TVideoAlgParam.h"
#include "TFusionAlgParam.h"
#include "TNetParam.h"
#include "TTrigAlgParam.h"

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_ALL			"getParam_all"            //C端发送命令号，查询调试参数
#define TOPIC_GETPARAMREPLY_ALL		"getParamReply_all"       //S端发送命令号，返回调试参数
#define TOPIC_SETPARAM_ALL			"setParam_all"            //C端发送命令号，设置调试参数
#define TOPIC_SETPARAMREPLY_ALL		"setParamReply_all"       //S端发送命令号，返回设置成功消息

//所有参数结构体的集合体
struct TAllParam
{
	TStationParam 				m_stStationParam;			//基站固有参数
	TStartControlParam 			m_stStartControlParam;		//启动控制参数
	TWorkflowParam 				m_stWorkflowParam;			//工作流参数
	TAuthorizationParam 		m_stAuthorizationParam;		//授权管理参数
	TLidarParam 				m_stLidarParam;				//雷达参数
	TCameraParam 				m_stCameraParam;			//相机参数
	TPcAlgParam 				m_stPcAlgParam;				//点云算法参数
	TVideoAlgParam 				m_stVideoAlgParam;			//视频算法参数
	TFusionAlgParam 			m_stFusionAlgParam;			//融合算法参数
	TNetParam 					m_stNetParam;				//转发相关的网络参数
    TTrigAlgParam               m_stTrigAlgParam;           //触发算法参数 add 2021.11.16
	std::vector<std::string> 	m_vecTopic;					//记录改变的参数对应的topic，则订阅方可以据此来决定修改本地的哪些参数

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TCameraParam);
		int l_nStrTemp = 0;
		for (size_t i = 0; i < m_vecTopic.size(); i++)
		{
			l_nStrTemp += m_vecTopic[i].length();
		}

		int l_nSizeTemp = m_stStationParam.GetSize()
						+ m_stStartControlParam.GetSize()
		 				+ m_stWorkflowParam.GetSize()
		  				+ m_stAuthorizationParam.GetSize()
		   				+ m_stLidarParam.GetSize()
		    			+ m_stCameraParam.GetSize()
			 			+ m_stPcAlgParam.GetSize()
			  			+ m_stVideoAlgParam.GetSize()
			   			+ m_stFusionAlgParam.GetSize()
			    		+ m_stNetParam.GetSize()
                        + m_stTrigAlgParam.GetSize();
		
		return l_nBaseSize + l_nStrTemp + l_nSizeTemp;
	}
};


#endif
