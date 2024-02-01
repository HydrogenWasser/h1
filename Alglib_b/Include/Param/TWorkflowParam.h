/*******************************************************
 文件：TWorkflowParam.h
 作者：
 描述：工作流参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TWORKFLOWPARAM_H
#define TWORKFLOWPARAM_H

#include <vector>
#include <string>

//定义FastDDS通信的topic
#define TOPIC_SETPARAM_WORKFLOWPARAM				"setParam_workflowParam"				//C端发送命令号，设置服务端当前工作流所有参数，目前只允许设置工作流的运行状态
																							//将来可以扩展增加、删除当前工作流中服务器或活动，但不可修改它们的名字

#define TOPIC_SETPARAMREPLY_WORKFLOWPARAM			"setParamReply_workflowParam"			//S端发送命令号，返回设置成功的消息

#define TOPIC_GETPARAM_WORKFLOWPARAM				"getParam_workflowParam"				//C端发送命令号，查询服务端当前工作流所有参数
#define TOPIC_GETPARAMREPLY_WORKFLOWPARAM			"getParamReply_workflowParam"			//S端发送命令号，返回服务端当前工作流所有参数


#define PUBLIC_DOMAIN_ID 1	//公用域ID，其他域ID不能与该ID相同

//活动参数结构体
struct TActivityParam
{
	std::string 			m_strActivityName;		//活动名称
	std::vector<int32_t>	m_vecFastddsDomainId;	//Fastdds域ID，有些活动可能拥有多个域
	int16_t					m_eActStatus;			//活动的工作状态,对外传递时使用int16_t，内部使用时转换成枚举WORK_STATUS类型

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TActivityParam);
        int l_nSizeTemp = m_vecFastddsDomainId.size() * sizeof(int32_t);
		return l_nBaseSize + l_nSizeTemp;
	}
};

//服务器参数结构体
struct TServiceInfo
{
	std::string 				m_strServiceName;		//服务器名称
	int16_t		 				m_eServiceStatus;		//服务器的工作状态,对外传递时使用int16_t，内部使用时转换成枚举WORK_STATUS类型
	std::vector<int32_t>		m_vecFastddsDomainId;	//Fastdds域ID，有些服务器可能拥有多个域
	std::vector<TActivityParam> m_vecActivityParam; 	//活动参数

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TServiceInfo);
		int l_nStrSize = m_strServiceName.length();
        int l_nSizeTemp = m_vecFastddsDomainId.size() * sizeof(int32_t);
		for (size_t i = 0; i < m_vecActivityParam.size(); i++)
		{
			l_nSizeTemp += m_vecActivityParam[i].GetSize();
		}
		
		return l_nBaseSize + l_nStrSize + l_nSizeTemp;
	}
};

//工作流参数结构体
struct TWorkflowParam
{
	std::string 				m_strSource;			//参数来源：显示端、接口服务器、点云算法服务器等，订阅方根据来源决定是否使用该参数
	std::string 				m_strWorkflowName;		//工作流名称
	int16_t		 				m_eWfStatus;			//工作流的工作状态,对外传递时使用int16_t，内部使用时转换成枚举WORK_STATUS类型
	std::vector<std::string>	m_vecAllWorkflowNames;	//当前机器内所有工作流的名称
	std::vector<TServiceInfo> 	m_vecServiceInfo;		//服务器信息

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TWorkflowParam);
		int l_nStrSize = m_strSource.length() + m_strWorkflowName.length();
        int l_nSizeTemp = 0;
		for (size_t i = 0; i < m_vecAllWorkflowNames.size(); i++)
		{
			l_nSizeTemp += m_vecAllWorkflowNames[i].length();
		}
		for (size_t i = 0; i < m_vecServiceInfo.size(); i++)
		{
			l_nSizeTemp += m_vecServiceInfo[i].GetSize();
		}
		return l_nBaseSize + l_nStrSize + l_nSizeTemp;
	}
};

#endif
