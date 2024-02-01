/*******************************************************
 文件：TStartControlParam.h
 作者：
 描述：启动控制参数，用于控制服务器程序的启动流程
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TSTARTCONTROLPARAM_H
#define TSTARTCONTROLPARAM_H

#include <string>

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_STARTCONTROL			"getParam_startControl"            //C端发送命令号，查询启动控制参数
#define TOPIC_GETPARAMREPLY_STARTCONTROL	"getParamReply_startControl"       //S端发送命令号，返回启动控制参数
#define TOPIC_SETPARAM_STARTCONTROL			"setParam_startControl"            //C端发送命令号，设置启动控制参数
#define TOPIC_SETPARAMREPLY_STARTCONTROL	"setParamReply_startControl"       //S端发送命令号，返回设置成功消息

//启动控制相关的参数结构体
struct TStartControlParam
{
    bool 			m_bDebug;				//是否调试
	bool 			m_bFusion;				//是否融合
	bool 			m_bTestFusion;			//是否测试融合,启用则表示：即使界面勾选了融合，依然转发纯点云识别结果
	bool 			m_bAutoStartWorkflow;	//是否自动启动工作流
	bool 			m_bAutoConnectDev;		//是否自动连接设备
	bool 			m_bWriteLog;			//是否写日志文件
	bool 			m_bPrintLog;			//是否打印日志
	unsigned char 	m_ucLogLevel;			//日志等级
	std::string		m_strDefaultWfName;		//默认工作流

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TStartControlParam);
		int l_strSize = m_strDefaultWfName.length();
		return l_nBaseSize + l_strSize;
	}
};


#endif
