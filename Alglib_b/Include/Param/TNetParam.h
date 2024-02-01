/*******************************************************
 文件：TNetParam.h
 作者：
 描述：转发相关的网络参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TNETPARAM_H
#define TNETPARAM_H

#include <vector>
#include <string>

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_FORWARDNET		"getParam_forwardNet"		//C端发送命令号，查询转发网络参数内容
#define TOPIC_GETPARAMREPLY_FORWARDNET	"getParamReply_forwardNet"	//S端发送命令号，返回转发网络参数内容
#define TOPIC_SETPARAM_FORWARDNET		"setParam_forwardNet"		//C端发送命令号，设置转发网络参数内容
#define TOPIC_SETPARAMREPLY_FORWARDNET	"setParamReply_forwardNet"	//S端发送命令号，返回设置成功消息

//网络地址结构体
struct TAddress
{
	bool 			bIpv6;		//是否是ipv6地址
	std::string 	m_strIp;	//ip地址,考虑ipv6,所以直接用字符串表示
	unsigned int 	m_unPort;	//端口号

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TAddress);
		int l_strSize = m_strIp.length();
		return l_nBaseSize + l_strSize;
	}
};

//网络参数结构体
struct TNetParam
{
	bool 						m_bSendSrcPc;						//是否发送原始点云
	bool 						m_bSendResult;						//是否发送结果数据
	unsigned int 				m_unUdpServicePort;					//UDP服务端口号
	std::vector<TAddress> 		m_vecForwardOrgSrcLidarAddress;		//转发的原始点云所属雷达源地址
	std::vector<TAddress> 		m_vecForwardOrgPcDataDstDev;		//转发原始点云的目的地址
	std::vector<TAddress> 		m_vecForwardResultDstDev;			//转发结果数据的目的地址（点云/融合识别结果协议）
	std::vector<TAddress> 		m_vecForwardResultToGlobalAppDstDev;//转发给全域软件的结果数据的目的地址(全域使用的协议和点云识别结果协议可能有差别)
	std::vector<TAddress> 		m_vecForwardTriggerSnapDstDev;		//转发触发抓拍数据（触发算法返回的字节流，发送给目的相机，用于通知相机抓拍）的目的地址
	std::vector<TAddress> 		m_vecTriggerSelectCameraDev;		//触发回传图像选择的源相机的ip地址（选择该ip对应的相机图像，处理后组织到最终要回传的信息结构中）
	std::vector<TAddress> 		m_vecForwardTriggerInfoDstDev;		//触发回传最终信息体的目的地址，http发送
	std::vector<TAddress> 		m_vecForwardEventDstDev;			//转发检测事件的目的地址
	std::vector<TAddress> 		m_vecKafkaAddress;					//kafka设备IP
	std::vector<std::string> 	m_vecKafkaTopic;					//kafka topic,按顺序:结果数据,原始点云
	std::vector<TAddress> 		m_vecSlaveStationAddress;			//从基站ip

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TNetParam);
		int l_nSizeTemp = 0;
		for (size_t i = 0; i < m_vecForwardOrgSrcLidarAddress.size(); i++)
		{
			l_nSizeTemp += m_vecForwardOrgSrcLidarAddress[i].GetSize();
		}
		for (size_t i = 0; i < m_vecForwardOrgPcDataDstDev.size(); i++)
		{
			l_nSizeTemp += m_vecForwardOrgPcDataDstDev[i].GetSize();
		}
		for (size_t i = 0; i < m_vecForwardResultDstDev.size(); i++)
		{
			l_nSizeTemp += m_vecForwardResultDstDev[i].GetSize();
		}
		for (size_t i = 0; i < m_vecForwardResultToGlobalAppDstDev.size(); i++)
		{
			l_nSizeTemp += m_vecForwardResultToGlobalAppDstDev[i].GetSize();
		}
		for (size_t i = 0; i < m_vecForwardTriggerSnapDstDev.size(); i++)
		{
			l_nSizeTemp += m_vecForwardTriggerSnapDstDev[i].GetSize();
		}
		for (size_t i = 0; i < m_vecTriggerSelectCameraDev.size(); i++)
		{
			l_nSizeTemp += m_vecTriggerSelectCameraDev[i].GetSize();
		}
		for (size_t i = 0; i < m_vecForwardTriggerInfoDstDev.size(); i++)
		{
			l_nSizeTemp += m_vecForwardTriggerInfoDstDev[i].GetSize();
		}
		for (size_t i = 0; i < m_vecForwardEventDstDev.size(); i++)
		{
			l_nSizeTemp += m_vecForwardEventDstDev[i].GetSize();
		}
		for (size_t i = 0; i < m_vecKafkaAddress.size(); i++)
		{
			l_nSizeTemp += m_vecKafkaAddress[i].GetSize();
		}
		for (size_t i = 0; i < m_vecSlaveStationAddress.size(); i++)
		{
			l_nSizeTemp += m_vecSlaveStationAddress[i].GetSize();
		}
		for (size_t i = 0; i < m_vecKafkaTopic.size(); i++)
		{
			l_nSizeTemp += m_vecKafkaTopic[i].length();
		}

		return l_nBaseSize + l_nSizeTemp;
	}
};

#endif
