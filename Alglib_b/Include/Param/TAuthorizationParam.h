/*******************************************************
 文件：TAuthorizationParam.h
 作者：
 描述：用于授权的参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TAUTHORIZATIONPARAM_H
#define TAUTHORIZATIONPARAM_H

#include <string>

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_AUTHORIZATION			"getParam_Authorization"            //C端发送命令号，查询授权参数
#define TOPIC_GETPARAMREPLY_AUTHORIZATION		"getParamReply_Authorization"       //S端发送命令号，返回授权参数
#define TOPIC_SETPARAM_AUTHORIZATION			"setParam_Authorization"            //C端发送命令号，设置授权参数
#define TOPIC_SETPARAMREPLY_AUTHORIZATION		"setParamReply_Authorization"       //S端发送命令号，返回设置成功消息

enum ACTIVATION_CODE_ERROR_TYPE
{
	ACTIVATION_CODE_UNKNOW,			//未知
	ACTIVATION_CODE_ERROR,			//激活码错误
	ACTIVATION_CODE_REPEAT,			//激活码重复，即该激活码已用过，不能再次使用
	ACTIVATION_CODE_TIMEOUT,		//激活码超时，超过使用时效
	ACTIVATION_CODE_DEVICE_ERR		//设备错误，该设备无法被激活
};


//授权相关的参数结构体（若是1046/NX/FPGA这种分布式架构，则此结构体需做相应改动）
struct TAuthorizationParam
{
	unsigned short				m_usActivationType;		//激活类型，0：按时长激活，1：永久激活
	float		 				m_fDaysLeft;			//剩余使用天数，服务端查询剩余时长后填充（授权文件中保留的是分钟数，这里转换成天数），用于返回给显示端显示
	std::string					m_strLocalId;			//本机识别码，服务端根据绑定规则生成，返回给显示端显示
	std::string					m_strActivationCode;	//激活码，显示端填充该字符串，用于给服务器授权，授权时长的单位是分钟
	ACTIVATION_CODE_ERROR_TYPE 	m_eErrCode; 			//激活码的错误代码，初始要置为ACTIVATION_CODE_UNKNOW

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TAuthorizationParam);
		int l_strSize = m_strLocalId.length() + m_strActivationCode.length();
		return l_nBaseSize + l_strSize;
	}
};


#endif
