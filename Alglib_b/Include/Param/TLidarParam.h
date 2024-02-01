/*******************************************************
 文件：TLidarParam.h
 作者：
 描述：激光雷达参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TLIDARPARAM_H
#define TLIDARPARAM_H

#include <vector>
#include <string>

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_LIDAR		"getParam_lidar"			//C端发送命令号，查询雷达参数内容
#define TOPIC_GETPARAMREPLY_LIDAR	"getParamReply_lidar"		//S端发送命令号，返回雷达参数内容
#define TOPIC_SETPARAM_LIDAR		"setParam_lidar"			//C端发送命令号，设置雷达参数内容
#define TOPIC_SETPARAMREPLY_LIDAR	"setParamReply_lidar" 		//S端发送命令号，返回设置成功消息

//网络地址结构体
// struct TAddress
// {
// 	bool 			bIpv6;		//是否是ipv6地址
// 	std::string 	m_strIp;	//ip地址,考虑ipv6,所以直接用字符串表示
// 	unsigned int 	m_unPort;	//端口号
// };

struct TLidarDev
{
	std::string			m_strSrcIp;	  		//雷达源IP
	std::string			m_strDstIp;	  		//雷达目的IP
	std::string			m_strRedisIp;	  	//雷达数据RedisIP
	std::string			m_strRedisTopic;	//订阅雷达数据的Topic
	std::string			m_strDataType;	  	//数据获取方式：1- pcap抓包2- udp接收3- redis订阅
	std::string			m_strLidarType;	  	//雷达类型
	std::string			m_strBmpPath1;		//Bmp
	std::string			m_strBmpPath2;		//Bmp2，配合扩大参数范围使用
	uint16_t			m_usPort;	  		//udp端口/redis端口/抓包目的端口
	uint8_t				m_ucLidarId;	  	//雷达ID
	double				m_dLon;	  			//雷达经度
	double				m_dLat;	  			//雷达纬度
	float				m_fAngle;	  		//雷达北向夹角
	bool 			    m_bRotationTranslation;	//是否旋转平移
	float				m_fSelfRotTrans[6];	//自身的调平参数及和主雷达的匹配参数，旋转xyz,平移xyz
	float				m_fRangeRotTrans1[6];	//扩大检测范围旋转平移预处理参数1，旋转xyz,平移xyz
	float				m_fRangeRotTrans2[6];	//扩大检测旋转平移预处理参数1，旋转xyz,平移xyz
	float				m_fUseAngle[4];	  	//角度限制,
	uint8_t				m_ucBuf[8];	  		//预留位
    std::vector<float> 	m_vecAngle;  		//对应各个相机的角度，从前往后，1，2，3，4

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TLidarDev);
		int l_strSize = m_strSrcIp.length() + m_strDstIp.length() + m_strRedisIp.length() + m_strRedisTopic.length();
		return l_nBaseSize + l_strSize + m_vecAngle.size() * sizeof(float);
	}
};

struct TLidarParam
{
	std::string	 			m_strNICname;		//网卡名，即从哪个网卡抓取雷达数据
	std::vector<TLidarDev> 	m_vecLidarDev;		//雷达设备

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TLidarParam);
		int l_strSize = m_strNICname.length();
		int l_nSizeTemp = 0;
		for (size_t i = 0; i < m_vecLidarDev.size(); i++)
		{
			l_nSizeTemp += m_vecLidarDev[i].GetSize();
		}
		
		return l_nBaseSize + l_strSize + l_nSizeTemp;
	}
};


#endif
