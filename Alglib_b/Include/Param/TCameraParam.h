/*******************************************************
 文件：TCameraParam.h
 作者：
 描述：相机参数据结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TCAMERAPARAM_H
#define TCAMERAPARAM_H

#include <vector>
#include <string>

// using namespace std; //不要像这样在头文件中直接使用命名空间

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_CAMERA			"getParam_camera"			//C端发送命令号，查询相机参数内容
#define TOPIC_GETPARAMREPLY_CAMERA		"getParamReply_camera"		//S端发送命令号，返回相机参数内容
#define TOPIC_SETPARAM_CAMERA			"setParam_camera"			//C端发送命令号，设置相机参数内容
#define TOPIC_SETPARAMREPLY_CAMERA		"setParamReply_camera"		//S端发送命令号，返回设置成功消息


struct T485Param
{
	bool 			m_bNetTranf;	//是否通过网络传输相机校准时间戳
	std::string 	m_strUsbDev;	//USB串口设备文件路径
	unsigned int 	m_unBaudRate;	//波特率
	float 			m_fTimeout;		//超时时间
	unsigned int 	m_unId;			//id,对应相机id
	std::string 	m_strIp;		//网络传输ip
    uint32_t 		m_unPort;		//网络传输port

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(T485Param);
		int l_strSize = m_strUsbDev.length() + m_strIp.length();
		return l_nBaseSize + l_strSize;
	}
};

//隧道項目新增的与视频算法相关的參數
struct TSuiDaoVideoParam
{
	int 			m_nBitStreamSize;	//字节流大小
	int 			m_nBorderSx;
	int 			m_nBorderSy;
	int 			m_nBorderEx;
	int 			m_nBorderEy;
	float 			m_fGroundZ;
	int 			m_nBorderY;
	int 			m_nKernelSize;
	int 			m_nBinaryThresh;
	float 			m_fPercentage;
	int 			m_nMaxAge;
	int 			m_nInitCount;
	int 			m_nOutputCount;
	int 			m_nFromAndTo;

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TSuiDaoVideoParam);
		return l_nBaseSize;
	}
};

struct TCameraDev
{
	std::string 		m_strCameraIp;				//相机IP
	unsigned char 		m_ucCameraId;				//相机序号
	std::string 		m_strCameraUser;			//相机用户名
	std::string 		m_strCameraPwd;				//相机用户密码
	std::vector<float> 	m_vecInParameter;			//内参
	std::vector<float> 	m_vecRotateMatrix;			//旋转矩阵
	std::vector<float> 	m_vecTranslationMatrix;		//平移矩阵
	std::vector<float> 	m_vecDistMatrix;			//畸变系数
	// unsigned int m_unBitStream;					//？？？	
	T485Param 			m_485Param;					//485参数
	TSuiDaoVideoParam	m_stSuiDaoVideoParam;		//隧道項目新增的与视频算法相关的參數

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TCameraDev);
		int l_strSize = m_strCameraIp.length() + m_strCameraUser.length() + m_strCameraPwd.length();
		int l_nSizeTemp = (m_vecInParameter.size() + m_vecRotateMatrix.size() + m_vecTranslationMatrix.size() + m_vecDistMatrix.size()) * sizeof(float);
		return l_nBaseSize + l_strSize + l_nSizeTemp + m_485Param.GetSize();
	}
};

struct TCameraParam
{
	unsigned char 			m_unCameraCount;	//相机个数
	bool 					m_bUse485;			//是否使用485
	bool 					m_bUseVideoServer;  //是否从视频服务器拉取视频
	std::vector<TCameraDev> m_vecCameraDev;		//相机设备参数

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TCameraParam);
		int l_nSizeTemp = 0;
		for (size_t i = 0; i < m_vecCameraDev.size(); i++)
		{
			l_nSizeTemp += m_vecCameraDev[i].GetSize();
		}
		
		return l_nBaseSize + l_nSizeTemp;
	}
};

#endif
