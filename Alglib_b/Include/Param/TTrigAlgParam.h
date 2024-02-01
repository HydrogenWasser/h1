/*******************************************************
 文件：TTrigAlgParam.h
 作者：
 描述：卡口相机触发算法参数结构体
 版本：v1.0
 日期：2021-11-16
 *******************************************************/

#ifndef TTRIGALGPARAM_H
#define TTRIGALGPARAM_H

#include <vector>
#include <string>

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_TRIGGER		    "getParam_trigger"			//C端发送命令号，查询雷达参数内容
#define TOPIC_GETPARAMREPLY_TRIGGER	    "getParamReply_trigger"		//S端发送命令号，返回雷达参数内容
#define TOPIC_SETPARAM_TRIGGER		    "setParam_trigger"			//C端发送命令号，设置雷达参数内容
#define TOPIC_SETPARAMREPLY_TRIGGER	    "setParamReply_trigger" 		//S端发送命令号，返回设置成功消息


struct  TLanePoint //
{
    double              m_fLaneLon;
    double              m_fLaneLat;
    unsigned int        m_unPixelX;
    unsigned int        m_unPixelY;
    double              m_fPixelDx1;
    double              m_fPixelDy1;
    double              m_fPixelDx2;
    double              m_fPixelDy2;
    // 下边四个是对应decrease情况的值
    double              m_fPixelDx1_de;
    double              m_fPixelDy1_de;
    double              m_fPixelDx2_de;
    double              m_fPixelDy2_de;
	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TLanePoint);
		return l_nBaseSize;
	}
};

struct TLanePara
{
    std::vector<TLanePoint>      m_vecLanePoint; // 每条车道的触发区域的4个角点的经纬度
	int        			m_nCertainLinesPixelY; //固定线Y参数
	int        			m_nCertainLinesPixelX[2]; //固定线X参数,需要xy坐标
	int					m_nShiftingPixelX; //x偏移参数
    int					m_nShiftingPixelY; //y偏移参数


    //获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TLanePara);
        int l_nSizeTemp = 0;
        for (size_t i = 0; i < m_vecLanePoint.size(); i++)
        {
            l_nSizeTemp += m_vecLanePoint[i].GetSize();
        }

        return l_nBaseSize + l_nSizeTemp;

	}

};

struct TTrigAlgParam
{
    float               m_fLimitAngle_1;
    float               m_fLimitAngle_2;
    float               m_fLimitAngle_3;
    float               m_fLimitAngle_4;
    std::string         m_strIncrease;
    std::string         m_strDecrease;
    unsigned int        m_unLaneNum;
    std::vector<TLanePara>      m_vecLanePoly;      //存放所有车道的触发区域的经纬度
    unsigned int        m_unCameraPixelX;
    unsigned int        m_unCameraPixelY;
    float               m_fLaneStandardAngle;
    float               m_fSpeedCorrectionFactor;
    float               m_fPCDelayTime; //点云算法时间
    float               m_fDisCorrectionFactor; //距离修正稀疏
	bool 				m_bAxisXTriggerMode; //固定x坐标
	// std::string			m_strAxisXTriggerMode;
	bool 				m_bAxisYTriggerMode; //固定y坐标
	// std::string			m_strAxisYTriggerMode;
    std::string			m_strIsLineOrSquare;
    unsigned int        m_unXSquareExtendSmall;
    unsigned int        m_unXSquareExtendBig;
    unsigned int        m_unySquareExtendUpSmall;
    unsigned int        m_unySquareExtendDownSmall;
    unsigned int        m_unySquareExtendUpBig;
    unsigned int        m_unySquareExtendDownBig;
    bool                m_bIsBigOrSmall;
    bool                m_bIsSpeed;
    unsigned int        m_unSpeedLimit;
    unsigned int        m_unLengthLimit;
	std::string			m_strExcuteLocation; //广东祈福隧道



	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TTrigAlgParam);
		int l_strSize = m_strIncrease.length() + 
						m_strDecrease.length() + 
						m_strExcuteLocation.length();
		int l_nSizeTemp = 0;
		for (size_t i = 0; i < m_vecLanePoly.size(); i++)
		{
			l_nSizeTemp += m_vecLanePoly[i].GetSize();
		}
		
		return l_nBaseSize + l_strSize + l_nSizeTemp;
	}

};



#endif
