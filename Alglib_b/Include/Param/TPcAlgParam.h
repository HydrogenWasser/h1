/*******************************************************
 文件：TPcAlgParam.h
 作者：
 描述：点云算法参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TPCALGPARAM_H
#define TPCALGPARAM_H

#include <iostream>
#include <vector>
//定义FastDDS通信的topic
#define TOPIC_GETPARAM_PCALG		"getParam_pcAlg"		//C端发送命令号，查询点云算法参数内容
#define TOPIC_GETPARAMREPLY_PCALG	"getParamReply_pcAlg"	//S端发送命令号，返回点云算法参数内容
#define TOPIC_SETPARAM_PCALG		"setParam_pcAlg"		//C端发送命令号，设置点云算法参数内容
#define TOPIC_SETPARAMREPLY_PCALG	"setParamReply_pcAlg"	//S端发送命令号，返回设置成功消息

//点云算法参数结构体
struct TPcAlgParam
{
	bool			m_bOnline;						//是否是在线模式，默认在线，无需序列化到配置文件
	/*******点云算法选择********/
	bool 			m_PcRead;						//只完成点云处理
	int 			m_PcProcessNum;					//点云处理选择
	int 			m_PcDetectNum;					//点云检测选择
	int 			m_PcTrackNum;					//点云跟踪选择
	std::string 	m_strCfgPath;					//点云检测配置文件路径， 不可为空

	/*******裁剪算法参数*******/
	bool 			m_bCropFilter;					//是否使用裁剪過濾,bmp裁剪+地面点过滤
    bool 			m_bRotTrans;					//是否使用旋轉

	std::string 	m_strBmpPath;					//bmp图地址，可以为空
	std::string 	m_strSplitBmpPath;				//扩范围后，目标分割BMP图路径
	bool	 		m_bPcProType;					//使用扩大范围方式处理点云，1：扩大范围，2：传统方式

	/*******地面点过滤算法参数*******/
	int 			m_nGridSizeSmall = 1;				//地面点过滤参数，默认为1   // zqj20230527 使用后处理计算和过滤检测框
	int 			m_nGridSizeBig = 5;					//地面点过滤参数，默认为5
	float 			m_fGridSizeFilter = 2.5;				//地面点过滤参数，默认为2.5
	float 			m_fThreshSmall = 0.2;					//地面点过滤参数， 默认0.2
	float 			m_fThreshBig = 0.4;					//地面点过滤参数， 默认0.4
	float 			m_fHThreshX = -7;					//地面点过滤参数， 默认-7
	float 			m_fHThreshY = -7;					//地面点过滤参数， 默认-7
	float 			m_fDisThresh = 80;					//地面点过滤参数， 默认80
	float           m_fRangeLow = 0.3;     			//点在垂直区间分布的最低点统计阈值
	float           m_fRangeHigh = 3.0;    			//点在垂直区间分布的最高点统计阈值
	uint16_t    	m_usNThresh = 1;        		//某格栅中在统计区间的点的个数

	unsigned int 	m_unMinPointCount;				//最小点数量?？？
	unsigned int 	m_unBmpSize;					//BMP大小
	float 			m_fCropBackSize;				//背景帧裁剪大小

	/*******点云检测参数**********/
	std::vector<std::string>	m_vecPcClass;		//点云检测类别

	/*******聚类算法参数*******/
	float 			m_fClusterRadius;				//聚类半径
	float 			m_fClusterDis;					//聚类点云选择的距离
	float 			m_fClusterMinZ;					//聚类点云的高度限制（min）
	float 			m_fClusterMaxZ;					//聚类点云的高度限制（max）

	/*******跟踪算法参数*******/
	float 			m_fCarThreshold;				//car类别过滤置信度的阈值
	float 			m_fConeThreshold;				//cone类别过滤置信度的阈值
	float 			m_fTrackMaxAge;					//跟踪的最大寿命
	float 			m_fTrackMinHits;				//跟踪的最小命中数目
	std::vector<float>	m_vecFixTruckParam;			//???
	float 			m_DeltaThresh;					//???
	

	/*******主/从基站参数，从基站通过该旋转矩阵，将识别结果坐标转换到主基站的坐标系下*******/
	float 			m_fRoughLanRotationX;			//粗转换矩阵旋转x
	float 			m_fRoughLanRotationY;			//粗转换矩阵旋转y
	float 			m_fRoughLanRotationZ;			//粗转换矩阵旋转z
	float 			m_fRoughLanTranslationX;		//粗转换矩阵平移x
	float 			m_fRoughLanTranslationY;		//粗转换矩阵平移y
	float 			m_fRoughLanTranslationZ;		//粗转换矩阵平移z

	float 			m_fFineLanRotationX;			//精转换矩阵旋转x
	float 			m_fFineLanRotationY;			//精转换矩阵旋转y
	float 			m_fFineLanRotationZ;			//精转换矩阵旋转z
	float 			m_fFineLanTranslationX;			//精转换矩阵平移x
	float 			m_fFineLanTranslationY;			//精转换矩阵平移y
	float 			m_fFineLanTranslationZ;			//精转换矩阵平移z
	
	float 			m_fBoxRotationAngle;			//Box旋转角度?？？
	int16_t 		m_sAloneToTogetherX;			//多基站中主基站中心点x像素-1000
	int16_t 		m_sAloneToTogetherY;			//多基站中主基站中心点y像素-1000
	bool 			m_bFixLanAngle;					//是否按车道修正航向角
	bool 			m_bSaveBmp;						//是否保存bmp图
	bool 			m_bSaveBackBin;					//是否保存背景帧

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TPcAlgParam);
		int l_nStrSize = m_strBmpPath.length() + m_strCfgPath.length();
		return l_nBaseSize + l_nStrSize;
	}
};


#endif
