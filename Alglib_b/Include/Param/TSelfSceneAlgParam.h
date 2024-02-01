/*******************************************************
 文件：TSelfFusionAlgParam.h
 作者：
 描述：融合算法内部使用的参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/
#pragma once
// #include "TFusionAlgParam.h"
// #include "TCameraParam.h"
// #include "TLidarParam.h"
#include "JSON/json.hpp"
#include "TLidarParam.h"
//融合算法参数结构体
struct TSelfSceneAlgParam
{
    TSelfSceneAlgParam(){}
    virtual ~TSelfSceneAlgParam(){}

	bool 				m_bFusion;						//是否融合 TStartControlParam.h：false 只传递点云数据
	bool 				m_ucStationId;					//基站ID  TStationParam.h
	
	std::vector<int> 	m_vecUsedCameraId;				//当前使用所有相机的Id  TVideoAlgParam.h
	// TFusionAlgParam     m_stFusionAlgParam;				//算法参数结构体
	// TCameraParam  		m_stCameraParam;				//相机参数
	// TLidarParam 		m_stLidarParam;					//雷达参数

    uint16_t            m_usFusionWaitTime;             //最长等待时间
	// std::string			m_strCfgPath;					//配置文件所在目录
	int 				m_FusionNum;					//融合算法选择
	std::string 		m_strFusionCfgPath;				//算法配置文件相对路径
	std::string			m_strRootPath;					//算法配置文件根目录路径
	int					m_CameraId;						//隧道融合算法对应的相机ID
	nlohmann::json 	    m_fusion_parameter;
	TLidarParam     	m_stLidarParam;			//雷达参数结构体
	// nlohmann::json *m_fusion_param_handler;

};
