/*******************************************************
 文件：TSelfTriggerAlgParam.h
 作者：
 描述：触发算法内部使用的参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/
#pragma once
#include "TTrigAlgParam.h"

enum TRIGGER_ALG_TYPE
{
	TRIGGER 		//触发算法
};

//触发算法参数结构体(需兼容多雷达)
struct TSelfTriggerAlgParam
{
	TSelfTriggerAlgParam() : m_eTriggerAlgType(TRIGGER){}
	virtual ~TSelfTriggerAlgParam(){}

	TRIGGER_ALG_TYPE	m_eTriggerAlgType;			//当前选取的触发算法类型
	std::string			m_strCfgPath;				//配置文件所在目录
	TTrigAlgParam     	m_stTriggerAlgParam;		//触发算法参数结构体
};
