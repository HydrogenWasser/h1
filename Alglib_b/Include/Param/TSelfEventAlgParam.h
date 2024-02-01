#pragma once
#include <string>
#include "TCameraParam.h"

struct TSelfEventAlgParam
{
    TSelfEventAlgParam(){}
    virtual ~TSelfEventAlgParam(){}

	std::string 		m_strEventCfgPath;			
	std::string			m_strRootPath;					
	int					m_CameraId;			
	TCameraParam  		m_stCameraParam;			//相机参数结构体			
};

enum EVENT_ALG_TYPE
{
	EVENT_PRETREATMENT, 		
	EVENT_DETECT,				
    /* ... */
};