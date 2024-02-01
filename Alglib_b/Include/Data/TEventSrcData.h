/*******************************************************
 文件名：TEventSrcData.h
 作者：
 描述：Event Input Data结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TEVENTSRCDATA_H
#define TEVENTSRCDATA_H

#include <vector>
#include <string>
#include "TPcAlgResult.h"

struct TEventSrcData
{
	TPcSrcAlgResult m_cPcSrcAlgResult;
	bool m_bEventFlag; 	//事件1 4传true   其他事件传false
};

#endif
