/*******************************************************
 文件名：ExportAlgLib.h
 作者：
 描述：算法库的算法接口类导出函数头文件
 版本：v1.0
 日期：2022-09-21
 *******************************************************/

#pragma once
#include <string>
#include "IPcAlg.h"
#include "IVideoAlg.h"
#include "IFusionAlg.h"
#include "ITriggerAlg.h"
#include "ISceneAlg.h"
#include "IEventAlg.h"

extern "C" __attribute__((visibility("default"))) IPcAlg *CreatePcAlgObj(const std::string &p_strExePath);
extern "C" __attribute__((visibility("default"))) IVideoAlg *CreateVideoAlgObj(const std::string &p_strExePath);
extern "C" __attribute__((visibility("default"))) IFusionAlg *CreateFusionAlgObj(const std::string &p_strExePath);
extern "C" __attribute__((visibility("default"))) ITriggerAlg *CreateTriggerAlgObj(const std::string &p_strExePath);
extern "C" __attribute__((visibility("default"))) ISceneAlg *CreateSceneAlgObj(const std::string &p_strExePath);
extern "C" __attribute__ ((visibility("default"))) IEventAlg* CreateEventAlgObj(const std::string& p_strExePath);