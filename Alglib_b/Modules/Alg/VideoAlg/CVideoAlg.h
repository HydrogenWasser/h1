/*******************************************************
 文件名：CVideoAlg.h
 作者：
 描述：视频算法接口实现，用于视频算法的运行及结果数据处理
 版本：v1.0
 日期：2022-10-19
 *******************************************************/

#pragma once

#include "IVideoAlg.h"
#include "TVideoResult.h"
#include "TVideoSrcData.h"
#include "TSelfVideoAlgParam.h"
#include "TVideoAlgParam.h"
#include "TVideoSrcData.h"
// #include "TVideoResult.h"
#include "TVideoAlgResult.h"
#include "CGeneralTrans.h"
#include "yolov5.h"
#include "yolov6.h"
#include "use_deepsort.h"
#include "Process_reflect.h"
#include "Camera_reflect.h"
#include "ParaMgrVideo.h"

#include <iostream>
#include <vector>
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor-blas/xlapack.hpp>
#include <fstream>

class CVideoAlg :public IVideoAlg, CGeneralTrans
{
public:
    CVideoAlg(const std::string& p_strExePath);
    virtual ~CVideoAlg();

    //设置初始化以及执行算法需要的各类参数，成功返回true，失败返回false
    virtual bool  SetAlgParam(const TSelfVideoAlgParam* p_pAlgParam) override;
    
    //初始化算法接口对象，内部主要处理只需初始化一次的操作，比如模型加载之类的，成功返回true，失败返回false
    virtual bool  InitAlgorithm(void* p_pInitParam=nullptr) override;

    //执行算法函数，传入原始数据体，算法执行成功返回处理后的数据或者检测结果（由算法类型而定），失败返回nullptr
    //参数1：数据源，目前约定传入 TPcSrcData* 类型数据 
    //参数2：点云算法类型，默认 PC_DETECT_AND_TRACK
    //返回值：算法处理后的数据空间地址，根据算法类型不同有些差异，具体如下：
    //PC_PRETREATMENT类型：返回 TPcSrcData* 类型数据 
    //PC_DETECT类型：返回 TPcResult* 类型数据 
    //PC_DETECT_AND_TRACK类型：返回 TPcResult* 类型数据 
    virtual void* RunAlgorithm(void* p_pSrcData) override;

private:
    TSelfVideoAlgParam *m_stSelfVideoAlgParam;
    TVideoAlgResult m_stVideoAlgResult;
    std::string m_strOutPath;
    IVideoAlgBase *m_VideoDetector;
    std::vector<IVideoAlgBase*> m_arrVideoTracker;

    Process_reflect *m_AlgRsultReflect; //xcb add
    nlohmann::json m_parameter;
#ifdef SAVE_INFO
    //保存性能数据
	std::ofstream write;
#endif

};
