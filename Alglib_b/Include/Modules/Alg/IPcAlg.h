
/*******************************************************
 文件名：IPcAlg.h
 作者：
 描述：算法库的点云算法接口类
 版本：v1.0
 日期：2022-09-21
 *******************************************************/

#pragma once
#include <iostream>
#include "TSelfPcAlgParam.h"

struct IPcAlg
{
    IPcAlg(){}
    virtual ~IPcAlg(){}

    //设置初始化以及执行算法需要的各类参数，成功返回true，失败返回false
    virtual bool  SetAlgParam(const TSelfPcAlgParam* p_pAlgParam)                               = 0;
    
    //初始化算法接口对象，内部主要处理只需初始化一次的操作，比如模型加载之类的，成功返回true，失败返回false
    virtual bool  InitAlgorithm(void* p_pInitParam=nullptr)                                     = 0;

    //执行算法函数，传入原始数据体，算法执行成功返回处理后的数据或者检测结果（由算法类型而定），失败返回nullptr
    //参数1：数据源，目前约定传入 TPcSrcData* 类型数据 
    //参数2：点云算法类型，默认 PC_DETECT_AND_TRACK
    //返回值：算法处理后的数据空间地址，根据算法类型不同有些差异，具体如下：
    //PC_PRETREATMENT类型：返回 TPcSrcData* 类型数据 
    //PC_DETECT类型：返回 TPcResult* 类型数据 
    //PC_DETECT_AND_TRACK类型：返回 TPcResult* 类型数据 
    // virtual void* RunAlgorithm(void* p_pSrcData, PC_ALG_TYPE p_eAlgType=PC_DETECT_AND_TRACK)    = 0;
    virtual void* RunAlgorithm(void* p_pSrcData)                                                 = 0;
};
