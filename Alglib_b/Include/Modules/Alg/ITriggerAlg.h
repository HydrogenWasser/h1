/*******************************************************
 文件名：IPcAlg.h
 作者：
 描述：算法库的触发算法接口类
 版本：v1.0
 日期：2022-09-21
 *******************************************************/

#pragma once
#include <iostream>
#include "TSelfTriggerAlgParam.h"

struct ITriggerAlg
{
    ITriggerAlg(){}
    virtual ~ITriggerAlg(){}
    
    //设置初始化以及执行算法需要的各类参数，成功返回true，失败返回false
    virtual bool  SetAlgParam(const TSelfTriggerAlgParam* p_pAlgParam)                            = 0;
    
    //初始化算法接口对象，内部主要处理只需初始化一次的操作，比如模型加载之类的，成功返回true，失败返回false
    virtual bool  InitAlgorithm(void* p_pInitParam=nullptr)                                       = 0;

    //执行算法函数，传入点云检测结果数据，执行成功返回算法结果（由算法类型而定），失败返回nullptr
    //参数1：数据源，目前约定传入 TPcResult* 类型数据 
    //参数2：触发算法类型，默认 TRIGGEWR
    //返回值：算法处理后的数据空间地址，根据算法类型不同可能有些差异(目前只有TRIGGEWR类型)，具体如下：
    //TRIGGEWR类型：返回 TTriggerAlgResult* 类型数据 
    virtual void* RunAlgorithm(void* p_pPcResult, TRIGGER_ALG_TYPE p_eAlgType=TRIGGER)             = 0;

};

