/*******************************************************
 文件名：sort.h
 作者：
 描述：存储轨迹，predict，
 版本：v1.0
 日期：2023-4-4
 *******************************************************/
#ifndef ISCENEALG_H
#define ISCENEALG_H

#include <vector>
#include "TSelfSceneAlgParam.h"
// #include "xtensor.hpp"

// #include "TFusionResult.h"

// 场景枚举
enum scene_type
{
    v2x = 0,
    twin_display = 1,
    ServiceArea = 2
    
};

struct ISceneAlg
{
    ISceneAlg(){}
    virtual ~ISceneAlg(){}

    //设置初始化以及执行算法需要的各类参数，成功返回true，失败返回false
    virtual bool  SetAlgParam(const TSelfSceneAlgParam* p_pAlgParam)                            = 0;
    
    //初始化算法接口对象，内部主要处理只需初始化一次的操作，比如模型加载之类的，成功返回true，失败返回false
    // virtual bool  InitAlgorithm(scene_type* p_pInitParam=nullptr)                                      = 0;
    virtual bool  InitAlgorithm(void* p_pInitParam=nullptr)                                      = 0;

    //执行算法函数，传入融合前的结果，算法执行成功返回融合后的结果（由算法类型而定），失败返回nullptr
    //参数1：数据源，目前约定传入 TFusionResult* 类型数据
    //参数2：融合算法类型，默认 FUSION_PC_VIDEO
    //返回值：算法处理后的数据空间地址，根据算法类型不同可能有些差异，具体如下：
    //FUSION_PC_VIDEO类型：返回 TFusionResult* 类型数据
    virtual void* RunAlgorithm(void *l_pFusionResult)     = 0;
};


#endif