/*******************************************************
 文件名：CPcAlg.h
 作者：
 描述：点云算法接口实现，用于点运算法的运行及结果数据处理
 版本：v1.0
 日期：2020-03-22
 *******************************************************/

#pragma once

#include "IPcAlg.h"
// #include "TPcResult.h"
#include "TPcSrcData.h"
#include "TSelfPcAlgParam.h"
#include "TPcAlgResult.h"
#include "TPcAlgParam.h"
#include "TPcSrcData.h"
#include "CGeneralTrans.h"

#include "SORT_tunnel.h"
#include "VoxelNet.h"
#include "pointpillar.h"

#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include "real_time_function.h"
#include "CPostProcess.h"
#include <fstream>
// #define SAVE_INFO

class CPcAlg :public IPcAlg, CGeneralTrans
{
public:
    CPcAlg(const std::string& p_strExePath);
    virtual ~CPcAlg();

    //设置初始化以及执行算法需要的各类参数，成功返回true，失败返回false
    virtual bool  SetAlgParam(const TSelfPcAlgParam* p_pAlgParam) override;
    
    //初始化算法接口对象，内部主要处理只需初始化一次的操作，比如模型加载之类的，成功返回true，失败返回false
    virtual bool  InitAlgorithm(void* p_pInitParam=nullptr) override;

    //执行算法函数，传入原始数据体，算法执行成功返回处理后的数据或者检测结果（由算法类型而定），失败返回nullptr
    //参数1：数据源，目前约定传入 TPcSrcData* 类型数据 
    //参数2：点云算法类型，默认 PC_DETECT_AND_TRACK
    //返回值：算法处理后的数据空间地址，根据算法类型不同有些差异，具体如下：
    //PC_PRETREATMENT类型：返回 TPcSrcData* 类型数据 
    //PC_DETECT类型：返回 TPcResult* 类型数据 
    //PC_DETECT_AND_TRACK类型：返回 TPcResult* 类型数据 
    // virtual void* RunAlgorithm(void* p_pSrcData, PC_ALG_TYPE p_eAlgType=PC_DETECT) override;
    virtual void* RunAlgorithm(void* p_pSrcData) override;

private:
    //点云预处理，包括裁剪、地面点滤除、旋转
    // virtual bool Pretreatment(void* p_pPcSrcData, bool p_bIsFilter, std::string p_sIpSeq);
    virtual bool Pretreatment(void* p_pPcSrcData);
private:
    TSelfPcAlgParam *m_stSelfPcAlgParam;
    TPcAlgResult m_stPcAlgResult;
    std::string m_strOutPath;

    IPcAlgBase *m_PcDetector;
    pc_process *m_PcProcess;
    IPcAlgBase *m_tTracker;
    CPostProcess* m_PcPostProcess;

#ifdef SAVE_INFO
    //保存性能数据
	std::ofstream write;
#endif

};
