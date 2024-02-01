/*******************************************************
 文件名：CFusionAlg.h
 作者：
 描述：融合算法接口实现，用于融合算法的运行及结果数据处理
 版本：v1.0
 日期：2020-10-22
 *******************************************************/

#pragma once

#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "IFusionAlg.h"
#include "TFusionResult.h"
#include "TFusionSrcData.h"
#include "TSelfFusionAlgParam.h"
#include "TFusionAlgParam.h"
#include "CGeneralTrans.h"
#include "fusion_algorithm_crossing.h"
#include "fusion_algorithm_tunnel.h" 
#include "fusion_algorithm_intersection.h" 
#include "fusion_algorithm_ServiceArea.h"
#include "ParaMgr.h"


class CFusionAlg :public IFusionAlg,CGeneralTrans
{
public:
    CFusionAlg(const std::string& p_strOutPath);
    virtual ~CFusionAlg();

    //设置初始化以及执行算法需要的各类参数，成功返回true，失败返回false
    virtual bool  SetAlgParam(const TSelfFusionAlgParam* p_pAlgParam) override;

    //初始化算法接口对象，内部主要处理只需初始化一次的操作，比如模型加载之类的，成功返回true，失败返回false
    virtual bool  InitAlgorithm(void* p_pInitParam=nullptr) override;

    //执行算法函数，传入融合前的结果，算法执行成功返回融合后的结果（由算法类型而定），失败返回nullptr
    //参数1：数据源，目前约定传入 TFusionResult* 类型数据
    //参数2：融合算法类型，默认 FUSION_PC_VIDEO_WITH_TRACK
    //返回值：算法处理后的数据空间地址，根据算法类型不同可能有些差异，具体如下：
    //FUSION_PC_VIDEO_WITH_TRACK：返回 TFusionResult* 类型数据
    virtual void* RunAlgorithm(void* p_pSrcData) override;

    virtual TSelfFusionAlgParam* getFuAlgParam(){
        return m_stSelfFuAlgParam;
    };

private:

    virtual void setFusionResult(TFusionResult* m_tFuResult, TSelfFusionAlgParam* p_pAlgParam, const Fusion_Res& result);

    // service for ScenneAlg
    virtual void setFusionResult4Scene(TFusionOutputResult* m_tFuTrackResult, TSelfFusionAlgParam* p_pAlgParam, const Fusion_Res& result);

    virtual xt::xarray<float> VideoAlgResTransfer(TVideoResult *p_pVideoResult, TSelfFusionAlgParam* p_pAlgParam);

    virtual xt::xarray<float> PcAlgResTransfer(TPcResult *p_pPcResult, TSelfFusionAlgParam* p_pAlgParam);

    virtual int VideoAlgClassToFusionId(const std::string& p_strClass, TSelfFusionAlgParam* p_pAlgParam);

    virtual int PcAlgClassToFusionId(const std::string& p_strClass, TSelfFusionAlgParam* p_pAlgParam);

    virtual void load_csv(string _file_path);   // suyan


private:
    TSelfFusionAlgParam *m_stSelfFuAlgParam;
    TFusionResult m_tFusionResult;
    TFusionSrcData m_tFusionSrcData;
    std::string m_strOutPath;
    IFusionAlgBase *m_Fusionalgorithm;
    double 			_dLidarLon;			//激光器原点经度
	double 			_dLidarLat;			//激光器原点纬度
	float 			_fLidarNorthAngle;		//激光器与正北方向夹角

    xt::xarray<int> m_camera_reflect_limit;  // 相机反映射限制  suyan

};