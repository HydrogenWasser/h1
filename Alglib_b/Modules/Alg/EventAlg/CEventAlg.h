/*******************************************************
 文件名：CEventAlgorithm.h
 作者：
 描述：算法接口实现，用于Event算法的运行及结果数据处理
 版本：v1.0
 日期：2020-03-22
 *******************************************************/

#ifndef CEVENTALG_H
#define CEVENTALG_H
// #include "CTrackAlgParam.h"
// #include "CTrackTargetData.h"
// #include "CServiceApp.h"
// #include "ICoreApplication.h"
#include "IEventAlg.h"
#include "TEventB3Data.h"
#include "TEventB4Data.h"
#include "TEventB5Data.h"
#include "TEventData.h"
// #include "TSelfEventAlgParam.h"
#include <iostream>
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include "traffic_flow_matrix.h"

class Traffic_Flow;
class CEventAlg : public IEventAlg //, public TGeneralAlgLoad
{
public:
    CEventAlg(const std::string& p_strExePath);
    virtual ~CEventAlg();
    // bool InitAlgorithm(int p_nRadarId) override;
    // bool RunAlgorithm(void *p_pInputParam, void* pB3Data, void* pB4Data, void* pB5Data, void* p_nRadarId) override;
    // bool SetAlgConfigParam(void *p_pConfigParam) override;

    virtual bool SetAlgParam(const TSelfEventAlgParam* p_pAlgParam) override;
    virtual bool InitAlgorithm(void* p_pInitParam=nullptr) override;
    // virtual void* RunAlgorithm(void* p_pSrcData, EVENT_ALG_TYPE p_eAlgType=EVENT_DETECT) override; 
    // virtual void* RunAlgorithm(void* p_pSrcData, void* pB3Data, void* pB4Data, void* pB5Data, void* p_nRadarId) override; 
    virtual void* RunAlgorithm(void* p_pSrcData,void* p_nRadarId) override; 
    //获取算法输出的B3帧结果数据，转换成对应的dds数据体
    bool GetB3AlgResult(std::vector<std::vector<double>> &p_refB3Data,int stationId, TEventB3Data& eventData, int b3_cnt);

    //获取算法输出的B4帧结果数据，转换成对应的dds数据体 
    bool GetB4AlgResult(std::vector<std::shared_ptr<B4_context_info>> &p_refB4Data, int stationId, TEventB4Data& eventData, int b4_cnt);
    // bool GetB4AlgResult(std::vector<std::vector<double>> &p_refB4Data, int stationId, TEventB4Data& eventData, int b4_cnt); // cjm 0927

    //获取算法输出的B5帧结果数据，转换成对应的dds数据体
//    bool GetB5AlgResult(CEventB5Data &p_refB5Data);
    // bool GetB5AlgResult(std::vector<std::vector<double>> &p_refB5Data, int stationId, TEventB5Data& eventB5Data);
    bool GetB5AlgResult(std::vector<std::shared_ptr<B5_event_info>> &p_refB5Data, int stationId, TB5Data& eventB5Data);


private:
    std::string m_strOutPath;
    Traffic_Flow *m_traffic_flow;
    TSelfEventAlgParam* m_stSelfEventAlgParam;
    TB5Data m_pEventB5Result;

    TEventB3Data pEventB3Data; // cjm 0925

    TEventB4Data pEventB4Data; // cjm 0925

    TEventData pEventData; // cjm 0928

    // xt::xarray<double> TrackAlgResTransfer(CTrackResult *p_pTrackResult, int id);
};

#endif
