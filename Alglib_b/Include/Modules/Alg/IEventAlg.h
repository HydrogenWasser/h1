
#ifndef IEVENTALGORITHM_H
#define IEVENTALGORITHM_H
#include <iostream>
#include "TSelfEventAlgParam.h"

struct IEventAlg
{
    IEventAlg(){}
    virtual ~IEventAlg(){}
    // virtual bool InitAlgorithm() = 0;
    // virtual bool RunAlgorithm(void* p_pInputParam) = 0;
    // virtual bool SetAlgConfigParam(void *p_pConfigParam) = 0;

    virtual bool  SetAlgParam(const TSelfEventAlgParam* p_pAlgParam)                        = 0;
    
    virtual bool  InitAlgorithm(void* p_pInitParam=nullptr)                                 = 0;

    // virtual void* RunAlgorithm(void* p_pSrcData, EVENT_ALG_TYPE p_eAlgType=EVENT_DETECT)    = 0; 
    // virtual void* RunAlgorithm(void* p_pSrcData, void* pB3Data, void* pB4Data, void* pB5Data, void* p_nRadarId) =0;
    virtual void* RunAlgorithm(void* p_pSrcData, void* p_nRadarId) =0; 
};


#endif