#ifndef SCENEALG_H
#define SCENEALG_H

#include "ISceneAlg.h"
#include "CGeneralTrans.h"
#include <string>
#include "ParaMgrScene.h"
#include "V2xSceneAlg.h"
#include "TwinDisplaySceneAlg.h"
// #include "ServiceAreaSceneAlg.h"

// #include "xtensor.hpp"

class CSceneAlg : public ISceneAlg,CGeneralTrans
{

public:

    // CSceneAlg();
    CSceneAlg(const std::string& p_strOutPath);
    virtual ~CSceneAlg();
    virtual bool  SetAlgParam(const TSelfSceneAlgParam* p_pAlgParam) override;

    // 在该方法中，根据传入进来的枚举类型，create不同的场景算法对象
    virtual bool  InitAlgorithm(void* p_pInitParam=nullptr) override;


    virtual void* RunAlgorithm(void *l_pFusionResult) override;

    ISceneAlgBase *create_alg(scene_type type);

private: 

    virtual void setFusionResult(TFusionResult* m_tFuResult, TSelfSceneAlgParam* p_pAlgParam, const Fusion_Res_scene& result);

    

    


private:

    double 			_dLidarLon;			//激光器原点经度
	double 			_dLidarLat;			//激光器原点纬度
	float 			_fLidarNorthAngle;	//激光器与正北方向夹角

    std::string m_strOutPath;
    TSelfSceneAlgParam  *m_stSelfSceneAlgParam; 

    ISceneAlgBase *m_scene_alg_handler = nullptr;

};
#endif