#include "ExportAlgLib.h"
#include "CPcAlg.h"
#include "CVideoAlg.h"
#include "CFusionAlg.h"
#include "CTriggerAlg.h"
#include "CSceneAlg.h"
#include "CEventAlg.h"

extern "C" __attribute__ ((visibility("default"))) IPcAlg* CreatePcAlgObj(const std::string& p_strExePath)
{
    return new CPcAlg(p_strExePath);
}

extern "C" __attribute__ ((visibility("default"))) IVideoAlg* CreateVideoAlgObj(const std::string& p_strExePath)
{
    return new CVideoAlg(p_strExePath);
}
extern "C" __attribute__ ((visibility("default"))) IFusionAlg* CreateFusionAlgObj(const std::string& p_strExePath)
{
    return new CFusionAlg(p_strExePath);
}
extern "C" __attribute__ ((visibility("default"))) ITriggerAlg* CreateTriggerAlgObj(const std::string& p_strExePath)
{
    return new CTriggerAlg(p_strExePath);
}

extern "C" __attribute__((visibility("default"))) ISceneAlg *CreateSceneAlgObj(const std::string &p_strExePath)
{
    return new CSceneAlg(p_strExePath);
}
extern "C" __attribute__((visibility("default"))) IEventAlg *CreateEventAlgObj(const std::string &p_strExePath)
{
    return new CEventAlg(p_strExePath);
}