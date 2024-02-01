/*******************************************************
 文件：TStationParam.h
 作者：
 描述：基站固有参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TSTATIONPARAM_H
#define TSTATIONPARAM_H

#include <string>
#include <vector>

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_STATION		"getParam_station"			//C端发送命令号，查询雷达参数内容
#define TOPIC_GETPARAMREPLY_STATION	"getParamReply_station"		//S端发送命令号，返回雷达参数内容
#define TOPIC_SETPARAM_STATION		"setParam_station"			//C端发送命令号，设置雷达参数内容
#define TOPIC_SETPARAMREPLY_STATION	"setParamReply_station" 	//S端发送命令号，返回设置成功消息

//基站的固有参数
struct TStationParam
{
    bool                        m_bIsMainStation;               //标识当前基站是主基站还是从基站
    unsigned char               m_ucStationId;                  //从配置文件中获取，1~255，若是高速公路，则按行驶方向递增；若是路口，则按照与正北顺时针夹角从小到大递增
    std::string                 m_strTerminalId;	            //终端ID，用于kafka识别基站等用途
    std::vector<std::string>    m_vecSlaveStationTerminaId;     //附属从基站终端ID

	//获取本结构的总大小
    int GetSize()
	{
		int l_nBaseSize = sizeof(TStationParam);
		int l_strSize = m_strTerminalId.length();
        int l_nSizeTemp = 0;
        for (size_t i = 0; i < m_vecSlaveStationTerminaId.size(); i++)
        {
            l_nSizeTemp += m_vecSlaveStationTerminaId[i].length();
        }
        
		return l_nBaseSize + l_strSize + l_nSizeTemp;
	}
};


#endif
