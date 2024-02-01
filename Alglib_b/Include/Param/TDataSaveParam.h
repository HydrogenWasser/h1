/*******************************************************
 文件：TDataSaveParam.h
 作者：
 描述：数据保存的参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TDATASAVEPARAM_H
#define TDATASAVEPARAM_H

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_DATASAVE			"getParam_dataSave"            //C端发送命令号，查询调试参数
#define TOPIC_GETPARAMREPLY_DATASAVE		"getParamReply_dataSave"       //S端发送命令号，返回调试参数
#define TOPIC_SETPARAM_DATASAVE			"setParam_dataSave"            //C端发送命令号，设置调试参数
#define TOPIC_SETPARAMREPLY_DATASAVE		"setParamReply_dataSave"       //S端发送命令号，返回设置成功消息

//数据保存相关的参数结构体
struct TDataSaveParam
{
	bool 			m_bSaveSrcData;				//是否保存原始数据
	unsigned int 	m_unSaveFileSizeLimit;		//单个点云文件存储大小限制（最大值）,单位：Mbyte
	unsigned int 	m_unMinSpaceLimit;			//最小限制空间,小于该大小则不再保存数据（或者开始循环覆盖）,单位：Mbyte
};


#endif
