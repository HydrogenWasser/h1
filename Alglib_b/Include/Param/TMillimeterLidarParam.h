/*******************************************************
 文件：TMillimeterLidarParam.h
 作者：
 描述：毫米波雷达参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TMILLIMETERLIDARPARAM_H
#define TMILLIMETERLIDARPARAM_H

#include <vector>

//定义FastDDS通信的topic
#define TOPIC_GETPARAM_MILLIMETERLIDAR			"getParam_millimeterLidar"			//C端发送命令号，查询毫米波雷达参数
#define TOPIC_GETPARAMREPLY_MILLIMETERLIDAR		"getParamReply_millimeterLidar"		//S端发送命令号，返回毫米波雷达参数
#define TOPIC_SETPARAM_MILLIMETERLIDAR			"setParam_millimeterLidar"			//C端发送命令号，设置毫米波雷达参数
#define TOPIC_SETPARAMREPLY_MILLIMETERLIDAR		"setParamReply_millimeterLidar"		//S端发送命令号，返回设置成功消息


struct TMillimeterLidarParam
{
    /* data */

};


#endif
