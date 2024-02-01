/*******************************************************
 文件名：TCommandData.h
 作者：
 描述：data start stop 
 版本：v1.0
 日期：2020-04-06
 *******************************************************/

#ifndef TCOMMANDDATA_H
#define TCOMMANDDATA_H

#include <vector>
#include <string>

#define TOPIC_GETDATA_COMMANDDATA		"getData_commandData"			//C端发送命令号，查询命令字数据
#define TOPIC_GETDATAREPLY_COMMANDDATA	"getDataReply_commandData"		//S端发送命令号，回复命令字数据

//数据启停协议
struct TSSProtocol
{
    unsigned char m_arrDstIp[4];
    unsigned short m_usPcSrcDataPort;
    unsigned short m_usPcResDataPort;
    unsigned char m_ucDataType;
    unsigned char m_ucRequestType;

};

//B0帧瞬时事件
struct TCommandData{
    unsigned short m_usBeginBit = 0;
    unsigned char m_ucSerialNum = 0;
    unsigned char m_ucCommandNum = 0;
    unsigned char m_ucSubComNum = 0;
    unsigned char m_ucStatusBit = 0;
    unsigned short m_usMsgLen = 0;
    //std::vector< TB0MsgContent> MsgContent ;
    TSSProtocol m_tMsgContent ;
    unsigned char m_ucCheck = 0;
    unsigned char m_ucCutoffBit = 0;
};
#endif
