/*******************************************************
 文件：TVideoCropRect.h
 作者：
 描述：视频界面框选的裁剪矩形框参数结构体
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef TVIDEOCROPRECT_H
#define TVIDEOCROPRECT_H


//视频界面框选的裁剪矩形框参数结构体
struct TVideoCropRect
{
	unsigned char 	m_ucVideoId;		//框选的裁剪矩形框属于哪张图片
	int 			m_nTopLeftX;		//图片上框选区域的左上角顶点坐标的x
	int 			m_nTopLeftY;		//图片上框选区域的左上角顶点坐标的x
	int 			m_nBottomRightX;	//图片上框选区域的左上角顶点坐标的x
	int 			m_nBottomRightY;	//图片上框选区域的左上角顶点坐标的x

	//获取本结构的总大小
	int GetSize()
	{
		int l_nBaseSize = sizeof(TVideoCropRect);
		return l_nBaseSize;
	}
};


#endif
