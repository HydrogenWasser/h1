# Getting Started

> 此文档极其重要，需要仔细阅读，并严格遵守。
***

## Overview
- [整体结构](#1整体结构)
- [接口定义](#2接口定义)
- [如何更新算法](#3如何更新算法)
- [如何开发模块](#4如何开发模块)
- [如何部署算法](#5如何部署算法)
- [后记](#6后记)

## 1、整体结构

```bash
AlgLib
├── build                           编译文件夹
├── data                            测试数据
│   ├── pc_data                     点云单帧测试数据
│   ├── video_data                  视频单帧测试数据
├── docs                            文档
├── include                         各个结构体定义
│   ├── Data                        各个模块的原始数据以及算法输出结果传输类型定义
│   ├── Modules                     算法模块声明
│   │   ├── Alg                     所有算法模块的父类声明
│   │   ├── Common                  一些工程通用函数
│   ├── Param                       各个算法模块的初始化参数类型定义
│   ├── ThirdPart                   所有第三方库的头文件
├── Modules                         算法模块实现
│   ├── Alg                         算法模块实现
│   │   ├── Include                 一些转换函数以及匈牙利算法实现
│   │   ├── FusionAlg               融合算法实现
│   │   │   ├── Crossing            路口融合跟踪算法
│   │   │   ├── tunnel            　隧道融合算法
│   │   │   ├── IFusionAlgBase.h    融合跟踪算法基类定义
│   │   ├── PointCloudAlg           点云算法实现
│   │   │   ├── PcDetection         点云检测
|   |   |   |   ├──pointpillar      PointPillar点云检测算法
|   |   |   |   ├──second           SECOND点云检测算法
│   │   │   ├── PcPretreatment      点云预处理
│   │   │   ├── tracker             点云跟踪
│   │   │   ├── IPcAlgBase.h        点云检测和跟踪算法基类定义
│   │   ├── VideoAlg                视频算法实现
│   │   │   ├── VideoDetection      视频检测算法
|   |   |   |   ├── yolov5          YoloV5视频检测算法
|   |   |   |   ├── yolov6          YoloV6视频检测算法
│   │   │   ├── VideoTrack          视频跟踪算法
│   │   │   ├── IVideoAlgBase.h     视频检测和跟踪算法基类定义
│   │   ├── TriggerAlg              相机触发算法实现
│   │   ├── CMakeLists.txt  
│   │   ├── ExportAlgLib.cpp        函数导出定义
├── OutPut                          工程输出
│   ├── Configs                     配置文件存放路径
│   │   ├── Alg      
│   │   │   ├── Alg.xml             算法选择配置文件              
│   │   │   ├── CppConfigs          各个算法模块所用的配置文件
│   │   │   |   ├── fusion          融合算法
│   │   │   |   |   ├── crossing    路口融合算法
│   │   │   |   |   ├── tunnel      隧道融合算法
│   │   │   |   |   ├── fusion.yaml 融合算法配置文件
│   │   │   |   ├── point_cloud     点云算法
|   │   │   |   |   ├── chedao      点云跟踪算法的车道线配置文件
|   │   │   |   |   ├── crop_images 点云预处理的bmp图
|   │   │   |   |   ├── pointpillar PointPillar点云检测算法的模型权重
|   │   │   |   |   ├── second      SECOND点云检测算法的配置文件和模型权重
│   │   │   |   |   ├── point_cloud.yaml 点云算法配置文件
│   │   │   |   ├── video           视频算法
|   │   │   |   |   ├── yolov5      YoloV5视频检测算法的模型权重
|   │   │   |   |   ├── yolov6      YoloV6视频检测算法的模型权重
│   │   │   |   |   ├── video.yaml  视频算法配置文件
│   ├── Lib                         算法模块动态库输出路径
│   ├── TPL                         第三方库的存放路径
│   ├── testAlgLib                  测试用例
├── CMakeLists.txt  
├── main.cpp                        测试程序 
├── README.md                       文档
```
***
## 2、接口定义

目前的算法模块都是经过调整的，已经非常规范。各位算法工程师在开发中应严格遵守目前的接口定义。以下是目前的接口定义。

### 2.1、点云处理

当前的版本重新定义为点云处理模块，包含点云预处理以及后处理等所有的非检测、跟踪部分的程序。

#### 2.1.1、创建
```c++
/***************************************************************
 * @file       real_time_function.cpp
 * @brief      点云处理对象的构造函数
 * @input      AlgParams: 参数结构体指针
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
pc_process::pc_process(TSelfPcAlgParam * AlgParams);
```
`TSelfPcAlgParam`可以在`/path/of/AlgLib/Include/Param/TSelfPcAlgParam.h`找到。该文件内容不可随意修改，需要与项目负责人沟通后进行修改，并与软件工程师同步沟通。
#### 2.1.2、调用

调用提供以下几个接口

- 裁剪与过滤
```c++
/***************************************************************
 * @file       real_time_function.cpp
 * @brief      点云预处理(裁剪与地面店过滤)
 * @input      points:          输入点云, 维度为2; 
 *             crop:            裁剪参数，true为裁剪，false为不裁剪; 
 *             filter_ground:   地面点过滤，true为过滤，false为不过滤;
 *             单位:             米.
 * @return     背景图为空时输出位false，其他情况下输出为true
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
bool pc_process::filter_points(xt::xarray<float> & points, const bool &crop, const bool & filter_ground);
```
> 多雷达情况，可以参考本工程`filter`分支中的实现，此分支暂不提供多雷达实现。

- 旋转平移
```c++
/***************************************************************
 * @file       real_time_function.cpp
 * @brief      点云平移旋转
 * @input      points: 输入点云, 维度为2; 单位为米。
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
void pc_process::rotate(xt::xarray<float> & points);
```

### 2.2、点云检测模块

#### 2.2.1、创建
```c++
/***************************************************************
 * @file       VoxelNet.cpp
 * @brief      点云检测对象的构造函数
 * @input      AlgParams: 参数结构体指针
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
VoxelNet::VoxelNet(TSelfPcAlgParam * AlgParams);

PointPillar::PointPillar(TSelfPcAlgParam * AlgParams);
```
`SECOND_cpp`中的检测主程序叫`VoxelNet`叫这个名字，如果有后续算法更新，名字可以修改，但是接口不可更改;

#### 2.2.2、调用
点云检测模块仅提供一个接口，所有处理在这个接口内完成，不再掺杂其他操作，输入输出必须保持这个形式。

```c++
/***************************************************************
 * @file       VoxelNet.cpp
 * @brief      点云检测主函数
 * @input      points_xt:   输入点云，尺寸N * 4; 单位: 米
 *             batch_size:  输入batch，默认为1，当前版本不支持修改。
 * @return     点云检测结果， 尺寸N * 9 (x, y, z, w, l, h, yaw, cls, conf)， N可为0.单位: 米、弧度
 *             输出目前使用：厘米、角度制
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
xt::xarray<float> VoxelNet::RUN(xt::xarray<float> &points_xt);

xt::xarray<float> PointPillar::RUN(xt::xarray<float> &points_xt);
```

### 2.3、点云跟踪模块

#### 2.3.1、创建
```c++
 /***************************************************************
 * @file       SORT_tunnel.cpp
 * @brief      点云跟踪对象的构造函数
 * @input      AlgParams: 参数结构体指针
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
Sort::Sort(TSelfPcAlgParam * AlgParams);
```

#### 2.3.2、调用

点云跟踪模块同样仅提供一个接口

```c++
/***************************************************************
 * @file       SORT_tunnel.cpp
 * @brief      点云跟踪主函数
 * @input      dets: 点云检测结果，尺寸N * 9 (x, y, z, w, l, h, yaw, cls, conf)， N可为0.单位: 米、弧度
 * @return     点云跟踪结果， 尺寸N * 16. N可为0. 单位: 米、角度( **注意** )
 *  *          输出目前使用：厘米、角度制
 *             [0]  : x
 *             [1]  : y
 *             [2]  : w
 *             [3]  : l
 *             [4]  : yaw
 *             [5]  : z
 *             [6]  : h
 *             [7]  : cls
 *             [8]  : speed
 *             [9]  : id
 *             [10] : conf
 *             [11] : hit_num
 *             [12] : kf_z 
 *             [13] : kf_h
 *             [14] : kf_cls 
 *             [15] : kf_conf
 * @author     陈向阳
 * @date       2022.10.12

 **************************************************************/
xt::xarray<float> Sort::RUN(xt::xarray<float> &dets);
```
此处输出，必须保证前11位输出，后五位输出不作要求，具体因需求而异，且算法输出的顺序不可更改，必须保证这个顺序。

### 2.4、视频检测

#### 2.4.1、创建
```c++
 /***************************************************************
 * @file       yolov5.cpp
 * @brief      视频检测对象的构造函数
 * @input      m_AlgParams: 参数结构体指针
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
yolov5::yolov5(TSelfVideoAlgParam *m_AlgParams);

yolov6::yolov6(TSelfVideoAlgParam *m_AlgParams);
```

#### 2.4.2、调用


```c++
/***************************************************************
 * @file       yolov5.cpp
 * @brief      视频检测主函数
 * @input      img_batch: 输入图像(寒武纪尺寸与工控机不同，还需要确认)
 * @return     视频检测结果， vector中每个元素: 尺寸N * 6 (t_l_x, t_l_y, w, h, conf, cls_id)， N可为0. 像素值均为输出尺寸下的坐标;
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
std::vector<xt::xarray<float>> yolov5::RUN(std::vector<cv::Mat> & img_batch);

std::vector<xt::xarray<float>> yolov6::RUN(std::vector<cv::Mat> & img_batch);
```

### 2.5、视频跟踪

#### 2.5.1、创建
```c++
 /***************************************************************
 * @file       use_deepsort.cpp
 * @brief      视频跟踪对象的构造函数
 * @input      m_AlgParams: 参数结构体指针
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
Use_DeepSort::Use_DeepSort(TSelfVideoAlgParam* Alg_params);
```
名字可以修改，但是接口不可更改;

#### 2.5.2、调用

```c++
/***************************************************************
 * @file       use_deepsort.cpp
 * @brief      视频跟踪主函数
 * @input      input: 视频检测结果;尺寸N * 6 (t_l_x, t_l_y, w, h, conf, cls_id)， N可为0. 像素值均为输出尺寸下的坐标;
 * @return     尺寸N * 7 (t_l_x, t_l_y, r_b_x, r_b_y, conf, cls, id)N可为0. 像素值均为输出尺寸下的坐标;
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
xt::xarray<float> Use_DeepSort::RUN(xt::xarray<float> & input);
```
此算法目前使用不多了，接口也许有不合理之处。暂时不做限制，可以修改，但是做好注释以及文档说明。

### 2.6、融合
提供离线调试接口

#### 2.6.1、创建
```c++
 /***************************************************************
 * @file       fusion_algorithm.cpp
 * @brief      融合跟踪对象的构造函数
 * @input      m_stSelfFuAlgParam: 参数结构体指针
 * @author     张诗晨
 * @date       2022.11.14
 **************************************************************/
Fusion_Algorithm_crossing::Fusion_Algorithm_crossing(TSelfFusionAlgParam *m_stAlgParam);

Fusion_Algorithm_TunnelAll::Fusion_Algorithm_TunnelAll(TSelfFusionAlgParam *m_stAlgParam);

```

#### 2.6.2、调用

```c++
/***************************************************************
 * @file       fusion_algorithm.cpp
 * @brief      融合跟踪主函数
 * @input      input: 视频及点云检测（跟踪）结果输入:
 * 米、角度制
 *                             [  0,   1,    2,    3,     4,    5,    6]
 * 视频输入(video视频跟踪结果输出):(左上x，左上y，右下x，右下y，label，score，id)
 *                             [0, 1, 2, 3, 4, 5,      6,        7,     8,   9,     10,          11~35         ]
 * 点云输入(data点云跟踪结果输出) :(x, y, z, w, l, h, angle(y偏x,-180~180), class, speed, id, conf(0~1), XYZ*8(8个角点坐标0~7))
 *                                   /|\z
 *                                   1|_________5
 *                                   /|        /|
 *                                 3/_|______7/ |----->x
 *                                  | /0      | /4
 *                                 2|/________|/
 *                                  /         6
 *                                \/_y
 * 相机个数输入 (cam_num)
 * @return     output:融合跟踪后结果输出：
 * 米、角度制   
 *             [0, 1, 2, 3,      4,       5, 6,   7,        8,     9,     10,     11,   12,            13,      14,  15,   16,    17,   18,   19,    20  ]
 *             (x, y, w, l, angle(0~360), z, h, label, speed(m/s), id, conf(0~1), hits, 车道号, timesinceupdate, 1, frame, 左上x, 左上y, 右下x, 右下y, 相机id)
 * @author     张诗晨
 * @date       2022.11.14
 **************************************************************/
Fusion_Res Fusion_Algorithm_crossing::RUN(std::vector <xt::xarray<float>> &video, xt::xarray<float> &data, int cam_num);

Fusion_Res Fusion_Algorithm_TunnelAll::RUN(std::vector <xt::xarray<float>> &video, xt::xarray<float> &data, int cam_num);

```
***

## 3、如何更新算法

以视频算法的更新为例，假设此时我们要开发`yolov7`算法。

通过阅读第二部分的接口定义, 可以看到，我们已经对算法模块的接口以及调用做了严格限制。因此，首先我们需要对自己的测试程序做如下的修改。

### 3.1、初始化接口

首先按照模板完成yolov7.h,继承视频检测算法基类
```c++
class yolov7 : public IVideoAlgBase {
public:
    yolov7();
    ~yolov7();
    yolov7(TSelfVideoAlgParam *m_AlgParams);

    std::vector<xt::xarray<float>> RUN(std::vector<cv::Mat> & img_batch);
    xt::xarray<float> RUN(xt::xarray<float> &input) {};   

private:
    /*
    *私有变量
    */
}
```

然后按照模板编写构造函数
```c++
yolov7::yolov7(TSelfVideoAlgParam *m_AlgParams)
{
    /*
     * 1. 在此构造函数内完成成员的初始化，以及开辟相应的内存空间。
     * 2. 深度学习检测算法还应该完成权重的读取，engine的生成或读取等操作。
     * 3. 如果m_AlgParams中的参数不能满足要求，应及时与团队沟通，万不可自由更改。
     */
}
```

### 3.2、调用接口

```c++
std::vector<xt::xarray<float>> yolov7::RUN(std::vector<cv::Mat> & img_batch)
{
    /*
     * 1. 在此函数内完成检测
     * 2. 将检测结果转换为std::vector<xt::xarray<float>>格式
     * 3. 将xt::xarray每一列的内容按照要求对照修改。
     */
}
```

### 3.3、内存管理
```c++
yolov7::~yolov7()
{
    /*
     * 1. 释放所开辟的内存空间，若无，可以不写该函数
     */
}
```

### 3.4、修改使用方式
以上的操作完成以后，我们就完成了一个算法的更新操作。下面还需要在工程中替换。

#### 3.4.1、声明
```c++
// CVideoAlg.h中
yolov5 *m_VideoDetector; --> yolov7 *m_VideoDetector;
```
#### 3.4.2、实例化
```c++
// CVideoAlg.cpp中
bool CVideoAlg::InitAlgorithm(void* p_pInitParam)
{
    /* ... */
    m_VideoDetector = new yolov5(&m_stSelfVideoAlgParam); --> 
    m_VideoDetector = new yolov7(&m_stSelfVideoAlgParam);
                    
    /* ... */
    return true;
}
```
这样就算是完整的完成了一个算法的更新。其他模块可以参照这种方式去更新。

### 3.5、算法兼容

可以在以上程序中看到，切换一个算法非常简单，因此如果有需要可以将不同场景的算法都放到这个工程中，如果想切换算法，只需要按照上述方法修改即可轻松实现多版本算法的兼容。

***

## 4、如何开发模块

如果我们要开发一个工程中不存在的、全新的模块，可以参考此部分进行开发。

我们以事件检测算法为例，来说明模块开发流程。

### 4.1、准备文件

#### 4.1.1、输入与输出数据

算法的输入需要首先定义好，在`/data/AlgLib/Include/Data/`中创建`TEventSrcData.h`文件(工程中已创建)。在这个文件中应该定义好数据流的格式以及内容。

然后定义好算法的输出，在`/data/AlgLib/Include/Data/`中创建`TEventResult.h`文件(工程中已创建)。在这个文件中应该定义好数据流的格式以及内容。

#### 4.1.2、参数
算法的初始化，需要定义许多参数，我们统一以结构体指针形式去创建。在`/data/AlgLib/Include/Param`中创建`TSelfEventAlgParam.h`文件，在这个文件中定义好参数结构体的格式以及内容。

#### 4.1.3、基类设计

我们需要为算法模块统一创建一个基类，供后续算法的更新以及使用。在`/data/AlgLib/Include/Modules/Alg`中创建`IEventAlg.h`文件。文件内容为事件检测模块基类对象。可以仿照`IPcAlg.h`中的形式进行设计。

```c++
struct IEventAlg
{
    IEventAlg(){}
    virtual ~IEventAlg(){}

    virtual bool  SetAlgParam(const TSelfEventAlgParam* p_pAlgParam)                        = 0;
    
    virtual bool  InitAlgorithm(void* p_pInitParam=nullptr)                                 = 0;

    virtual void* RunAlgorithm(void* p_pSrcData, EVENT_ALG_TYPE p_eAlgType=EVENT_DETECT)    = 0;
};

```

这个模板是固定的，不可以再额外增加参数或函数了，基类的设计只能如此，不允许修改。

### 4.2、准备源码

在`/data/AlgLib/Modules/Alg`中添加目录，以及对应的算法文件(`CEventAlg.cpp与CEventAlg.h`)。可参考`VideoAlg`或`PointCloudAlg`文件夹的格式创建。
```bash
/data/AlgLib/Modules/Alg
├── Include                       
├── VideoAlg                      
├── EventAlg                      
│   ├── EventDetection             事件检测算法
│   │   ├── include                头文件
│   │   ├── src                    源码
│   ├── CEventAlg.cpp              主程序
│   ├── CEventAlg.h
```
`.cpp, .c, .cxx`等源码文件需要放在src文件夹中，`.h, .hpp`等文件需要放在include中。

### 4.3、编写事件算法对象

这一部分全部都在`CEventAlg.cpp与CEventAlg.h`中完成。

#### 4.3.1、设计类对象
我们需要创建一个事件算法对象，继承`4.1.3`节中设计好的基类。如果需要使用`CGeneralTrans.h`中的函数，也可以将`CGeneralTrans`加到继承中。

```c++
class CEventAlg :public IEventAlg, CGeneralTrans
{
public:
    CEventAlg(const std::string& p_strExePath);
    virtual ~CEventAlg();
    virtual bool SetAlgParam(const TSelfEventAlgParam* p_pAlgParam) override;
    virtual bool InitAlgorithm(void* p_pInitParam=nullptr) override;
    virtual void* RunAlgorithm(void* p_pSrcData, Event_ALG_TYPE p_eAlgType=EVENT_DETECT) override;
private:
    /* 可以添加私有成员，此处允许增加、删除、修改。但需要注意代码格式 */ 
}
```
这样一个完整的事件检测对象就设计好了，接下来就可以实现这几个接口函数了。

#### 4.3.2、实现接口函数
在`CEventAlg.cpp`编写我们的接口函数实现程序。

前几个函数较为简单，就不再介绍了，重点说一下最后一个函数`RunAlgorithm`的实现。

在这个函数中，有一个参数是`p_eAlgType`，它是算法类型，事件可以有多种算法，并不是定死的一种事件检测。可以设置一个枚举类将所有可能的算法类型列举出来。在`TSelfEventAlgParam.h`中添加:

```c++
enum EVENT_ALG_TYPE
{
	EVENT_PRETREATMENT, 		
	EVENT_DETECT,				
    /* ... */
};
```
接下来解析函数的实现。
- 首先是函数的输入数据。此函数定义为`void*`的输入，可以转换为任何类型。因此，首先需要将`void* p_pSrcData`转换为我们在`4.1.1`中的`/data/AlgLib/Include/Data/TEventSrcData.h`中定义好的数据结构体类型，
- 然后是执行算法。在这个地方需要实现事件检测算法的内容。即需要引入`/data/AlgLib/Modules/Alg/EventAlg/EventDetection/`，调用算法源码中的事件检测算法，得到输出结果(注意: 源码中也需要事先定义好算法的接口，一般来说这个部分的调用接口一旦确定就不能再改变了)。
- 最后是输出。我们需要定义一个变量，类型就是我们在`4.1.1`中的`/data/AlgLib/Include/Data/TEventResult.h`中定义好的输出类型。然后将事件检测的输出结果转换为该类型，然后直接输出即可。


这样一个完成的接口函数就完成了。


### 4.4、添加接口类导出函数

文件准备好以后，需要将该函数设置为可导出函数。在`/data/AlgLib/Include/Modules/Alg/ExportAlgLib.h`中修改。
首先添加头文件
```c++
#include "IEventAlg.h"
```
然后增加以下内容即可。
```c++
extern "C" __attribute__ ((visibility("default"))) IEventAlg* CreateEventAlgObj(const std::string& p_strExePath);
```

然后需要增加该函数的实现。在`/data/AlgLib/Modules/Alg/ExportAlgLib.cpp`中增加该函数的实现。
```c++
extern "C" __attribute__ ((visibility("default"))) IEventAlg* CreateEventAlgObj(const std::string& p_strExePath)
{
    return new CEventAlg(p_strExePath);
}
```

### 4.5、添加到CMakeList中

接下来需要将上述添加的内容添加到`CMakeList.txt`中进行编译。此处涉及到一个问题，即第三方库。我们需要对引入第三方库持谨慎态度，不可随意引入。若是非引不可，那么需要按格式添加到`/data/AlgLib/Include/ThirdPart`与`/data/AlgLib/OutPut/TPL`中，然后在`CMakeList.txt`中做相应的包含与链接。

具体如何将该模块添加到`CMakeList.txt`中进行编译，可以参考`VideoAlg`与`PointCloudAlg`，不再细讲了。


到此结束，模块就开发完成了。接下来就可以在`/data/AlgLib/main.cpp`中添加函数进行测试，测试方法可以参考[`DEMO.md`](DEMO.md)。测试无误以后，就可以上线部署了。

篇幅有限不能细讲，如有问题，请在[`DEPLOY_ERROR.md`](DEPLOY_ERROR.md)中记录清楚，并寻求团队帮助。
***

## 5、如何部署算法

完成开发以后，需要将其部署。

该工程的输出产物，输出给软件工程师，即可完成融合软件的部署。主要是以下文件
- 本工程的导出文件，即`AlgLib/Include/Modules/Alg/`下的头文件，放到融合软件系统工程的 `ISFP/Include/Modules/Handlers/AlgLibInclude/` 目录下;
- 本工程生成的算法库，即 `AlgLib/OutPut/Lib/libAlg.so` 放到融合软件系统工程的 `ISFP/OutPut/Lib/` 目录下;
- 本工程使用的第三方库，即 `AlgLib/OutPut/TPL/Alg` 整个目录替换 `ISFP/OutPut/TPL/Alg` 目录，或者确认两个目录的差异，同步差异项也可以;
- 本工程使用的配置文件，即 `AlgLib/OutPut/Configs/Alg/CppConfigs/` 整个目录替换 `ISFP/OutPut/Configs/Alg/CppConfigs/` 目录，或者确认两个目录的差异，同步差异项也可以.
***

## 6、后记

> 版本维护不易，请大家按照格式要求进行开发。
> 有问题请多沟通。