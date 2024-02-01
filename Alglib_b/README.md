<img src="docs/imgs/wanji.png" align="right" width="8%">

# AlgLib

`AlgLib` 是万集新版融合软件中的算法开发工程。作为一个纯算法项目，由算法工程师独立维护。 此项目算法由点云检测、点云跟踪、视频检测、视频跟踪、融合跟踪算法等模块组成。作为算法输出工程供软件工程师使用。

如果第一次使用本工程，请仔细阅读此文档。

***

**Highlights**: 
* `AlgLib` has been Released to `v1.0` (2022.10).

## Overview
- [Changelog](#changelog)
- [Design Pattern](#alglib-design-pattern)
- [Alg Model](#model-zoo)
- [Installation](#installation)
- [Quick Demo](docs/DEMO.md)
- [Getting Started](docs/GETTING_STARTED.md)
- [Other](#other)


## Changelog

[2022-10-26] 添加离线测试功能，供算法工程师测试使用。并发布第一版工程。

[2022-10-24] 将视频检测算法由`YOLOv4`更新至`YOLOv5`, 原有算法存在重大缺陷，已删除。并添加视频跟踪模块，供调试使用。

[2022-10-21] 添加了标准化的点云检测算法模块 `SECOND_cpp`, 此版本为最终版本，后续不再进行维护。并添加点云跟踪模块，供调试使用。

***
## Introduction


### `AlgLib` 工具箱用来做什么?

旧版本的融合软件，由算法与软件交叉开发，存在诸多问题(不一一列举了)。因此，在经过标准化改进后，将融合软件中的算法部分拆分出来，由算法工程师独立维护开发。

算法工程师使用`AlgLib` 工具箱，为软件工程师提供对应的头文件与动态库，两者使用统一的接口，独立开发，即可完成整个融合软件的部署。

此开发流程可以简化开发流程，算法工程师从此不需要再关注融合软件，只需关注自己的算法模块即可。降低开发难度，加快开发时间。


### `AlgLib` design pattern

#### 1、算法类接口保持统一，不可再修改。
- "AlgLib/Include/Modules/Alg"目录下“I”开头的头文件内的接口函数形式，不得**增加、删除、修改**，(包括参数和返回值)
- 新增算法模块，需要在`ExportAlgLib.h`、`ExportAlgLib.cpp` 中添加新的导出函数，与已有的保持统一的格式，不得修改导出函数形式

#### 2、参数与数据接口体保持统一。
- `AlgLib/Include/Param` 和 `AlgLib/Include/Data` 下的文件原则上不允许修改，要与融合软件系统工程内的保持一致。
- 若后续因需求确实需要修改，需要跟软件一起协商，统一调整



#### 3、配置文件统一。
- 所有算法的配置文件，全部存放到 `AlgLib/OutPut/Configs/Alg`, 目录下.

#### 4、第三方库统一。
- 库文件统一存放在 `AlgLib/OutPut/TPL/Alg` 目录下，子级目录可以自己定
- 当前的第三方库已经覆盖绝大部分开发常用库，慎重添加第三方库。如需添加，需要与软件团队确认。

#### 5、算法与软件配合方式。
该工程的输出产物，输出给软件工程师，即可完成融合软件的部署。主要是以下文件
- 本工程的导出文件，即`AlgLib/Include/Modules/Alg`下的头文件，放到融合软件系统工程的 `ISFP/Include/Modules/Handlers/AlgLibInclude` 目录下;
- 本工程生成的算法库，即 `libAlg.so` 放到融合软件系统工程的 `ISFP/OutPut/Lib` 目录下;
- 本工程使用的第三方库，即 `AlgLib/OutPut/TPL/Alg` 整个目录替换 `ISFP/OutPut/TPL/Alg` 目录，或者确认两个目录的差异，同步差异项也可以;
- 本工程使用的配置文件，即 `AlgLib/OutPut/Configs/Alg` 整个目录替换 `ISFP/OutPut/Configs/Alg` 目录，或者确认两个目录的差异，同步差异项也可以.

***

## Alg Model

### 1、点云模块

当前点云检测模块依然使用`SECOND_cpp`。`SECOND_cpp`的完整开发依赖于以下两个工程:
- 训练工程
- 权重转换工程

如有需要，请至团队负责人处获取。

### 2、视频模块

原有的`YOLOv4`模块存在重大缺陷，运行速度非常慢，甚至比python版本都要慢很多。因此直接替换为了`YOLOv5`。后续不再维护`YOLOv4`，如坚持使用`YOLOv4`，请对应人员自己负责。

现有的模块[`YOLOv5`](https://github.com/ultralytics/YOLOv5)开发维护只需要一个训练工程即可。可直接github拉取。

### 3、融合跟踪模块

> 融合跟踪算法，基于海淀路口项目初步开发，此项目重新调整框架。主要算法为点云与视频检测结果先融合后跟踪，去掉了特殊策略，为通用版。

### 4、跟踪模块(点云与视频)

当前的模块保留了点云跟踪与视频跟踪，后续不再进行单独维护，如坚持使用，请对应人员自己负责。

***
## Installation

此工程依赖于`Docker images`，镜像位于FTP服务器。目前推荐使用`isfp_cpp_v2`.
> FTP地址: `/wj123/cxy/docker_images/isfp_cpp_v2.tar`

使用以上镜像可保证成功运行，且该镜像经过了特殊的定制与压缩，体积较小，功能齐全。由于目前的融合软件环境较为驳杂，建议都以此镜像为统一标准。

如使用以前的版本镜像出现问题，需要进行第三方库适配，请对应人员负责。

> **说明**:  没有任何一个镜像能够适配所有的机器，所有的环境。因此如果出现问题，请积极解决并上报。具体的问题描述以及解决，可以记录于[`DEPLOY_ERROR.md`](docs/DEPLOY_ERROR.md)文件中, 供其他人参考解决。

***
## Quick Demo
根据 [DEMO.md](docs/DEMO.md), 可以快速开始运行起点云、视频、融合等算法。

***
## Getting Started

根据 [GETTING_STARTED.md](docs/GETTING_STARTED.md) 学习如何使用此工程进行开发工作，并学习相关接口类的定义。

***
## Other
>- 请按照[GETTING_STARTED.md](docs/GETTING_STARTED.md)定义好的方式进行开发，万不可随心所欲。
>- 如果遇到解决不了的问题，或是本文档中没有说清楚的问题，请寻找团队帮助。