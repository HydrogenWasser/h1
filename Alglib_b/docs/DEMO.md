# Quick Demo

此文档提供了一个快速运行工程的方法，可以参考一下。

## 1、点云模块

### 1.1、准备数据

工程中提供了5帧32线雷达点云数据供参考，暂时为csv数据，也可以使用npy, pcd, bin等格式的数据，但是需要自己编写对应的数据读取函数。

下方路径需要明确。
```c++
void test_pc()
{
    std::string pc_path = "/data/AlgLib/data/pc_data/";
    /* ... */
}
```


### 1.2、设置参数
```c++
TSelfPcAlgParam l_stPcAlgParam;
TLidarDev a;
l_stPcAlgParam.m_stPcAlgParam.m_BackImgPath = "/data/AlgLib/OutPut/Configs/Alg/CppConfigs/point_cloud/crop-70.bmp";
l_stPcAlgParam.m_stPcAlgParam.m_CfgPath = "/data/AlgLib/OutPut/Configs/Alg/CppConfigs/point_cloud/second/second_cpp.yaml";
l_stPcAlgParam.m_stPcAlgParam.m_bUseCropRoi = 0;                // 是否裁剪
l_stPcAlgParam.m_stPcAlgParam.m_bUseGroundPointsFilter = 0;     // 是否过滤
l_stPcAlgParam.m_stLidarParam.m_vecLidarDev.push_back(a);
l_stPcAlgParam.m_ePcAlgType = PC_DETECT_AND_TRACK;
l_stPcAlgParam.m_nLidarIndex = 0; //对应雷达ID
```

以上为简单的一些测试参数，如果是现场部署，需要读取`xml`文件中的参数，此处仅仅为运行起算法模块。其中`m_CfgPath`是必选参数，否则会报错。

### 1.3、修改程序
首先在main函数中将```testPc();```释放出来;
```c++
int main(int argc, char* argv[])
{
    testPc();
    // testVideo();
    return 0;
}
```
- 单独运行点云预处理模块，那么`testPc()`的调用如下：
    ```c++
    l_pObj->RunAlgorithm(&l_stPcSrcData, PC_PRETREATMENT);
    ```

- 如果要单独运行点云检测模块，那么`testPc()`的调用如下：
    ```c++
    l_pObj->RunAlgorithm(&l_stPcSrcData, PC_DETECT);
    ```
- 如果要运行点云检测与跟踪模块，那么`testPc()`的调用如下：
    ```c++
    l_pObj->RunAlgorithm(&l_stPcSrcData, PC_DETECT_AND_TRACK);
    ```

### 1.4、运行
```bash
cd /path/of/AlgLib
mkdir build && cd build 
cmake -DCMAKE_BUILD_TYPE=Release .. && make -j100 && cd ../OutPut
./testAlgLib
```
### 1.5、切换其他算法
需要修改对应的算法参数，参考[Algorithm.md](Algorithm.md)中的对应关系，在[Alg.xml](../OutPut/Configs/Alg/Alg.xml)中更改对应的参数，即可实现算法的切换

***
## 2、视频模块
### 2.1、准备数据

工程中提供了10帧图片供测试，也可以使用自己的数据。如果使用视频数据，需要自己编写读取函数。

下方路径需要明确。
```c++
void testVideo()
{
    std::string img_path = "/data/offline_data/img_1080/";;
    /* ... */
}
```


### 2.2、设置参数
```c++
TSelfVideoAlgParam l_stVideoAlgParam;
l_stVideoAlgParam.m_stVideoAlgParam.m_Weight_Path = "/data/AlgLib/OutPut/Configs/Alg/CppConfigs/video/yolov5/yolov5s-wj.wts";
l_stVideoAlgParam.m_eVideoAlgType = VIDEO_DETECT;
for (int i = 0; i < CamNUm; ++i) 
{
    TCameraDev l_stCameraDev;
    l_stCameraDev.m_ucCameraId = i;
    l_stVideoAlgParam.m_stCameraParam.m_vecCameraDev.push_back(l_stCameraDev);
}
```

以上为简单的一些测试参数，如果是现场部署，需要读取`xml`文件中的参数，此处仅仅为运行起算法模块。其中`m_Weight_Path`是必选参数，否则会报错。

### 2.3、修改程序
首先在main函数中将```testVideo();```释放出来;
```c++
int main(int argc, char* argv[])
{
    // testPc();
    testVideo();
    return 0;
}
```

- 如果要单独运行视频检测模块，那么`testVideo()`的调用如下：
    ```c++
    l_pObj->RunAlgorithm(&l_stPcSrcData, Video_DETECT);
    ```
- 如果要运行视频检测与跟踪模块，那么`testVideo()`的调用如下：
    ```c++
    l_pObj->RunAlgorithm(&l_stPcSrcData, VIDEO_DETECT_AND_TRACK);
    ```

### 2.4、运行
```bash
cd /path/of/AlgLib
mkdir build && cd build 
cmake -DCMAKE_BUILD_TYPE=Release .. && make -j100 && cd ../OutPut
./testAlgLib
```
### 2.5、切换其他算法
需要修改对应的算法参数，参考[Algorithm.md](Algorithm.md)中的对应关系，在[Alg.xml](../OutPut/Configs/Alg/Alg.xml)中更改对应的参数，即可实现算法的切换


### 2.6、常见错误
视频模块最常见的错误在于引擎文件与系统不匹配导致。常见错误提示如下:
```bash
[10/26/2022-07:25:57] [E] [TRT] INVALID_CONFIG: The engine plan file is generated on an incompatible device, expecting compute 8.0 got compute 8.6, please rebuild.
[10/26/2022-07:25:57] [E] [TRT] engine.cpp (1646) - Serialization Error in deserialize: 0 (Core engine deserialization failure)
[10/26/2022-07:25:57] [E] [TRT] INVALID_STATE: std::exception
[10/26/2022-07:25:57] [E] [TRT] INVALID_CONFIG: Deserialize the cuda engine failed.
```
这个问题是由于在一个GPU上生成的`.engine`文件在另一个不同算力的GPU上无法运行。解决办法非常简单，只需要将`l_stVideoAlgParam.m_stVideoAlgParam.m_Weight_Path`同级目录下的同名的`.engine`文件删除即可成功运行。

执行以上操作后，再次运行时候，会发现程序卡住, 终端提醒如下:
```bash
Loading weights: /data/AlgLib/OutPut/Configs/Alg/CppConfigs/video/yolov5/yolov5s-wj.wts
Building engine, please wait for a while...
```
这是因为找不到`.engine`文件了，因此需要根据`.tws`文件生成`.engine`文件。这个操作根据机器的不同时间也不尽相同，一般会持续三分钟左右。
## 3、融合跟踪模块
