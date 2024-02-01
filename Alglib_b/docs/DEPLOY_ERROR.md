
# Deploy_Error

此文档用于记录工程部署过程中遇到的一些问题以及解决办法，请描述清楚。

### 目录
- [1、opencv与libav库冲突](#1opencv与libav库冲突)
- [2、编译`spconv`](#2编译spconv)
- [3、视频engine文件序列化](#3视频engine文件序列化)

***
### 1、opencv与libav库冲突
> 在使用工程中的环境读取视频时候，会出现读取失败，或编译失败的问题。后经过验证，当单独链接`libopencv*.so`的时候程序正常运行，一旦加入`libav*.so`的时候就会出现未定义的函数的问题。

解决办法:
- 尝试分开这两个库。(暂时还未尝试解决)

### 2、编译`spconv`
遇到`libspconv.so, libcuhash.so`的问题，一般都是重新编译就能解决问题了。

在推荐镜像`isfp_cpp_v2.tar`中编译`spconv`，与原有方式略有不同。
`spconv`库在`/root`下已经提供。

1. 需要首先下载`libtorch`库, FTP地址: `/wj123/cxy/docker_images/libtorch-cxx11-abi-shared-with-deps-1.11.0+cu113.zip`， 解压至任意路径
2. 安装`libboost`库，
    ```bash
    apt-get update
    apt-get install libboost-dev
    ```
3. 修改`/path/of/spconv/setup.py`
    - 修改1
        ```python
        # import torch
        # ...
        from setuptools.command.build_ext import build_ext

        # if 'LIBTORCH_ROOT' not in os.environ:
        #     raise ValueError("You must set LIBTORCH_ROOT to your torch c++ library.")

        # LIBTORCH_ROOT = str(Path(torch.__file__).parent)
        LIBTORCH_ROOT = "/path/of/libtorch"
        SPCONV_FORCE_BUILD_CUDA = os.getenv("SPCONV_FORCE_BUILD_CUDA")

        PYTHON_VERSION = "{}.{}".format(sys.version_info.major, sys.version_info.minor)

        #remove_device = re.search(r"(\+|\.)(dev|cu|cpu)", torch.__version__)
        #PYTORCH_VERSION = torch.__version__
        PYTORCH_VERSION = "1.11.0"
        #if remove_device is not None:
        #    PYTORCH_VERSION = torch.__version__[:remove_device.start()]
        PYTORCH_VERSION = list(map(int, PYTORCH_VERSION.split(".")))
        PYTORCH_VERSION_NUMBER = PYTORCH_VERSION[0] * 10000 + PYTORCH_VERSION[1] * 100 + PYTORCH_VERSION[2]
        class CMakeExtension(Extension):
            # ...
        ```
    - 修改2
        ```python
        #if not torch.cuda.is_available() and SPCONV_FORCE_BUILD_CUDA is None:
        #    cmake_args += ['-DSPCONV_BuildCUDA=OFF']
        if 0:
            pass
        else:
            cuda_flags = ["\"--expt-relaxed-constexpr\""]
            # must add following flags to use at::Half
            # but will remove raw half operators.
            cuda_flags += ["-D__CUDA_NO_HALF_OPERATORS__", "-D__CUDA_NO_HALF_CONVERSIONS__"]
            # cuda_flags += ["-D__CUDA_NO_HALF2_OPERATORS__"] 
            cmake_args += ['-DCMAKE_CUDA_FLAGS=' + " ".join(cuda_flags)]
        cfg = 'Debug' if self.debug else 'Release'
        ```
    - 修改3
        ```python
        setup(
            name='spconv',
            version='1.2.1',
            author='Yan Yan',
            author_email='scrin@foxmail.com',
            description='spatial sparse convolution for pytorch',
            long_description='',
            #setup_requires = ['torch>=1.3.0'],
            packages=packages,
            package_dir = {'spconv': 'spconv'},
            ext_modules=[CMakeExtension('spconv', library_dirs=[])],
            cmdclass=dict(build_ext=CMakeBuild),
            zip_safe=False,
        )
        ```
4. 编译
    ```bash
    python setup.py bdist_wheel
    cd build/lib.linux-x86_64-3.8/spconv
    ```
    若编译算法库AlgLib，
    ```bash
    cp *.so /path/of/AlgLib/OutPut/TPL/Alg
    ```
    若编译融合软件isfp_baseline，
    ```bash
    cp libcuhash.so /path/of/isfp_baseline/OutPut/TPL/
    cp libspconv.so /path/of/isfp_baseline/OutPut/TPL/
    ```

5. 编译‘python setup.py bdist_wheel’时报错
    ‘/root/spconv/include/spconv/reordering.cu.h:18:10: fatal error: THC/THCNumerics.cuh: No such file or directory
    18 | #include <THC/THCNumerics.cuh>
        |          ^~~~~~~~~~~~~~~~~~~~~
    compilation terminated’，
    - 修改/root/spconv/include/spconv/reordering.cu.h文件，将第18行删掉，重新编译。

### 3、视频engine文件序列化
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
