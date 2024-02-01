# 需要的CMake的最小版本号
cmake_minimum_required(VERSION 3.10)

# 执行文件另取别名
project(testAlgLib)
set(CMAKE_CXX_FLAGS "-O0 -g -ffast-math -ffast-math -use_fast_math")  # add by zqj20230514 for xtensor
    # "-O3"在"-O2"基础上增加一些默认项
    # -ffast-math 危险的选项，仅在编译不需要严格遵守IEEE规范且浮点计算密集的程序考虑采用。
    # -fsingle-precision-constant 浮点常量作为单精度常量对待，而不是隐式地将其转换为双精度
# SET(CMAKE_CXX_COMPILE_FEATURES g++)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-fPIC)
add_compile_options(-shared)
add_definitions(-w)   # 不显示warning
# 设置输出路径
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/OutPut/Lib) #动态库 
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/OutPut/Lib) #静态库 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/OutPut/) #执行文件 

# 内存检测工具只用于测试，提交代码时请注释
# add_definitions("-Wall -g")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -fsanitize=leak")
# add_definitions("-Wall -g")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fno-omit-frame-pointer")


# 包含头文件的路径
include_directories(Include)
include_directories(${CMAKE_SOURCE_DIR}/Include/Param)
include_directories(${CMAKE_SOURCE_DIR}/Include/Data)
include_directories(${CMAKE_SOURCE_DIR}/Include/Modules/Alg)
include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart)              # add by zqj20230516 for pc_postporecess
include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/eigen3)       # add by zqj20230516 for pc_postporecess
include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/av_opencv)
include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/tiny_xml)

# 链接动态库的路径
link_directories(${CMAKE_SOURCE_DIR}/OutPut/Lib)
link_directories(${CMAKE_SOURCE_DIR}/OutPut/TPL)
include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/cpp_algorithm)

# 链接动态库的路径
link_directories(${CMAKE_SOURCE_DIR}/OutPut/Lib)
link_directories(${CMAKE_SOURCE_DIR}/OutPut/TPL)
link_directories(${CMAKE_SOURCE_DIR}/OutPut/TPL/Alg/)
link_directories(${CMAKE_SOURCE_DIR}/OutPut/TPL/Alg/libtorch)
link_directories(${CMAKE_SOURCE_DIR}/OutPut/TPL/Alg/TensorRtLib)
link_directories(${CMAKE_SOURCE_DIR}/OutPut/TPL/av_opencv)

# 子目录的路径
add_subdirectory(Modules)

# 源文件的路径
aux_source_directory(. HW_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/OutPut/TPL/Alg/tiny_xml HW_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcPostProcess HW_SRC_FILES)  # add by zqj for pc_postprocess

# 生成可执行程序
add_executable(${PROJECT_NAME} ${HW_SRC_FILES} main.cpp)

# 链接动态库
target_link_libraries(${PROJECT_NAME} PRIVATE Alg glog )
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_core blas)   # add by zqj for nms
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_imgproc)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_imgcodecs)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_highgui)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_calib3d)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_dnn)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_features2d)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_flann)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_ml)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_objdetect)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_photo)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_stitching)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_video)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_videoio)
